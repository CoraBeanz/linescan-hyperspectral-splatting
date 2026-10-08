"""scan_mirror_bridge: the ROS side of the scan-mirror ESP32.

The ESP32 firmware (firmware/ in this repo) works in microsteps and its own microsecond clock,
and talks plain text over USB serial (mirror_protocol.py, firmware/PROTOCOL.md). This node turns
that into ROS:

  /joint_states   sensor_msgs/JointState  scan_mirror_joint's angle, so TF and RViz follow the
                                          mirror (0, the 45 deg rest, until homed)
  ~/line          ScanLine                one per scan line, stamped in ROS time
  ~/state         MirrorState             the ESP32's state, homed, busy, angle; 10 times a second
  ~/home          std_srvs/Trigger        power the motor and find the hall sensor; needed again
                                          whenever the ESP32 restarts
  ~/move          MoveMirror              turn to an angle and wait
  ~/start_sweep   StartSweep              start a sweep; returns once the ESP32 accepts it
  ~/stop          std_srvs/Trigger        stop whatever is running

and it listens to /line_camera/frame (FrameStamp), when line_camera runs, to lock sweeps to the
camera's frames (below).

It honours the arm's e-stop: when the servo driver stops the arm (/arm_safety/state, from a
fault or an e-stop) or anything publishes true on /estop, it sends the ESP32 a STOP, which ends
a sweep, a move or homing. While the arm stays stopped it refuses ~/home, ~/move and
~/start_sweep, until /arm_safety/reset. arm_safety: false leaves the mirror out of it.

A sweep is the firmware's stare scan: on each line's tick the mirror steps, settles and holds
still until the next tick, and the ESP32 reports when it settled. Once a second the bridge sends
a PING and maps the ESP32's clock onto ROS time from the round trips (clock_sync.py), so a line's
stamp is the ROS time the mirror settled, good to about a millisecond, and the arm's pose can be
looked up at that instant. The bridge keeps trying to (re)open the port, so the ESP32 can be
plugged in after launch.

With the camera's frames coming in, a sweep is locked to them (frame_lock.py): the line period
becomes a whole number of frame periods, long enough for one exposure plus a move, and the ticks
go where the moves fall between exposures. The bridge then keeps them there with NUDGE (phase)
and PERIOD (rate) as each line reports when its tick was. Without frames, or with
frame_lock: false, sweeps run on their own clock as asked.
"""

import itertools
import math
import threading
import time
import traceback
from dataclasses import dataclass

import numpy as np
import rclpy
import serial
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from so101_scan_interfaces.msg import FrameStamp, MirrorState, ScanLine
from so101_scan_interfaces.srv import MoveMirror, StartSweep
from so101_scan_safety.watch import RESET_HINT, SafetyWatch, describe
from so101_scan_sweep import frame_lock as fl
from so101_scan_sweep import mirror_protocol as mp
from so101_scan_sweep.clock_sync import ClockSync

HOME_HINTS = {"not_found": "no magnet within a turn: check the hall sensor's wiring, then flip the magnet over"}


class Waiter:
    """Waits in one thread for a message the reader thread receives."""

    def __init__(self, match):
        self.match = match
        self.message = None
        self._done = threading.Event()

    def offer(self, message):
        if self._done.is_set() or not self.match(message):
            return False
        self.message = message
        self._done.set()
        return True

    def wait(self, timeout):
        return self.message if self._done.wait(timeout) else None


class MirrorLink:
    """The serial port: reopens it as needed, reads lines in a thread, matches replies by id."""

    def __init__(self, port, baud, on_message, on_state, log):
        self.port, self.baud = port, baud
        self.on_message, self.on_state, self.log = on_message, on_state, log
        self.ser = None
        self.connected = False
        self._waiters = []
        self._ids = itertools.count(1)
        self._lock = threading.Lock()
        self._write_lock = threading.RLock()  # also guards ser and connected
        self._running = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.skipped_lines = 0

    def start(self):
        self._running = True
        self._thread.start()

    def stop(self):
        self._running = False
        self._thread.join(timeout=2.0)
        self._close()

    def new_id(self):
        with self._lock:
            return str(next(self._ids))

    def expect(self, match):
        w = Waiter(match)
        with self._lock:
            self._waiters.append(w)
        return w

    def forget(self, waiter):
        with self._lock:
            if waiter in self._waiters:
                self._waiters.remove(waiter)

    def send(self, text):
        with self._write_lock:
            ser = self.ser
            if not self.connected or ser is None:
                return False
            try:
                ser.write(text.encode())
                return True
            except Exception as e:  # unplugged: pyserial raises more than SerialException
                self.log.warning("write to %s failed: %s" % (self.port, e))
                self._close()
                return False

    def send_command(self, verb, **fields):
        """Send a command without waiting; its reply goes to on_message like everything else."""
        return self.send(mp.command(verb, **fields, id=self.new_id()))

    def request(self, verb, timeout, **fields):
        """Send a command and wait for its reply (OK or ERR); None if none came in time."""
        cid = self.new_id()
        w = self.expect(lambda m: m.kind != "EV" and m.id == cid)
        try:
            if not self.send(mp.command(verb, **fields, id=cid)):
                return None
            return w.wait(timeout)
        finally:
            self.forget(w)

    def _open(self):
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = self.port, self.baud, 0.05
        # DTR and RTS drive the ESP32's reset and boot pins: keep both released, or opening
        # the port restarts the board
        s.dtr, s.rts = False, False
        s.open()
        s.reset_input_buffer()
        return s

    def _close(self):
        with self._write_lock:
            was, self.connected = self.connected, False
            ser, self.ser = self.ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if was:
            self.on_state(False)

    def _run(self):
        buf = b""
        warned = False
        while self._running:
            if not self.connected:
                try:
                    ser = self._open()
                except Exception as e:
                    if not warned:
                        self.log.warning("waiting for the mirror ESP32 on %s (%s)" % (self.port, e))
                        warned = True
                    time.sleep(1.0)
                    continue
                warned = False
                buf = b""
                with self._write_lock:
                    self.ser, self.connected = ser, True
                self.log.info("connected to the mirror ESP32 on %s" % self.port)
                self.on_state(True)
            ser = self.ser
            try:
                if ser is None:
                    raise serial.SerialException("the port was closed")
                buf += ser.read(ser.in_waiting or 1)
            except Exception as e:
                if self._running:
                    self.log.warning("lost the mirror ESP32 on %s: %s" % (self.port, e))
                self._close()
                continue
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._dispatch(raw.decode(errors="replace"))
            if len(buf) > 4096:  # no newline in sight: not the firmware talking
                buf = b""

    def _dispatch(self, text):
        found = mp.find_message(text)
        try:
            if found is None:
                raise mp.ProtocolError("not a protocol line")
            message = mp.parse(found)
        except mp.ProtocolError:
            # the boot ROM's output after a reset, or a line garbled on the wire
            if text.strip():
                self.skipped_lines += 1
                self.log.debug("skipped %r" % text)
            return
        self.deliver(message)

    def deliver(self, message):
        """Hand a message to the waiters it matches, then to on_message."""
        with self._lock:
            waiters = list(self._waiters)
        for w in waiters:
            w.offer(message)
        try:
            self.on_message(message)
        except Exception:
            self.log.error("handling %s failed:\n%s" % (message, traceback.format_exc()))


@dataclass
class Sweep:
    id: int
    n_lines: int
    period_us: float        # on the ESP32's clock
    seen: int = 0
    started: bool = False   # the ESP32 has accepted it (OK SCAN)
    lock: fl.LockPlan = None  # locked to the camera's frames
    nudge_pending: bool = False
    period_line: int = 0    # the line at which PERIOD was last sent
    nudges: int = 0
    phase_errors_ns: list = None
    busy_max_us: int = 0    # the longest move and settle of a line, as reported


def why(reply, verb):
    """What to tell the caller when a command got an ERR reply, or none."""
    if reply is None:
        return "no reply from the mirror ESP32 to %s" % verb
    return "the mirror ESP32 refused %s (%s)" % (verb, reply.get("msg") or reply.get("code", "?"))


class ScanMirrorBridge(Node):
    def __init__(self, link_factory=None, **node_kwargs):
        super().__init__("scan_mirror", **node_kwargs)
        p = self.declare_parameter
        self.port = p("port", "/dev/scan_mirror").value
        self.baud = p("baud", mp.BAUD).value
        self.min_angle = p("min_angle", -0.8).value
        self.max_angle = p("max_angle", 0.8).value
        self.joint_name = p("joint_name", "scan_mirror_joint").value
        self.frame_id = p("frame_id", "scan_mirror_link").value
        ping_period = p("ping_period", 1.0).value
        status_period = p("status_period", 0.1).value
        self.home_timeout = p("home_timeout", 75.0).value  # the firmware gives up after 60 s
        self.move_timeout = p("move_timeout", 10.0).value
        frame_topic = p("frame_topic", "/line_camera/frame").value
        self.frame_lock = p("frame_lock", True).value
        self.lock_margin_ns = p("lock_margin", 0.001).value * 1e9     # either side of a window
        self.nudge_threshold_ns = p("nudge_threshold", 0.0003).value * 1e9
        self.lock_lead_ns = p("lock_lead", 0.15).value * 1e9         # first tick at least this far ahead
        arm_safety = p("arm_safety", True).value

        self.clock_sync = ClockSync()
        self.rad_per_step = 2.0 * math.pi / 6400  # until the ESP32's INFO says otherwise
        self.tick_us = 50       # its step timer; INFO says too
        self.firmware = None    # the fields of its INFO reply
        self.mirror_cfg = {}    # its settings (CFG): vstart and settle set how long a line's move takes
        self.frames = fl.FrameClock()
        self.last_esp_now = None  # its clock in the last PING or STATUS reply
        self.esp_state = "disconnected"
        self.homed = False
        self.pos = 0
        self.sweep_id = 0
        self.next_sweep_id = 0
        self.active = None      # the Sweep the ESP32 is running
        self._pings = {}        # command id -> ROS time sent, ns
        self._ping_lock = threading.Lock()
        self._warned_unsynced = False
        self._lock = threading.Lock()
        self._events = {"LINE": self.on_line, "SCAN_DONE": self.on_scan_done, "HOMED": self.on_homed,
                        "MOVED": self.on_position, "STOPPED": self.on_position, "BOOT": self.on_boot,
                        "HOME_FAILED": self.on_problem, "FAULT": self.on_problem, "WARN": self.on_problem}

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.line_pub = self.create_publisher(ScanLine, "~/line", 100)
        self.state_pub = self.create_publisher(MirrorState, "~/state", 10)

        services = ReentrantCallbackGroup()
        timers = MutuallyExclusiveCallbackGroup()
        self.create_subscription(FrameStamp, frame_topic, self.on_frame, 50,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(Trigger, "~/home", self.handle_home, callback_group=services)
        self.create_service(Trigger, "~/stop", self.handle_stop, callback_group=services)
        self.create_service(MoveMirror, "~/move", self.handle_move, callback_group=services)
        self.create_service(StartSweep, "~/start_sweep", self.handle_start_sweep, callback_group=services)
        self.create_timer(ping_period, self.send_ping, callback_group=timers)
        self.create_timer(status_period, self.poll_status, callback_group=timers)

        make = link_factory or MirrorLink
        self.link = make(self.port, self.baud, self.on_message, self.on_link_state, self.get_logger())
        self.safety = None
        if arm_safety:
            estop = MutuallyExclusiveCallbackGroup()
            self.safety = SafetyWatch(
                self, on_stop=lambda msg: self.emergency_stop("the arm stopped (%s)" % describe(msg)),
                callback_group=estop)
            self.create_subscription(Bool, "/estop", self.on_estop, 10, callback_group=estop)
        self.link.start()

    # --- conversions ----------------------------------------------------------------------

    def step_to_angle(self, step):
        return step * self.rad_per_step

    def angle_to_step(self, angle):
        return int(round(angle / self.rad_per_step))

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def esp_to_ros_ns(self, t_us):
        if self.clock_sync.ready:
            return self.clock_sync.to_ros_ns(t_us)
        if not self._warned_unsynced:
            self.get_logger().warning("line before the clock was synced; stamped with arrival time")
            self._warned_unsynced = True
        return self.now_ns()

    # --- ESP32 -> ROS ---------------------------------------------------------------------

    def on_link_state(self, connected):
        if connected:
            self.restart_link()
        else:
            self.end_sweep("the mirror ESP32 was disconnected")
            with self._lock:
                self.esp_state, self.homed = "disconnected", False
            self.publish_state()

    def restart_link(self):
        """A new connection, or the ESP32 restarted: ask what it runs and resync the clock."""
        self.clock_sync.reset()
        self.last_esp_now = None
        self.link.send_command("INFO")
        self.link.send_command("CFG")
        threading.Thread(target=self._ping_burst, daemon=True).start()

    def _ping_burst(self):
        for _ in range(8):
            self.send_ping()
            time.sleep(0.03)

    def on_message(self, m):
        try:
            if m.kind == "EV":
                handler = self._events.get(m.name)
                if handler:
                    handler(m)
            elif m.kind == "ERR":
                if m.name in ("NUDGE", "PERIOD"):
                    self.end_nudge()
                    self.get_logger().warning("ESP32 refused the frame lock's %s: %s" % (m.name, m))
                else:
                    # the service that sent the command reports it to its caller too
                    self.get_logger().info("ESP32: %s" % m)
            elif m.name == "PING":
                t = m.int("t")
                self.check_clock(t)
                with self._ping_lock:
                    sent = self._pings.pop(m.id, None)
                if sent is not None:
                    self.clock_sync.add(sent, self.now_ns(), t)
            elif m.name == "STATUS":
                self.check_clock(m.int("t"))
                state = m.get("state", "?")
                with self._lock:
                    self.esp_state, self.homed, self.pos = state, m.get("homed") == "1", m.int("pos")
                    # it leaves "scanning" only as it sends SCAN_DONE, so that line was lost
                    lost = self.active is not None and self.active.started and state not in ("scanning", "stopping")
                if lost:
                    self.end_sweep("the ESP32 stopped scanning and its SCAN_DONE was lost")
                self.publish_joint()
                self.publish_state()
            elif m.name == "SCAN":
                with self._lock:
                    if self.active is not None:
                        self.active.started = True
                        self.sweep_id, self.esp_state = self.active.id, "scanning"
                self.publish_state()
            elif m.name == "INFO":
                self.on_info(m)
            elif m.name == "CFG":
                if "settle" in m.fields:     # the full list, not the reply to a change
                    self.mirror_cfg = dict(m.fields)
            elif m.name == "NUDGE":
                self.end_nudge()
        except mp.ProtocolError as e:
            self.get_logger().warning("ESP32 sent %r: %s" % (str(m), e))

    def check_clock(self, t_us):
        """Its clock going back means the ESP32 restarted and the EV BOOT got lost on the way."""
        last, self.last_esp_now = self.last_esp_now, t_us
        if last is not None and t_us < last:
            self.link.deliver(mp.Message("EV", "BOOT", {"reset": "unknown, its clock went back", "t": str(t_us)}))

    def on_info(self, m):
        self.firmware = dict(m.fields)
        if "tick_us" in m.fields:
            self.tick_us = m.int("tick_us")
        if m.int("proto") != mp.PROTOCOL_VERSION:
            self.get_logger().error("the mirror ESP32 speaks protocol %s and this bridge speaks %d; "
                                    "flash the firmware from this repo" % (m.get("proto"), mp.PROTOCOL_VERSION))
        steps_per_rev = m.int("usteps") * m.int("full_steps")
        self.rad_per_step = 2.0 * math.pi / steps_per_rev
        self.get_logger().info("mirror ESP32 firmware %s, protocol %s, %d microsteps per turn"
                               % (m.get("fw"), m.get("proto"), steps_per_rev))

    def on_boot(self, m):
        self.get_logger().warning("the mirror ESP32 has just started (reset: %s); its motor is off "
                                  "until ~/home" % m.get("reset", "?"))
        self.end_sweep("the mirror ESP32 restarted")
        with self._lock:
            self.esp_state, self.homed = "disabled", False
        self.restart_link()
        self.publish_state()

    def on_homed(self, m):
        with self._lock:
            self.homed, self.pos = True, m.int("pos")
        self.get_logger().info("mirror homed; the hall window is %s microsteps wide" % m.get("width", "?"))
        self.publish_joint()
        self.publish_state()

    def on_position(self, m):
        with self._lock:
            self.pos = m.int("pos")
        self.publish_joint()

    def on_problem(self, m):
        log = self.get_logger().error if m.name == "FAULT" else self.get_logger().warning
        log("ESP32: %s" % m)

    def on_frame(self, msg):
        self.frames.add(Time.from_msg(msg.header.stamp).nanoseconds, Time.from_msg(msg.exposure_start).nanoseconds,
                        Time.from_msg(msg.exposure_end).nanoseconds)

    def end_nudge(self):
        with self._lock:
            if self.active is not None:
                self.active.nudge_pending = False

    def track_lock(self, sweep, n, t, ready):
        """After line n of a locked sweep (its tick at t on the ESP32's clock): how far to NUDGE
        the ticks still to come (us, 0 for not at all), and a new PERIOD (us) or None."""
        if ready >= 0 and n > 0:
            busy = ready - t
            sweep.busy_max_us = max(sweep.busy_max_us, busy)
            if busy * 1000 > sweep.lock.busy_ns + sweep.lock.slack_ns / 2:
                self.get_logger().warning(
                    "sweep %d line %d: the mirror took %.1f ms to move and settle, more than the %.1f ms the "
                    "frame lock left for it; frames may catch it moving" % (
                        sweep.id, n, busy * 1e-3, (sweep.lock.busy_ns + sweep.lock.slack_ns / 2) * 1e-6),
                    throttle_duration_sec=5.0)
        now = self.now_ns()
        fit = self.frames.fit(now) if self.clock_sync.ready else None
        if fit is None:
            return 0, None   # no frames for now: the ticks carry on as they are
        err = fl.phase_error_ns(fit, sweep.lock, self.clock_sync.to_ros_ns(t))
        sweep.phase_errors_ns.append(err)
        ns_per_us = self.clock_sync.ns_per_us
        nudge = 0
        if not sweep.nudge_pending and abs(err) > self.nudge_threshold_ns and n < sweep.n_lines - 1:
            nudge = int(round(-err / ns_per_us))
            sweep.nudge_pending = True
            sweep.nudges += 1
        period = None
        want = sweep.lock.frames_per_line * fit.period_ns / ns_per_us
        if abs(want - sweep.period_us) > 0.05 and n - sweep.period_line >= 10 and n < sweep.n_lines - 2:
            period, sweep.period_us, sweep.period_line = want, want, n
        return nudge, period

    def on_line(self, m):
        n, t, pos = m.int("n"), m.int("t"), m.int("pos")
        ready = m.int("ready") if "ready" in m.fields else -1
        with self._lock:
            sweep = self.active
            self.pos = pos
            if sweep is not None:
                sweep.seen += 1
        if sweep is None:
            self.get_logger().debug("line %d of a sweep this bridge didn't start; dropped" % n)
            return
        settled = ready >= 0
        at = ready if settled else t
        stamp_ns = self.esp_to_ros_ns(at)
        # the line's hold ends at the next tick (the period after this one: a PERIOD sent now
        # starts a tick later), or earlier if this line's NUDGE pulls the ticks in
        period_us = sweep.period_us
        nudge, new_period = self.track_lock(sweep, n, t, ready) if sweep.lock is not None else (0, None)
        msg = ScanLine()
        msg.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        msg.header.frame_id = self.frame_id
        # the next line's tick, less one step-timer tick: the timer can act that much early
        moves_on = t + period_us - self.tick_us + min(nudge, 0)
        msg.hold_until = Time(nanoseconds=stamp_ns + int(round((moves_on - at) * 1000))).to_msg()
        msg.sweep_id = sweep.id
        msg.index = n
        msg.step = pos
        msg.angle = self.step_to_angle(pos)
        msg.settled = settled
        msg.last = n == sweep.n_lines - 1
        self.line_pub.publish(msg)
        if nudge:
            self.link.send_command("NUDGE", dt=nudge)
        if new_period is not None:
            self.link.send_command("PERIOD", us=new_period)
        self.publish_joint(msg.header.stamp, msg.angle)
        if not settled:
            self.get_logger().warning("sweep %d line %d: the mirror was still moving when the next line "
                                      "was due; use a longer line_period" % (sweep.id, n),
                                      throttle_duration_sec=5.0)

    def on_scan_done(self, m):
        with self._lock:
            if self.esp_state == "scanning":
                self.esp_state = "idle"
        self.end_sweep(None if m.get("aborted") == "0" else "it was stopped")
        self.publish_state()

    def end_sweep(self, reason):
        with self._lock:
            sweep, self.active = self.active, None
        if sweep is not None and (reason or sweep.seen < sweep.n_lines):
            self.get_logger().warning("sweep %d ended after %d of %d lines%s"
                                      % (sweep.id, sweep.seen, sweep.n_lines, ": " + reason if reason else ""))
        if sweep is not None and sweep.lock is not None and sweep.phase_errors_ns:
            e = np.abs(np.array(sweep.phase_errors_ns))
            self.get_logger().info("sweep %d kept to the camera's frames within %.2f ms (typically %.2f ms), "
                                   "%d nudges; moves took up to %.1f ms of the %.1f ms planned"
                                   % (sweep.id, e.max() * 1e-6, np.median(e) * 1e-6, sweep.nudges,
                                      sweep.busy_max_us * 1e-3, sweep.lock.busy_ns * 1e-6))

    def publish_joint(self, stamp=None, angle=None):
        js = JointState()
        js.header.stamp = stamp or self.get_clock().now().to_msg()
        js.name = [self.joint_name]
        if angle is None:
            with self._lock:
                angle = self.step_to_angle(self.pos) if self.homed else 0.0
        js.position = [angle]
        self.joint_pub.publish(js)

    def publish_state(self):
        st = MirrorState()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = self.frame_id
        with self._lock:
            st.state, st.homed, st.sweep_id, st.step = self.esp_state, self.homed, self.sweep_id, self.pos
            st.busy = self.esp_state in mp.BUSY_STATES or self.active is not None
        st.angle = self.step_to_angle(st.step) if st.homed else float("nan")
        self.state_pub.publish(st)

    # --- timers ---------------------------------------------------------------------------

    def send_ping(self):
        if not self.link.connected:
            return
        cid = self.link.new_id()
        with self._ping_lock:
            while len(self._pings) > 20:   # never answered
                self._pings.pop(next(iter(self._pings)))
            self._pings[cid] = self.now_ns()
        self.link.send(mp.command("PING", id=cid))

    def poll_status(self):
        if self.link.connected:
            self.link.send_command("STATUS")
        else:
            self.publish_joint()
            self.publish_state()

    # --- services -------------------------------------------------------------------------

    @staticmethod
    def fail(response, message):
        response.success, response.message = False, message
        return response

    def handle_home(self, _request, response):
        if not self.link.connected:
            return self.fail(response, "mirror ESP32 not connected on %s" % self.port)
        stopped = self.arm_stopped()
        if stopped:
            return self.fail(response, stopped)
        # the motor is off after every start of the ESP32; ENABLE powers it and clears a fault
        reply = self.link.request("ENABLE", 3.0)
        if reply is None or reply.kind == "ERR":
            return self.fail(response, why(reply, "ENABLE"))
        done = self.link.expect(lambda m: m.kind == "EV" and m.name in ("HOMED", "HOME_FAILED", "BOOT"))
        try:
            reply = self.link.request("HOME", 2.0)
            if reply is None or reply.kind == "ERR":
                return self.fail(response, why(reply, "HOME"))
            ev = done.wait(self.home_timeout)
        finally:
            self.link.forget(done)
        if ev is None:
            return self.fail(response, "homing took over %.0f s" % self.home_timeout)
        if ev.name == "BOOT":
            return self.fail(response, "the mirror ESP32 restarted while homing")
        if ev.name == "HOME_FAILED":
            reason = ev.get("reason", "?")
            hint = HOME_HINTS.get(reason)
            return self.fail(response, "homing failed: %s%s" % (reason, "; " + hint if hint else ""))
        response.success = True
        response.message = "homed; the hall window is %s microsteps wide" % ev.get("width", "?")
        return response

    def on_estop(self, msg):
        if msg.data:
            self.emergency_stop("e-stop on /estop")

    def emergency_stop(self, cause):
        """STOP without waiting for the reply, so it goes out at once whatever else is running."""
        sent = self.link.send_command("STOP")
        self.get_logger().warning("%s; %s" % (cause, "stopping the mirror" if sent else
                                              "the mirror ESP32 isn't connected"))

    def arm_stopped(self):
        if self.safety is not None and self.safety.stopped:
            return "the arm is stopped (%s); %s" % (describe(self.safety.state), RESET_HINT)
        return None

    def handle_stop(self, _request, response):
        if not self.link.connected:
            return self.fail(response, "mirror ESP32 not connected on %s" % self.port)
        stopped = self.link.expect(lambda m: m.kind == "EV" and m.name in ("STOPPED", "BOOT"))
        try:
            reply = self.link.request("STOP", 2.0)
            if reply is None or reply.kind == "ERR":
                return self.fail(response, why(reply, "STOP"))
            ev = stopped.wait(2.0)
        finally:
            self.link.forget(stopped)
        response.success = True
        response.message = ("stopped at %.4f rad" % self.step_to_angle(ev.int("pos"))
                            if ev is not None and ev.name == "STOPPED" else "stop sent")
        return response

    def _check(self, *angles):
        if not self.link.connected:
            return "mirror ESP32 not connected on %s" % self.port
        if self.firmware is not None and self.firmware.get("proto") != str(mp.PROTOCOL_VERSION):
            return "the mirror ESP32 speaks protocol %s, not %d" % (self.firmware.get("proto"), mp.PROTOCOL_VERSION)
        with self._lock:
            homed = self.homed
        if not homed:
            return "not homed; call ~/home first"
        for a in angles:
            if not self.min_angle <= a <= self.max_angle:
                return "%.4f rad is outside [%.3f, %.3f]" % (a, self.min_angle, self.max_angle)
        return self.arm_stopped()

    def handle_move(self, request, response):
        step = self.angle_to_step(request.angle)
        response.angle = self.step_to_angle(step)
        error = self._check(response.angle)
        if error:
            return self.fail(response, error)
        done = self.link.expect(lambda m: m.kind == "EV" and (
            m.name in ("STOPPED", "FAULT", "BOOT") or (m.name == "MOVED" and m.get("pos") == str(step))))
        try:
            reply = self.link.request("MOVE", 2.0, pos=step)
            if reply is None or reply.kind == "ERR":
                return self.fail(response, why(reply, "MOVE"))
            ev = done.wait(self.move_timeout)
        finally:
            self.link.forget(done)
        if ev is None:
            return self.fail(response, "the mirror did not get to %.4f rad within %.0f s"
                             % (response.angle, self.move_timeout))
        if ev.name != "MOVED":
            return self.fail(response, "the move ended early: %s" % ev)
        response.success, response.message = True, "at %.4f rad" % response.angle
        return response

    def plan_lock(self, request):
        """(LockPlan, t0 on the ESP32's clock, period in ESP32 us) for a sweep locked to the
        camera's frames, or None when there are none to lock to."""
        if not self.frame_lock or not self.clock_sync.ready:
            return None
        fit = self.frames.fit(self.now_ns())
        if fit is None:
            return None
        cfg = self.mirror_cfg
        busy = fl.busy_ns(request.steps_per_line, float(cfg.get("vstart", 1600)), float(cfg.get("settle", 3000)),
                          self.tick_us)
        lock = fl.plan_lock(fit, request.line_period * 1e9, busy, self.lock_margin_ns)
        first = fl.first_tick_ns(fit, lock, self.now_ns() + self.lock_lead_ns)
        return lock, int(round(self.clock_sync.to_esp_us(first))), lock.line_period_ns / self.clock_sync.ns_per_us

    def handle_start_sweep(self, request, response):
        start = self.angle_to_step(request.start_angle)
        response.start_angle = self.step_to_angle(start)
        response.rad_per_step = self.rad_per_step
        end = start + request.steps_per_line * max(request.n_lines - 1, 0)
        error = self._check(response.start_angle, self.step_to_angle(end))
        if error is None and (request.n_lines < 1 or request.line_period < 0.001):
            error = "need n_lines >= 1 and line_period >= 0.001 s"
        locked = self.plan_lock(request) if error is None else None
        lock, t0, period_us = locked if locked else (None, None, request.line_period * 1e6)
        sweep = None
        if error is None:
            with self._lock:
                if self.active is not None:
                    error = "sweep %d is still running" % self.active.id
                else:
                    self.next_sweep_id += 1
                    sweep = self.active = Sweep(self.next_sweep_id, request.n_lines, period_us, lock=lock,
                                                phase_errors_ns=[])
        if sweep is not None:
            reply = self.link.request("SCAN", 2.0, mode="stare", start=start, step=request.steps_per_line,
                                      lines=request.n_lines, period=sweep.period_us, t0=t0)
            if reply is None or reply.kind == "ERR":
                error = why(reply, "SCAN")
                with self._lock:
                    if self.active is sweep:
                        self.active = None
            else:
                response.sweep_id = sweep.id
        response.accepted = error is None
        response.frame_locked = lock is not None
        if lock is not None:
            response.line_period = lock.line_period_ns * 1e-9
            response.frames_per_line = lock.frames_per_line
        else:
            response.line_period = request.line_period
        if error:
            response.message = error
        elif lock is not None:
            response.message = ("sweep %d: %d lines locked to the camera, %d frame%s a line (%.1f ms), "
                                "%d still for its whole exposure" % (
                                    response.sweep_id, request.n_lines, lock.frames_per_line,
                                    "" if lock.frames_per_line == 1 else "s", lock.line_period_ns * 1e-6,
                                    lock.good_frames))
        else:
            response.message = "sweep %d: %d lines%s" % (response.sweep_id, request.n_lines,
                                                          ", not locked to a camera" if self.frame_lock else "")
        return response

    def destroy_node(self):
        self.link.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScanMirrorBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
