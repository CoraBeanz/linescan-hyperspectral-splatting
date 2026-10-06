"""scan_mirror_bridge: the ROS side of the scan-mirror ESP32.

The ESP32 works in microsteps and its own microsecond clock (mirror_protocol.py). This node
turns that into ROS:

  /joint_states        sensor_msgs/JointState  scan_mirror_joint's angle, so TF and RViz follow
                                               the mirror (0, the 45 deg rest, until homed)
  ~/line               ScanLine                one per scan line, stamped in ROS time
  ~/state              MirrorState             homed, busy, step, angle; 10 times a second
  ~/home               std_srvs/Trigger        find the hall sensor; needed once per power-up
  ~/move               MoveMirror              turn to an angle and wait
  ~/start_sweep        StartSweep              start a sweep; returns once the ESP32 accepts it
  ~/stop               std_srvs/Trigger        stop whatever is running

Line times: the ESP32 stamps each line with its clock. Once a second the bridge sends a PING
and maps that clock onto ROS time from the round trips (clock_sync.py), so a line's stamp is
the ROS time the mirror settled, good to about a millisecond, and the arm's pose can be looked
up at that instant. It keeps trying to (re)open the port, so the ESP32 can be plugged in after
launch or reset.
"""

import math
import threading
import time

import rclpy
import serial
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from so101_scan_interfaces.msg import MirrorState, ScanLine
from so101_scan_interfaces.srv import MoveMirror, StartSweep
from so101_scan_sweep import mirror_protocol as mp
from so101_scan_sweep.clock_sync import ClockSync


class Waiter:
    """Waits in one thread for an event the reader thread receives."""

    def __init__(self, match):
        self.match = match
        self.event = None
        self._done = threading.Event()

    def offer(self, event):
        if self._done.is_set() or not self.match(event):
            return False
        self.event = event
        self._done.set()
        return True

    def wait(self, timeout):
        return self.event if self._done.wait(timeout) else None


class MirrorLink:
    """The serial port: reopens it as needed, reads lines in a thread, matches replies."""

    def __init__(self, port, baud, on_event, on_state, log):
        self.port, self.baud = port, baud
        self.on_event, self.on_state, self.log = on_event, on_state, log
        self.ser = None
        self.connected = False
        self._waiters = []
        self._lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._running = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.bad_lines = 0

    def start(self):
        self._running = True
        self._thread.start()

    def stop(self):
        self._running = False
        self._thread.join(timeout=2.0)
        self._close()

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
            if not self.connected:
                return False
            try:
                self.ser.write(text.encode())
                return True
            except (OSError, serial.SerialException) as e:
                self.log.warning("write to %s failed: %s" % (self.port, e))
                self._close()
                return False

    def request(self, text, match, timeout):
        """Send a command and wait for the reply `match` accepts; None on timeout."""
        w = self.expect(match)
        try:
            if not self.send(text):
                return None
            return w.wait(timeout)
        finally:
            self.forget(w)

    def _open(self):
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = self.port, self.baud, 0.05
        # Most ESP32 boards reset when DTR/RTS change on open; keep both released.
        s.dtr, s.rts = False, False
        s.open()
        s.reset_input_buffer()
        return s

    def _close(self):
        was = self.connected
        self.connected = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        if was:
            self.on_state(False)

    def _run(self):
        buf = b""
        warned = False
        while self._running:
            if not self.connected:
                try:
                    self.ser = self._open()
                except (OSError, serial.SerialException) as e:
                    if not warned:
                        self.log.warning("waiting for the mirror ESP32 on %s (%s)" % (self.port, e))
                        warned = True
                    time.sleep(1.0)
                    continue
                warned = False
                buf = b""
                self.connected = True
                self.log.info("connected to the mirror ESP32 on %s" % self.port)
                self.on_state(True)
            try:
                buf += self.ser.read(self.ser.in_waiting or 1)
            except (OSError, serial.SerialException) as e:
                self.log.warning("lost the mirror ESP32 on %s: %s" % (self.port, e))
                self._close()
                continue
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._dispatch(raw.decode(errors="replace"))

    def _dispatch(self, text):
        if not text.strip():
            return
        try:
            event = mp.parse(text)
        except mp.ProtocolError as e:
            # boot messages after a reset land here too
            self.bad_lines += 1
            self.log.debug("ignored %r: %s" % (text, e))
            return
        with self._lock:
            waiters = list(self._waiters)
        for w in waiters:
            w.offer(event)
        self.on_event(event)


class ScanMirrorBridge(Node):
    def __init__(self, link_factory=None, **node_kwargs):
        super().__init__("scan_mirror", **node_kwargs)
        p = self.declare_parameter
        self.port = p("port", "/dev/scan_mirror").value
        self.baud = p("baud", 115200).value
        self.microsteps_per_rev = p("microsteps_per_rev", 6400).value
        self.direction = 1 if p("direction", 1).value >= 0 else -1
        self.home_angle = p("home_angle", -0.698132).value
        self.min_angle = p("min_angle", -0.8).value
        self.max_angle = p("max_angle", 0.8).value
        self.joint_name = p("joint_name", "scan_mirror_joint").value
        self.frame_id = p("frame_id", "scan_mirror_link").value
        ping_period = p("ping_period", 1.0).value
        status_period = p("status_period", 0.1).value
        self.home_timeout = p("home_timeout", 20.0).value
        self.move_timeout = p("move_timeout", 10.0).value
        self.rad_per_step = self.direction * 2.0 * math.pi / self.microsteps_per_rev

        self.clock_sync = ClockSync()
        self.homed = False
        self.busy = False
        self.step = 0
        self.sweep_id = 0
        self.next_sweep_id = 0
        self.active = None  # (sweep_id, n_lines) while a sweep runs
        self.lines_seen = 0
        self._pings = {}  # seq -> ROS time sent, ns
        self._ping_seq = 0
        self._ping_lock = threading.Lock()
        self._warned_unsynced = False
        self._lock = threading.Lock()

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.line_pub = self.create_publisher(ScanLine, "~/line", 100)
        self.state_pub = self.create_publisher(MirrorState, "~/state", 10)

        services = ReentrantCallbackGroup()
        timers = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, "~/home", self.handle_home, callback_group=services)
        self.create_service(Trigger, "~/stop", self.handle_stop, callback_group=services)
        self.create_service(MoveMirror, "~/move", self.handle_move, callback_group=services)
        self.create_service(StartSweep, "~/start_sweep", self.handle_start_sweep, callback_group=services)
        self.create_timer(ping_period, self.send_ping, callback_group=timers)
        self.create_timer(status_period, self.poll_status, callback_group=timers)

        make = link_factory or MirrorLink
        self.link = make(self.port, self.baud, self.on_event, self.on_link_state, self.get_logger())
        self.link.start()

    # --- conversions ----------------------------------------------------------------------

    def step_to_angle(self, step):
        return self.home_angle + step * self.rad_per_step

    def angle_to_step(self, angle):
        return int(round((angle - self.home_angle) / self.rad_per_step))

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def esp_to_stamp(self, t_us):
        if self.clock_sync.ready:
            return Time(nanoseconds=self.clock_sync.to_ros_ns(t_us)).to_msg()
        if not self._warned_unsynced:
            self.get_logger().warning("line before the clock was synced; stamped with arrival time")
            self._warned_unsynced = True
        return self.get_clock().now().to_msg()

    # --- ESP32 -> ROS ---------------------------------------------------------------------

    def on_link_state(self, connected):
        if connected:
            self.clock_sync.reset()
            # a quick burst of pings so line stamps are good from the start
            threading.Thread(target=self._ping_burst, daemon=True).start()
        else:
            with self._lock:
                self.busy = False
                self.active = None
            self.publish_state()

    def _ping_burst(self):
        for _ in range(8):
            self.send_ping()
            time.sleep(0.03)

    def on_event(self, ev):
        if isinstance(ev, mp.Pong):
            with self._ping_lock:
                sent = self._pings.pop(ev.seq, None)
            if sent is not None:
                self.clock_sync.add(sent, self.now_ns(), ev.t_us)
        elif isinstance(ev, mp.Status):
            with self._lock:
                self.homed, self.busy, self.step = ev.homed, ev.busy, ev.step
                if ev.sweep_id:
                    self.sweep_id = ev.sweep_id
            self.publish_joint()
            self.publish_state()
        elif isinstance(ev, mp.Homed):
            with self._lock:
                self.homed, self.step = True, ev.step
            self.publish_joint()
        elif isinstance(ev, mp.Line):
            self.on_line(ev)
        elif isinstance(ev, mp.Done):
            with self._lock:
                active, self.active = self.active, None
                self.busy = False
            if active and active[0] == ev.sweep_id and self.lines_seen < active[1]:
                self.get_logger().warning("sweep %d ended after %d of %d lines"
                                          % (ev.sweep_id, self.lines_seen, active[1]))
            self.publish_state()
        elif isinstance(ev, mp.Err):
            self.get_logger().warning("ESP32 refused %s: %s" % (ev.command, ev.reason))
        elif isinstance(ev, mp.Log):
            self.get_logger().info("ESP32: %s" % ev.text)

    def on_line(self, ev):
        with self._lock:
            self.step = ev.step
            active = self.active
            if active and active[0] == ev.sweep_id:
                self.lines_seen += 1
        msg = ScanLine()
        msg.header.stamp = self.esp_to_stamp(ev.t_us)
        msg.header.frame_id = self.frame_id
        msg.sweep_id = ev.sweep_id
        msg.index = ev.index
        msg.step = ev.step
        msg.angle = self.step_to_angle(ev.step)
        msg.last = bool(active and active[0] == ev.sweep_id and ev.index == active[1] - 1)
        self.line_pub.publish(msg)
        self.publish_joint(msg.header.stamp, msg.angle)

    def publish_joint(self, stamp=None, angle=None):
        js = JointState()
        js.header.stamp = stamp or self.get_clock().now().to_msg()
        js.name = [self.joint_name]
        if angle is None:
            angle = self.step_to_angle(self.step) if self.homed else 0.0
        js.position = [angle]
        self.joint_pub.publish(js)

    def publish_state(self):
        st = MirrorState()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = self.frame_id
        with self._lock:
            st.homed, st.busy, st.sweep_id, st.step = self.homed, self.busy, self.sweep_id, self.step
        st.angle = self.step_to_angle(st.step) if st.homed else float("nan")
        self.state_pub.publish(st)

    # --- timers ---------------------------------------------------------------------------

    def send_ping(self):
        if not self.link.connected:
            return
        with self._ping_lock:
            self._ping_seq += 1
            seq = self._ping_seq
            for old in [s for s in self._pings if s < seq - 20]:
                del self._pings[old]  # never answered
            self._pings[seq] = self.now_ns()
        self.link.send(mp.ping(seq))

    def poll_status(self):
        if self.link.connected:
            self.link.send(mp.status())
        else:
            self.publish_joint()
            self.publish_state()

    # --- services -------------------------------------------------------------------------

    def _reply_to(self, command, *args):
        def match(ev):
            if isinstance(ev, mp.Err):
                return ev.command == command
            return isinstance(ev, mp.Ok) and ev.command == command and ev.args[:len(args)] == args
        return match

    def handle_home(self, _request, response):
        if not self.link.connected:
            response.success, response.message = False, "mirror ESP32 not connected on %s" % self.port
            return response
        homed = self.link.expect(lambda ev: isinstance(ev, (mp.Homed, mp.Err)) and
                                 (isinstance(ev, mp.Homed) or ev.command == "HOME"))
        try:
            reply = self.link.request(mp.home(), self._reply_to("HOME"), 1.0)
            if reply is None or isinstance(reply, mp.Err):
                response.success = False
                response.message = reply.reason if reply else "no reply to HOME"
                return response
            done = homed.wait(self.home_timeout)
        finally:
            self.link.forget(homed)
        if isinstance(done, mp.Homed):
            response.success, response.message = True, "homed"
        else:
            response.success = False
            response.message = done.reason if done else "no hall sensor within %.0f s" % self.home_timeout
        return response

    def handle_stop(self, _request, response):
        reply = self.link.request(mp.stop(), self._reply_to("STOP"), 1.0)
        response.success = isinstance(reply, mp.Ok)
        response.message = "stopped" if response.success else "no reply to STOP"
        return response

    def _check(self, *angles):
        if not self.link.connected:
            return "mirror ESP32 not connected on %s" % self.port
        if not self.homed:
            return "not homed; call ~/home first"
        for a in angles:
            if not self.min_angle <= a <= self.max_angle:
                return "%.4f rad is outside [%.3f, %.3f]" % (a, self.min_angle, self.max_angle)
        return None

    def handle_move(self, request, response):
        step = self.angle_to_step(request.angle)
        response.angle = self.step_to_angle(step)
        error = self._check(response.angle)
        if error is None:
            reply = self.link.request(mp.goto(step), self._reply_to("GOTO"), 1.0)
            if reply is None or isinstance(reply, mp.Err):
                error = reply.reason if reply else "no reply to GOTO"
        if error is None:
            deadline = time.monotonic() + self.move_timeout
            with self._lock:
                self.busy = True
            while time.monotonic() < deadline:
                time.sleep(0.05)
                with self._lock:
                    if not self.busy and self.step == step:
                        break
            else:
                error = "the mirror did not reach step %d within %.0f s" % (step, self.move_timeout)
        response.success = error is None
        response.message = error or "at %.4f rad" % response.angle
        return response

    def handle_start_sweep(self, request, response):
        start_step = self.angle_to_step(request.start_angle)
        start = self.step_to_angle(start_step)
        end = self.step_to_angle(start_step + request.steps_per_line * max(request.n_lines - 1, 0))
        error = self._check(start, end)
        if error is None and (request.n_lines < 1 or request.line_period <= 0.0):
            error = "need n_lines >= 1 and line_period > 0"
        if error is None:
            with self._lock:
                self.next_sweep_id += 1
                sweep_id = self.next_sweep_id
                self.active = (sweep_id, request.n_lines)
                self.lines_seen = 0
            text = mp.sweep(sweep_id, start_step, request.steps_per_line, request.n_lines,
                            int(round(request.line_period * 1e6)))
            reply = self.link.request(text, self._reply_to("SWEEP", str(sweep_id)), 1.0)
            if reply is None or isinstance(reply, mp.Err):
                error = reply.reason if reply else "no reply to SWEEP"
                with self._lock:
                    self.active = None
            else:
                with self._lock:
                    self.busy, self.sweep_id = True, sweep_id
                response.sweep_id = sweep_id
        response.accepted = error is None
        response.message = error or "sweep %d: %d lines" % (response.sweep_id, request.n_lines)
        response.start_angle = start
        response.rad_per_step = self.rad_per_step
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
