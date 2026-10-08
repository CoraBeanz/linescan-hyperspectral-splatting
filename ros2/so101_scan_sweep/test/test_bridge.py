"""scan_mirror_bridge against the simulated ESP32: homing, sweeps, line stamps, refusals, the
ESP32 restarting, and sweeps locked to a camera's frames."""

import math
import os
import select
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from so101_scan_interfaces.msg import ArmSafety, FrameStamp, MirrorState, ScanLine
from so101_scan_interfaces.srv import MoveMirror, StartSweep
from so101_scan_safety import watch
from so101_scan_sweep import frame_lock as fl
from so101_scan_sweep.bridge import MirrorLink, ScanMirrorBridge, Sweep
from so101_scan_sweep.clock_sync import ClockSync
from so101_scan_sweep.fake_mirror import FakeMirror

RAD_PER_STEP = 2 * math.pi / 6400


@pytest.fixture
def fake(tmp_path):
    mirror = FakeMirror(link=str(tmp_path / "mirror"), drift_ppm=50.0, seed=1).start()
    yield mirror
    mirror.stop()


class Client:
    def __init__(self, context):
        self.node = rclpy.create_node("bridge_test", context=context)
        self.lines, self.joints, self.state = [], [], None
        self.node.create_subscription(ScanLine, "/scan_mirror/line", self.lines.append, 100)
        self.node.create_subscription(JointState, "/joint_states", self.joints.append, 100)
        self.node.create_subscription(MirrorState, "/scan_mirror/state", self.on_state, 10)
        self.clients = {
            "home": self.node.create_client(Trigger, "/scan_mirror/home"),
            "stop": self.node.create_client(Trigger, "/scan_mirror/stop"),
            "move": self.node.create_client(MoveMirror, "/scan_mirror/move"),
            "start_sweep": self.node.create_client(StartSweep, "/scan_mirror/start_sweep"),
        }

    def on_state(self, msg):
        self.state = msg

    def call(self, name, request, timeout=30.0):
        client = self.clients[name]
        assert client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        assert done.wait(timeout), name
        return future.result()

    def wait_for(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, "timed out"
            time.sleep(0.02)


@pytest.fixture
def ros(fake):
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=int(os.environ.get("TEST_DOMAIN_ID", 77)))
    bridge = ScanMirrorBridge(context=context, parameter_overrides=[
        Parameter("port", value=fake.link), Parameter("ping_period", value=0.2)])
    client = Client(context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(bridge)
    executor.add_node(client.node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client.wait_for(lambda: client.state is not None and bridge.clock_sync.ready)
    yield client, bridge
    executor.shutdown()
    bridge.destroy_node()
    client.node.destroy_node()
    rclpy.shutdown(context=context)


def sweep_request(start, steps, n, period):
    return StartSweep.Request(start_angle=start, steps_per_line=steps, n_lines=n, line_period=period)


def home(client):
    res = client.call("home", Trigger.Request())
    assert res.success, res.message
    client.wait_for(lambda: client.state.homed and client.state.state == "idle")


def test_needs_homing_first(ros, fake):
    client, _ = ros
    client.wait_for(lambda: client.state.state == "disabled")   # as the ESP32 starts: motor off
    res = client.call("start_sweep", sweep_request(0.0, 2, 5, 0.02))
    assert not res.accepted and "not homed" in res.message
    assert not client.call("move", MoveMirror.Request(angle=0.1)).success
    home(client)
    assert client.state.angle == pytest.approx(0.0)              # HOME parks at the 45 deg rest
    verbs = [line.split()[0] for line in fake.received]
    assert verbs.index("ENABLE") < verbs.index("HOME")           # the motor is powered first


def test_homing_failure_is_reported(ros, fake):
    client, _ = ros
    fake.hall = False
    res = client.call("home", Trigger.Request())
    assert not res.success and "not_found" in res.message and "magnet" in res.message


def ns(stamp):
    return rclpy.time.Time.from_msg(stamp).nanoseconds


def check_sweep(client, fake, res, n, period):
    """The lines of one sweep: all there, at the right angles, stamped when the fake settled."""
    client.wait_for(lambda: any(m.sweep_id == res.sweep_id and m.last for m in client.lines))
    lines = [m for m in client.lines if m.sweep_id == res.sweep_id]
    assert [m.index for m in lines] == list(range(n))
    for m in lines:
        assert m.angle == pytest.approx(res.start_angle + 2 * m.index * RAD_PER_STEP, abs=1e-9)
        assert m.step == round(m.angle / RAD_PER_STEP) and m.settled
        # the mirror holds still from the stamp until the next line is due
        assert 0 < ns(m.hold_until) - ns(m.header.stamp) <= period * 1e9 + 1e6
    assert [m.last for m in lines] == [False] * (n - 1) + [True]
    # each stamp is the ROS time the fake reported the line ready, from its drifting, offset clock
    ready = {i: wall for scan, i, wall in fake.line_times if scan == fake.scans}
    errors_ms = [abs(ns(m.header.stamp) - ready[m.index]) * 1e-6 for m in lines]
    assert max(errors_ms) < 3.0, errors_ms
    return lines


def test_sweep_lines_angles_and_stamps(ros, fake):
    client, _ = ros
    home(client)
    start = -0.05
    res = client.call("start_sweep", sweep_request(start, 2, 25, 0.02))
    assert res.accepted, res.message
    assert res.rad_per_step == pytest.approx(RAD_PER_STEP)
    assert abs(res.start_angle - start) <= RAD_PER_STEP / 2
    lines = check_sweep(client, fake, res, 25, 0.02)
    # the mirror joint followed the sweep
    mirror = [j.position[0] for j in client.joints if j.name == ["scan_mirror_joint"]]
    assert any(abs(a - lines[-1].angle) < 1e-9 for a in mirror)
    client.wait_for(lambda: client.state.sweep_id == res.sweep_id and not client.state.busy)


def test_refuses_out_of_range_and_bad_requests(ros):
    client, _ = ros
    home(client)
    assert not client.call("start_sweep", sweep_request(0.7, 10, 50, 0.02)).accepted  # ends past 0.8 rad
    assert not client.call("start_sweep", sweep_request(0.0, 2, 0, 0.02)).accepted
    assert not client.call("start_sweep", sweep_request(0.0, 2, 5, 0.0005)).accepted  # under 1 ms a line
    assert not client.call("move", MoveMirror.Request(angle=1.5)).success
    res = client.call("start_sweep", sweep_request(0.0, 1, 100, 0.02))
    assert res.accepted
    second = client.call("start_sweep", sweep_request(0.0, 1, 5, 0.02))
    assert not second.accepted and "still running" in second.message


def test_move_rounds_to_a_step(ros, fake):
    client, _ = ros
    home(client)
    res = client.call("move", MoveMirror.Request(angle=0.1))
    assert res.success, res.message
    assert abs(res.angle - 0.1) <= RAD_PER_STEP / 2
    assert fake.pos == round(res.angle / RAD_PER_STEP)


def test_stop_ends_a_sweep(ros):
    client, _ = ros
    home(client)
    res = client.call("start_sweep", sweep_request(0.0, 1, 500, 0.02))
    assert res.accepted
    client.wait_for(lambda: len(client.lines) >= 5)
    stop = client.call("stop", Trigger.Request())
    assert stop.success and "stopped at" in stop.message
    client.wait_for(lambda: client.state.sweep_id == res.sweep_id and not client.state.busy)
    n = len(client.lines)
    time.sleep(0.2)
    assert len(client.lines) == n < 500


def test_the_arm_stopping_stops_the_mirror(ros):
    client, bridge = ros
    home(client)
    arm = client.node.create_publisher(ArmSafety, watch.TOPIC, watch.QOS)
    estop = client.node.create_publisher(Bool, "/estop", 10)

    def sweep_then(stop):
        res = client.call("start_sweep", sweep_request(0.0, 1, 500, 0.02))
        assert res.accepted, res.message
        client.wait_for(lambda: sum(m.sweep_id == res.sweep_id for m in client.lines) >= 5)
        stop()
        client.wait_for(lambda: client.state.sweep_id == res.sweep_id and not client.state.busy)
        n = len(client.lines)
        time.sleep(0.2)
        assert len(client.lines) == n
        assert sum(m.sweep_id == res.sweep_id for m in client.lines) < 500

    arm.publish(ArmSafety(state=ArmSafety.OK))
    client.wait_for(lambda: bridge.safety.state is not None)
    # the driver stops the arm: the sweep ends, and nothing else moves the mirror until a reset
    sweep_then(lambda: arm.publish(ArmSafety(state=ArmSafety.HOLDING, reason="e-stop (/arm_safety/estop)")))
    refused = client.call("start_sweep", sweep_request(0.0, 1, 5, 0.02))
    assert not refused.accepted
    assert "the arm is stopped (holding: e-stop (/arm_safety/estop))" in refused.message
    assert "arm_estop --reset" in refused.message
    assert "arm is stopped" in client.call("move", MoveMirror.Request(angle=0.1)).message
    assert "arm is stopped" in client.call("home", Trigger.Request()).message
    arm.publish(ArmSafety(state=ArmSafety.RESUMING))
    client.wait_for(lambda: not bridge.safety.stopped)
    # /estop stops it too, without the driver
    sweep_then(lambda: estop.publish(Bool(data=True)))


@pytest.mark.parametrize("boot_lost", [False, True])
def test_esp32_restart_mid_sweep(ros, fake, boot_lost):
    """A reset ends the sweep and the homing, whether or not its EV BOOT gets through."""
    client, bridge = ros
    home(client)
    first = client.call("start_sweep", sweep_request(0.0, 1, 500, 0.02))
    assert first.accepted
    client.wait_for(lambda: len(client.lines) >= 5)
    if boot_lost:
        fake.drop.add("BOOT")       # then only its clock going back gives the reset away
    fake.reboot()
    client.wait_for(lambda: not client.state.homed and client.state.state == "disabled" and not client.state.busy)
    assert client.state.sweep_id == first.sweep_id
    assert not client.call("start_sweep", sweep_request(0.0, 2, 5, 0.02)).accepted   # not homed
    client.wait_for(lambda: bridge.clock_sync.ready)
    home(client)
    res = client.call("start_sweep", sweep_request(0.0, 2, 10, 0.02))
    assert res.accepted, res.message
    check_sweep(client, fake, res, 10, 0.02)    # stamps are right on the restarted clock


def test_lost_scan_done(ros, fake):
    """If SCAN_DONE never arrives, STATUS shows the ESP32 isn't scanning and the sweep ends."""
    client, _ = ros
    home(client)
    fake.drop.add("SCAN_DONE")
    res = client.call("start_sweep", sweep_request(0.0, 2, 10, 0.02))
    assert res.accepted
    client.wait_for(lambda: client.state.sweep_id == res.sweep_id and not client.state.busy)
    assert client.call("start_sweep", sweep_request(0.0, 2, 5, 0.02)).accepted


def test_send_survives_the_port_vanishing():
    """send() racing the reader thread's close: no exception, just nothing sent."""
    link = MirrorLink("/dev/null-mirror", 921600, lambda m: None, lambda c: None, None)
    link.connected, link.ser = True, None
    assert not link.send("PING id=1\n")


# --- locking sweeps to a camera's frames ----------------------------------------------------

class FrameStamps:
    """Publishes /line_camera/frame as line_camera does, from a free-running camera clock: each
    frame once its last row is read out, with the window in which the slit's rows exposed."""

    def __init__(self, node, period_s=1 / 30 * (1 + 150e-6), exposure_s=0.005, rows_s=(0.002, 0.019)):
        self.pub = node.create_publisher(FrameStamp, "/line_camera/frame", 50)
        self.period_ns = period_s * 1e9
        self.offsets = (rows_s[0] - exposure_s) * 1e9, rows_s[1] * 1e9
        self.exposure_s = exposure_s
        self.windows = []          # (start, end) ROS ns of every frame published
        self.t0 = time.time_ns() + 30_000_000
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        k = 0
        while not self._stop.is_set():
            sof = int(self.t0 + k * self.period_ns)
            start, end = sof + int(self.offsets[0]), sof + int(self.offsets[1])
            if self._stop.wait(max(0.0, (end + 1_000_000 - time.time_ns()) * 1e-9)):
                return
            msg = FrameStamp()
            msg.header.stamp = rclpy.time.Time(nanoseconds=sof).to_msg()
            msg.header.frame_id = "spectrograph_optical_frame"
            msg.sequence = k
            msg.exposure_start = rclpy.time.Time(nanoseconds=start).to_msg()
            msg.exposure_end = rclpy.time.Time(nanoseconds=end).to_msg()
            msg.exposure, msg.gain = self.exposure_s, 1.0
            self.windows.append((start, end))
            self.pub.publish(msg)
            k += 1

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._thread.join(timeout=2.0)


def test_sweep_locks_to_the_camera(ros, fake):
    """The mirror moves only between exposures, and every line has a whole exposure while still."""
    client, bridge = ros
    home(client)
    n = 45
    with FrameStamps(client.node) as camera:
        client.wait_for(lambda: bridge.frames.fit(bridge.now_ns()) is not None)
        res = client.call("start_sweep", sweep_request(0.0, 2, n, 0.0333))
        assert res.accepted and res.frame_locked, res.message
        assert res.frames_per_line == 1 and "locked to the camera" in res.message
        assert res.line_period == pytest.approx(camera.period_ns * 1e-9, rel=2e-5)
        client.wait_for(lambda: any(m.sweep_id == res.sweep_id and m.last for m in client.lines))
        # the last line's frame is read out after its line is reported; wait for the scan's end
        client.wait_for(lambda: any(scan == fake.scans and i == n for scan, i, _ in fake.ticks))
        done = [w for scan, i, w in fake.ticks if scan == fake.scans and i == n][0]
        client.wait_for(lambda: camera.windows[-1][1] > done)
        windows = list(camera.windows)
    lines = [m for m in client.lines if m.sweep_id == res.sweep_id]
    assert [m.index for m in lines] == list(range(n)) and all(m.settled for m in lines)
    # the fake's own times: line i's move runs from its tick until it is ready
    ticks = {i: w for scan, i, w in fake.ticks if scan == fake.scans}
    ready = {i: w for scan, i, w in fake.line_times if scan == fake.scans}
    for i in range(1, n):
        caught = [w for w in windows if w[0] < ready[i] and w[1] > ticks[i]]
        assert not caught, "line %d: an exposure caught the mirror moving" % i
    for i in range(n):
        still = [w for w in windows if w[0] >= ready[i] and w[1] <= ticks[i + 1]]
        assert len(still) == 1, "line %d: %d exposures while the mirror held still" % (i, len(still))
        # and the stamps say so too, with the margin the matcher uses
        m = lines[i]
        hold = (ns(m.header.stamp) + 500_000, ns(m.hold_until) - 500_000)
        assert any(w[0] >= hold[0] and w[1] <= hold[1] for w in windows), i


def test_sweep_without_frames_is_not_locked(ros):
    client, _ = ros
    home(client)
    res = client.call("start_sweep", sweep_request(0.0, 2, 5, 0.02))
    assert res.accepted and not res.frame_locked and res.frames_per_line == 0
    assert res.line_period == pytest.approx(0.02)


def test_lock_tracking_nudges_and_periods():
    """track_lock: a tick late by more than the threshold gets one NUDGE back, and a changed
    frame period a PERIOD, without a ROS graph."""
    period = 33_333_333.0
    clock = fl.FrameClock()
    t0 = 1_700_000_000_000_000_000
    for k in range(60):
        sof = int(t0 + k * period)
        clock.add(sof, sof - 3_000_000, sof + 20_000_000)
    sync = ClockSync(min_samples=1)
    sync.add(t0, t0 + 1_000_000, 5_000_000)          # ESP32 us 5_000_000 is ROS t0 + 0.5 ms
    fit = clock.fit()
    plan = fl.plan_lock(fit, 0.0333e9, fl.busy_ns(2, 1600, 3000, 50), 1e6)
    logs = []
    host = SimpleNamespace(clock_sync=sync, frames=clock, nudge_threshold_ns=300_000,
                           now_ns=lambda: t0 + int(60 * period),
                           get_logger=lambda: SimpleNamespace(warning=lambda *a, **k: logs.append(a)))
    sweep = Sweep(1, 100, plan.line_period_ns / 1000, lock=plan, phase_errors_ns=[])
    on_phase = fl.first_tick_ns(fit, plan, t0 + 40 * period)
    esp = lambda ros_ns: sync.to_esp_us(ros_ns)      # noqa: E731

    track = ScanMirrorBridge.track_lock
    assert track(host, sweep, 3, esp(on_phase + 100_000), -1) == (0, None)        # within the threshold
    nudge, new_period = track(host, sweep, 4, esp(on_phase + 800_000), -1)
    assert nudge == pytest.approx(-800, abs=2) and new_period is None and sweep.nudge_pending
    assert track(host, sweep, 5, esp(on_phase + 800_000), -1)[0] == 0              # until the OK NUDGE
    sweep.nudge_pending = False
    assert track(host, sweep, 6, esp(on_phase - 700_000), -1)[0] == pytest.approx(700, abs=2)
    # the camera runs 100 ppm slow: the next check after ten lines sends the new period
    sweep.nudge_pending = False
    sweep.period_us = plan.line_period_ns / 1000 * (1 - 100e-6)
    _, new_period = track(host, sweep, 20, esp(on_phase), -1)
    assert new_period == pytest.approx(plan.line_period_ns / 1000, abs=0.01)
    assert sweep.period_us == new_period and sweep.period_line == 20
    assert track(host, sweep, 25, esp(on_phase), -1)[1] is None
    # a move that took longer than the lock allowed is reported
    track(host, sweep, 30, esp(on_phase), int(esp(on_phase) + 9000))
    assert logs and "frames may catch it moving" in logs[0][0]


class Port:
    """The fake ESP32's port, spoken to directly."""

    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        self.buf = b""

    def send(self, text):
        os.write(self.fd, (text + "\n").encode())

    def read_until(self, prefix, timeout=5.0):
        deadline = time.monotonic() + timeout
        while True:
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                text = line.decode(errors="replace").strip()
                if text.startswith(prefix):
                    return dict(w.split("=", 1) for w in text.split()[2:] if "=" in w)
            left = deadline - time.monotonic()
            assert left > 0, "no %r from the fake ESP32" % prefix
            if select.select([self.fd], [], [], left)[0]:
                self.buf += os.read(self.fd, 4096)

    def close(self):
        os.close(self.fd)


def test_fake_mirror_nudge_and_period(fake):
    port = Port(fake.link)
    try:
        port.send("ENABLE id=1")
        port.read_until("OK ENABLE")
        port.send("HOME id=2")
        port.read_until("EV HOMED")
        port.send("CFG id=3")
        cfg = port.read_until("OK CFG")
        assert (cfg["vstart"], cfg["settle"]) == ("1600", "3000")
        port.send("NUDGE dt=100 id=4")
        port.read_until("ERR NUDGE code=no_scan")
        port.send("SCAN mode=stare start=0 step=2 lines=8 period=20000 delay=30000 id=5")
        t0 = int(port.read_until("OK SCAN")["t0"])
        ticks = [int(port.read_until("EV LINE")["t"]) for _ in range(2)]
        port.send("NUDGE dt=-3000 id=6")
        nxt = int(port.read_until("OK NUDGE")["next"])
        assert nxt == ticks[1] + 20000 - 3000
        ticks.append(int(port.read_until("EV LINE")["t"]))
        port.send("PERIOD us=25000.5 id=7")
        port.read_until("OK PERIOD us=25000.5")
        ticks += [int(port.read_until("EV LINE")["t"]) for _ in range(5)]
        done = port.read_until("EV SCAN_DONE")
        assert ticks[0] == t0 and ticks[2] == nxt
        # the tick after the PERIOD keeps its time; the ones after that are 25000.5 us apart
        gaps = np.diff(ticks + [int(done["t"])])
        assert list(gaps[:3]) == [20000, 17000, 20000]
        assert gaps[3:] == pytest.approx(25000.5, abs=1)
        assert fake.nudges == [-3000] and fake.periods == [25000.5]
    finally:
        port.close()
