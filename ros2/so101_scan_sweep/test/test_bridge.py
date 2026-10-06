"""scan_mirror_bridge against the simulated ESP32: homing, sweeps, line stamps, refusals and
the ESP32 restarting."""

import math
import os
import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from so101_scan_interfaces.msg import MirrorState, ScanLine
from so101_scan_interfaces.srv import MoveMirror, StartSweep
from so101_scan_sweep.bridge import MirrorLink, ScanMirrorBridge
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
