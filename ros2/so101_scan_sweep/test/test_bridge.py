"""scan_mirror_bridge against the simulated ESP32: homing, sweeps, line stamps and refusals."""

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
from so101_scan_sweep.bridge import ScanMirrorBridge
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


def test_needs_homing_first(ros):
    client, _ = ros
    res = client.call("start_sweep", sweep_request(0.0, 2, 5, 0.02))
    assert not res.accepted and "not homed" in res.message
    assert not client.call("move", MoveMirror.Request(angle=0.1)).success
    assert client.call("home", Trigger.Request()).success
    client.wait_for(lambda: client.state.homed)
    assert client.state.angle == pytest.approx(-0.698132)


def test_sweep_lines_angles_and_stamps(ros, fake):
    client, bridge = ros
    assert client.call("home", Trigger.Request()).success
    start = -0.05
    res = client.call("start_sweep", sweep_request(start, 2, 25, 0.02))
    assert res.accepted, res.message
    assert res.rad_per_step == pytest.approx(RAD_PER_STEP)
    assert abs(res.start_angle - start) <= RAD_PER_STEP / 2
    client.wait_for(lambda: client.lines and client.lines[-1].last)
    lines = [m for m in client.lines if m.sweep_id == res.sweep_id]
    assert [m.index for m in lines] == list(range(25))
    for m in lines:
        assert m.angle == pytest.approx(res.start_angle + 2 * m.index * RAD_PER_STEP, abs=1e-9)
        assert m.step == bridge.angle_to_step(m.angle)
    assert [m.last for m in lines] == [False] * 24 + [True]
    # each stamp is the ROS time the fake sent the line, from its drifting, offset clock
    sent = {i: wall for sid, i, wall in fake.line_times if sid == res.sweep_id}
    errors_ms = [abs(rclpy.time.Time.from_msg(m.header.stamp).nanoseconds - sent[m.index]) * 1e-6 for m in lines]
    assert max(errors_ms) < 3.0, errors_ms
    # and the mirror joint followed the sweep
    assert any(abs(j.position[0] - lines[-1].angle) < 1e-9 for j in client.joints if j.name == ["scan_mirror_joint"])


def test_refuses_out_of_range_and_bad_requests(ros):
    client, _ = ros
    assert client.call("home", Trigger.Request()).success
    assert not client.call("start_sweep", sweep_request(0.7, 10, 50, 0.02)).accepted  # ends past 0.8 rad
    assert not client.call("start_sweep", sweep_request(0.0, 2, 0, 0.02)).accepted
    assert not client.call("move", MoveMirror.Request(angle=1.5)).success


def test_move_rounds_to_a_step(ros, fake):
    client, _ = ros
    assert client.call("home", Trigger.Request()).success
    res = client.call("move", MoveMirror.Request(angle=0.1))
    assert res.success, res.message
    assert abs(res.angle - 0.1) <= RAD_PER_STEP / 2
    assert fake.step == round((res.angle + 0.698132) / RAD_PER_STEP)


def test_stop_ends_a_sweep(ros):
    client, _ = ros
    assert client.call("home", Trigger.Request()).success
    res = client.call("start_sweep", sweep_request(0.0, 1, 500, 0.02))
    assert res.accepted
    client.wait_for(lambda: len(client.lines) >= 5)
    assert client.call("stop", Trigger.Request()).success
    client.wait_for(lambda: not client.state.busy)
    n = len(client.lines)
    time.sleep(0.2)
    assert len(client.lines) == n < 500
