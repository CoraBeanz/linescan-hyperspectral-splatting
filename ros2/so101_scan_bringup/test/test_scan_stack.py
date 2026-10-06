"""The whole scan stack without hardware.

scan_arm.launch.py runs as it would on the Jetson, with the real STS3215 driver talking to
the fake servo bus and the mirror bridge talking to the simulated ESP32. Then:
  * scan_sweep runs a two-viewpoint plan: the arm goes where the plan says, every line is
    logged, and each logged pose is the URDF's pose for the logged joints and mirror angle;
  * Ctrl-C during a scan stops the mirror and still writes what was logged;
  * Ctrl-C during move_arm stops the arm where it is.
"""

import csv
import json
import math
import os
import signal
import subprocess
import threading
import time

import numpy as np
import pytest
import rclpy
import yaml
from sensor_msgs.msg import JointState

from so101_scan_description.kinematics import Robot
from so101_scan_interfaces.msg import MirrorState
from so101_scan_sweep.line_log import LinePoser, quaternion

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
VIEWS = {"down": [0.0, -8.3, 60.5, -52.2, -87.2], "side": [14.0, -4.1, 56.9, -46.3, -111.4]}
N_LINES = 15
DOMAIN = 60 + os.getpid() % 40


@pytest.fixture(scope="module")
def env():
    e = dict(os.environ)
    e["ROS_DOMAIN_ID"] = str(DOMAIN)
    return e


@pytest.fixture(scope="module")
def stack(tmp_path_factory, env):
    tmp = tmp_path_factory.mktemp("stack")
    # the fake bus gets its own process: in this one, the GIL shared with rclpy below could
    # hold up its replies past the driver's 10 ms timeout
    link = tmp / "so101"
    bus = subprocess.Popen(["ros2", "run", "so101_scan_hardware", "sts_fake_bus", "--link", str(link)],
                           env=env, stdout=subprocess.DEVNULL, start_new_session=True)
    deadline = time.monotonic() + 20
    while not link.exists():
        assert time.monotonic() < deadline and bus.poll() is None, "the fake servo bus didn't start"
        time.sleep(0.1)
    cal = tmp / "calibration.yaml"
    cal.write_text("joints:\n" + "".join(
        "  %s: {id: %d, zero_ticks: 2048, sign: 1, min_ticks: 200, max_ticks: 3900}\n" % (n, i + 1)
        for i, n in enumerate(ARM)))
    log = open(tmp / "launch.log", "w")
    launch = subprocess.Popen(
        ["ros2", "launch", "so101_scan_bringup", "scan_arm.launch.py", "port:=%s" % link,
         "calibration_file:=%s" % cal, "mirror:=fake"],
        env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    yield tmp
    os.killpg(launch.pid, signal.SIGINT)
    try:
        launch.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(launch.pid, signal.SIGKILL)
    log.close()
    os.killpg(bus.pid, signal.SIGINT)
    bus.wait(timeout=10)


@pytest.fixture(scope="module")
def watch(stack):
    """The latest /joint_states and /scan_mirror/state, from this process."""
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=DOMAIN)
    node = rclpy.create_node("stack_test", context=context)
    latest = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: latest.update(dict(zip(m.name, m.position))), 50)
    node.create_subscription(MirrorState, "/scan_mirror/state", lambda m: latest.update(mirror=m), 10)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    yield latest
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown(context=context)


def write_plan(path, n_lines=N_LINES, period=0.02):
    path.write_text(yaml.safe_dump({
        "name": "stack_test",
        "move": {"max_joint_speed_deg": 90, "min_move_s": 0.5, "settle_s": 0.2},
        "sweep": {"start_angle_deg": -2.0, "steps_per_line": 2, "n_lines": n_lines, "line_period_s": period},
        "viewpoints": [{"name": n, "joints_deg": dict(zip(ARM, q))} for n, q in VIEWS.items()],
    }))
    return path


def run_sweep(stack, env, plan, out):
    """scan_sweep, retried while the launch is still coming up."""
    deadline = time.monotonic() + 60
    while True:
        run = subprocess.run(["ros2", "run", "so101_scan_sweep", "scan_sweep", "--plan", str(plan),
                              "--output", str(out)], env=env, capture_output=True, text=True, timeout=120)
        if run.returncode == 0 or time.monotonic() > deadline or "not available" not in run.stderr + run.stdout:
            return run
        time.sleep(2)


def test_plan_runs_and_poses_match_the_urdf(stack, env):
    out = stack / "scan"
    run = run_sweep(stack, env, write_plan(stack / "plan.yaml"), out)
    launch_log = (stack / "launch.log").read_text()
    assert run.returncode == 0, run.stdout + run.stderr + "\n--- launch log ---\n" + launch_log[-4000:]

    info = json.loads((out / "scan.json").read_text())
    assert info["lines"] == 2 * N_LINES and not info["interrupted"]
    for vp in info["viewpoints"]:
        assert vp["move"]["ok"], vp
        assert vp["lines_logged"] == N_LINES
        assert vp["line_period_mean_s"] == pytest.approx(0.02, abs=0.002)
        assert vp["arm_motion_max_rad"] < 0.002   # held still during the sweep

    robot = Robot((out / "robot.urdf").read_text())
    poser = LinePoser(robot)
    with open(out / "lines.csv") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 2 * N_LINES
    for row in rows:
        name = list(VIEWS)[int(row["viewpoint"])]
        q = {j: float(row[j]) for j in ARM}
        # the fake servos land on the goal to within a tick or two
        np.testing.assert_allclose([q[j] for j in ARM], np.radians(VIEWS[name]), atol=0.004)
        head, cam = poser.poses(q, float(row["mirror_angle"]))
        np.testing.assert_allclose([float(row["cam_" + k]) for k in "xyz"], cam[:3, 3], atol=1e-6)
        qx, qy, qz, qw = (float(row["cam_q" + k]) for k in "xyzw")
        assert abs(np.dot([qx, qy, qz, qw], quaternion(cam[:3, :3]))) == pytest.approx(1.0, abs=1e-6)
        np.testing.assert_allclose([float(row["head_" + k]) for k in "xyz"], head[:3, 3], atol=1e-6)
    # lines of one sweep are 2 microsteps of mirror apart
    angles = [float(r["mirror_angle"]) for r in rows[:N_LINES]]
    np.testing.assert_allclose(np.diff(angles), 2 * 2 * math.pi / 6400, atol=1e-9)


def test_ctrl_c_stops_the_mirror_and_keeps_the_log(stack, env, watch):
    out = stack / "interrupted"
    plan = write_plan(stack / "long.yaml", n_lines=400, period=0.02)
    proc = subprocess.Popen(["ros2", "run", "so101_scan_sweep", "scan_sweep", "--plan", str(plan),
                             "--output", str(out)], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)
    deadline = time.monotonic() + 60
    while not (watch.get("mirror") and watch["mirror"].busy):
        assert time.monotonic() < deadline and proc.poll() is None, proc.stdout.read() if proc.poll() else ""
        time.sleep(0.05)
    time.sleep(1.0)
    os.killpg(proc.pid, signal.SIGINT)  # what Ctrl-C in the terminal does
    output = proc.communicate(timeout=30)[0]
    deadline = time.monotonic() + 5
    while watch["mirror"].busy:
        assert time.monotonic() < deadline, "the mirror is still sweeping\n" + output
        time.sleep(0.05)
    info = json.loads((out / "scan.json").read_text())
    assert info["interrupted"], output
    assert 10 < info["lines"] < 400


def test_ctrl_c_stops_move_arm(stack, env, watch):
    start = {j: watch[j] for j in ARM}
    goal = [math.degrees(start[j]) for j in ARM]
    goal[0] += 60.0  # pan 60 deg at 10 deg/s: six seconds
    proc = subprocess.Popen(["ros2", "run", "so101_scan_sweep", "move_arm", "--speed-deg", "10",
                             "--joints-deg"] + ["%.3f" % g for g in goal],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)
    deadline = time.monotonic() + 30
    while abs(watch["shoulder_pan"] - start["shoulder_pan"]) < math.radians(3):
        assert time.monotonic() < deadline and proc.poll() is None, proc.stdout.read() if proc.poll() else ""
        time.sleep(0.05)
    os.killpg(proc.pid, signal.SIGINT)
    output = proc.communicate(timeout=30)[0]
    assert "stopped" in output, output
    time.sleep(0.5)
    held = watch["shoulder_pan"]
    time.sleep(1.0)
    assert watch["shoulder_pan"] == pytest.approx(held, abs=0.002)        # not moving
    assert held < math.radians(goal[0]) - math.radians(20)                # well short of the goal
