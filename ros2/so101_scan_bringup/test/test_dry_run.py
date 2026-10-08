"""Dry runs and coverage plans on the running stack, without hardware.

scan_arm.launch.py with the mock arm, the simulated mirror ESP32 and the simulated camera.
make_plan plans views around a box; scan_sweep --dry-run plays that plan on the stack: the
markers show every line and the box's coverage, the report says what it found, and nothing
moves (the arm's joints and the mirror stay where they were, and no scan folder appears).
A plan the stack would refuse (the mirror swept past the bridge's limits) or that comes too
close to the table is reported, with exit code 1.
"""

import os
import signal
import subprocess
import threading
import time

import pytest
import rclpy
import yaml
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray

from so101_scan_interfaces.msg import MirrorState
from so101_scan_sweep.sweep import LATCHED

DOMAIN = 60 + (os.getpid() + 29) % 40
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


@pytest.fixture(scope="module")
def env(tmp_path_factory):
    e = dict(os.environ)
    e["ROS_DOMAIN_ID"] = str(DOMAIN)
    e["SO101_SCAN_DATA"] = str(tmp_path_factory.mktemp("data"))
    return e


@pytest.fixture(scope="module")
def stack(tmp_path_factory, env):
    tmp = tmp_path_factory.mktemp("dry")
    log = open(tmp / "launch.log", "w")
    launch = subprocess.Popen(
        ["ros2", "launch", "so101_scan_bringup", "scan_arm.launch.py", "use_mock_hardware:=true", "mirror:=fake",
         "camera:=fake"], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    yield tmp
    os.killpg(launch.pid, signal.SIGINT)
    try:
        launch.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(launch.pid, signal.SIGKILL)
    log.close()


@pytest.fixture(scope="module")
def watch(stack):
    """Every /joint_states and /scan_mirror/state, and the latest /scan/markers, from this process."""
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=DOMAIN)
    node = rclpy.create_node("dry_run_test", context=context)
    seen = {"joints": [], "mirror": [], "markers": {}}

    def on_markers(msg):
        for m in msg.markers:
            seen["markers"][(m.ns, m.id)] = m

    node.create_subscription(JointState, "/joint_states", lambda m: seen["joints"].append(
        dict(zip(m.name, m.position))), 200)
    node.create_subscription(MirrorState, "/scan_mirror/state", seen["mirror"].append, 50)
    node.create_subscription(MarkerArray, "/scan/markers", on_markers, LATCHED)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield seen
    executor.shutdown()
    thread.join(timeout=5.0)
    node.destroy_node()
    rclpy.shutdown(context=context)


def ros2_run(env, *args, timeout=180):
    return subprocess.run(["ros2", "run", *args], env=env, capture_output=True, text=True, timeout=timeout)


def dry_run(env, plan, *extra):
    """scan_sweep --dry-run, retried while the launch is still coming up."""
    deadline = time.monotonic() + 60
    while True:
        run = ros2_run(env, "so101_scan_sweep", "scan_sweep", "--plan", str(plan), "--dry-run", *extra)
        said = run.stdout + run.stderr
        if time.monotonic() > deadline or not any(s in said for s in (
                "isn't running", "no /robot_description", "isn't homed and", "no arm joints")):
            return run
        time.sleep(2)


def arm_joints(seen):
    return [[j[n] for n in ARM] for j in seen["joints"] if all(n in j for n in ARM)]


def test_coverage_plan_plays_without_moving_anything(stack, env, watch):
    plan = stack / "box.yaml"
    run = ros2_run(env, "so101_scan_sweep", "make_plan", "--object", "0.26", "0", "--size", "0.04", "0.04", "0.02",
                   "--tilts", "0", "35", "--azimuths", "90", "180", "270", "--out", str(plan))
    assert run.returncode == 0, run.stdout + run.stderr
    doc = yaml.safe_load(plan.read_text())
    assert doc["object"]["size_m"] == [0.04, 0.04, 0.02] and doc["coverage"]["faces"]["top"]["seen"] == 1.0
    n_views = len(doc["viewpoints"])
    n_lines = doc["sweep"]["n_lines"]
    assert n_views >= 3

    time.sleep(1.0)
    before = len(watch["joints"])
    run = dry_run(env, plan, "--speed", "20")
    assert run.returncode == 0, run.stdout + run.stderr + (stack / "launch.log").read_text()[-3000:]
    assert "dry run of object: %d viewpoints" % n_views in run.stdout
    assert "0 problems" in run.stdout and "the object: " in run.stdout
    assert "scan stack isn't running" not in run.stdout          # it found the stack

    # nothing moved: the mock arm held its joints and the mirror never left its rest
    held = arm_joints(watch)[max(0, before - 5):]
    assert held and max(abs(a - b) for row in held for a, b in zip(row, held[0])) < 1e-6
    assert not any(m.busy for m in watch["mirror"])
    assert not os.path.exists(os.path.join(env["SO101_SCAN_DATA"], "scans"))

    # what it drew: every line of every viewpoint, the box, and its points, every one seen
    deadline = time.monotonic() + 5
    while ("dry_run_coverage", 0) not in watch["markers"]:
        assert time.monotonic() < deadline, sorted(watch["markers"])
        time.sleep(0.1)
    lines = [m for (ns, _), m in watch["markers"].items() if ns == "dry_run_lines" and m.action == m.ADD]
    assert len(lines) == n_views and all(len(m.points) == 2 * n_lines for m in lines)
    assert ("dry_run_object", 0) in watch["markers"] and ("dry_run_skeleton", 0) in watch["markers"]
    colours = watch["markers"][("dry_run_coverage", 0)].colors
    assert sum(c.g > 0.5 and c.r < 0.5 for c in colours) > len(colours) // 2     # green: seen by two or more


def test_problems_are_reported(stack, env):
    plan = stack / "bad.yaml"
    plan.write_text(yaml.safe_dump({
        "name": "bad",
        "sweep": {"start_angle_deg": -60.0, "steps_per_line": 2, "n_lines": 20, "line_period_s": 0.0333},
        "viewpoints": [{"name": "down", "joints_deg": {"shoulder_pan": 0, "shoulder_lift": -8.3, "elbow_flex": 60.5,
                                                       "wrist_flex": -52.2, "wrist_roll": -87.2}}],
    }))
    run = dry_run(env, plan, "--no-play", "--clearance", "0.3")
    assert run.returncode == 1, run.stdout + run.stderr
    assert "outside the bridge's -45.84..45.84 deg" in run.stdout, run.stdout
    assert "under the 300 mm clearance" in run.stdout, run.stdout
