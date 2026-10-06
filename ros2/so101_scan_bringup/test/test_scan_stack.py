"""The whole scan stack without hardware.

scan_arm.launch.py runs as it would on the Jetson, with the real STS3215 driver talking to
the fake servo bus and the mirror bridge talking to the simulated ESP32; scan_sweep then runs
a two-viewpoint plan. The test checks that the arm went where the plan said, that every line
was logged, and that each logged pose is the URDF's pose for the logged joints and mirror angle.
"""

import csv
import json
import math
import os
import signal
import subprocess
import time

import numpy as np
import pytest
import yaml

from so101_scan_hardware.fake_bus import FakeBus
from so101_scan_sweep.line_log import LinePoser, quaternion
from so101_scan_description.kinematics import Robot

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
VIEWS = {"down": [0.0, -8.3, 60.5, -52.2, -87.2], "side": [14.0, -4.1, 56.9, -46.3, -111.4]}
N_LINES = 15


@pytest.fixture
def env():
    e = dict(os.environ)
    e["ROS_DOMAIN_ID"] = str(60 + os.getpid() % 40)
    return e


@pytest.fixture
def stack(tmp_path, env):
    bus = FakeBus(link=str(tmp_path / "so101")).start()
    cal = tmp_path / "calibration.yaml"
    cal.write_text("joints:\n" + "".join(
        "  %s: {id: %d, zero_ticks: 2048, sign: 1, min_ticks: 200, max_ticks: 3900}\n" % (n, i + 1)
        for i, n in enumerate(ARM)))
    log = open(tmp_path / "launch.log", "w")
    launch = subprocess.Popen(
        ["ros2", "launch", "so101_scan_bringup", "scan_arm.launch.py", "port:=%s" % bus.link,
         "calibration_file:=%s" % cal, "mirror:=fake"],
        env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    yield tmp_path
    os.killpg(launch.pid, signal.SIGINT)
    try:
        launch.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(launch.pid, signal.SIGKILL)
    log.close()
    bus.stop()


def test_plan_runs_and_poses_match_the_urdf(stack, env):
    out = stack / "scan"
    plan = {
        "name": "stack_test",
        "move": {"max_joint_speed_deg": 90, "min_move_s": 0.5, "settle_s": 0.2},
        "sweep": {"start_angle_deg": -2.0, "steps_per_line": 2, "n_lines": N_LINES, "line_period_s": 0.02},
        "viewpoints": [{"name": n, "joints_deg": dict(zip(ARM, q))} for n, q in VIEWS.items()],
    }
    (stack / "plan.yaml").write_text(yaml.safe_dump(plan))
    # the arm's controllers and the mirror take a few seconds to come up
    deadline = time.monotonic() + 60
    while True:
        run = subprocess.run(["ros2", "run", "so101_scan_sweep", "scan_sweep", "--plan", str(stack / "plan.yaml"),
                              "--output", str(out)], env=env, capture_output=True, text=True, timeout=120)
        if run.returncode == 0 or time.monotonic() > deadline or "not available" not in run.stderr + run.stdout:
            break
        time.sleep(2)
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
