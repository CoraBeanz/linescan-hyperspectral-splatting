"""A scan with the spectrograph camera, without hardware.

scan_arm.launch.py with the mock arm, the simulated mirror ESP32 and camera:=fake (line_camera
on a simulated camera and spectrograph). A white is taken with capture_reference (the
simulated camera can't be capped for a dark), then scan_sweep runs a plan that requires the
camera: the bridge locks the sweep to the
camera's frames, every line is recorded and binned, and scan_to_dataset turns the folder into
the splat trainer's dataset, finding the references through scan.json.
"""

import csv
import json
import os
import signal
import subprocess
import time

import numpy as np
import pytest
import yaml

N_LINES = 30
DOMAIN = 60 + (os.getpid() + 17) % 40


@pytest.fixture(scope="module")
def env(tmp_path_factory):
    e = dict(os.environ)
    e["ROS_DOMAIN_ID"] = str(DOMAIN)
    e["SO101_SCAN_DATA"] = str(tmp_path_factory.mktemp("data"))   # where line_camera keeps references
    return e


@pytest.fixture(scope="module")
def stack(tmp_path_factory, env):
    tmp = tmp_path_factory.mktemp("capture")
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


def ros2_run(env, *args, timeout=120, retry_on=("isn't there", "isn't running", "frames came")):
    """ros2 run, retried while the launch is still coming up."""
    deadline = time.monotonic() + 60
    while True:
        run = subprocess.run(["ros2", "run", *args], env=env, capture_output=True, text=True, timeout=timeout)
        said = run.stdout + run.stderr
        if run.returncode == 0 or time.monotonic() > deadline or not any(r in said for r in retry_on):
            return run
        time.sleep(2)


def test_scan_records_the_camera_and_converts(stack, env):
    run = ros2_run(env, "so101_scan_camera", "capture_reference", "white", "--frames", "4")
    assert run.returncode == 0, run.stdout + run.stderr + (stack / "launch.log").read_text()[-3000:]
    refs = os.path.join(env["SO101_SCAN_DATA"], "references")
    assert os.path.exists(os.path.join(refs, "reference", "white", "meta.json"))

    plan = stack / "plan.yaml"
    plan.write_text(yaml.safe_dump({
        "name": "capture_test",
        "move": {"max_joint_speed_deg": 90, "min_move_s": 0.5, "settle_s": 0.2},
        "sweep": {"start_angle_deg": -2.0, "steps_per_line": 2, "n_lines": N_LINES, "line_period_s": 0.02},
        "camera": {"record": "required"},
        "viewpoints": [{"name": "down", "joints_deg": {"shoulder_pan": 0, "shoulder_lift": -8.3, "elbow_flex": 60.5,
                                                       "wrist_flex": -52.2, "wrist_roll": -87.2}}],
    }))
    out = stack / "scan"
    run = ros2_run(env, "so101_scan_sweep", "scan_sweep", "--plan", str(plan), "--output", str(out),
                   retry_on=("not available",))
    assert run.returncode == 0, run.stdout + run.stderr + (stack / "launch.log").read_text()[-3000:]

    info = json.loads((out / "scan.json").read_text())
    assert info["camera"]["recording"] and info["camera"]["calibrated"], info["camera"]
    assert info["camera"]["references"] == refs
    vp = info["viewpoints"][0]
    # locked: asked for 20 ms, got one 30 fps frame a line
    assert vp["frame_locked"] and vp["frames_per_line"] == 1 and vp["lines_logged"] == N_LINES
    assert vp["line_period_s"] == pytest.approx(1 / 30, rel=0.01)
    with open(out / "frames" / "frames.csv") as f:
        rows = list(csv.DictReader(f))
    assert [r["status"] for r in rows] == ["ok"] * N_LINES, info["camera"].get("stopped")
    binned = np.load(out / "binned" / ("sweep_%03d.npy" % vp["sweep_id"]))
    assert binned.shape[:2] == (N_LINES, 64) and np.isfinite(binned).all()

    run = subprocess.run(["ros2", "run", "so101_scan_camera", "scan_to_dataset", str(out), str(stack / "dataset")],
                         env=env, capture_output=True, text=True, timeout=120)
    assert run.returncode == 0, run.stdout + run.stderr
    doc = json.loads((stack / "dataset" / "dataset.json").read_text())
    assert doc["values"] == "reflectance" and doc["metadata"]["white"]["folder"].startswith(refs)
    assert (doc["num_lines"], doc["num_sweeps"], doc["width"], doc["num_bands"]) == (N_LINES, 1, 64, 46)
    lines = np.load(stack / "dataset" / "lines.npy")
    assert lines.shape == (N_LINES, 64, 46) and np.isfinite(lines).all()
