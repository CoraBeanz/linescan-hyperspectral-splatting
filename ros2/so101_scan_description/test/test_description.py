"""The URDF builds, its arm is TheRobotStudio's SO-101 to the number, its head matches the CAD
model, and the reflected line camera sits where the mirror puts it."""

import importlib.util
import json
import math
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import xacro

from so101_scan_description.kinematics import Robot

PKG = Path(__file__).resolve().parent.parent
REPO = PKG.parent.parent
XACRO = PKG / "urdf" / "so101_scan.urdf.xacro"
VENDOR_URDF = REPO / "cad" / "vendor" / "so101" / "so101_new_calib.urdf"
BUILD_REPORT = REPO / "cad" / "build_report.json"
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


@pytest.fixture(scope="session", autouse=True)
def package_index(tmp_path_factory):
    """Let $(find so101_scan_description) resolve to this source tree, installed or not."""
    prefix = tmp_path_factory.mktemp("prefix")
    (prefix / "share" / "ament_index" / "resource_index" / "packages").mkdir(parents=True)
    (prefix / "share" / "ament_index" / "resource_index" / "packages" / "so101_scan_description").touch()
    (prefix / "share" / "so101_scan_description").symlink_to(PKG)
    old = os.environ.get("AMENT_PREFIX_PATH", "")
    os.environ["AMENT_PREFIX_PATH"] = str(prefix) + (os.pathsep + old if old else "")
    yield
    os.environ["AMENT_PREFIX_PATH"] = old


def build(**args):
    mappings = {k: str(v) for k, v in args.items()}
    return xacro.process_file(str(XACRO), mappings=mappings).toprettyxml(indent="  ")


@pytest.fixture(scope="module")
def mock_urdf():
    return build(use_mock_hardware="true")


@pytest.fixture(scope="module")
def robot(mock_urdf):
    return Robot(mock_urdf)


def head_params():
    root = ET.parse(PKG / "urdf" / "scan_head_params.xacro").getroot()
    return {p.get("name"): float(p.get("value")) for p in root.iter("{http://www.ros.org/wiki/xacro}property")}


def calibration_yaml(path):
    path.write_text("joints:\n" + "".join(
        "  %s: {id: %d, zero_ticks: %d, sign: %d, min_ticks: %d, max_ticks: %d}\n"
        % (n, i + 1, 2000 + i, -1 if n == "elbow_flex" else 1, 900, 3100) for i, n in enumerate(ARM)))
    return path


# --- it builds ---------------------------------------------------------------------------

def test_arm_xacro_is_up_to_date():
    spec = importlib.util.spec_from_file_location("gen", PKG / "scripts" / "so101_urdf_to_xacro.py")
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    assert gen.generate() == (PKG / "urdf" / "so101_arm.xacro").read_text(), \
        "urdf/so101_arm.xacro is stale: run scripts/so101_urdf_to_xacro.py"


def test_mock_urdf_is_valid(mock_urdf, robot, tmp_path):
    assert robot.movable() == ARM + ["scan_mirror_joint"]
    root = ET.fromstring(mock_urdf)
    hw = root.find("ros2_control/hardware")
    assert hw.find("plugin").text == "mock_components/GenericSystem"
    assert [j.get("name") for j in root.findall("ros2_control/joint")] == ARM
    if shutil.which("check_urdf"):
        f = tmp_path / "so101_scan.urdf"
        f.write_text(mock_urdf)
        out = subprocess.run(["check_urdf", str(f)], capture_output=True, text=True)
        assert out.returncode == 0, out.stdout + out.stderr


def test_every_mesh_exists(mock_urdf):
    missing = []
    for mesh in ET.fromstring(mock_urdf).iter("mesh"):
        uri = mesh.get("filename")
        rel = uri.replace("package://so101_scan_description/", "")
        local = PKG / rel
        if rel.startswith("meshes/so101/"):  # installed from cad/vendor by CMakeLists.txt
            local = REPO / "cad" / "vendor" / "so101" / "assets" / Path(rel).name
            if not local.parent.exists():
                pytest.skip("SO-101 meshes not fetched (cad/vendor/so101/fetch_assets.py)")
        if not local.exists():
            missing.append(uri)
    assert not missing


def test_without_ros2_control():
    assert ET.fromstring(build(ros2_control="false")).find("ros2_control") is None


def test_real_arm_needs_a_calibration_file():
    with pytest.raises(Exception, match="calibration_file"):
        build()


def test_calibration_goes_to_the_driver(tmp_path):
    root = ET.fromstring(build(calibration_file=calibration_yaml(tmp_path / "cal.yaml"), port="/dev/ttyACM7",
                               torque="false"))
    hw = root.find("ros2_control/hardware")
    params = {p.get("name"): p.text for p in hw.findall("param")}
    assert hw.find("plugin").text == "so101_scan_hardware/FeetechStsSystem"
    assert params["port"] == "/dev/ttyACM7" and params["torque"] == "false"
    elbow = root.find("ros2_control/joint[@name='elbow_flex']")
    jp = {p.get("name"): p.text for p in elbow.findall("param")}
    assert jp == {"id": "3", "zero_ticks": "2002", "sign": "-1", "min_ticks": "900", "max_ticks": "3100"}
    assert [s.get("name") for s in elbow.findall("state_interface")] == \
        ["position", "velocity", "load", "voltage", "temperature"]


# --- the arm is TheRobotStudio's ----------------------------------------------------------

def test_arm_matches_vendored_urdf(robot):
    vendor = Robot(VENDOR_URDF.read_text())
    rng = np.random.default_rng(1)
    pairs = [("shoulder_link",) * 2, ("upper_arm_link",) * 2, ("lower_arm_link",) * 2,
             ("wrist_link",) * 2, ("gripper_link", "wrist_roll_link")]
    for _ in range(20):
        q = {n: float(rng.uniform(-1.5, 1.5)) for n in ARM}
        for theirs, ours in pairs:
            np.testing.assert_allclose(robot.fk(ours, q), vendor.fk(theirs, q), atol=1e-9)


def test_zero_pose_and_directions_match_calibration_prompts(robot):
    """sts_calibrate asks for this zero pose and these directions; the URDF must agree."""
    def at(link, **q):
        return robot.fk(link, q)

    shoulder = at("upper_arm_link")[:3, 3]
    elbow = at("lower_arm_link")[:3, 3]
    wrist = at("wrist_link")[:3, 3]
    # zero: upper arm up, forearm level and forward
    assert elbow[2] - shoulder[2] > 0.1 and abs(elbow[0] - shoulder[0]) < 0.03
    assert wrist[0] - elbow[0] > 0.12 and abs(wrist[2] - elbow[2]) < 0.01
    # the head points forward along the forearm, window facing right (-Y)
    head = at("scan_head_link")
    np.testing.assert_allclose(head[:3, 2], [1, 0, 0], atol=0.06)
    np.testing.assert_allclose(head[:3, 1], [0, -1, 0], atol=0.06)

    d = 0.2
    assert at("lower_arm_link", shoulder_pan=d)[1, 3] < elbow[1] - 0.005            # pan: right (-Y)
    assert at("lower_arm_link", shoulder_lift=d)[0, 3] > elbow[0] + 0.01            # lift: forward
    assert at("wrist_link", elbow_flex=d)[2, 3] < wrist[2] - 0.01                   # elbow: forearm down
    assert at("scan_head_link", wrist_flex=d)[2, 3] < head[2, 3] - 0.005            # wrist: head down
    assert at("scan_head_link", wrist_roll=d)[:3, 1][2] > 0.1                       # roll: window right -> up


# --- the head is the CAD model's ----------------------------------------------------------

def test_head_params_match_build_report():
    p = head_params()
    report = json.loads(BUILD_REPORT.read_text())
    st = report["stations_mm"]
    assert p["mirror_shaft_y"] * 1e3 == pytest.approx(st["y_shaft"], abs=0.01)
    assert p["mirror_shaft_z"] * 1e3 == pytest.approx(st["z_shaft"], abs=0.01)
    assert p["objective_z"] * 1e3 == pytest.approx(st["z_obj"], abs=0.01)
    # the optical axis meets the mirror face where the CAD put the mirror station
    face_y = p["mirror_shaft_y"] + p["mirror_face_offset"] * math.cos(p["mirror_rest"])
    face_z = p["mirror_shaft_z"] + p["mirror_face_offset"] * math.sin(p["mirror_rest"])
    assert face_y == pytest.approx(p["objective_y"], abs=2e-5)
    assert face_z * 1e3 == pytest.approx(st["z_mirror"], abs=0.02)
    assert p["head_mass"] * 1e3 == pytest.approx(report["mass"]["total_g"], abs=0.1)
    np.testing.assert_allclose([p["head_com_x"], p["head_com_y"], p["head_com_z"]],
                               np.array(report["mass"]["com_head_mm"]) * 1e-3, atol=1e-5)


# --- the reflected line camera ------------------------------------------------------------

@pytest.mark.parametrize("theta", [0.0, 0.1, -0.1, 0.2, -0.698132, 1.0])
def test_line_camera_is_the_objective_seen_in_the_mirror(robot, theta):
    q = {"scan_mirror_joint": theta}
    obj = robot.fk("spectrograph_optical_frame", q, base="scan_head_link")
    face = robot.fk("scan_mirror_face", q, base="scan_head_link")
    cam = robot.fk("line_camera_optical_frame", q, base="scan_head_link")
    n, m = face[:3, 2], face[:3, 3]
    reflect = np.eye(3) - 2 * np.outer(n, n)
    np.testing.assert_allclose(cam[:3, 3], obj[:3, 3] - 2 * np.dot(obj[:3, 3] - m, n) * n, atol=1e-9)
    # reflect the axes, then flip y to keep the frame right-handed
    np.testing.assert_allclose(cam[:3, :3], reflect @ obj[:3, :3] @ np.diag([1, -1, 1]), atol=1e-9)
    # the view turns twice as fast as the mirror: +Y of the head at rest, toward +Z as theta grows
    # (to 1e-6: the parameters are written to the micrometre and microradian)
    np.testing.assert_allclose(cam[:3, 2], [0, math.cos(2 * theta), math.sin(2 * theta)], atol=1e-6)
    # pixel order along the slit is the objective's
    np.testing.assert_allclose(cam[:3, 0], obj[:3, 0], atol=1e-9)


def test_scan_line_is_in_front_of_the_window(robot):
    p = head_params()
    st = json.loads(BUILD_REPORT.read_text())["stations_mm"]
    cam = robot.fk("line_camera_optical_frame", {}, base="scan_head_link")
    line = robot.fk("scan_line_frame", {}, base="scan_head_link")
    # at rest the mirror folds the axis by 90 deg: the objective appears level with the mirror
    # station, as far toward -Y as it really sits above the mirror
    seen = [0, p["objective_y"] * 1e3 - (st["z_obj"] - st["z_mirror"]), st["z_mirror"]]
    np.testing.assert_allclose(cam[:3, 3] * 1e3, seen, atol=0.05)
    np.testing.assert_allclose(line[:3, 3], cam[:3, 3] + [0, p["scene_distance"], 0], atol=1e-6)
