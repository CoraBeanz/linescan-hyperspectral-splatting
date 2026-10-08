"""A simulated session is a scan session as the rig writes one, plus its truth."""

import csv
import json

import numpy as np
import pytest

from conftest import PLAN
from hsisim.session import Settings, load_plan, simulate
from hsisim.truth import ScanTruth, pose_matrix, read_lines_csv
from hsical.frames import load_session
from hsical.session_check import inspect_session
from so101_scan_description.kinematics import Robot
from so101_scan_camera.session import CSV_COLUMNS as CAMERA_COLUMNS
from so101_scan_sweep.line_log import CSV_COLUMNS, LinePoser
from so101_scan_sweep.plan import ARM_JOINTS


def test_lines_csv_is_scan_sweeps(scan):
    base, info = scan
    with open(base / "lines.csv", newline="") as f:
        rows = list(csv.reader(f))
    assert rows[0] == list(CSV_COLUMNS)
    cols = read_lines_csv(base / "lines.csv")
    n = PLAN["lines"]
    assert len(cols["index"]) == info["lines"] == 2 * n
    assert cols["sweep_id"].tolist() == [1] * n + [2] * n          # the bridge numbers sweeps from 1
    assert cols["viewpoint"].tolist() == [0] * n + [1] * n
    assert cols["index"].tolist() == list(range(n)) * 2
    assert (cols["settled"] == 1).all()
    stamps, hold = cols["stamp_ns"], cols["hold_until_ns"]
    assert np.all(np.diff(stamps) > 0) and np.all(hold > stamps)
    # as the ESP32 runs a line: the mirror moves at 1600 steps/s from the line's tick, is stamped
    # 3 ms after it gets there (line 0 is already there at its tick), and holds until 50 us
    # before the next tick
    t = read_lines_csv(base / "truth" / "frames_true.csv")
    move = round(PLAN["steps_per_line"] / 1600 * 1e9)
    for s in (slice(0, n), slice(n, 2 * n)):
        tick = t["tick_ns"][s]
        assert stamps[s][0] == tick[0]
        assert (stamps[s][1:] == tick[1:] + move + 3_000_000).all()
        assert (hold[s][:-1] == tick[1:] - 50_000).all()
    steps = np.diff(cols["mirror_angle"][:n]) / (2 * np.pi / 6400)
    assert steps == pytest.approx(np.full(n - 1, PLAN["steps_per_line"]), abs=1e-6)


def test_logged_poses_are_the_urdf_of_the_logged_joints(scan):
    base, _ = scan
    poser = LinePoser(Robot((base / "robot.urdf").read_text()))
    cols = read_lines_csv(base / "lines.csv")
    for i in range(len(cols["index"])):
        head, cam = poser.poses({j: cols[j][i] for j in ARM_JOINTS}, cols["mirror_angle"][i])
        assert pose_matrix(cols, "head", i) == pytest.approx(head, abs=2e-8)
        assert pose_matrix(cols, "cam", i) == pytest.approx(cam, abs=2e-8)


def test_logged_joints_are_encoder_ticks_near_the_truth(scan):
    base, info = scan
    logged, true = read_lines_csv(base / "lines.csv"), read_lines_csv(base / "truth" / "lines_true.csv")
    tick = 2 * np.pi / 4096
    for j in ARM_JOINTS:
        assert logged[j] / tick == pytest.approx(np.round(logged[j] / tick), abs=1e-4)
        assert np.abs(logged[j] - true[j]).max() < np.radians(1.5)
    assert np.abs(logged["mirror_angle"] - true["mirror_angle"]).max() < np.radians(0.3)
    # the head really sits a little off where the joints say: millimetres, not centimetres
    d = np.hypot(np.hypot(logged["cam_x"] - true["cam_x"], logged["cam_y"] - true["cam_y"]),
                 logged["cam_z"] - true["cam_z"])
    assert 1e-5 < np.median(d) < 5e-3


def test_scan_json(scan):
    base, info = scan
    on_disk = json.loads((base / "scan.json").read_text())
    assert on_disk == json.loads(json.dumps(info))
    for key in ("format", "name", "started", "finished", "plan", "viewpoints", "frames", "units", "lines",
                "camera", "calibration_session", "simulated"):
        assert key in on_disk, key
    assert on_disk["format"] == "so101_scan lines v1"
    assert [v["sweep_id"] for v in on_disk["viewpoints"]] == [1, 2]
    assert [v["name"] for v in on_disk["viewpoints"]] == PLAN["views"]
    assert all(v["lines_logged"] == PLAN["lines"] for v in on_disk["viewpoints"])
    assert on_disk["calibration_session"] == "calibration"
    assert on_disk["frames"]["line_camera"] == "line_camera_optical_frame"
    assert on_disk["simulated"]["instrument"]["scale"] == 4.0


def test_robot_urdf(scan):
    base, _ = scan
    robot = Robot((base / "robot.urdf").read_text())
    for link in ("base_link", "scan_head_link", "line_camera_optical_frame", "scan_line_frame"):
        assert robot.root.find(f"link[@name='{link}']") is not None, link


def test_frames_are_hsical_frame_sets(scan):
    base, _ = scan
    sets, _ = load_session(base / "frames")
    assert sorted(sets) == ["sweep_001", "sweep_002"]
    for name, fs in sets.items():
        assert fs.kind == "scene" and fs.exposure_us == 20000.0
        files = fs.files()
        assert [f.name for f in files] == [f"frame_{k:04d}.npy" for k in range(PLAN["lines"])]
        assert sorted(p.name for p in fs.path.iterdir()) == sorted(["meta.json"] + [f.name for f in files])
        frame = np.load(files[0])
        assert frame.dtype == np.uint16 and frame.shape == (616, 820)   # 3280 x 2464 binned by 4
        assert frame.max() <= 1023 and frame.min() >= 0


def test_frames_index_is_the_capture_nodes(scan):
    """frames/frames.csv and frames/camera.json, as so101_scan_camera writes them."""
    base, info = scan
    lines = read_lines_csv(base / "lines.csv")
    with open(base / "frames" / "frames.csv", newline="") as f:
        rows = list(csv.reader(f))
    assert rows[0] == list(CAMERA_COLUMNS)
    rows = [dict(zip(rows[0], r)) for r in rows[1:]]
    assert [(int(r["sweep_id"]), int(r["index"])) for r in rows] == list(zip(lines["sweep_id"], lines["index"]))
    assert {r["status"] for r in rows} == {"ok"}
    cam = json.loads((base / "frames" / "camera.json").read_text())
    sensor = cam["sensor"]
    seq = np.array([int(r["seq"]) for r in rows])
    n = PLAN["lines"]
    for v, s in zip(info["viewpoints"], (slice(0, n), slice(n, 2 * n))):
        # locked to the camera: a whole number of frames a line, and the first good one kept
        assert v["frame_locked"] and v["line_period_s"] == pytest.approx(v["frames_per_line"] / sensor["fps"])
        assert (np.diff(seq[s]) == v["frames_per_line"]).all()
    r0, r1 = sensor["slit_rows"]
    window = round(((r1 - r0) * sensor["line_time_us"] + cam["exposure_us"]) * 1000)
    margin = round(cam["timing"]["match_margin_us"] * 1000)
    for r, stamp, hold in zip(rows, lines["stamp_ns"], lines["hold_until_ns"]):
        start, end, sof = int(r["exposure_start_ns"]), int(r["exposure_end_ns"]), int(r["sof_ns"])
        # the slit rows' rolling-shutter window, all of it while the mirror held still
        assert abs(end - start - window) <= 1 and stamp + margin <= start and end <= hold - margin
        assert end == sof + round(r1 * sensor["line_time_us"] * 1000)
        assert float(r["exposure_us"]) == 20000.0 and int(r["n_frames"]) >= 1
        frame = np.load(base / r["file"])                    # relative to the session
        assert int(r["saturated_px"]) == int((frame[r0:r1 + 1] >= 1023).sum())
    assert cam["format"] == "so101_scan frames v1"
    assert {k: sensor[k] for k in ("width", "height", "bits", "black_level", "bayer")} == {
        "width": 820, "height": 616, "bits": 10, "black_level": 64, "bayer": "RGGB"}
    assert sensor["fps"] == 30.0 and sensor["line_time_us"] == pytest.approx(2 * 18.904)  # 1640 x 1232, binned
    assert 0 < r0 < 100 and 516 < r1 < 615                   # the slit's image, most of the sensor
    assert cam["slit_reversed"] is False and cam["calibration"] is None
    assert cam["timing"]["stamp_offset_us"] == 0.0
    assert cam["exposure_us"] == 20000.0 and cam["gain"] == 1.0


def test_exposure_must_fit_in_a_frame(tmp_path):
    plan = load_plan("ring", lines=2, views=["down"])
    with pytest.raises(ValueError, match="doesn't fit"):       # 33.3 ms frames at 30 fps
        simulate(tmp_path / "out", plan, Settings(exposure_us=40000.0, calibration=False), log=lambda *m: None)
    assert not (tmp_path / "out").exists()


def test_references_and_calibration_session(scan):
    base, _ = scan
    refs, _ = load_session(base / "reference")
    assert refs["white"].kind == "white" and refs["white"].meta["reflectance"] == 0.98
    assert refs["dark"].kind == "dark"
    white, dark = refs["white"].summary()[0], refs["dark"].summary()[0]
    assert np.percentile(white - dark, 99.9) == pytest.approx(0.8 * (1023 - 64), rel=0.1)
    assert inspect_session(base / "calibration", log=lambda *a: None)


def test_truth_reads_back(scan):
    base, _ = scan
    t = ScanTruth(base)
    assert len(t) == 2 * PLAN["lines"]
    assert t.weights.shape == (len(t), t.h.size, len(t.materials))
    rho = t.reflectance(3, np.array([550.0, 850.0]))
    assert rho.shape == (t.h.size, 2)
    assert np.nanmax(rho) <= 1.0
    assert t.pure(3, 0.02).dtype == bool
    assert t.line_number(2, 0) == PLAN["lines"]


def test_same_seed_same_session(tmp_path):
    plan = load_plan("ring", lines=2, steps_per_line=16, views=["down"])
    s = Settings(binning=4, calibration=False, seed=11)
    a, b = tmp_path / "a", tmp_path / "b"
    simulate(a, plan, s, log=lambda *m: None)
    simulate(b, plan, s, log=lambda *m: None)
    assert (a / "lines.csv").read_bytes() == (b / "lines.csv").read_bytes()
    for k in range(2):
        name = f"frames/sweep_001/frame_{k:04d}.npy"
        assert (np.load(a / name) == np.load(b / name)).all()
    assert not (a / "calibration").exists()


def test_viewpoint_needs_joints(tmp_path):
    plan = load_plan({"name": "x", "target_m": [0.26, 0.0, -0.0024],
                      "viewpoints": [{"name": "nowhere", "joints_deg": {}}]})
    with pytest.raises(ValueError):
        simulate(tmp_path / "out", plan, Settings(calibration=False), log=lambda *m: None)
    assert not (tmp_path / "out").exists()
