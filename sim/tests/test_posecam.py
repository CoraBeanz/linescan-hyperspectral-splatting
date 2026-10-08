"""The pose camera, the tag board, a head off its CAD numbers, and binned lines for headcal."""

import csv
import json

import numpy as np
import pytest

pytest.importorskip("cv2")

from hsisim import repo  # noqa: E402,F401
from hsisim import scenes, spectra  # noqa: E402
from hsisim.session import Settings, load_plan, simulate  # noqa: E402

QUIET = dict(log=lambda *a: None)
VIEWS = ["t0_a0_+0_+0_+0", "t20_a90_+0_+0_+0"]


def _pose(row):
    from scipy.spatial.transform import Rotation
    t = np.eye(4)
    t[:3, 3] = [float(row[k]) for k in ("x", "y", "z")]
    t[:3, :3] = Rotation.from_quat([float(row[k]) for k in ("qx", "qy", "qz", "qw")]).as_matrix()
    return t


def test_tagboard_is_headcals_board():
    """Points on the simulated board are black where OpenCV draws the board black."""
    from headcal import geometry as g
    from headcal.board import Board
    scene = scenes.make("tagboard", target=(0.25, 0.02, -0.0024))
    board, pose = scene.board, scene.board_pose
    assert isinstance(board, Board)
    px = 8
    drawn = board.raster(px)                                 # 1 white, 0 black, area-averaged
    rng = np.random.default_rng(1)
    xy = rng.uniform([0, 0], [board.squares_x * board.square_mm, board.squares_y * board.square_mm], (4000, 2))
    clear = drawn[(xy[:, 1] * px).astype(int), (xy[:, 0] * px).astype(int)]
    keep = (clear == 0) | (clear == 1)                       # not on an edge of the drawing
    pts = g.apply(pose, np.c_[xy[keep] * 1e-3, np.zeros(keep.sum())])
    hits = scene.cast(pts + [0, 0, 0.05], np.tile([0.0, 0.0, -1.0], (len(pts), 1)))
    black = hits.material == spectra.material_id("carbon_black")
    assert np.mean(black == (clear[keep] == 0)) > 0.999
    assert np.allclose(hits.point[:, 2], -0.0024 + scenes.CARD)


@pytest.fixture(scope="module")
def stills(tmp_path_factory):
    """Two views of headcal's calibration scan, with the head off its CAD numbers and the pose camera on."""
    base = tmp_path_factory.mktemp("tagboard")
    plan = load_plan("headcal", lines=8, steps_per_line=8, views=VIEWS)
    simulate(base, plan, Settings(scene="tagboard", pose_camera=True, head_errors=1.0, calibration=False), **QUIET)
    return base


def test_stills_are_in_the_recorders_format(stills):
    from headcal.posecam import load_stills, load_grey
    info, got = load_stills(stills / "pose")
    assert (info["width"], info["height"], info["sensor_width"]) == (1632, 1232, 3264)
    assert len(got) == len(VIEWS)
    with open(stills / "lines.csv", newline="") as f:
        lines = list(csv.DictReader(f))
    for s, sweep in zip(got, (1, 2)):
        stamps = [int(r["stamp_ns"]) for r in lines if int(r["sweep_id"]) == sweep]
        # inside the window headcal solve takes a sweep's stills from
        assert stamps[0] + 200_000_000 <= s.stamp_ns <= stamps[-1]
        img = load_grey(s.path)
        assert img.shape == (1232, 1632) and 60 < np.percentile(img, 1) < 90 and np.percentile(img, 99.9) < 1023


def test_board_corners_land_where_the_truth_puts_them(stills):
    """The corners OpenCV finds in each still are where the true lens and pose camera put them."""
    import cv2
    from headcal import geometry as g
    from headcal.board import Board
    from headcal.model import PinholeCamera
    from headcal.posecam import detect, load_grey, load_stills
    truth = json.loads((stills / "truth" / "pose_camera.json").read_text())
    head = json.loads((stills / "truth" / "head.json").read_text())
    lens = PinholeCamera.from_json(truth["lens"])
    board, board_in_base = Board(**{k: v for k, v in head["board"].items() if k != "format"}), \
        np.array(head["board_in_base"])
    with open(stills / "truth" / "stills_true.csv", newline="") as f:
        true = list(csv.DictReader(f))
    # OpenCV 4's ChArUco detector puts the same corners half a pixel further down and right than
    # OpenCV 5's, which has pixel centres at integers as headcal's lens model does
    shift = 0.5 if int(cv2.__version__.split(".")[0]) < 5 else 0.0
    for s, row in zip(load_stills(stills / "pose")[1], true):
        ids, uv = detect(board, load_grey(s.path))
        assert len(ids) >= 60, len(ids)
        err = lens.project(g.apply(g.inv(_pose(row)) @ board_in_base, board.corners(ids))) - (uv - shift)
        assert np.sqrt((err ** 2).sum(1).mean()) < 0.2


def test_the_true_head_is_off_cad_and_lines_csv_logs_cad(stills):
    """With head errors, the frames come from the true head and lines.csv from the CAD one."""
    from headcal.model import HeadGeometry
    head = json.loads((stills / "truth" / "head.json").read_text())
    true, cad = HeadGeometry.from_json(head["head"]), HeadGeometry.from_json(head["cad"])
    # the URDF the frames were rendered with is the true head
    built = HeadGeometry.from_urdf(repo.robot(stills / "truth" / "head_calibration.yaml"))
    for part in ("mount", "mirror", "objective", "pose_camera"):
        assert np.allclose(getattr(built, part), getattr(true, part), atol=1e-8), part
    assert np.abs(true.mount[:3, 3] - cad.mount[:3, 3]).max() > 1e-4
    assert np.allclose(HeadGeometry.from_urdf(repo.robot()).mount, cad.mount)
    with open(stills / "truth" / "lines_true.csv", newline="") as f:
        t_rows = list(csv.DictReader(f))
    with open(stills / "lines.csv", newline="") as f:
        l_rows = list(csv.DictReader(f))
    for t, logged in zip(t_rows, l_rows):
        cam = HeadGeometry.from_json(head["head"]).line_camera(float(t["mirror_angle"]))
        head_pose = _pose({k: t["head_" + k] for k in ("x", "y", "z", "qx", "qy", "qz", "qw")})
        cam_pose = _pose({k: t["cam_" + k] for k in ("x", "y", "z", "qx", "qy", "qz", "qw")})
        assert np.allclose(head_pose @ cam, cam_pose, atol=1e-7)
        logged_cam = np.array([float(logged["cam_" + k]) for k in ("x", "y", "z")])
        assert np.linalg.norm(logged_cam - cam_pose[:3, 3]) > 2e-4


def test_binned_lines_are_line_cameras(calibrated, tmp_path):
    """`hsisim bin` writes what line_camera writes with a calibration, and headcal reads it."""
    import shutil
    from headcal.linecam import load_scan
    from hsisim.binned import bin_session
    from so101_scan_camera.binning import LineBinner, load_calibration
    base, cal, _ = calibrated
    session = tmp_path / "scan"
    shutil.copytree(base, session, ignore=shutil.ignore_patterns("calibration", "cal", "spectra", "binned"))
    bin_session(session, cal, **QUIET)
    binner = LineBinner(load_calibration(str(cal)), 256)
    lines = np.load(session / "binned" / "sweep_001.npy")
    assert lines.shape == (12, 256, binner.shape[1]) and lines.dtype == np.float32
    want = binner.bin(np.load(session / "frames" / "sweep_001" / "frame_0005.npy"))[0]
    assert np.array_equal(lines[5], want, equal_nan=True)
    assert (session / "binned" / "reference_white.npy").exists()
    cam = json.loads((session / "frames" / "camera.json").read_text())
    assert cam["binning"]["slit_bins"] == 256 and cam["calibration"]["maps_sha256"]
    sweeps, _ = load_scan(session, log=lambda *a: None)
    assert [s.sweep_id for s in sweeps] == [1, 2] and sweeps[0].image.shape == (12, 256)


def test_headcal_finds_the_simulated_head(calibrated, tmp_path):
    """headcal solve on a simulated calibration scan of a head off its CAD numbers, with a perfect
    arm: it finds the true head, as it does on its own synthetic scans."""
    from headcal import solve
    from hsisim.binned import bin_session
    from hsisim.handeye import LINES, STEPS, compare, logged_line_errors, plan_views
    _, cal, _ = calibrated
    session = tmp_path / "scan"
    plan = load_plan("headcal", lines=LINES, steps_per_line=STEPS, views=plan_views(4))
    simulate(session, plan, Settings(scene="tagboard", pose_camera=True, head_errors=1.0, errors=0.0,
                                     calibration=False), **QUIET)
    bin_session(session, cal, **QUIET)
    result = solve.run(str(session), str(session / "pose"), None, str(tmp_path / "headcal"), log=lambda *a: None)
    got, cad = compare(session, result)
    assert got["slit_reversed_ok"]
    # headcal's own self-test limits (calibration/headcal/selftest.py)
    assert got["line_mm_median"] < 0.4 and got["line_mm_max"] < 0.8, got
    assert got["pose_mm"] < 0.5 and got["pose_deg"] < 0.15, got
    assert got["f_px"] < 0.5 and got["c_px"] < 1.0 and got["k_rel"] < 0.003, got
    assert cad["line_mm_median"] > 2.0     # the CAD head is well off
    calibrated_mm = logged_line_errors(session, tmp_path / "headcal" / "head_calibration.yaml")
    assert np.median(calibrated_mm) < 0.4
