"""headcal: the head's hand-eye calibration (calibration/headcal).

The pieces one at a time (the board against OpenCV's drawing, the head model against the URDF,
the hand-eye solve, the recorder), then a small synthetic calibration scan from end to end,
whose head_calibration.yaml has to build, through xacro, into the URDF the solver describes.
`python -m headcal selftest` runs the full-size scan against tighter limits.
"""

import ast
import math
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
pytest.importorskip("xacro")
pytest.importorskip("yaml")

from headcal import board as board_mod  # noqa: E402
from headcal import geometry as g  # noqa: E402
from headcal import plan as plan_mod  # noqa: E402
from headcal import record, repo, solve, synth  # noqa: E402
from headcal.model import HeadGeometry, PinholeCamera  # noqa: E402

HEADCAL = Path(__file__).resolve().parent.parent / "headcal"


@pytest.fixture(scope="module")
def cad_urdf():
    return repo.build_urdf()


@pytest.fixture(scope="module")
def robot(cad_urdf):
    return repo.robot(cad_urdf)


@pytest.fixture(scope="module")
def nominal(robot):
    return HeadGeometry.from_urdf(robot)


def off_cad(nominal, seed=1):
    """A head a millimetre and a degree or so off the CAD's, as synth makes them."""
    return synth.make_truth(nominal, seed=seed).head


# --- geometry -------------------------------------------------------------------------------

def test_rotations_round_trip():
    rng = np.random.default_rng(0)
    for _ in range(20):
        w = rng.normal(0, 1.0, 3)
        np.testing.assert_allclose(g.rot_log(g.rot_exp(w)), w, atol=1e-9)
        r = g.rot_exp(w)
        np.testing.assert_allclose(g.rpy_matrix(*g.rpy(r)), r, atol=1e-12)
        np.testing.assert_allclose(g.quat_matrix(g.quat_wxyz(r)), r, atol=1e-12)
    t = g.exp6([0.01, -0.02, 0.03, 0.1, -0.2, 0.3])
    np.testing.assert_allclose(t @ g.inv(t), np.eye(4), atol=1e-12)
    np.testing.assert_allclose(g.pose_from_array(g.pose_array(t)), t, atol=1e-12)
    pts = rng.normal(0, 0.05, (10, 3))
    np.testing.assert_allclose(g.kabsch(pts, g.apply(t, pts)), t, atol=1e-9)


# --- the board -----------------------------------------------------------------------------

def test_board_drawing_matches_opencv():
    b = board_mod.Board()
    ppm = 12
    w, h = (round(v * 1e3 * ppm) for v in b.size_m)
    theirs = b.cv_board().generateImage((w, h), marginSize=0, borderBits=1)
    ours = b.raster(ppm)
    assert ours.shape == theirs.shape
    assert np.mean((ours < 0.5) != (theirs < 128)) < 1e-3


def test_board_corners_and_scaling(tmp_path):
    b = board_mod.Board()
    assert b.n_corners == (b.squares_x - 1) * (b.squares_y - 1) == len(b.cv_board().getChessboardCorners())
    np.testing.assert_allclose(b.corners(), np.asarray(b.cv_board().getChessboardCorners()), atol=1e-12)
    b.save(tmp_path / "board.json")
    assert board_mod.Board.load(tmp_path / "board.json") == b
    printed = b.scaled(198.0)     # 20 squares measured 198 mm: the printer shrank it 1 %
    assert printed.square_mm == pytest.approx(9.9) and printed.marker_mm == pytest.approx(6.93)
    svg = b.svg("letter")
    assert 'width="279.4mm" height="215.9mm"' in svg and "Laser printer" in svg


def test_board_is_found_in_its_own_picture():
    b = board_mod.Board()
    img = (b.texture(px_per_mm=6)[0] * 255).astype(np.uint8)
    from headcal import posecam
    ids, uv = posecam.detect(b, img, cv2.aruco.CharucoDetector(b.cv_board()))
    assert len(ids) > 0.95 * b.n_corners


# --- the head model -------------------------------------------------------------------------

@pytest.mark.parametrize("t", [-0.3, 0.0, 0.25])
def test_line_camera_matches_the_urdf(robot, nominal, t):
    want = robot.fk("line_camera_optical_frame", {"scan_mirror_joint": t}, base="scan_head_link")
    np.testing.assert_allclose(nominal.line_camera(t), want, atol=1e-9)
    # the scan line's ends are the slit's ends, and the line has no width
    line = robot.fk("scan_line_frame", {"scan_mirror_joint": t}, base="scan_head_link")
    ends = g.apply(line, [[-nominal.half_line, 0, 0], [0, 0, 0], [nominal.half_line, 0, 0]])
    h, w = nominal.slit_project(ends, np.full(3, t))
    np.testing.assert_allclose(h, [-1, 0, 1], atol=1e-6)
    np.testing.assert_allclose(w, 0, atol=1e-9)


def test_slit_rays_and_projection_agree(nominal):
    head = off_cad(nominal)
    head.slit_k1 = 0.02
    h = np.linspace(-0.95, 0.95, 7)
    t = np.full(7, 0.1)
    d, c = head.slit_ray(h, t)
    hh, w = head.slit_project(c + 0.12 * d, t)
    np.testing.assert_allclose(hh, h, atol=1e-9)
    np.testing.assert_allclose(w, 0, atol=1e-12)


def test_head_calibration_yaml_builds_the_same_urdf(tmp_path, nominal, cad_urdf):
    head = off_cad(nominal)
    (tmp_path / "head_calibration.yaml").write_text(head.calibration_yaml("test"))
    built = repo.robot(repo.build_urdf(str(tmp_path / "head_calibration.yaml")))
    patched = repo.robot(head.patch_urdf(cad_urdf))
    again = HeadGeometry.from_urdf(built)
    np.testing.assert_allclose(again.mount, head.mount, atol=1e-8)
    np.testing.assert_allclose(again.pose_camera, head.pose_camera, atol=1e-8)
    assert again.half_line == pytest.approx(head.half_line, abs=1e-9)   # written to 9 digits
    for t in (-0.2, 0.0, 0.3):
        q = {"scan_mirror_joint": t}
        for link in ("line_camera_optical_frame", "scan_line_frame", "pose_camera_optical_frame"):
            a = built.fk(link, q, base="wrist_roll_link")
            np.testing.assert_allclose(a, patched.fk(link, q, base="wrist_roll_link"), atol=1e-9, err_msg=link)
        np.testing.assert_allclose(built.fk("line_camera_optical_frame", q, base="scan_head_link"),
                                   head.line_camera(t), atol=1e-8)


def test_pinhole_matches_opencv():
    cam = PinholeCamera(1632, 1232, 1350.0, 1352.0, 810.0, 620.0, np.array([0.14, -0.3, 0.0008, -0.0006, 0.0]))
    rng = np.random.default_rng(2)
    p = np.column_stack([rng.uniform(-0.1, 0.1, (50, 2)), rng.uniform(0.15, 0.3, 50)])
    theirs, _ = cv2.projectPoints(p, np.zeros(3), np.zeros(3), cam.matrix, cam.dist)
    np.testing.assert_allclose(cam.project(p), theirs[:, 0], atol=1e-6)
    rays = cam.rays(cam.project(p))
    np.testing.assert_allclose(rays[:, :2] / rays[:, 2:], p[:, :2] / p[:, 2:], atol=1e-6)


# --- hand-eye ---------------------------------------------------------------------------------

def test_park_martin_hand_eye():
    rng = np.random.default_rng(4)
    x = g.exp6([0.01, 0.02, 0.05, 0.3, -0.2, 2.0])
    z = g.exp6([0.25, 0.0, 0.0, math.pi, 0.0, 0.3])
    wrists = [g.exp6(np.concatenate([rng.normal([0.2, 0, 0.2], 0.05), rng.normal(0, 0.5, 3)])) for _ in range(8)]
    cams = [g.inv(w @ x) @ z for w in wrists]
    x_est, z_est = solve.hand_eye_init(wrists, cams)
    np.testing.assert_allclose(x_est, x, atol=1e-9)
    np.testing.assert_allclose(z_est, z, atol=1e-9)


# --- the plan ------------------------------------------------------------------------------

def test_plan_looks_at_the_board(robot):
    p = synth.load_plan()
    assert len(p["viewpoints"]) == len(plan_mod.VIEWS) and p["camera"]["record"] == "required"
    limits = robot.limits()
    t_mid = plan_mod.mirror_angles(p["sweep"])[p["sweep"]["n_lines"] // 2]
    for vp in p["viewpoints"]:
        q = {j: math.radians(v) for j, v in vp["joints_deg"].items()}
        assert all(limits[j][0] <= q[j] <= limits[j][1] for j in q), vp["name"]
        line = robot.fk("scan_line_frame", dict(q, scan_mirror_joint=t_mid))
        # mid-sweep, the scan line's centre is on the board (40 mm off its centre at most) and
        # within 13 mm of focus
        assert np.hypot(*(line[:2, 3] - p["target_m"][:2])) < 0.045, vp["name"]
        assert abs(line[2, 3] - p["target_m"][2]) < 0.013, vp["name"]
        assert plan_mod.lowest_point(robot, q) > plan_mod.TABLE_Z + 0.011, vp["name"]


def test_plan_is_a_scan_sweep_plan():
    repo.line_log()     # puts so101_scan_sweep on the path
    from so101_scan_sweep import plan as sweep_plan
    p = sweep_plan.load(str(synth.PLAN))
    assert len(p.viewpoints) == len(plan_mod.VIEWS)


# --- the recorder (Jetson) -------------------------------------------------------------------

@pytest.mark.parametrize("name", ["record.py", "__init__.py", "__main__.py"])
def test_jetson_side_is_python_36(name):
    """JetPack 4 has Python 3.6: no walrus, no positional-only arguments, no f-string '='."""
    tree = ast.parse((HEADCAL / name).read_text())
    for node in ast.walk(tree):
        assert not isinstance(node, ast.NamedExpr), name
        if isinstance(node, (ast.FunctionDef, ast.Lambda)):
            assert not node.args.posonlyargs, name
        if isinstance(node, ast.JoinedStr):
            raise AssertionError("%s uses an f-string (fine on 3.6, but keep the Jetson side plain)" % name)


def test_grey_averages_colour_cells():
    raw = np.array([[10, 20, 30, 40], [30, 40, 50, 60]], np.uint16)
    np.testing.assert_array_equal(record.grey(raw), [[25, 45]])
    assert record.change(np.full((4, 4), 164), np.full((4, 4), 164)) == 0.0


def test_sharpness_prefers_the_sharper_view():
    rng = np.random.default_rng(3)
    sharp = 64 + 400 * (rng.uniform(0, 1, (48, 64)) > 0.5)
    soft = 64 + 400 * cv2.GaussianBlur((sharp - 64) / 400.0, (0, 0), 2.0)
    assert record.sharpness(sharp) > 3 * record.sharpness(soft)
    assert record.sharpness(2 * (sharp - 64) + 64) == pytest.approx(record.sharpness(sharp))


class FakeCamera:
    """Frames as hsical's V4L2Camera.grab gives them: the arm moves, holds, moves, holds."""

    def __init__(self, scenes):
        self.scenes, self.i = scenes, 0
        self.size = (64, 48)
        self.cfg = {"device": "/dev/fake"}

    def grab(self, scratch, exposure_us, gain, n):
        rng = np.random.default_rng(self.i)
        scene = self.scenes[min(self.i, len(self.scenes) - 1)]
        self.i += 1
        img = 64 + scene * 400.0 + rng.normal(0, 2.0, scene.shape)
        return [np.clip(img, 0, 1023).astype(np.uint16)], {}


def test_recorder_keeps_one_still_per_hold(tmp_path):
    rng = np.random.default_rng(9)
    a, b = rng.uniform(0, 1, (48, 64)), rng.uniform(0, 1, (48, 64))
    moving = [rng.uniform(0, 1, (48, 64)) for _ in range(3)]
    scenes = moving[:1] + [a] * 4 + moving[1:] + [b] * 4
    n = record.record(FakeCamera(scenes), str(tmp_path / "pose"), period=0.0, max_grabs=len(scenes),
                      log=lambda *x: None)
    assert n == 2
    lines = (tmp_path / "pose" / "frames.csv").read_text().splitlines()
    assert lines[0] == "index,stamp_ns,file,exposure_us,gain" and len(lines) == 3
    still = np.load(tmp_path / "pose" / "still_0000.npy")
    assert still.shape == (24, 32) and still.dtype == np.uint16
    assert not (tmp_path / "pose" / ".grab").exists()
    # while a view holds, another still every `every` seconds (here every grab)
    n = record.record(FakeCamera(scenes), str(tmp_path / "again"), period=0.0, max_grabs=len(scenes), every=0.0,
                      log=lambda *x: None)
    assert n == 6


# --- end to end, small -----------------------------------------------------------------------

@pytest.fixture(scope="module")
def small_run(tmp_path_factory, nominal, cad_urdf):
    """Eight viewpoints of a synthetic calibration scan, calibrated (about 30 s)."""
    out = tmp_path_factory.mktemp("headcal")
    truth = synth.make_truth(nominal, seed=21)
    p = synth.load_plan()
    p = dict(p, viewpoints=p["viewpoints"][::3][:8])
    synth.write_scan(str(out), truth, plan=p, nominal_urdf=cad_urdf, log=lambda *a: None)
    res = solve.run(str(out / "scan"), str(out / "pose"), str(out / "board.json"), str(out / "cal"),
                    log=lambda *a: None)
    return out, truth, res, p


def test_small_calibration_is_close(small_run, robot):
    out, truth, res, p = small_run
    got = synth.compare(truth, res, plan=p, robot=robot)
    assert got["slit_reversed_ok"]
    assert got["line_mm_median"] < 1.0 and got["line_mm_max"] < 2.0, got
    assert got["pose_mm"] < 1.5 and got["pose_deg"] < 0.5, got     # eight views: the arm noise counts more
    assert got["f_px"] < 2.0 and got["k_rel"] < 0.01, got


def test_small_calibration_outputs(small_run):
    out, truth, res, p = small_run
    cal = out / "cal"
    for name in ("head_calibration.yaml", "robot.urdf", "calibration.json", "pose_camera.yaml", "report.md"):
        assert (cal / name).stat().st_size > 100, name
    report = (cal / "report.md").read_text()
    assert "head_calibration.yaml" in report and "scan_to_dataset" in report
    # the YAML through xacro gives the URDF the solver wrote, joint for joint
    built = repo.robot(repo.build_urdf(str(cal / "head_calibration.yaml")))
    written = repo.robot((cal / "robot.urdf").read_text())
    for name in ("scan_head_mount", "scan_mirror_joint", "spectrograph_optical_joint", "line_camera_fold_joint",
                 "line_camera_optical_joint", "pose_camera_joint", "scan_mirror_face_joint"):
        np.testing.assert_allclose(built.joints[name]["origin"], written.joints[name]["origin"], atol=1e-9,
                                   err_msg=name)
    for t in (-0.1, 0.0, 0.1):
        np.testing.assert_allclose(written.fk("line_camera_optical_frame", {"scan_mirror_joint": t},
                                              base="scan_head_link"), res.head.line_camera(t), atol=1e-8)


def test_too_few_pose_camera_views(tmp_path, small_run):
    out, *_ = small_run
    import shutil
    shutil.copytree(out / "pose", tmp_path / "pose")
    index = tmp_path / "pose" / "frames.csv"
    index.write_text("\n".join(index.read_text().splitlines()[:9]) + "\n")   # 8 stills: 4 viewpoints'
    with pytest.raises(RuntimeError, match="needs 6 or more"):
        solve.run(str(out / "scan"), str(tmp_path / "pose"), str(out / "board.json"), None, log=lambda *a: None)
