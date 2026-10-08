"""A synthetic calibration scan with a known answer, in the files the real rig writes.

The truth is the CAD head with every unknown disturbed (the head turned on the horn, the pose
camera and the objective off their seats, the mirror's shaft tilted and its home angle wrong,
the scan line a few percent long, some distortion), a real-looking pose-camera lens, and an arm
whose joints read a little off. The scan goes through the plan's viewpoints: at each, the pose
camera takes a still of the board (rendered through its lens, with noise), and the line camera
sweeps it (every line rendered through its own reflected camera, blurred by the slit, the lens
and the focus, with noise). Out come the folders a real run leaves:

    scan/       scan_sweep and line_camera's scan folder: lines.csv (logged with the CAD URDF,
                as the rig would), scan.json, robot.urdf, plan.yaml, frames/, binned/
    pose/       the pose-camera recorder's folder: camera.json, frames.csv, still_NNNN.npy
                (with stills taken while the arm moved, which the solver must leave out)
    board.json  the board
    truth.json  the answer
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import board as board_mod
from . import geometry as g
from . import linecam, plan as plan_mod, repo
from .model import HeadGeometry, PinholeCamera

PLAN = Path(__file__).resolve().parent / "plans" / "headcal.yaml"
BLACK_LEVEL = 64.0


@dataclass
class Truth:
    head: HeadGeometry
    camera: PinholeCamera
    board: board_mod.Board
    board_in_base: np.ndarray
    joint_offsets: dict = field(default_factory=dict)   # rad, the arm's reading minus the truth
    joint_noise: float = 0.0                            # rad, per viewpoint
    slit_reversed: bool = False                         # what the sensor does
    slit_reversed_flag: bool = None                     # what camera.json says (None: the truth)
    seed: int = 0

    def to_json(self):
        return dict(head=self.head.to_json(), camera=self.camera.to_json(), board=self.board.to_json(),
                    board_in_base=g.pose_array(self.board_in_base).tolist(),
                    joint_offsets=self.joint_offsets, joint_noise=self.joint_noise,
                    slit_reversed=self.slit_reversed, seed=self.seed)


def board_on_table(board, centre_xy, yaw, z=plan_mod.TABLE_Z + 0.0015):
    """board -> base_link for a board lying face up (printed side toward +z) on the table."""
    r = g.rotation_about([0, 0, 1], yaw) @ np.diag([1.0, -1.0, -1.0])
    t = np.array([centre_xy[0], centre_xy[1], z]) - r @ board.centre
    return g.make(r, t)


def make_truth(nominal, board=None, seed=7, errors=1.0, camera_scale=1.0, joint_offsets_deg=None,
               joint_noise_deg=0.03, slit_reversed=False, slit_reversed_flag=None, pose_camera_turn_deg=0.0):
    """The truth: the nominal head disturbed by about `errors` times what a printed, hand-built
    head is likely to be off."""
    rng = np.random.default_rng(seed)
    d = math.radians
    e = errors
    head = nominal.copy()

    def nudge(sig_mm, sig_deg, mask=(1, 1, 1, 1, 1, 1)):
        xi = np.concatenate([rng.normal(0, sig_mm * 1e-3 * e, 3), rng.normal(0, d(sig_deg) * e, 3)])
        return xi * np.asarray(mask, float)

    head.mount = g.perturb(head.mount, nudge(0.6, 1.5))
    head.pose_camera = g.perturb(head.pose_camera, nudge(1.0, 1.5))
    head.pose_camera = g.perturb(head.pose_camera, [0, 0, 0, 0, 0, d(pose_camera_turn_deg)])  # turned on its axis
    head.objective = g.perturb(head.objective, nudge(0.5, 0.6))
    head.mirror = g.perturb(head.mirror, nudge(0.4, 0.5, (0, 1, 1, 0, 1, 1)))
    head.mirror = g.perturb(head.mirror, [0, 0, 0, d(rng.normal(0, 1.5)) * e, 0, 0])   # the home angle
    head.half_line *= 1.0 + rng.normal(0, 0.02) * e
    head.slit_k1 = 0.015 * e
    w, h = int(round(1632 * camera_scale)), int(round(1232 * camera_scale))
    f = 1357.0 * camera_scale * (1.0 + rng.normal(0, 0.01))
    cam = PinholeCamera(w, h, f, f * (1.0 + rng.normal(0, 0.002)), (w - 1) / 2 + rng.normal(0, 12 * camera_scale),
                        (h - 1) / 2 + rng.normal(0, 12 * camera_scale), np.array([0.14, -0.30, 0.0008, -0.0006, 0.0]))
    board = board or board_mod.Board()
    centre = np.array(plan_mod.BOARD_CENTRE) + rng.normal(0, 0.008, 2)
    b2b = board_on_table(board, centre, d(rng.uniform(-20, 20)))
    offs = {k: d(v) for k, v in (joint_offsets_deg or {}).items()}
    return Truth(head, cam, board, b2b, offs, d(joint_noise_deg), slit_reversed, slit_reversed_flag, seed)


def load_plan(path=PLAN):
    import yaml
    with open(path) as f:
        return yaml.safe_load(f)


# --- rendering ---------------------------------------------------------------------------------------

class StillRenderer:
    """Pose-camera stills of the board: every pixel's ray (2 x 2 per pixel) through the lens."""

    def __init__(self, cam, texture):
        self.cam, self.texture = cam, texture
        ys, xs = np.mgrid[0:cam.height, 0:cam.width]
        self.rays = []
        for oy in (-0.25, 0.25):
            for ox in (-0.25, 0.25):
                uv = np.stack([xs.ravel() + ox, ys.ravel() + oy], axis=-1)
                self.rays.append(cam.rays(uv).astype(np.float32))
        r2 = ((xs - cam.cx) ** 2 + (ys - cam.cy) ** 2) / (cam.fx ** 2)
        self.vignette = (1.0 / (1.0 + r2)) ** 2          # cos^4 fall-off

    def render(self, board_in_cam, rng, level=700.0):
        rb, tb = board_in_cam[:3, :3], board_in_cam[:3, 3]
        c = -rb.T @ tb                                    # the camera in the board frame
        acc = 0.0
        for d in self.rays:
            db = d @ rb.astype(np.float32)
            with np.errstate(divide="ignore", invalid="ignore"):
                s = -c[2] / db[:, 2]
            x = np.where(s > 0, c[0] + s * db[:, 0], np.nan)
            y = np.where(s > 0, c[1] + s * db[:, 1], np.nan)
            acc = acc + self.texture.sample(x.reshape(self.cam.height, -1), y.reshape(self.cam.height, -1))
        sig = level * self.vignette * acc / len(self.rays)
        img = BLACK_LEVEL + sig + rng.normal(0, 1.0, sig.shape) * np.sqrt(2.0 ** 2 + 0.25 * sig)
        return np.clip(np.round(img), 0, 1023).astype(np.uint16)


def render_line_sweep(truth, board_in_head, t, width, texture, rng, level=1500.0):
    """A sweep as line_camera would bin it: mean counts (lines, slit bins), in the sensor's slit
    order (reversed if the truth says so)."""
    import cv2
    head = truth.head
    img = linecam.render_sweep(head, board_in_head, t, width, texture, sub_u=3, sub_w=3)
    # focus: the board's distance from the scan line's focus, at the middle of the sweep
    _, _, dist = linecam.board_points(head, board_in_head, [float(np.median(t))], np.array([0.0]))
    blur_rad = 0.004 * abs(1.0 / float(dist[0, 0]) - 1.0 / head.scene_distance)   # 4 mm aperture
    f_px = width / (2.0 * head.k)
    step = float(np.median(np.abs(np.diff(t))))
    sig_u = math.hypot(0.4, 0.4 * blur_rad * f_px)
    sig_l = math.hypot(0.3, 0.4 * blur_rad / (2.0 * step))
    img = cv2.GaussianBlur(img, (0, 0), sigmaX=sig_u, sigmaY=sig_l)
    u = (np.arange(width) + 0.5) / width * 2.0 - 1.0
    lamp = 1.0 - 0.3 * u ** 2                                 # the lamp and the lens's fall-off
    sig = level * img * lamp[None, :]
    counts = sig + rng.normal(0, 1.0, sig.shape) * np.sqrt(4.0 + 0.3 * sig)
    if truth.slit_reversed:
        counts = counts[:, ::-1]
    return counts


# --- the scan -------------------------------------------------------------------------------------------

def write_scan(out, truth=None, plan=None, nominal_urdf=None, width=256, bands=4, max_views=None,
               log=print):
    """Render the synthetic calibration scan into out/ (scan/, pose/, board.json, truth.json)."""
    out = Path(out)
    plan = plan or load_plan()
    urdf = nominal_urdf or repo.build_urdf()
    robot = repo.robot(urdf)
    nominal = HeadGeometry.from_urdf(robot)
    truth = truth or make_truth(nominal)
    rng = np.random.default_rng(truth.seed + 1)
    ll = repo.line_log()
    poser = ll.LinePoser(robot)

    scan, pose = out / "scan", out / "pose"
    for p in (scan / "frames", scan / "binned", pose):
        p.mkdir(parents=True, exist_ok=True)
    (scan / "robot.urdf").write_text(urdf)
    truth.board.save(out / "board.json")
    texture = linecam.Texture(truth.board, px_per_mm=12, blur_px=0.5)
    stills = StillRenderer(truth.camera, texture)

    views = plan["viewpoints"][:max_views] if max_views else plan["viewpoints"]
    t = plan_mod.mirror_angles(plan["sweep"])
    nm_edges = np.linspace(500.0, 900.0, bands + 1)
    np.savez(scan / "binned" / "binning.npz", slit_edges=np.linspace(0.0, width, width + 1), nm_edges=nm_edges,
             pixels=np.full((width, bands), 40, np.int32), response=np.zeros(0))
    lines = ll.LinesCsv(str(scan / "lines.csv"))
    fcsv = open(scan / "frames" / "frames.csv", "w", newline="")
    fw = csv.writer(fcsv)
    fw.writerow(["sweep_id", "index", "status", "file", "seq", "sof_ns", "exposure_start_ns", "exposure_end_ns",
                 "exposure_us", "gain", "saturated_px", "n_frames"])
    scsv = open(pose / "frames.csv", "w", newline="")
    sw = csv.writer(scsv)
    sw.writerow(["index", "stamp_ns", "file", "exposure_us", "gain"])
    n_still = 0
    t0 = 1_791_000_000 * 10 ** 9
    period = int(plan["sweep"]["line_period_s"] * 1e9)
    info_views = []
    names = repo.ARM_JOINTS

    def still(q_true, stamp):
        nonlocal n_still
        wrist = robot.fk("wrist_roll_link", q_true)
        board_in_cam = g.inv(wrist @ truth.head.mount @ truth.head.pose_camera) @ truth.board_in_base
        img = stills.render(board_in_cam, rng)
        name = "still_%04d.npy" % n_still
        np.save(pose / name, img)
        sw.writerow([n_still, stamp, name, 8000.0, 1.0])
        n_still += 1

    previous = None
    for v, vp in enumerate(views):
        sid = v + 1
        q_read = {j: math.radians(vp["joints_deg"][j]) for j in names}
        q_true = {j: q_read[j] - truth.joint_offsets.get(j, 0.0) + rng.normal(0, truth.joint_noise) for j in names}
        start = t0 + v * 30 * 10 ** 9
        if previous is not None:   # a still while the arm moves: it has no sweep and must be left out
            mid = {j: 0.5 * (previous[j] + q_true[j]) for j in names}
            still(mid, start - 6 * 10 ** 9)
        previous = q_true
        wrist = robot.fk("wrist_roll_link", q_true)
        board_in_head = g.inv(wrist @ truth.head.mount) @ truth.board_in_base
        counts = render_line_sweep(truth, board_in_head, t, width, texture, rng)
        weights = np.linspace(0.8, 1.2, bands)
        binned = (BLACK_LEVEL + counts[..., None] * weights / weights.sum()).astype(np.float32)
        np.save(scan / "binned" / ("sweep_%03d.npy" % sid), binned)
        for i, ang in enumerate(t):
            stamp = start + i * period
            head_pose, cam_pose = poser.poses(q_read, float(ang))
            lines.write(vp["name"], sid, i, stamp, stamp + period - 10 ** 6, True, float(ang), head_pose, cam_pose,
                        q_read)
            fw.writerow([sid, i, "ok", "", i, stamp + period - 2 * 10 ** 6, stamp + 10 ** 6,
                         stamp + period - 3 * 10 ** 6, 20000.0, 1.0, 0, 1])
        still(q_true, start + int(0.4 * len(t) * period))
        info_views.append({"name": vp["name"], "joints_goal": vp["joints_deg"]})
        log("  viewpoint %d/%d %s" % (v + 1, len(views), vp["name"]))
    lines.close()
    fcsv.close()
    scsv.close()

    flag = truth.slit_reversed if truth.slit_reversed_flag is None else truth.slit_reversed_flag
    (scan / "frames" / "camera.json").write_text(json.dumps(
        {"format": "so101_scan frames v1", "sensor": "synthetic (headcal synth)", "slit_reversed": bool(flag),
         "binning": {"slit_bins": width, "nm_edges": nm_edges.tolist()}}, indent=2) + "\n")
    (scan / "scan.json").write_text(json.dumps(
        {"format": "so101_scan lines v1", "name": plan["name"], "plan": plan, "viewpoints": info_views,
         "frames": {"base": "base_link", "head": "scan_head_link", "line_camera": "line_camera_optical_frame",
                    "scene_distance_m": poser.scene_distance, "scan_line_half_length_m": poser.half_line},
         "simulated": {"by": "headcal synth", "seed": truth.seed}}, indent=2) + "\n")
    import yaml
    (scan / "plan.yaml").write_text(yaml.safe_dump(plan, sort_keys=False))
    (pose / "camera.json").write_text(json.dumps(
        {"format": "headcal pose frames v1", "sensor": "synthetic Pi camera v2 (headcal synth)",
         "width": truth.camera.width, "height": truth.camera.height,
         "grey": "2 x 2 colour cells averaged, raw counts (black level 64)"}, indent=2) + "\n")
    (out / "truth.json").write_text(json.dumps(truth.to_json(), indent=2) + "\n")
    return truth


# --- against the truth ------------------------------------------------------------------------------------

def compare(truth, result, plan=None, robot=None):
    """How far the calibration is from the truth, in the terms that matter downstream.

    line_mm:  where the line camera's rays meet the board, calibrated head against the true one,
              both carried by the true arm pose (so the arm's own errors don't count): over every
              viewpoint, three mirror angles and three slit positions
    pose_mm, pose_deg: the pose camera on the wrist
    f_px, c_px: the pose camera's focal lengths and centre; k_rel: the scan line's length"""
    plan = plan or load_plan()
    robot = robot or repo.robot(repo.build_urdf())
    t_all = plan_mod.mirror_angles(plan["sweep"])
    ts = [t_all[0], t_all[len(t_all) // 2], t_all[-1]]
    th, ch = truth.head, result.head
    errs = []
    for vp in plan["viewpoints"]:
        q = {j: math.radians(vp["joints_deg"][j]) for j in repo.ARM_JOINTS}
        wrist = robot.fk("wrist_roll_link", q)
        b_in_wrist = g.inv(wrist) @ truth.board_in_base
        for t in ts:
            for hh in (-0.9, 0.0, 0.9):
                pts = []
                for head in (th, ch):
                    hh_d = hh + head.slit_k1 * hh ** 3   # same point of the slit: undistorted h
                    d, c = head.slit_ray(np.array([hh_d]), np.array([t]))
                    d = head.mount[:3, :3] @ d[0]
                    c = g.apply(head.mount, c[0])
                    n, p0 = b_in_wrist[:3, 2], b_in_wrist[:3, 3]
                    s = n @ (p0 - c) / (n @ d)
                    pts.append(c + s * d)
                errs.append(np.linalg.norm(pts[0] - pts[1]))
    errs = np.array(errs) * 1e3
    a, b = th.mount @ th.pose_camera, ch.mount @ ch.pose_camera
    tc, cc = truth.camera, result.camera
    return dict(line_mm_median=float(np.median(errs)), line_mm_max=float(errs.max()),
                pose_mm=float(np.linalg.norm(a[:3, 3] - b[:3, 3]) * 1e3),
                pose_deg=math.degrees(g.angle_between(a[:3, :3], b[:3, :3])),
                f_px=float(max(abs(tc.fx - cc.fx), abs(tc.fy - cc.fy))),
                c_px=float(max(abs(tc.cx - cc.cx), abs(tc.cy - cc.cy))),
                k_rel=float(abs(ch.k / th.k - 1.0)),
                slit_reversed_ok=bool(result.slit_reversed == truth.slit_reversed))
