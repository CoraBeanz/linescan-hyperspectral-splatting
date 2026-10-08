"""The solver: from a calibration scan to the head's geometry.

It works in two steps, so the arm's own errors can't leak into how the cameras sit:

  1. Cameras. At every viewpoint the pose camera sees the board, so the board's pose in the
     head is known there without the arm. A bundle adjustment then fits, all together, the
     pose camera's lens, every viewpoint's board pose, and the line camera's geometry (the
     objective's place and turn, the mirror shaft's place and tilt, the mirror's home angle,
     the scan line's length and its distortion), to every board corner both cameras saw. The
     pose camera's place in the head stays fixed here: moving it would move everything else
     with it (the head frame is only defined up to that), and step 3 places it.
  2. Hand-eye. The arm's forward kinematics say where the wrist was at each viewpoint, and
     step 1 says where the board was in the head; one board pose on the table and one head
     pose on the wrist must explain them all (A X B = Z). What doesn't fit is the arm's own
     error, reported per viewpoint.
  3. Gauge. The head frame is then put back where the CAD has it, as near as the cameras and
     the mirror allow: one rigid move of all three toward their CAD seats, the same move
     taken out of the mount. The mount then carries what is common (the head turned on the
     servo horn, the wrist servo's zero), and each part keeps its own error.

Before step 1 the pose camera is placed by a classic hand-eye solve (Park and Martin) of its
own, from the board poses it sees, so a camera mounted the wrong way round costs nothing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import geometry as g
from . import linecam, posecam
from .model import HeadGeometry, PinholeCamera, slit_coordinate

# measurement noise the residuals are divided by
SIGMA_POSE_PX = 0.3          # pose camera corners
SIGMA_SLIT_PX = 0.15         # line camera, along the slit
SIGMA_LINES = 0.15           # line camera, across: fractions of a line
SIGMA_ARM_M = 0.001          # hand-eye: where the board lands from each viewpoint
# priors on the line camera's geometry, around the CAD (or the scan URDF's) numbers
PRIOR_MM = 3.0
PRIOR_DEG = 3.0
PRIOR_LOG_K = 0.10
PRIOR_K1 = 0.10

LINE_PARAMS = ["objective x", "objective y", "objective z", "objective turn across the slit",
               "slit roll", "mirror shaft y", "mirror shaft z", "mirror home angle", "mirror shaft tilt (y)",
               "mirror shaft tilt (z)", "scan line length (log)", "slit distortion k1"]
LENS_PARAMS = ["fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2"]


@dataclass
class View:
    sweep: linecam.Sweep
    wrist: np.ndarray                    # wrist_roll_link -> base_link at the sweep's joints
    detection: posecam.Detection = None
    board_in_pc: np.ndarray = None       # board -> pose camera (PnP)
    pnp_rms: float = np.nan
    board_in_head: np.ndarray = None     # board -> head (step 1)
    line_obs: dict = None                # ids, line, u, t, h, score
    arm_mm: float = np.nan               # step 2 residual
    arm_deg: float = np.nan

    @property
    def name(self):
        return self.sweep.viewpoint


@dataclass
class Result:
    head: HeadGeometry
    camera: PinholeCamera
    board_in_base: np.ndarray
    nominal: HeadGeometry
    views: list
    stats: dict = field(default_factory=dict)
    sigmas: dict = field(default_factory=dict)
    warnings: list = field(default_factory=list)
    slit_reversed: bool = False
    joint_offsets: dict = field(default_factory=dict)


# --- gathering ----------------------------------------------------------------------------------------------

def match_stills(sweeps, stills, settle_ns=200_000_000):
    """{sweep_id: [stills taken while that sweep ran]}: the arm holds still from the first line
    to the last, so a still in that window was taken from that viewpoint."""
    out = {s.sweep_id: [] for s in sweeps}
    for st in stills:
        for s in sweeps:
            if s.start_ns + settle_ns <= st.stamp_ns <= s.end_ns:
                out[s.sweep_id].append(st)
                break
    return out


def gather(sweeps, robot, stills, board, log=print):
    """Views (one per sweep) with their pose-camera detection."""
    import cv2
    det = cv2.aruco.CharucoDetector(board.cv_board())
    by_sweep = match_stills(sweeps, stills)
    views = []
    for s in sweeps:
        v = View(s, robot.fk("wrist_roll_link", s.joints))
        best = None
        for st in by_sweep[s.sweep_id]:
            img = posecam.load_grey(st.path)
            ids, uv = posecam.detect(board, img, det)
            if len(ids) and (best is None or len(ids) > len(best.ids)):
                best = posecam.Detection(st, ids, uv, (img.shape[1], img.shape[0]))
        v.detection = best
        if s.joint_spread > math.radians(0.3):
            log("  ! sweep %d (%s): the arm moved %.2f deg during the sweep" % (s.sweep_id, s.viewpoint,
                                                                              math.degrees(s.joint_spread)))
        views.append(v)
    n_still = sum(len(x) for x in by_sweep.values())
    log("%d sweeps; %d pose-camera stills taken during them (%d others left out); board found in %d"
        % (len(sweeps), n_still, len(stills) - n_still, sum(v.detection is not None for v in views)))
    return views


# --- step 0: the pose camera on its own -------------------------------------------------------------------

def hand_eye_init(wrists, board_in_cams, min_turn_deg=5.0):
    """Park and Martin's closed-form hand-eye solve: the camera in the wrist frame (X), and the
    board in the base frame (Z), from wrist -> base poses W_i and board -> camera poses C_i.

    The board stays put, so W_i X C_i = Z for every i, and for any two viewpoints
    A X = X B with A = W_j^-1 W_i (how the wrist moved) and B = C_j C_i^-1 (how the board seemed
    to move). The rotation's axis-angle vectors then satisfy a = R_X b, an orthogonal
    Procrustes problem; the translation follows by least squares. Pairs that turn less than
    min_turn_deg say little about the axis and are left out."""
    alphas, betas, pairs = [], [], []
    for i in range(len(wrists)):
        for j in range(i + 1, len(wrists)):
            a = g.inv(wrists[j]) @ wrists[i]
            b = board_in_cams[j] @ g.inv(board_in_cams[i])
            la, lb = g.rot_log(a[:3, :3]), g.rot_log(b[:3, :3])
            if min(np.linalg.norm(la), np.linalg.norm(lb)) < math.radians(min_turn_deg):
                continue
            alphas.append(la)
            betas.append(lb)
            pairs.append((a, b))
    if len(pairs) < 3:
        raise RuntimeError("the viewpoints don't turn the head enough for a hand-eye solve (need tilts from "
                           "several directions)")
    u, _, vt = np.linalg.svd(np.array(betas).T @ np.array(alphas))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r_x = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    lhs = np.concatenate([a[:3, :3] - np.eye(3) for a, _ in pairs])
    rhs = np.concatenate([r_x @ b[:3, 3] - a[:3, 3] for a, b in pairs])
    t_x = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
    x = g.make(r_x, t_x)
    z = g.mean_pose([w @ x @ b for w, b in zip(wrists, board_in_cams)])
    return x, z


# --- step 1: both cameras --------------------------------------------------------------------------------

def apply_line_params(base, p):
    """base's line camera moved by the 12 line parameters (LINE_PARAMS)."""
    h = base.copy()
    h.objective = g.perturb(base.objective, [p[0], p[1], p[2], 0.0, p[3], p[4]])
    h.mirror = g.perturb(base.mirror, [0.0, p[5], p[6], p[7], p[8], p[9]])
    h.half_line = base.half_line * math.exp(min(max(p[10], -1.0), 1.0))   # bounded: a wild trial step can't overflow
    h.slit_k1 = base.slit_k1 + p[11]
    return h


def lens_vector(cam):
    return np.array([cam.fx, cam.fy, cam.cx, cam.cy, *cam.dist[:4]])


def lens_from(cam, p):
    return PinholeCamera(cam.width, cam.height, p[0], p[1], p[2], p[3], np.array([p[4], p[5], p[6], p[7], 0.0]))


class Bundle:
    """Step 1's least squares: x = [lens change (8), line camera (12), board pose per view (6 each)]."""

    def __init__(self, views, base, cam, board, use_line=True):
        self.views = [v for v in views if v.board_in_head is not None]
        self.base, self.cam0, self.board = base, cam, board
        self.use_line = use_line
        self.b0 = [v.board_in_head.copy() for v in self.views]
        self.pc_inv = g.inv(base.pose_camera)
        self.n_shared = 20
        # every parameter starts at zero (the lens as a change from cam), so the first trust region is small
        self.lens0 = lens_vector(cam)
        self.x0 = np.zeros(20 + 6 * len(self.views))
        self.prior_sigma = np.array([PRIOR_MM * 1e-3] * 3 + [math.radians(PRIOR_DEG)] * 2 + [PRIOR_MM * 1e-3] * 2
                                    + [math.radians(PRIOR_DEG) * 3] + [math.radians(PRIOR_DEG)] * 2
                                    + [PRIOR_LOG_K, PRIOR_K1])
        self.steps = np.array([1e-3, 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-7, 1e-7] + [1e-7] * 3 + [1e-7] * 2
                              + [1e-7] * 2 + [1e-7] * 3 + [1e-7, 1e-7])
        self.view_steps = np.array([1e-7] * 3 + [1e-7] * 3)
        self.corners = [board.corners(v.detection.ids) if v.detection is not None else None for v in self.views]
        self.line_pts = [board.corners(v.line_obs["ids"]) if (use_line and v.line_obs) else None for v in self.views]

    def unpack(self, x):
        cam = lens_from(self.cam0, self.lens0 + x[:8])
        head = apply_line_params(self.base, x[8:20])
        poses = [b @ g.exp6(x[20 + 6 * i:26 + 6 * i]) for i, b in enumerate(self.b0)]
        return cam, head, poses

    def view_residuals(self, i, cam, head, pose):
        v = self.views[i]
        out = []
        if self.corners[i] is not None:
            p = g.apply(self.pc_inv @ pose, self.corners[i])
            out.append(((cam.project(p) - v.detection.uv) / SIGMA_POSE_PX).ravel())
        if self.line_pts[i] is not None:
            o = v.line_obs
            hh, w = head.slit_project(g.apply(pose, self.line_pts[i]), o["t"])
            width = v.sweep.width
            out.append((hh - o["h"]) * width / 2.0 / SIGMA_SLIT_PX)
            out.append(w / (2.0 * v.sweep.step) / SIGMA_LINES)
        return np.concatenate(out) if out else np.zeros(0)

    def residuals(self, x, split=False):
        cam, head, poses = self.unpack(x)
        blocks = [np.nan_to_num(self.view_residuals(i, cam, head, p), nan=1e4, posinf=1e4, neginf=-1e4)
                  for i, p in enumerate(poses)]
        prior = x[8:20] / self.prior_sigma
        return blocks + [prior] if split else np.concatenate(blocks + [prior])

    def jacobian(self, x):
        """Finite differences, grouped: every view's board pose touches only that view's rows, so
        one nudge moves the same parameter of every view at once (26 evaluations, not 20 + 6V)."""
        r0 = self.residuals(x, split=True)
        sizes = [len(b) for b in r0]
        offs = np.concatenate([[0], np.cumsum(sizes)])
        r0c = np.concatenate(r0)
        n = len(x)
        jac = np.zeros((len(r0c), n))
        for k in range(self.n_shared):
            xk = x.copy()
            xk[k] += self.steps[k]
            jac[:, k] = (self.residuals(xk) - r0c) / self.steps[k]
        for j in range(6):
            xk = x.copy()
            xk[20 + j::6] += self.view_steps[j]
            rk = self.residuals(xk, split=True)
            for i in range(len(self.views)):
                jac[offs[i]:offs[i + 1], 20 + 6 * i + j] = (rk[i] - r0[i]) / self.view_steps[j]
        return jac

    def solve(self, max_nfev=60):
        from scipy.optimize import least_squares
        res = least_squares(self.residuals, self.x0, jac=self.jacobian, method="trf", loss="soft_l1", f_scale=3.0,
                            x_scale="jac", max_nfev=max_nfev)
        self.x0 = res.x
        return res


def covariance(jac, resid, n_prior=0):
    """Parameter covariance from the Jacobian at the solution, scaled by the residuals' spread."""
    m, n = jac.shape
    dof = max(m - n_prior - n, 1)
    s2 = float(resid[: m - n_prior] @ resid[: m - n_prior]) / dof
    jtj = jac.T @ jac
    try:
        return np.linalg.inv(jtj) * max(s2, 1e-12), s2
    except np.linalg.LinAlgError:
        return np.linalg.pinv(jtj) * max(s2, 1e-12), s2


# --- matching the line camera's corners ------------------------------------------------------------------

def coarse_offsets(views, head, texture, board, max_frac=0.35, log=print):
    """Each sweep's shift against its predicted sweep, for the first round of matching.

    The board's corners repeat every square, so one sweep on its own can lock on a square off.
    The model's error shifts every sweep by about the same amount, though, so the maps are added
    up first, and each sweep then takes its best shift within a third of a square of the common
    one."""
    maps, periods = {}, []
    ml = int(max_frac * min(v.sweep.image.shape[0] for v in views))
    mu = int(max_frac * min(v.sweep.width for v in views))
    for v in views:
        s = v.sweep
        pred = linecam.flatten(linecam.render_sweep(head, v.board_in_head, s.t, s.width, texture))
        maps[id(v)] = linecam.coarse_map(linecam.flatten(s.image), pred, ml, mu)
        line, u, _ = linecam.predict_corners(head, g.apply(v.board_in_head, board.corners()), s)
        sc = linecam.corner_scales(board, line, u)
        if np.isfinite(sc).all(axis=1).any():
            periods.append(np.nanmedian(sc, axis=0))
    common = linecam.map_peak(sum(maps.values()) / len(maps))
    period = np.median(periods, axis=0) if periods else np.array([ml, mu])
    window = np.maximum(np.round(period / 3.0), 2).astype(int)
    log("  sweeps sit (%d rows, %d px) off the CAD model's prediction" % common[:2])
    return {id(v): linecam.map_peak(maps[id(v)], common[:2], window) for v in views}


def match_view(v, head, texture, board, coarse=None, search_frac=0.3, min_score=0.75, flip=False):
    """Find the board's corners in a view's sweep with the current model, after shifting the
    prediction by `coarse` (rows, columns, score) if given. Returns the observation dict (or None)
    and the coarse score."""
    s = v.sweep
    obs = linecam.flatten(s.image[:, ::-1] if flip else s.image)
    pts = g.apply(v.board_in_head, board.corners())
    line, u, hcoord = linecam.predict_corners(head, pts, s)
    if np.isfinite(line).sum() < 3:
        return None, 0.0
    pred = linecam.flatten(linecam.render_sweep(head, v.board_in_head, s.t, s.width, texture))
    off, score = (0, 0), 1.0
    if coarse is not None:
        off, score = coarse[:2], coarse[2]
    scale = linecam.corner_scales(board, line, u)
    ok = np.isfinite(scale).all(axis=1)
    half = np.where(ok[:, None], np.maximum(np.round(0.38 * np.nan_to_num(scale)), 3), 3).astype(int)
    search = np.maximum(np.round(search_frac * np.nan_to_num(scale)), 2).astype(int)
    ml, mu, sc = linecam.match_corners(obs, pred, line, u, off, half, search, min_score)
    keep = np.isfinite(ml) & ok
    if keep.sum() < 3:
        return None, score
    ids = np.flatnonzero(keep)
    return dict(ids=ids, line=ml[keep], u=mu[keep], t=s.t_at(ml[keep]), h=slit_coordinate(mu[keep], s.width),
                score=sc[keep], pred_line=line[keep], pred_u=u[keep]), score


def check_slit_order(views, head, texture, board, log=print):
    """True if the sweeps fit the board better with the slit's order reversed (camera.json's
    slit_reversed is wrong)."""
    scores = {False: [], True: []}
    for v in views:
        if v.board_in_head is None:
            continue
        s = v.sweep
        pred = linecam.flatten(linecam.render_sweep(head, v.board_in_head, s.t, s.width, texture))
        for flip in (False, True):
            obs = linecam.flatten(s.image[:, ::-1] if flip else s.image)
            scores[flip].append(linecam.coarse_offset(obs, pred)[2])
    a, b = float(np.median(scores[False])), float(np.median(scores[True]))
    log("  sweep vs model overlap: %.2f as recorded, %.2f with the slit reversed" % (a, b))
    return b > a + 0.1


# --- step 2: hand-eye -------------------------------------------------------------------------------------

def arm_points(board):
    w, h = board.size_m
    xs, ys = np.meshgrid([0.2 * w, 0.5 * w, 0.8 * w], [0.2 * h, 0.5 * h, 0.8 * h])
    return np.stack([xs.ravel(), ys.ravel(), np.zeros(9)], axis=-1)


def hand_eye(views, x0, z0, board, robot=None, joint_offsets=(), log=print):
    """Refine the head on the wrist (x) and the board on the table (z) so that every viewpoint
    puts the board in the same place; optionally also constant offsets of named arm joints.
    Returns x, z, offsets {joint: rad}, per-view (mm, deg)."""
    from scipy.optimize import least_squares
    pts = arm_points(board)
    vs = [v for v in views if v.board_in_head is not None]
    names = list(joint_offsets)

    def wrists(p):
        if not names:
            return [v.wrist for v in vs]
        out = []
        for v in vs:
            q = dict(v.sweep.joints)
            for k, n in enumerate(names):
                q[n] = q[n] + p[12 + k]
            out.append(robot.fk("wrist_roll_link", q))
        return out

    def fun(p):
        x = x0 @ g.exp6(p[:6])
        z = z0 @ g.exp6(p[6:12])
        zp = g.apply(z, pts)
        r = [(g.apply(w @ x @ v.board_in_head, pts) - zp).ravel() / SIGMA_ARM_M for w, v in zip(wrists(p), vs)]
        prior = p[12:] / math.radians(5.0)
        return np.concatenate(r + [prior])

    res = least_squares(fun, np.zeros(12 + len(names)), method="trf", loss="soft_l1", f_scale=2.0, x_scale=1e-3,
                        diff_step=1e-7)
    p = res.x
    x, z = x0 @ g.exp6(p[:6]), z0 @ g.exp6(p[6:12])
    for w, v in zip(wrists(p), vs):
        b = w @ x @ v.board_in_head
        d = g.apply(b, pts) - g.apply(z, pts)
        v.arm_mm = float(np.sqrt((d ** 2).sum(axis=1).mean()) * 1e3)
        v.arm_deg = math.degrees(g.angle_between(b[:3, :3], z[:3, :3]))
    return x, z, {n: float(p[12 + k]) for k, n in enumerate(names)}


# --- step 3: gauge ------------------------------------------------------------------------------------------

def frame_points(frames, size=0.02):
    pts = []
    for f in frames:
        o = f[:3, 3]
        pts += [o, o + size * f[:3, 0], o + size * f[:3, 1], o + size * f[:3, 2]]
    return np.array(pts)


def gauge(head_p, nominal, max_deg=5.0):
    """The rigid move G (head -> head') that brings the pose camera, the objective and the mirror
    closest to their seats in `nominal`, leaving out any part more than max_deg off after the
    first fit. Returns G and the parts used."""
    parts = {"pose camera": (head_p.pose_camera, nominal.pose_camera),
             "objective": (head_p.objective, nominal.objective),
             "mirror": (head_p.mirror, nominal.mirror)}
    use = list(parts)
    for _ in range(2):
        g_inv = g.kabsch(frame_points([parts[k][0] for k in use]), frame_points([parts[k][1] for k in use]))
        off = {k: math.degrees(g.angle_between((g_inv @ a)[:3, :3], b[:3, :3])) for k, (a, b) in parts.items()}
        keep = [k for k in parts if off[k] <= max_deg]
        if len(keep) < 2 or keep == use:
            break
        use = keep
    return g.inv(g_inv), use


def regauge(head_p, x, gauge_t):
    """head' geometry and mount x (head' -> wrist) in the head frame G maps from."""
    gi = g.inv(gauge_t)
    h = head_p.copy()
    h.mount = x @ gauge_t
    h.mirror = gi @ head_p.mirror
    h.objective = gi @ head_p.objective
    h.pose_camera = gi @ head_p.pose_camera
    return h


# --- the whole thing --------------------------------------------------------------------------------------

def calibrate(views, robot, nominal, board, slit_reversed=False, joint_offsets=(), rounds=3, log=print):
    texture = linecam.Texture(board, px_per_mm=10, blur_px=0.6)
    warnings = []
    have = [v for v in views if v.detection is not None]
    if len(have) < 6:
        raise RuntimeError("the pose camera found the board in only %d viewpoints; the calibration needs 6 or "
                           "more (check focus, exposure and that the recorder ran during the scan)" % len(have))

    # step 0: the pose camera's lens, its board poses, and where it sits on the wrist
    cam, rms = posecam.calibrate_lens(board, [v.detection for v in have])
    log("pose camera lens: f = %.1f x %.1f px, centre (%.1f, %.1f), k1 %.3f k2 %.3f; fit %.2f px rms"
        % (cam.fx, cam.fy, cam.cx, cam.cy, cam.dist[0], cam.dist[1], rms))
    for v in have:
        v.board_in_pc, v.pnp_rms = posecam.board_pose(cam, board, v.detection.ids, v.detection.uv)
    x_pc, z = hand_eye_init([v.wrist for v in have], [v.board_in_pc for v in have])
    head0 = nominal.copy()
    head0.pose_camera = g.inv(nominal.mount) @ x_pc
    turn = math.degrees(g.angle_between(head0.pose_camera[:3, :3], nominal.pose_camera[:3, :3]))
    shift = 1e3 * np.linalg.norm(head0.pose_camera[:3, 3] - nominal.pose_camera[:3, 3])
    log("pose camera on the wrist: %.1f deg and %.1f mm from the CAD seat" % (turn, shift))
    if turn > 20:
        warnings.append("The pose camera sits %.0f deg turned from its CAD seat (mounted another way round?). "
                        "The calibration handles it, but check it's what you meant." % turn)
    for v in have:
        v.board_in_head = head0.pose_camera @ v.board_in_pc

    # the slit's order, then the line camera's corners
    if check_slit_order(have, head0, texture, board, log):
        slit_reversed = not slit_reversed
        warnings.append("The sweeps match the board only with the slit's pixel order reversed: set "
                        "slit_reversed to %s in line_camera's settings (the calibration used it)."
                        % str(slit_reversed).lower())
        for v in views:
            v.sweep.image = v.sweep.image[:, ::-1]
    head = head0
    search = 0.4
    bundle = None
    coarse = coarse_offsets(have, head0, texture, board, log=log)
    x_prev = None
    for rnd in range(rounds):
        n_obs = 0
        for v in have:
            v.line_obs, score = match_view(v, head, texture, board, coarse=coarse[id(v)] if rnd == 0 else None,
                                           search_frac=search)
            n_obs += 0 if v.line_obs is None else len(v.line_obs["ids"])
        log("round %d: %d line-camera corners in %d sweeps"
            % (rnd + 1, n_obs, sum(v.line_obs is not None for v in have)))
        if n_obs < 30:
            raise RuntimeError("found only %d board corners in the line camera's sweeps; is the board lit, in "
                               "focus, and under the scan line?" % n_obs)
        bundle = Bundle(have, head0, cam, board)
        if rnd:
            bundle.x0[8:20] = x_prev[8:20]   # cam already carries the lens
        res = bundle.solve()
        x_prev = bundle.x0
        cam, head, poses = bundle.unpack(bundle.x0)
        for v, p in zip(bundle.views, poses):
            v.board_in_head = p
        # leave out corners more than 4 sigma off, then match again with a tighter search
        blocks = bundle.residuals(bundle.x0, split=True)
        for i, v in enumerate(bundle.views):
            if v.line_obs is None:
                continue
            k = len(v.line_obs["ids"])
            r = blocks[i][-2 * k:].reshape(2, k)
            bad = np.hypot(r[0], r[1]) > 4.0
            if bad.any():
                v.line_obs = {key: val[~bad] for key, val in v.line_obs.items()}
        search = 0.2
        log("  fit: %d residuals, cost %.1f after %d evaluations, %s" % (len(res.fun), res.cost, res.nfev, res.message))
    # final fit on the cleaned corners
    bundle = Bundle(have, head0, cam, board)
    bundle.x0[8:20] = x_prev[8:20]   # cam already carries the lens
    bundle.solve()
    cam, head_p, poses = bundle.unpack(bundle.x0)
    for v, p in zip(bundle.views, poses):
        v.board_in_head = p
    jac = bundle.jacobian(bundle.x0)
    resid = bundle.residuals(bundle.x0)
    cov, s2 = covariance(jac, resid, n_prior=12)
    sig = np.sqrt(np.maximum(np.diag(cov)[:20], 0))

    # step 2: the head on the wrist
    x0 = x_pc @ g.inv(head_p.pose_camera)      # head' on the wrist, from step 0's hand-eye
    x, z, offsets = hand_eye(have, x0, z, board, robot, joint_offsets, log)

    # step 3: back into the CAD's head frame
    gt, used = gauge(head_p, nominal)
    final = regauge(head_p, x, gt)

    stats = residual_stats(bundle, cam, head_p)
    stats["s2"] = s2
    stats["lens_rms_px"] = rms
    stats["gauge_parts"] = used
    stats["arm_mm"] = [v.arm_mm for v in have]
    for v in have:
        if v.arm_mm > 3 * max(np.median(stats["arm_mm"]), 0.5):
            warnings.append("Viewpoint %s doesn't fit the others: the board lands %.1f mm off from there (the arm "
                            "moved, slipped, or a servo misread?)" % (v.name, v.arm_mm))
    sigmas = {n: float(s) for n, s in zip(LENS_PARAMS + LINE_PARAMS, sig)}
    return Result(final, cam, z, nominal, views, stats, sigmas, warnings, slit_reversed, offsets)


def residual_stats(bundle, cam, head):
    blocks = bundle.residuals(bundle.x0, split=True)
    pose, along, across = [], [], []
    per_view = []
    for i, v in enumerate(bundle.views):
        b = blocks[i]
        n_pc = 2 * len(v.detection.ids) if v.detection is not None else 0
        pc = b[:n_pc].reshape(-1, 2) * SIGMA_POSE_PX
        k = len(v.line_obs["ids"]) if v.line_obs else 0
        lr = b[n_pc:n_pc + 2 * k].reshape(2, k) if k else np.zeros((2, 0))
        pose.append(np.hypot(pc[:, 0], pc[:, 1]))
        along.append(lr[0] * SIGMA_SLIT_PX)
        across.append(lr[1] * SIGMA_LINES)
        per_view.append(dict(name=v.name, pose_corners=n_pc // 2, line_corners=k,
                             pose_rms_px=float(np.sqrt(np.mean(pc ** 2) * 2)) if n_pc else None,
                             slit_rms_px=float(np.sqrt(np.mean(lr[0] ** 2))) * SIGMA_SLIT_PX if k else None,
                             lines_rms=float(np.sqrt(np.mean(lr[1] ** 2))) * SIGMA_LINES if k else None))

    def rms(a):
        a = np.concatenate(a) if a else np.zeros(0)
        return float(np.sqrt(np.mean(a ** 2))) if a.size else float("nan")

    return dict(pose_rms_px=rms(pose), slit_rms_px=rms(along), lines_rms=rms(across),
                n_pose=int(sum(len(a) for a in pose)), n_line=int(sum(len(a) for a in along)), per_view=per_view)


def run(scan, pose, board_path=None, out=None, nm=None, joint_offsets=(), log=print):
    """Calibrate from a scan folder and a pose-camera recording; write the results to out."""
    import os
    from . import board as board_mod, output, repo
    scan = os.path.abspath(os.path.expanduser(scan))
    pose = os.path.abspath(os.path.expanduser(pose))
    board = board_mod.Board.load(board_path) if board_path else board_mod.Board()
    sweeps, camera = linecam.load_scan(scan, nm=nm, log=log)
    with open(os.path.join(scan, "robot.urdf")) as f:
        urdf = f.read()
    robot = repo.robot(urdf)
    # start from the CAD numbers even if the scan ran with an earlier head calibration loaded
    nominal = HeadGeometry.from_urdf(repo.robot(repo.build_urdf()))
    if HeadGeometry.from_urdf(robot).origin_strings() != nominal.origin_strings():
        log("the scan's URDF carried an earlier head calibration; starting again from the CAD numbers")
    _, stills = posecam.load_stills(pose)
    views = gather(sweeps, robot, stills, board, log)
    result = calibrate(views, robot, nominal, board, slit_reversed=bool(camera.get("slit_reversed")),
                       joint_offsets=joint_offsets, log=log)
    if out:
        board_text = "%d x %d squares of %.3f mm" % (board.squares_x, board.squares_y, board.square_mm)
        output.write(result, out, urdf, dict(scan=scan, pose=pose, board=board.to_json(), board_text=board_text),
                     log)
    return result
