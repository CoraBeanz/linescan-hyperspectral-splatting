"""The line camera's side: a sweep of the board as an image, where the model says the board's
corners should be in it, and where they really are.

A sweep is one picture of the board built a line at a time: row i is scan line i (one mirror
angle), column j is slit bin j. headcal sums each line's spectrum into one brightness, so a
sweep of the tag board looks like a slightly warped photo of it. It isn't one camera's photo:
every row has its own camera (the objective reflected in the mirror at that angle), which is
why the corners are found by matching rather than with a checkerboard detector:

  1. render the sweep the model predicts (the board's known pattern seen through every line's
     camera, board pose from the pose camera), and slide it over the real sweep: the best
     overlap gives the offset a wrong mirror home or slit centre would cause. The tags make
     the pattern unique, so this can't lock on one square over
  2. around each corner the model predicts, match a small piece of the rendered sweep to the
     real one, to a fraction of a pixel: that corner's line (so its mirror angle) and slit
     position
  3. after the solver has moved the model, render and match again, closer
"""

from __future__ import annotations

import csv
import json
import os
from dataclasses import dataclass, field

import numpy as np

from . import board as board_mod
from .model import slit_coordinate, slit_pixels

SLIT_HALF_WIDTH = 0.0014   # the 50 um slit seen from the 15.6 mm objective: +-1.4 mrad across the line


@dataclass
class Sweep:
    sweep_id: int
    viewpoint: str
    t: np.ndarray               # reported mirror angle of each row (rad)
    image: np.ndarray           # (lines, slit bins) brightness, NaN where there is none
    joints: dict                # the arm's joints, mean over the sweep (rad)
    joint_spread: float         # largest change of any joint during the sweep (rad)
    start_ns: int
    end_ns: int
    obs: dict = field(default_factory=dict)   # matched corners: ids, line, u, t, h, score

    @property
    def width(self):
        return self.image.shape[1]

    def t_at(self, line):
        """Mirror angle at a fractional row."""
        rows = np.arange(len(self.t))
        return np.interp(line, rows, self.t, left=np.nan, right=np.nan)

    @property
    def step(self):
        return float(np.median(np.abs(np.diff(self.t))))


def _read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def load_scan(path, nm=None, arm_joints=None, log=print):
    """The sweeps of a scan folder (scan_sweep's lines.csv with line_camera's frames/ and binned/).

    nm: (lo, hi) wavelengths to add up for the brightness, default all. Rows are turned so slit
    bin 0 is at the head's -X end (camera.json's slit_reversed), as everywhere downstream."""
    path = os.path.abspath(os.path.expanduser(path))
    for need in ("lines.csv", "robot.urdf", os.path.join("frames", "camera.json")):
        if not os.path.exists(os.path.join(path, need)):
            raise FileNotFoundError("%s has no %s: is it a scan folder with line_camera's frames?" % (path, need))
    with open(os.path.join(path, "frames", "camera.json")) as f:
        camera = json.load(f)
    reversed_ = bool(camera.get("slit_reversed"))
    status = {}
    if os.path.exists(os.path.join(path, "frames", "frames.csv")):
        for row in _read_csv(os.path.join(path, "frames", "frames.csv")):
            status[(int(row["sweep_id"]), int(row["index"]))] = row["status"]
    band = None
    grid = os.path.join(path, "binned", "binning.npz")
    if nm and os.path.exists(grid):
        edges = np.load(grid)["nm_edges"]
        c = 0.5 * (edges[:-1] + edges[1:])
        band = (c >= nm[0]) & (c <= nm[1])
    rows = _read_csv(os.path.join(path, "lines.csv"))
    joints = arm_joints or [k for k in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
                            if rows and k in rows[0]]
    by_sweep = {}
    for r in rows:
        by_sweep.setdefault(int(r["sweep_id"]), []).append(r)
    sweeps = []
    for sid in sorted(by_sweep):
        rs = sorted(by_sweep[sid], key=lambda r: int(r["index"]))
        f = os.path.join(path, "binned", "sweep_%03d.npy" % sid)
        if not os.path.exists(f):
            log("  sweep %d: no binned lines (binned/sweep_%03d.npy), left out" % (sid, sid))
            continue
        arr = np.load(f).astype(np.float64)
        if band is not None:
            arr = arr[..., band]
        with np.errstate(invalid="ignore"):
            img = np.where(np.isnan(arr).all(axis=-1), np.nan, np.nansum(arr, axis=-1))
        idx = np.array([int(r["index"]) for r in rs])
        ang = np.array([float(r["mirror_angle"]) for r in rs])
        n = img.shape[0]
        fit = np.polyfit(idx, ang, 1) if len(idx) > 1 else (0.0, ang[0])
        t = np.polyval(fit, np.arange(n))
        known = idx < n
        t[idx[known]] = ang[known]
        for i in range(n):
            if status and status.get((sid, i), "missing") != "ok":
                img[i] = np.nan
        if reversed_:
            img = img[:, ::-1]
        q = np.array([[float(r[j]) for j in joints] for r in rs])
        sweeps.append(Sweep(sid, rs[0].get("viewpoint", str(sid)), t, img, dict(zip(joints, q.mean(axis=0))),
                            float(np.abs(q - q.mean(axis=0)).max()) if len(q) else 0.0,
                            int(rs[0]["stamp_ns"]), int(rs[-1].get("hold_until_ns") or rs[-1]["stamp_ns"])))
    return sweeps, camera


# --- rendering ---------------------------------------------------------------------------------------

def board_points(head, board_in_head, t, h, w=0.0):
    """Where the rays of slit coordinates h (N,) at mirror angles t (L,) (across offset w) meet the
    board: board-frame x, y (L, N) and the distance along each ray."""
    t = np.asarray(t, float)
    h = np.asarray(h, float)
    hu = h.copy()
    for _ in range(4):
        hu = hu - (hu + head.slit_k1 * hu ** 3 - h) / (1.0 + 3.0 * head.slit_k1 * hu ** 2)
    r, c = head.line_cameras(t)                                   # (L,3,3), (L,3)
    w = np.broadcast_to(np.asarray(w, float), h.shape)
    d_cam = np.stack([hu * head.k, w, np.ones_like(hu)], axis=-1)   # (N,3)
    d = np.einsum("lij,nj->lni", r, d_cam)                        # (L,N,3) head frame
    rb, tb = board_in_head[:3, :3], board_in_head[:3, 3]
    cb = (c - tb) @ rb                                            # (L,3) board frame
    db = d @ rb                                                   # (L,N,3)
    with np.errstate(divide="ignore", invalid="ignore"):
        s = -cb[:, None, 2] / db[..., 2]
    x = cb[:, None, 0] + s * db[..., 0]
    y = cb[:, None, 1] + s * db[..., 1]
    return x, y, s * np.linalg.norm(db, axis=-1)


class Texture:
    """The board's pattern for rendering, built once."""

    def __init__(self, board, px_per_mm=12, blur_px=0.6):
        import cv2
        self.board = board
        img, self.to_px = board.texture(px_per_mm)
        self.image = cv2.GaussianBlur(img, (0, 0), blur_px) if blur_px else img

    def sample(self, x, y):
        import cv2
        px, py = self.to_px(np.asarray(x), np.asarray(y))
        shape = np.shape(px)
        px = np.nan_to_num(px.reshape(shape[0], -1), nan=-1e5).astype(np.float32)
        py = np.nan_to_num(py.reshape(shape[0], -1), nan=-1e5).astype(np.float32)
        out = cv2.remap(self.image, px, py, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                        borderValue=board_mod.TABLE)
        return out.reshape(shape)


def render_sweep(head, board_in_head, t, width, texture, sub_u=2, sub_w=2, slit_half_width=SLIT_HALF_WIDTH):
    """The sweep the model predicts: the board's reflectance at every (line, slit bin), each value
    the mean over the bin's width and the slit's width."""
    u = (np.arange(width)[:, None] + (np.arange(sub_u) + 0.5)[None, :] / sub_u).ravel()
    h = slit_coordinate(u, width)
    out = 0.0
    for wv in slit_half_width * ((np.arange(sub_w) + 0.5) / sub_w * 2.0 - 1.0):
        x, y, s = board_points(head, board_in_head, t, h, wv)
        v = texture.sample(np.where(s > 0, x, np.nan), np.where(s > 0, y, np.nan))
        out = out + v.reshape(len(t), width, sub_u).mean(axis=-1)
    return out / sub_w


# --- where the corners should be -----------------------------------------------------------------------

def predict_corners(head, points_head, sweep):
    """Fractional row, slit pixel u and slit coordinate h at which each head-frame point (K,3)
    crosses the scan line during the sweep; NaN rows for points it never crosses."""
    pts = np.asarray(points_head, float)
    t_lo, t_hi = float(np.min(sweep.t)), float(np.max(sweep.t))
    t = np.full(len(pts), 0.5 * (t_lo + t_hi))
    eps = 1e-5
    for _ in range(8):  # Newton on w(t) = 0
        _, w0 = head.slit_project(pts, t)
        _, w1 = head.slit_project(pts, t + eps)
        dw = (w1 - w0) / eps
        step = np.where(np.abs(dw) > 1e-9, w0 / dw, 0.0)
        t = t - np.clip(step, -0.2, 0.2)
    h, w = head.slit_project(pts, t)
    rows = np.arange(len(sweep.t))
    order = np.argsort(sweep.t)
    line = np.interp(t, sweep.t[order], rows[order], left=np.nan, right=np.nan)
    ok = (np.abs(w) < 1e-6) & (np.abs(h) < 1.0) & np.isfinite(line)
    line = np.where(ok, line, np.nan)
    return line, np.where(ok, slit_pixels(h, sweep.width), np.nan), np.where(ok, h, np.nan)


# --- matching -------------------------------------------------------------------------------------------

def flatten(img, sigma_frac=0.15):
    """Brightness with slow changes divided out (lamp fall-off, vignetting) and gaps filled, for
    matching. float32."""
    import cv2
    a = np.asarray(img, np.float64).copy()
    bad = ~np.isfinite(a)
    fill = np.nanmedian(a) if (~bad).any() else 0.0
    a[bad] = fill
    sl, su = max(sigma_frac * a.shape[0], 2.0), max(sigma_frac * a.shape[1], 2.0)
    bg = cv2.GaussianBlur(a, (0, 0), sigmaX=su, sigmaY=sl)
    out = a / np.maximum(bg, 1e-9 * max(abs(fill), 1.0))
    out[bad] = 1.0
    return out.astype(np.float32)


def coarse_map(obs, pred, ml, mu):
    """How well the predicted sweep overlaps the real one shifted by (-ml..ml rows, -mu..mu
    columns): a (2 ml + 1, 2 mu + 1) map of normalised correlation."""
    import cv2
    pad = cv2.copyMakeBorder(obs, ml, ml, mu, mu, cv2.BORDER_CONSTANT, value=float(np.median(obs)))
    return cv2.matchTemplate(pad, pred.astype(np.float32), cv2.TM_CCOEFF_NORMED)


def map_peak(res, centre=None, window=None):
    """(rows, columns, score) of a coarse_map's best shift, optionally only within `window`
    (rows, columns) of the shift `centre`."""
    ml, mu = res.shape[0] // 2, res.shape[1] // 2
    r = res
    if centre is not None:
        r = np.full_like(res, -2.0)
        i0, j0 = int(centre[0]) + ml, int(centre[1]) + mu
        a, b = max(i0 - int(window[0]), 0), max(j0 - int(window[1]), 0)
        r[a:i0 + int(window[0]) + 1, b:j0 + int(window[1]) + 1] = res[a:i0 + int(window[0]) + 1,
                                                                      b:j0 + int(window[1]) + 1]
    i, j = np.unravel_index(int(np.argmax(r)), r.shape)
    return int(i) - ml, int(j) - mu, float(r[i, j])


def coarse_offset(obs, pred, max_frac=0.35):
    """(rows, columns, score): the shift of the predicted sweep that best overlaps the real one."""
    L, W = obs.shape
    return map_peak(coarse_map(obs, pred, int(max_frac * L), int(max_frac * W)))


def _subpixel(r, i, j):
    def peak(a, b, c):
        d = a - 2.0 * b + c
        return 0.0 if d >= 0 else 0.5 * (a - c) / d
    di = peak(r[i - 1, j], r[i, j], r[i + 1, j]) if 0 < i < r.shape[0] - 1 else 0.0
    dj = peak(r[i, j - 1], r[i, j], r[i, j + 1]) if 0 < j < r.shape[1] - 1 else 0.0
    return float(np.clip(di, -0.5, 0.5)), float(np.clip(dj, -0.5, 0.5))


def match_corners(obs, pred, line, u, offset=(0, 0), half=None, search=None, min_score=0.75):
    """Each predicted corner (fractional row `line`, slit pixel `u`) matched in the real sweep:
    a piece of the predicted sweep around it, slid over the real one. Returns (line, u, score)
    arrays; NaN where the match failed or ran off the sweep.

    half: (rows, columns) half-size of the piece per corner (K,2); search: how far to look,
    (K,2) or one pair."""
    import cv2
    L, W = obs.shape
    K = len(line)
    out_l = np.full(K, np.nan)
    out_u = np.full(K, np.nan)
    score = np.zeros(K)
    half = np.broadcast_to(np.asarray(half, int), (K, 2))
    search = np.broadcast_to(np.asarray(search, int), (K, 2))
    for k in range(K):
        if not (np.isfinite(line[k]) and np.isfinite(u[k])):
            continue
        cl, cu = line[k], u[k] - 0.5                      # array coordinates of the corner
        il, iu = int(round(cl)), int(round(cu))
        hl, hu = half[k]
        sl, su = search[k]
        ol, ou = il + int(offset[0]), iu + int(offset[1])
        if (il - hl < 0 or il + hl + 1 > L or iu - hu < 0 or iu + hu + 1 > W
                or ol - hl - sl < 0 or ol + hl + sl + 1 > L or ou - hu - su < 0 or ou + hu + su + 1 > W):
            continue
        tpl = pred[il - hl:il + hl + 1, iu - hu:iu + hu + 1]
        if float(tpl.std()) < 0.05 * float(abs(tpl.mean()) + 1e-9):
            continue
        reg = obs[ol - hl - sl:ol + hl + sl + 1, ou - hu - su:ou + hu + su + 1]
        res = cv2.matchTemplate(reg, tpl, cv2.TM_CCOEFF_NORMED)
        _, best, _, (pj, pi) = cv2.minMaxLoc(res)
        if pi in (0, res.shape[0] - 1) or pj in (0, res.shape[1] - 1) or best < min_score:
            continue
        di, dj = _subpixel(res, pi, pj)
        out_l[k] = cl + offset[0] + (pi - sl) + di
        out_u[k] = cu + 0.5 + offset[1] + (pj - su) + dj
        score[k] = best
    return out_l, out_u, score


def corner_scales(board, line, u):
    """Rows and columns from each corner to its neighbours (K,2), for sizing the match pieces."""
    nx = board.squares_x - 1
    K = len(line)
    ids = np.arange(K)
    out = np.full((K, 2), np.nan)
    for k in ids:
        dl, du = [], []
        for nb in (k - 1, k + 1, k - nx, k + nx):
            if 0 <= nb < K and (nb // nx == k // nx or nb % nx == k % nx) and np.isfinite(line[nb]) \
                    and np.isfinite(line[k]):
                dl.append(abs(line[nb] - line[k]))
                du.append(abs(u[nb] - u[k]))
        if dl:
            out[k] = (max(dl), max(du))
    return out
