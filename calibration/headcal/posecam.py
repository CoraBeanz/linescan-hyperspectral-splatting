"""The pose camera's side: its stills, the board corners found in them, and its lens.

Stills come from `python -m headcal record` on the Jetson (record.py): a folder with
camera.json, frames.csv (index, stamp_ns, file, exposure_us, gain) and one grey image a file
(.npy, the sensor's 2 x 2 colour cells averaged, or any image OpenCV reads).
"""

from __future__ import annotations

import csv
import json
import os
from dataclasses import dataclass

import numpy as np

from .model import PinholeCamera

FORMAT = "headcal pose frames v1"
MIN_CORNERS = 12


@dataclass
class Still:
    index: int
    stamp_ns: int
    path: str
    exposure_us: float = 0.0
    gain: float = 1.0


@dataclass
class Detection:
    still: Still
    ids: np.ndarray        # (N,) board corner ids
    uv: np.ndarray         # (N, 2) pixels
    size: tuple            # (width, height)


def load_stills(folder):
    folder = os.path.abspath(os.path.expanduser(folder))
    info_path = os.path.join(folder, "camera.json")
    if not os.path.exists(info_path):
        raise FileNotFoundError("%s has no camera.json: is it a headcal record folder?" % folder)
    with open(info_path) as f:
        info = json.load(f)
    if info.get("format") != FORMAT:
        raise ValueError("%s/camera.json isn't a headcal pose-camera recording" % folder)
    out = []
    with open(os.path.join(folder, "frames.csv"), newline="") as f:
        for row in csv.DictReader(f):
            out.append(Still(int(row["index"]), int(row["stamp_ns"]), os.path.join(folder, row["file"]),
                             float(row.get("exposure_us") or 0), float(row.get("gain") or 1)))
    return info, out


def load_grey(path):
    """A still as a float32 grey image."""
    if path.endswith(".npy"):
        a = np.load(path).astype(np.float32)
    else:
        import cv2
        a = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if a is None:
            raise FileNotFoundError(path)
        if a.ndim == 3:
            a = a.mean(axis=2)
        a = a.astype(np.float32)
    if a.ndim == 3:  # several frames: their mean
        a = a.mean(axis=0)
    return a


def to_uint8(img):
    """Stretched to 8 bits between the 0.5 and 99.5 percentiles, for the tag detector."""
    lo, hi = np.percentile(img, [0.5, 99.5])
    return np.clip((img - lo) * (255.0 / max(hi - lo, 1e-6)), 0, 255).astype(np.uint8)


def detect(board, img, detector=None):
    """(ids, uv) of the board's inner corners found in a grey image, ([], []) when too few."""
    import cv2
    det = detector or cv2.aruco.CharucoDetector(board.cv_board())
    corners, ids, _, _ = det.detectBoard(to_uint8(img))
    if ids is None or len(ids) < MIN_CORNERS:
        return np.zeros(0, int), np.zeros((0, 2))
    return ids.ravel().astype(int), corners.reshape(-1, 2).astype(np.float64)


def detect_all(board, stills, log=print):
    import cv2
    det = cv2.aruco.CharucoDetector(board.cv_board())
    out = []
    for s in stills:
        img = load_grey(s.path)
        ids, uv = detect(board, img, det)
        if len(ids):
            out.append(Detection(s, ids, uv, (img.shape[1], img.shape[0])))
        else:
            log("  pose camera still %d: board not found" % s.index)
    return out


def calibrate_lens(board, detections, flags=None):
    """The pose camera's lens from every detection (OpenCV calibrateCamera): (camera, rms px).
    The third radial term is held at 0: a 62 degree lens doesn't need it, and from a dozen
    views of a flat board it only trades off against the other two."""
    import cv2
    if len(detections) < 4:
        raise RuntimeError("the pose camera found the board in only %d stills; the lens needs at least 4"
                           % len(detections))
    obj = [board.corners(d.ids).astype(np.float32) for d in detections]
    img = [d.uv.astype(np.float32) for d in detections]
    w, h = detections[0].size
    f0 = 0.85 * w   # Pi camera v2: 62 degrees across
    k0 = np.array([[f0, 0, (w - 1) / 2], [0, f0, (h - 1) / 2], [0, 0, 1]])
    fl = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3 if flags is None else flags
    rms, k, dist, _, _ = cv2.calibrateCamera(obj, img, (w, h), k0, np.zeros(5), flags=fl)
    d = np.zeros(5)
    d[:min(5, dist.size)] = dist.ravel()[:5]
    return PinholeCamera(w, h, k[0, 0], k[1, 1], k[0, 2], k[1, 2], d), float(rms)


def board_pose(cam, board, ids, uv):
    """board -> camera (4x4) from one detection, and its reprojection rms (px)."""
    import cv2
    obj = board.corners(ids)
    ok, rvec, tvec = cv2.solvePnP(obj, uv, cam.matrix, np.asarray(cam.dist), flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, np.inf
    r, _ = cv2.Rodrigues(rvec)
    t = np.eye(4)
    t[:3, :3] = r
    t[:3, 3] = tvec.ravel()
    p = obj @ r.T + tvec.ravel()
    err = cam.project(p) - uv
    return t, float(np.sqrt(np.mean(np.sum(err ** 2, axis=1))))
