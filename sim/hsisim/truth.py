"""Read back what a simulated scan was made from (the session's truth/ folder).

    t = ScanTruth("sim_out")
    t.reflectance(line, nm)         # [h, nm]: what each slit position saw, line by line
    t.lines                         # lines_true.csv as a dict of numpy columns
    t.pure(line, margin)            # slit positions that saw one material only
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np


def read_lines_csv(path):
    """lines.csv (or lines_true.csv) as {column: numpy array}."""
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    head, body = rows[0], rows[1:]
    out = {}
    for k, name in enumerate(head):
        col = [r[k] for r in body]
        out[name] = np.array(col, dtype=np.int64 if name in ("viewpoint", "sweep_id", "index", "stamp_ns",
                                                              "hold_until_ns", "settled") else float)
    return out


def pose_matrix(cols, prefix, i):
    """4 x 4 pose from lines.csv columns <prefix>_x .. <prefix>_qw of row i."""
    x, y, z, w = (cols[f"{prefix}_q{k}"][i] for k in "xyzw")
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [cols[f"{prefix}_{k}"][i] for k in "xyz"]
    return T


class ScanTruth:
    def __init__(self, session):
        self.dir = Path(session) / "truth"
        self.info = json.loads((self.dir / "truth.json").read_text())
        self.materials = self.info["materials"]
        self.weights = np.load(self.dir / "weights.npy", mmap_mode="r")    # [line, h, material]
        self.depth = np.load(self.dir / "depth.npy", mmap_mode="r")
        self.h = np.load(self.dir / "h.npy").astype(float)
        self.nm = np.load(self.dir / "nm.npy").astype(float)
        self.spectra = np.load(self.dir / "materials.npy").astype(float)  # [material, nm]
        self.lines = read_lines_csv(self.dir / "lines_true.csv")
        maps = np.load(self.dir / "pixel_maps.npz")
        self.nm_map, self.h_map = maps["nm"], maps["h"]

    def __len__(self):
        return len(self.lines["index"])

    def line_number(self, sweep_id, index):
        hit = np.flatnonzero((self.lines["sweep_id"] == sweep_id) & (self.lines["index"] == index))
        if not hit.size:
            raise KeyError(f"no line {index} in sweep {sweep_id}")
        return int(hit[0])

    def material_spectra(self, nm=None):
        if nm is None:
            return self.spectra
        return np.stack([np.interp(nm, self.nm, s) for s in self.spectra])

    def reflectance(self, line, nm=None):
        """[h, nm] reflectance (times shading) along the slit for one line."""
        return np.asarray(self.weights[line], float) @ self.material_spectra(nm)

    def pure(self, line, margin=0.0, level=0.999):
        """[h] True where the slit saw a single material, everywhere within +-margin (in h)."""
        w = np.asarray(self.weights[line], float)
        total = w.sum(1)
        dom = w.argmax(1)
        ok = (total > 0) & (w.max(1) >= level * total)
        n = int(np.ceil(margin / (self.h[1] - self.h[0])))
        out = ok.copy()
        for s in range(1, n + 1):
            out[s:] &= ok[:-s] & (dom[:-s] == dom[s:])
            out[:-s] &= ok[s:] & (dom[s:] == dom[:-s])
        if n:
            out[:n] = False
            out[-n:] = False
        return out

    def dominant(self, line):
        return np.asarray(self.weights[line], float).argmax(1)

    def pose(self, line, which="cam"):
        return pose_matrix(self.lines, which, line)
