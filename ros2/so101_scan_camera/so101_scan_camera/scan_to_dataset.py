"""scan_to_dataset: turn a scan folder into the line-splat trainer's dataset.

    ros2 run so101_scan_camera scan_to_dataset ~/so101_scan/scans/ring_20261007-031500 ~/datasets/ring
    python3 splat/tools/scan_to_dataset.py <scan folder> <dataset folder>     # the same, without ROS
    splat_train ~/datasets/ring ~/runs/ring

It reads what scan_sweep (lines.csv, scan.json, robot.urdf) and line_camera (frames/, binned/,
reference/) wrote into the scan folder, and writes the dataset splat/README.md describes:
dataset.json, lines.npy, line_sweep.npy, line_mirror_angle.npy and sweep_head_pose.npy.

  values      reflectance against the white reference by default, band by band:
                  R x sum(line - dark) / (exposure x gain)
                    / sum(white - its dark) / (its exposure x gain)
              where R is the white's reflectance (its meta.json's, else 0.98 for PTFE),
              with the sums over each band's binned cells weighted by their pixel counts (dim
              pixels count for less, as in hsical). Without a white: radiance, relative to the
              calibration's lamp (counts per second over the calibration's response); or ask
              with --values radiance or counts (dark-subtracted counts per second at gain 1).
              A dark taken at the line's exposure and gain is subtracted, else the black level.
  bands       --nm-step wide, centred from --nm-min to --nm-max (10 nm, 500 to 950 nm: the 46
              bands of the synthetic dataset the trainer is tuned on), merged from the cells.
  pixels      one per slit bin (256 by default), pixel 0 at the slit's s_top end; camera.json's
              slit_reversed flips them so pixel 0 is at the head's -X end, as the trainer wants.
  poses       each sweep's head pose is the mean of its lines' (the arm holds still while the
              mirror sweeps), in base_link moved down to the table top (z = 0 there, where the
              trainer starts its Gaussians; --ground-z moves it), and each line keeps its mirror
              angle. The head model (the objective and the mirror) comes from the scan's
              robot.urdf, checked against the line-camera poses lines.csv recorded. --urdf gives
              another one, such as the robot.urdf of a later head calibration (calibration/headcal):
              the head poses are then worked out again from the joint readings in lines.csv.
  intrinsics  f from the scan line's length at the scene distance (scan.json's, or --urdf's),
              blur from the slit and optics numbers splat uses.

Lines without a frame, or with more than --max-missing of their values missing, are left out;
the rest of the missing values (a saturated pixel, a band outside the white's light) are
filled in from their neighbours along the spectrum, and dataset.json's metadata says how many.
References are looked for in --references, the scan folder's reference/, and the folders
scan.json names (camera.references: where line_camera kept the darks and whites taken outside a
recording; calibration_session). Raw frames are binned here when the scan has no binned lines
(it was taken without a calibration), with --calibration.
"""

import argparse
import csv
import json
import math
import os
import sys
from dataclasses import dataclass

import numpy as np

from so101_scan_camera import linesplat as ls
from so101_scan_camera.sources import BLACK_LEVEL

TABLE_Z = -0.0024          # the table top in base_link: the SO-101 base's underside


class ConvertError(RuntimeError):
    pass


def log(text):
    print(text, flush=True)


# --- reading the scan --------------------------------------------------------------------------

def read_json(path):
    with open(path) as f:
        return json.load(f)


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


@dataclass
class Grid:
    """The cells lines are binned on (binned/binning.npz)."""
    slit_edges: np.ndarray
    nm_edges: np.ndarray
    pixels: np.ndarray
    response: np.ndarray   # mean response per cell, or None

    @classmethod
    def load(cls, path):
        z = np.load(path)
        r = z["response"] if "response" in z and z["response"].size else None
        return cls(z["slit_edges"], z["nm_edges"], z["pixels"].astype(np.int64), r)

    @classmethod
    def of(cls, binner):
        return cls(binner.slit_edges, binner.nm_edges, binner.pixels.astype(np.int64), binner.response)

    def same(self, other):
        return (self.slit_edges.shape == other.slit_edges.shape and self.nm_edges.shape == other.nm_edges.shape
                and np.allclose(self.slit_edges, other.slit_edges, atol=1e-6)
                and np.allclose(self.nm_edges, other.nm_edges, atol=1e-6)
                and np.array_equal(self.pixels, other.pixels))

    @property
    def nm_centres(self):
        return 0.5 * (self.nm_edges[:-1] + self.nm_edges[1:])


@dataclass
class Reference:
    name: str
    kind: str
    exposure_us: float
    gain: float
    folder: str
    root: str
    reflectance: float = None     # a white's, when its meta.json says
    binned: np.ndarray = None


class Scan:
    def __init__(self, path, calibration=None, slit_bins=256):
        self.path = os.path.abspath(os.path.expanduser(path))
        p = lambda *a: os.path.join(self.path, *a)   # noqa: E731
        for need in ("lines.csv", "scan.json", "robot.urdf"):
            if not os.path.exists(p(need)):
                raise ConvertError("%s has no %s: is it a scan_sweep folder?" % (self.path, need))
        if not os.path.exists(p("frames", "camera.json")):
            raise ConvertError("%s has no frames/camera.json: line_camera wasn't recording during this scan"
                               % self.path)
        self.info = read_json(p("scan.json"))
        self.camera = read_json(p("frames", "camera.json"))
        self.lines = read_csv(p("lines.csv"))
        self.frames = {}
        if os.path.exists(p("frames", "frames.csv")):
            for row in read_csv(p("frames", "frames.csv")):
                self.frames[(int(row["sweep_id"]), int(row["index"]))] = row
        with open(p("robot.urdf")) as f:
            self.urdf = f.read()
        self.binner = None
        self.grid = Grid.load(p("binned", "binning.npz")) if os.path.exists(p("binned", "binning.npz")) else None
        self.calibration_path = calibration or (self.camera.get("calibration") or {}).get("path")
        if self.grid is None:
            # taken without a calibration: bin the raw frames here
            self.binner = self.make_binner(slit_bins=slit_bins)
            self.grid = Grid.of(self.binner)
        self._sweeps = {}

    def make_binner(self, slit_bins=None, grid=None):
        from so101_scan_camera.binning import LineBinner, load_calibration
        if not self.calibration_path:
            raise ConvertError("the scan's lines aren't binned and no calibration is known: give --calibration")
        if not os.path.exists(os.path.join(self.calibration_path, "maps.npz")):
            raise ConvertError("no calibration at %s (give --calibration)" % self.calibration_path)
        maps = load_calibration(self.calibration_path)
        want = (self.camera.get("calibration") or {}).get("maps_sha256")
        if want and maps.sha256 != want:
            log("  ! %s isn't the calibration the scan was binned with (its maps.npz differs)" % maps.path)
        if grid is None:
            return LineBinner(maps, slit_bins)
        step = float(grid.nm_edges[1] - grid.nm_edges[0])
        b = LineBinner(maps, len(grid.slit_edges) - 1, step, (float(grid.nm_edges[0]), float(grid.nm_edges[-1])))
        if not Grid.of(b).same(grid):
            raise ConvertError("the calibration at %s doesn't give the scan's binning grid" % maps.path)
        return b

    def binned_line(self, sweep_id, index, row):
        """Mean raw counts per cell [slit bins, fine bands] for one line, or None."""
        path = os.path.join(self.path, "binned", "sweep_%03d.npy" % sweep_id)
        if self.binner is None and os.path.exists(path):
            if sweep_id not in self._sweeps:
                self._sweeps[sweep_id] = np.load(path, mmap_mode="r")
            arr = self._sweeps[sweep_id]
            return np.array(arr[index], dtype=np.float64) if index < len(arr) else None
        if row.get("file"):
            if self.binner is None:
                self.binner = self.make_binner(grid=self.grid)
            img = np.load(os.path.join(self.path, row["file"]))
            if img.ndim == 3:
                return np.mean([self.binner.bin(f)[0] for f in img], axis=0).astype(np.float64)
            return self.binner.bin(img)[0].astype(np.float64)
        return None

    def references(self, extra_roots=()):
        roots = [os.path.abspath(os.path.expanduser(r)) for r in extra_roots if r]
        roots.append(self.path)
        for named in ((self.info.get("camera") or {}).get("references"), self.info.get("calibration_session")):
            if isinstance(named, str) and named:   # relative to the scan folder
                roots.append(os.path.normpath(os.path.join(self.path, os.path.expanduser(named))))
        out = []
        for root in dict.fromkeys(roots):
            folder = os.path.join(root, "reference")
            if not os.path.isdir(folder):
                continue
            root_grid = os.path.join(root, "binned", "binning.npz")
            same_grid = root == self.path or (os.path.exists(root_grid) and Grid.load(root_grid).same(self.grid))
            for name in sorted(os.listdir(folder)):
                meta_path = os.path.join(folder, name, "meta.json")
                if not os.path.exists(meta_path):
                    continue
                meta = read_json(meta_path)
                if meta.get("kind") not in ("dark", "white"):
                    continue
                ref = Reference(name, meta["kind"], float(meta.get("exposure_us") or 0), float(meta.get("gain", 1.0)),
                                os.path.join(folder, name), root, reflectance=meta.get("reflectance"))
                binned = os.path.join(root, "binned", "reference_%s.npy" % name)
                if same_grid and os.path.exists(binned):
                    ref.binned = np.load(binned).astype(np.float64)
                out.append(ref)
        return out

    def bin_reference(self, ref):
        if ref.binned is None:
            if self.binner is None:
                self.binner = self.make_binner(grid=self.grid)
            frames = [np.load(os.path.join(ref.folder, f)) for f in sorted(os.listdir(ref.folder))
                      if f.endswith(".npy")]
            frames = [f for a in frames for f in (a if a.ndim == 3 else [a])]
            if not frames:
                raise ConvertError("no frames in %s" % ref.folder)
            ref.binned = np.mean([self.binner.bin(f)[0] for f in frames], axis=0).astype(np.float64)
        return ref.binned


def matching_dark(darks, exposure_us, gain, tol=0.02):
    best = None
    for d in darks:
        if d.exposure_us and abs(d.exposure_us / exposure_us - 1) <= tol and abs(d.gain / gain - 1) <= tol:
            if best is None or abs(d.exposure_us - exposure_us) < abs(best.exposure_us - exposure_us):
                best = d
    return best


# --- values --------------------------------------------------------------------------------------

def band_matrix(grid, centres, step):
    """[fine bands, bands]: 1 where a binned cell's centre falls in a band."""
    c = grid.nm_centres
    lo, hi = centres - step / 2.0, centres + step / 2.0
    m = (c[:, None] >= lo[None, :]) & (c[:, None] < hi[None, :])
    empty = ~m.any(axis=0)
    if empty.any():
        raise ConvertError("bands %s nm have no binned cells: the scan was binned over %.0f..%.0f nm"
                           % (", ".join("%.0f" % v for v in centres[empty][:5]), grid.nm_edges[0], grid.nm_edges[-1]))
    return m.astype(np.float64)


def band_sums(per_cell, weights, m):
    """Weighted sums of a [slit, fine] array into [slit, bands]; NaN where any cell with pixels
    in the band is NaN."""
    nan = np.isnan(per_cell) & (weights > 0)
    out = (np.where(nan, 0.0, per_cell) * weights) @ m
    out[(nan.astype(np.float64) @ m) > 0] = np.nan
    return out


def fill_missing(values):
    """Fill NaNs along the spectrum from their neighbours (and along the slit where a whole
    spectrum is gone). Returns the count filled."""
    s, b = values.shape
    missing = np.isnan(values)
    n = int(missing.sum())
    if not n:
        return 0
    x = np.arange(b)
    for i in np.flatnonzero(missing.any(axis=1)):
        ok = ~missing[i]
        if ok.sum() >= 1:
            values[i, ~ok] = np.interp(x[~ok], x[ok], values[i, ok])
    still = np.isnan(values).all(axis=1)
    if still.any() and not still.all():
        rows = np.arange(s)
        for j in range(b):
            values[still, j] = np.interp(rows[still], rows[~still], values[~still, j])
    return n


# --- the dataset -----------------------------------------------------------------------------------

def mean_pose(poses):
    """Mean of 4x4 poses (positions averaged; rotations by their quaternions, sign-aligned), and
    the largest distance (m) and angle (rad) of any pose from it."""
    t = np.mean([p[:3, 3] for p in poses], axis=0)
    qs = np.array([ls.quat_wxyz(p[:3, :3]) for p in poses])
    qs[(qs @ qs[0]) < 0] *= -1
    q = qs.mean(axis=0)
    q /= np.linalg.norm(q)
    out = np.eye(4)
    out[:3, :3] = ls.quat_matrix(*q)
    out[:3, 3] = t
    dist = max(float(np.linalg.norm(p[:3, 3] - t)) for p in poses)
    ang = max(float(np.arccos(np.clip((np.trace(out[:3, :3].T @ p[:3, :3]) - 1) / 2, -1, 1))) for p in poses)
    return out, dist, ang


def csv_pose(row, prefix):
    x, y, z, qx, qy, qz, qw = (float(row[prefix + k]) for k in ("x", "y", "z", "qx", "qy", "qz", "qw"))
    return ls.pose_matrix([x, y, z, qw, qx, qy, qz])


def convert(scan_dir, out_dir, values="auto", nm_min=500.0, nm_max=950.0, nm_step=10.0, references=(),
            white_name="", dark_name="", white_reflectance=None, calibration=None, slit_bins=256,
            ground_z=TABLE_Z, max_missing=0.25, sweeps=None, force=False, urdf=None):
    from so101_scan_description.kinematics import Robot

    scan = Scan(scan_dir, calibration, slit_bins)
    grid = scan.grid
    out_dir = os.path.abspath(os.path.expanduser(out_dir))
    if os.path.isdir(out_dir) and os.listdir(out_dir) and not force:
        if not os.path.exists(os.path.join(out_dir, "dataset.json")):
            raise ConvertError("%s isn't empty and isn't a dataset; give another folder" % out_dir)
    centres = np.arange(nm_min, nm_max + nm_step / 2.0, nm_step)
    m = band_matrix(grid, centres, nm_step)
    pixels = grid.pixels.astype(np.float64)
    log("scan %s: %d lines in lines.csv, binned on %d slit bins x %d cells of %.1f nm"
        % (scan.path, len(scan.lines), grid.pixels.shape[0], grid.pixels.shape[1],
           grid.nm_edges[1] - grid.nm_edges[0]))

    refs = scan.references(references)
    darks = [r for r in refs if r.kind == "dark" and (not dark_name or r.name == dark_name)]
    whites = [r for r in refs if r.kind == "white" and (not white_name or r.name == white_name)]
    if white_name and not whites:
        raise ConvertError("no white reference named %s" % white_name)
    white = sorted(whites, key=lambda r: (r.name != "white", r.root != scan.path))[0] if whites else None
    if values == "auto":
        values = "reflectance" if white else ("radiance" if grid.response is not None else "counts")
    if values == "reflectance" and white is None:
        raise ConvertError("no white reference (reference/<name>/ with kind white): take one with "
                           "capture_reference white, or use --values radiance")
    if values == "radiance" and grid.response is None:
        raise ConvertError("the calibration has no spectral response: use --values counts or reflectance")

    dark_cache = {}

    def dark_for(exposure_us, gain):
        key = (round(exposure_us, 1), round(gain, 4))
        if key not in dark_cache:
            d = matching_dark(darks, exposure_us, gain)
            dark_cache[key] = (scan.bin_reference(d), d.name) if d else (BLACK_LEVEL, None)
        return dark_cache[key]

    def per_second(binned, exposure_us, gain):
        dark, _ = dark_for(exposure_us, gain)
        return (binned - dark) / (exposure_us * 1e-6 * gain)

    if values == "reflectance":
        if white_reflectance is None:
            white_reflectance = float(white.reflectance or 0.98)
        w = per_second(scan.bin_reference(white), white.exposure_us, white.gain)
        den = band_sums(w, pixels, m) / white_reflectance
        log("white: %s (%s), %.0f us, gain %.2f; its dark: %s" % (
            white.name, white.folder, white.exposure_us, white.gain,
            dark_for(white.exposure_us, white.gain)[1] or "the black level"))
    elif values == "radiance":
        den = band_sums(grid.response.astype(np.float64), pixels, m)
    else:
        den = pixels @ m
    # leave out values the white or the response barely reach: judged per pixel, as some values
    # come from far fewer pixels than others
    level = den / np.maximum(pixels @ m, 1.0)
    den = np.where(level > 0.02 * np.nanmax(level), den, np.nan)

    wanted = None if sweeps is None else {int(s) for s in sweeps}
    rows, data, dropped = [], [], {"no_frame": 0, "missing_values": 0, "not_asked": 0}
    filled = 0
    for row in scan.lines:
        sid, idx = int(row["sweep_id"]), int(row["index"])
        if wanted is not None and sid not in wanted:
            dropped["not_asked"] += 1
            continue
        fr = scan.frames.get((sid, idx))
        if fr is None or fr["status"] != "ok":
            dropped["no_frame"] += 1
            continue
        binned = scan.binned_line(sid, idx, fr)
        if binned is None or not np.isfinite(binned).any():
            dropped["no_frame"] += 1
            continue
        c = per_second(binned, float(fr["exposure_us"]), float(fr["gain"]))
        v = band_sums(c, pixels, m) / den
        if np.isnan(v).mean() > max_missing:
            dropped["missing_values"] += 1
            continue
        filled += fill_missing(v)
        if scan.camera.get("slit_reversed"):
            v = v[::-1]
        rows.append(row)
        data.append(v.astype(np.float32))
    if not rows:
        raise ConvertError("no lines with frames to write (%s)" % ", ".join("%d %s" % (n, k) for k, n in dropped.items()
                                                                           if n))

    # poses: one head pose per sweep, in a world frame with the table top at z = 0
    frames_info = scan.info.get("frames") or {}
    scene_distance = float(frames_info.get("scene_distance_m", 0.15))
    half_line = float(frames_info.get("scan_line_half_length_m", 0.021538))
    if urdf:
        # a head calibration made after the scan: the head's poses again, from the joint readings
        with open(os.path.expanduser(urdf)) as f:
            robot = Robot(f.read())
        head = ls.head_from_urdf(robot)
        arm = [j for j in robot.movable() if j != "scan_mirror_joint"]
        if any(j not in rows[0] for j in arm):
            raise ConvertError("lines.csv has no joint readings (%s) to work out the poses with --urdf from"
                               % ", ".join(arm))
        head_poses = [robot.fk("scan_head_link", {j: float(r[j]) for j in arm}) for r in rows]
        scene_distance = float(robot.fk("scan_line_frame", {}, base="line_camera_optical_frame")[2, 3])
        box = robot.root.find("link[@name='scan_line_frame']/visual/geometry/box")
        if box is not None:
            half_line = float(box.get("size").split()[0]) / 2
        log("head: %s (poses from the joint readings, scan line %.2f mm long at %.1f mm)"
            % (urdf, 2e3 * half_line, 1e3 * scene_distance))
    else:
        robot = Robot(scan.urdf)
        head = ls.head_from_urdf(robot)
        head_poses = [csv_pose(r, "head_") for r in rows]
        worst_pos = worst_ang = 0.0
        for row in rows:
            v = np.linalg.inv(csv_pose(row, "head_")) @ csv_pose(row, "cam_")
            want = ls.virtual_camera_in_head(head, float(row["mirror_angle"]))
            worst_pos = max(worst_pos, float(np.abs(v[:3, 3] - want[:3, 3]).max()))
            worst_ang = max(worst_ang, float(np.abs(v[:3, :3] - want[:3, :3]).max()))
        if worst_pos > 1e-5 or worst_ang > 1e-4:
            raise ConvertError("lines.csv's line-camera poses are up to %.3f mm and %.4f rad off the head model in "
                               "robot.urdf: the trainer would put the lines in the wrong place" % (worst_pos * 1e3,
                                                                                                 worst_ang))
    world = np.eye(4)
    world[2, 3] = -ground_z
    sweep_ids = list(dict.fromkeys(int(r["sweep_id"]) for r in rows))
    sweep_poses, spreads = [], {}
    for sid in sweep_ids:
        poses = [world @ p for r, p in zip(rows, head_poses) if int(r["sweep_id"]) == sid]
        pose, dist, ang = mean_pose(poses)
        sweep_poses.append(ls.pose_array(pose))
        spreads[str(sid)] = dict(lines=len(poses), max_offset_mm=round(dist * 1e3, 4),
                                 max_angle_deg=round(math.degrees(ang), 4))
        if dist > 1e-3 or ang > math.radians(0.2):
            log("  ! sweep %d: the head moved %.2f mm / %.2f deg during the sweep; the trainer takes one pose a "
                "sweep" % (sid, dist * 1e3, math.degrees(ang)))

    width = data[0].shape[0]
    intr = ls.intrinsics(width, scene_distance, half_line)
    lines = np.ascontiguousarray(np.stack(data), dtype="<f4")
    line_sweep = np.array([sweep_ids.index(int(r["sweep_id"])) for r in rows], dtype="<i4")
    line_angle = np.array([float(r["mirror_angle"]) for r in rows], dtype="<f8")
    head_pose = np.array(sweep_poses, dtype="<f8").reshape(-1, 7)

    os.makedirs(out_dir, exist_ok=True)
    meta = dict(
        source="so101_scan", scan=scan.path, scan_name=scan.info.get("name"), values=values,
        white=None if white is None else dict(name=white.name, folder=white.folder, exposure_us=white.exposure_us,
                                              gain=white.gain, reflectance=white_reflectance),
        darks={"%g us, gain %g" % k: v[1] or "black level %g" % BLACK_LEVEL for k, v in dark_cache.items()},
        calibration=scan.camera.get("calibration"), slit_reversed=bool(scan.camera.get("slit_reversed")),
        head_urdf=os.path.abspath(os.path.expanduser(urdf)) if urdf else "the scan's robot.urdf",
        world="base_link with z moved by %+.4f m, so the table top (base_link z = %.4f) is z = 0"
              % (-ground_z, ground_z),
        sweep_ids=sweep_ids, head_pose_spread=spreads,
        lines_written=len(rows), lines_left_out=dropped, values_filled=filled,
        values_filled_fraction=round(filled / lines.size, 6))
    doc = {
        "format": "linesplat-dataset", "version": 1, "units": "metres, radians, nanometres",
        "num_lines": int(lines.shape[0]), "num_sweeps": int(head_pose.shape[0]), "width": int(width),
        "num_bands": int(lines.shape[2]), "values": values, "wavelengths_nm": [float(c) for c in centres],
        "intrinsics": intr, "head": head.to_json(),
        "files": {"lines": "lines.npy", "line_sweep": "line_sweep.npy", "line_mirror_angle": "line_mirror_angle.npy",
                  "sweep_head_pose": "sweep_head_pose.npy"},
        "metadata": meta,
    }
    with open(os.path.join(out_dir, "dataset.json"), "w") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")
    np.save(os.path.join(out_dir, "lines.npy"), lines)
    np.save(os.path.join(out_dir, "line_sweep.npy"), line_sweep)
    np.save(os.path.join(out_dir, "line_mirror_angle.npy"), line_angle)
    np.save(os.path.join(out_dir, "sweep_head_pose.npy"), head_pose)
    left = ", ".join("%d %s" % (n, k.replace("_", " ")) for k, n in dropped.items() if n)
    log("wrote %s: %d lines in %d sweeps, %d px x %d bands of %s%s; %.2f%% of values filled in"
        % (out_dir, lines.shape[0], head_pose.shape[0], width, lines.shape[2], values,
           " (left out: %s)" % left if left else "", 100.0 * filled / lines.size))
    return doc


def main(argv=None):
    ap = argparse.ArgumentParser(description="Turn a scan folder into the line-splat trainer's dataset")
    ap.add_argument("scan", help="the scan folder (scan_sweep's output, with line_camera's frames/ and binned/)")
    ap.add_argument("out", help="the dataset folder to write")
    ap.add_argument("--values", choices=("auto", "reflectance", "radiance", "counts"), default="auto",
                    help="auto: reflectance with a white reference, else radiance")
    ap.add_argument("--nm-min", type=float, default=500.0, help="first band's centre, nm")
    ap.add_argument("--nm-max", type=float, default=950.0, help="last band's centre, nm")
    ap.add_argument("--nm-step", type=float, default=10.0, help="band width, nm")
    ap.add_argument("--references", action="append", default=[],
                    help="a folder whose reference/ has darks and whites (may be given more than once)")
    ap.add_argument("--white", default="", help="the white's folder name under reference/")
    ap.add_argument("--dark", default="", help="use only this dark")
    ap.add_argument("--white-reflectance", type=float,
                    help="the white's reflectance (default: its meta.json's, else 0.98 for PTFE)")
    ap.add_argument("--calibration", help="hsical calibration folder, to bin raw frames (default: camera.json's)")
    ap.add_argument("--slit-bins", type=int, default=256, help="pixels along the slit when binning raw frames here")
    ap.add_argument("--ground-z", type=float, default=TABLE_Z,
                    help="base_link z that becomes the dataset's z = 0 (default: the table top)")
    ap.add_argument("--max-missing", type=float, default=0.25, help="leave out lines missing more of their values")
    ap.add_argument("--sweeps", help="comma-separated sweep ids to keep (default: all)")
    ap.add_argument("--urdf", help="the head from this URDF instead of the scan's (e.g. a later head "
                                   "calibration's robot.urdf): poses worked out again from the joint readings")
    ap.add_argument("--force", action="store_true", help="write into a folder that isn't empty")
    args, _ = ap.parse_known_args(argv)
    try:
        convert(args.scan, args.out, args.values, args.nm_min, args.nm_max, args.nm_step, args.references,
                args.white, args.dark, args.white_reflectance, args.calibration, args.slit_bins, args.ground_z,
                args.max_missing, args.sweeps.split(",") if args.sweeps else None, args.force, args.urdf)
    except ConvertError as e:
        print("scan_to_dataset: %s" % e, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
