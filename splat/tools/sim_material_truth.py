#!/usr/bin/env python3
"""The true materials behind a splat trained on a simulator scan, for splat_materials to score.

    python3 splat/tools/sim_material_truth.py SIM DATASET SPLAT.lsplat OUT --poses TRAINED/sweep_head_pose.npy
    ./splat_materials SPLAT.lsplat SPLAT.lsplat --library OUT/library.csv \\
        --truth-labels OUT/labels.npy --truth-map OUT/view.npy \\
        --dataset DATASET --poses TRAINED/sweep_head_pose.npy --truth-lines OUT/lines.npy

SIM is the scan session `python -m hsisim scan` wrote, DATASET what scan_to_dataset.py made of
it and SPLAT.lsplat what splat_export made of the trained splat. This rebuilds the simulator's
scene and writes to OUT:

    library.csv  the scene's materials' reflectance at the dataset's wavelengths, with the
                 splat's map colors: the library splat_materials matches against
    labels.npy   int32 [N], each Gaussian's true material (a library column), the material of
                 the surface nearest its centre, or -1 if none is within 1.5 mm
    view.npy     int32 [360, 480], the material each pixel of the file's default view shows,
                 or -1 where its four corners' rays don't all meet the same one
    lines.npy    int32 [lines, width], the material each pixel of each scan line saw, from the
                 simulator's truth, or -1 where none covers 90% of it

It also matches the dataset's measured line spectra against the library directly, pixel by
pixel, as splat_materials matches Gaussians (spectral angle within 2x in brightness and 10
degrees), and prints how often that is right: what the instrument allows before any splat.

The Gaussians and the view are only as right as the splat's shape: where training gets the
geometry wrong (a relief flattened onto the board, say), a Gaussian's nearest surface and the
view's rays miss what it shows. The lines don't depend on that, since a trained splat
reproduces the lines whatever its shape.

The dataset's world is base_link moved so the table top is z = 0; dataset.json says by how much.
A trained splat can sit anywhere in it, though: moving the scene and every pose together changes
nothing in the lines, so training is free to drift (a 2 mm board starts out at z = 0, say).
--poses, the trained sweep poses, puts it back: the rigid motion that best takes them onto the
simulator's true head poses is applied to the Gaussians and the view first.
It needs what sim/ needs (sim/requirements.txt).
"""

import argparse
import csv
import json
import os
import re
import struct
import sys

import numpy as np

SIM = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir, "sim")
sys.path.insert(0, os.path.normpath(SIM))

from hsisim import scenes, spectra  # noqa: E402

TRUTH_RADIUS = 1.5e-3   # splat_materials' radius for synthetic truth
MAP_W, MAP_H = 480, 360  # splat_materials' map size

# The splat's names and map colors for the simulator's materials (splat/src/materials.cpp),
# plus the two only the simulator has.
NAMES = {
    "white_paper": ("white paper", "#d8d3c4"),
    "carbon_black": ("carbon black", "#5b626c"),
    "ir_black_dye": ("IR-transparent black dye", "#9a6fdc"),
    "leaf": ("leaf", "#4caf50"),
    "red_paint": ("red paint", "#e0463c"),
    "orange_plastic": ("orange plastic", "#f08a24"),
    "yellow_paint": ("yellow paint", "#f2d03b"),
    "green_paint": ("green paint", "#1fb5a8"),
    "blue_paint": ("blue paint", "#3f7fe0"),
    "gray18": ("18% gray", "#9aa0a8"),
    "wood": ("wood", "#a87648"),
    "ptfe": ("PTFE", "#f4f6fb"),
    "rare_earth": ("rare-earth tile", "#d4709c"),
}


def read_lsplat(path):
    """(header, Gaussian centres [N, 3]) of a splat_export file."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:2] == b"\x1f\x8b":
        raise SystemExit(f"{path} is gzipped; gunzip it first")
    if data[:4] != b"LSPV":
        raise SystemExit(f"{path} is not a splat_export file")
    _, length = struct.unpack_from("<II", data, 4)
    header = json.loads(data[12:12 + length])
    b = header["blocks"]["means"]
    means = np.frombuffer(data, "<f4", header["count"] * 3, 12 + length + b["offset"]).reshape(-1, 3)
    return header, means.astype(float)


def ground_shift(dataset):
    """How far base_link's z is below the dataset's (the table top's base_link z)."""
    with open(os.path.join(dataset, "dataset.json")) as f:
        world = json.load(f).get("metadata", {}).get("world", "")
    m = re.search(r"z moved by ([+-]?[0-9.]+) m", world)
    if not m:
        raise SystemExit(f"{dataset}/dataset.json doesn't say how its world relates to base_link")
    return float(m.group(1))


def load_scene(sim):
    with open(os.path.join(sim, "truth", "scene.json")) as f:
        info = json.load(f)
    return scenes.make(info["name"], tuple(info["target"]))


def nearest_materials(scene, points, radius=TRUTH_RADIUS):
    """The material id of the surface nearest each point, -1 if none is within `radius`. Rays
    from just outside the radius, along the 6 axes and the 8 diagonals, find the nearby faces."""
    dirs = [d for a in range(3) for d in (np.eye(3)[a], -np.eye(3)[a])]
    dirs += [np.array([sx, sy, sz]) / np.sqrt(3.0) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    best = np.full(len(points), np.inf)
    out = np.full(len(points), -1, np.int32)
    back = 2.0 * radius
    for d in dirs:
        o = points - back * d
        hits = scene.cast(o, np.broadcast_to(d, points.shape).copy())
        dist = np.linalg.norm(hits.point - points, axis=1)
        dist[hits.solid < 0] = np.inf
        closer = dist < best
        best[closer], out[closer] = dist[closer], hits.material[closer]
    out[best > radius] = -1
    return out


def line_truth(sim, dataset, column, pure=0.9):
    """[lines, width] the material each dataset pixel saw (a library column), -1 where none
    covers `pure` of it. Pixel u of a line spans slit positions h from -1 + 2u/W to -1 + 2(u+1)/W."""
    with open(os.path.join(dataset, "dataset.json")) as f:
        width = json.load(f)["width"]
    weights = np.load(os.path.join(sim, "truth", "weights.npy")).astype(np.float32)  # [line, h, material]
    h = np.load(os.path.join(sim, "truth", "h.npy")).astype(float)
    sweeps = np.load(os.path.join(dataset, "line_sweep.npy"))
    with open(os.path.join(sim, "truth", "lines_true.csv")) as f:
        rows = list(csv.DictReader(f))
    true_sweeps = np.array([int(r["sweep_id"]) for r in rows])
    if len(sweeps) != len(weights) or len(np.unique(sweeps)) != len(np.unique(true_sweeps)) or \
            np.any(np.diff(sweeps) * np.diff(true_sweeps) < 0) or np.any((np.diff(sweeps) != 0) != (np.diff(true_sweeps) != 0)):
        raise SystemExit(f"{dataset} doesn't have every line of {sim} in order; convert it again without leaving any out")
    with open(os.path.join(sim, "truth", "truth.json")) as f:
        ids = np.array([spectra.material_id(m) for m in json.load(f)["materials"]])
    edges = np.searchsorted(h, -1.0 + 2.0 * np.arange(width + 1) / width)
    share = np.add.reduceat(weights, edges[:-1], axis=1) / np.diff(edges)[None, :, None]
    best = share.argmax(axis=2)
    out = np.vectorize(lambda m: column.get(int(m), -1), otypes=[np.int32])(ids[best])
    out[share.max(axis=2) < pure] = -1
    return out


def spectral_angle_labels(spectra, library, max_angle_deg=10.0, brightness=2.0):
    """splat_materials' library match for each row of `spectra` [n, B] against `library`
    [M, B]: the column with the smallest spectral angle among those whose fitted scale is within
    1/brightness..brightness, or -1 if that angle is over max_angle_deg."""
    dots = spectra @ library.T
    norms = np.linalg.norm(spectra, axis=1)[:, None] * np.linalg.norm(library, axis=1)[None, :]
    angle = np.degrees(np.arccos(np.clip(dots / np.maximum(norms, 1e-12), -1.0, 1.0)))
    scale = dots / np.sum(library * library, axis=1)[None, :]
    angle[(scale < 1.0 / brightness) | (scale > brightness)] = np.inf
    best = angle.argmin(axis=1)
    return np.where(angle[np.arange(len(best)), best] <= max_angle_deg, best, -1)


def pose_matrix(v):
    """4 x 4 from [tx, ty, tz, qw, qx, qy, qz]."""
    t, (w, x, y, z) = v[:3], v[3:] / np.linalg.norm(v[3:])
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = t
    return T


def alignment(sim, dataset, poses, shift):
    """The rigid motion (R, t) that best takes the trained sweep head poses onto the true ones,
    in the dataset's world: Kabsch on each pose's origin and three points 10 cm along its axes."""
    with open(os.path.join(dataset, "dataset.json")) as f:
        sweep_ids = json.load(f)["metadata"]["sweep_ids"]
    with open(os.path.join(sim, "truth", "lines_true.csv")) as f:
        first = {}
        for r in csv.DictReader(f):
            first.setdefault(int(r["sweep_id"]), r)
    trained = np.load(poses).reshape(-1, 7)
    if len(trained) != len(sweep_ids):
        raise SystemExit(f"{poses} has {len(trained)} poses for {len(sweep_ids)} sweeps")
    src, dst = [], []
    for k, sid in enumerate(sweep_ids):
        r = first[sid]
        true = pose_matrix(np.array([float(r["head_" + c]) for c in ("x", "y", "z", "qw", "qx", "qy", "qz")]))
        true[2, 3] += shift
        for T, out in ((pose_matrix(trained[k]), src), (true, dst)):
            out.append(T[:3, 3])
            out.extend(T[:3, 3] + 0.1 * T[:3, j] for j in range(3))
    src, dst = np.array(src), np.array(dst)
    cs, cd = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - cs).T @ (dst - cd))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cd - R @ cs


def view_rays(view, w=MAP_W, h=MAP_H):
    """Origins and directions of the rays through each pixel's four quarter points, [4, h*w, 3],
    for splat_materials' pinhole of the file's default view (look_at, f from fov_deg)."""
    eye, target, up = (np.asarray(view[k], float) for k in ("eye", "target", "up"))
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    f = 0.5 * min(w, h) / np.tan(0.5 * np.radians(view["fov_deg"]))
    v, u = np.mgrid[0:h, 0:w].astype(float)
    dirs = []
    for du, dv in ((0.25, 0.25), (0.75, 0.25), (0.25, 0.75), (0.75, 0.75)):
        cu = ((u + du) - 0.5 * w) / f
        cv = ((v + dv) - 0.5 * h) / f
        d = cu.reshape(-1, 1) * x + cv.reshape(-1, 1) * y + z
        dirs.append(d / np.linalg.norm(d, axis=1, keepdims=True))
    return eye, np.stack(dirs)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("sim", help="the scan session hsisim wrote")
    ap.add_argument("dataset", help="what scan_to_dataset.py made of it")
    ap.add_argument("splat", help="splat_export's file for the splat trained on it (unzipped)")
    ap.add_argument("out", help="folder for library.csv, labels.npy, view.npy and lines.npy")
    ap.add_argument("--poses", help="the trained sweep_head_pose.npy, to undo the splat's drift")
    a = ap.parse_args(argv)

    scene = load_scene(a.sim)
    header, means = read_lsplat(a.splat)
    shift = ground_shift(a.dataset)
    to_base = np.array([0.0, 0.0, -shift])  # dataset point + this = base_link point
    R, t = np.eye(3), np.zeros(3)
    if a.poses:
        R, t = alignment(a.sim, a.dataset, a.poses, shift)
        angle = np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))
        print(f"the splat sits {np.linalg.norm(t + (R - np.eye(3)) @ means.mean(0)) * 1e3:.2f} mm and {angle:.2f} deg "
              f"from the truth at its centre; moved back")
    means = means @ R.T + t
    view = dict(header["view"])
    view["eye"], view["target"] = (R @ np.asarray(view[k], float) + t for k in ("eye", "target"))
    view["up"] = R @ np.asarray(view["up"], float)

    present = [m for m in scene.materials() if m != "void"]
    missing = [m for m in present if m not in NAMES]
    if missing:
        raise SystemExit(f"no splat name for {', '.join(missing)}; add them to NAMES")
    column = {spectra.material_id(m): k for k, m in enumerate(present)}
    to_column = np.vectorize(lambda i: column.get(int(i), -1), otypes=[np.int32])

    os.makedirs(a.out, exist_ok=True)
    nm = np.asarray(header["wavelengths_nm"], float)
    table = spectra.table(nm, present)
    with open(os.path.join(a.out, "library.csv"), "w") as f:
        f.write(f"# {scene.name}: the simulator's materials at {a.dataset}'s wavelengths\n")
        f.write("nm," + ",".join(NAMES[m][0] for m in present) + "\n")
        f.write("color," + ",".join(NAMES[m][1] for m in present) + "\n")
        for b, w in enumerate(nm):
            f.write(f"{w:g}," + ",".join(f"{v:.5f}" for v in table[:, b]) + "\n")

    labels = to_column(nearest_materials(scene, means + to_base)) if len(means) else np.zeros(0, np.int32)
    np.save(os.path.join(a.out, "labels.npy"), labels.astype(np.int32))

    eye, dirs = view_rays(view)
    ids = np.stack([scene.cast(np.tile(eye + to_base, (len(d), 1)), d).material for d in dirs])
    pure = (ids == ids[0]).all(axis=0)
    view_map = np.where(pure, to_column(ids[0]), -1).astype(np.int32).reshape(MAP_H, MAP_W)
    np.save(os.path.join(a.out, "view.npy"), view_map)

    lines = line_truth(a.sim, a.dataset, column)
    np.save(os.path.join(a.out, "lines.npy"), lines)
    measured = np.load(os.path.join(a.dataset, "lines.npy"))
    scored = lines >= 0
    got = spectral_angle_labels(measured[scored].astype(float), table)

    print(f"{scene.name}: {len(present)} materials, {len(means)} Gaussians, "
          f"{np.mean(labels >= 0):.1%} within {TRUTH_RADIUS * 1e3:g} mm of a surface; "
          f"{np.mean(view_map >= 0):.1%} of the default view and {np.mean(lines >= 0):.1%} of the line pixels "
          f"show one material")
    print(f"  {'':<26} {'Gaussians':>9} {'view px':>9} {'line px':>9}")
    for k, m in enumerate(present):
        print(f"  {NAMES[m][0]:<26} {int(np.sum(labels == k)):9d} {int(np.sum(view_map == k)):9d} "
              f"{int(np.sum(lines == k)):9d}")
    print(f"the measured lines, matched pixel by pixel: {np.mean(got == lines[scored]):.1%} of {int(scored.sum())} "
          f"line pixels that saw one material get it, {np.mean(got < 0):.1%} no match")
    for k, m in enumerate(present):
        sel = lines[scored] == k
        if sel.any():
            print(f"  {NAMES[m][0]:<26} {np.mean(got[sel] == k):6.1%}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
