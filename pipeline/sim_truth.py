"""The simulator's truth as the trainer's ground truth, and checks of a converted dataset against it.

    python3 pipeline/sim_truth.py SIM_SESSION DATASET        # writes DATASET/gt/ and prints the checks

A dataset that scan_to_dataset made from an hsisim session gets the same gt/ folder splat_synth
writes for its own synthetic datasets, so splat_train and splat_render score against it unchanged:

  gt/sweep_head_pose.npy    [S, 7] each sweep's true head pose, in the dataset's world
  gt/line_mirror_angle.npy  [L] each line's true mirror angle
  gt/lines_clean.npy        [L, W, B] what each pixel really saw: the truth's reflectance (times
                            shading) averaged over the pixel's stretch of the slit and over the
                            band, with no blur and no noise

and the conversion is checked against the same truth:

  reflectance   pixels that saw one material only, a few pixels from any edge: the dataset's
                value over the truth's, in every band where the truth is above 0.15
  slit          each line's profile along the slit (the mean over the bands) against the
                truth's: their correlation, the correlation with the truth mirrored (which is
                high if pixel 0 is at the wrong end), and how far apart they sit in pixels
  poses         how far the logged head poses the dataset was made from are from the truth

It needs numpy only. Dataset lines are matched to the session's lines by sweep and mirror angle,
so lines the converter left out are no problem.
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
sys.path[:0] = [os.path.join(REPO, "ros2", p) for p in ("so101_scan_camera", "so101_scan_description")]

from so101_scan_camera import linesplat as ls  # noqa: E402
from so101_scan_camera.scan_to_dataset import csv_pose, mean_pose  # noqa: E402

REFLECTANCE_FLOOR = 0.15     # bands darker than this are left out of the ratio, as in hsisim's evaluate
EDGE_MARGIN_PX = 3.0         # a pure pixel has one material this far either side of it too
PROFILE_CONTRAST = 0.02      # lines whose true profile varies less than this have no shape to compare


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def pixel_matrix(h, width):
    """[width, len(h)]: the mean over the truth's slit samples that fall in each pixel. Pixel p
    covers h from -1 + 2p / width to -1 + 2(p + 1) / width (pixel 0 at h = -1, the line camera's
    -x end, as the converter writes it)."""
    edges = np.linspace(-1.0, 1.0, width + 1)
    p = np.searchsorted(edges, h, side="right") - 1
    m = np.zeros((width, len(h)))
    inside = (p >= 0) & (p < width)
    m[p[inside], np.flatnonzero(inside)] = 1.0
    counts = m.sum(axis=1, keepdims=True)
    if (counts == 0).any():
        raise ValueError("the truth samples the slit too coarsely for %d pixels" % width)
    return m / counts


def band_matrix(nm, centres, step):
    """[len(nm), bands]: the mean over the truth's wavelengths in each band."""
    lo, hi = centres - step / 2.0, centres + step / 2.0
    m = ((nm[:, None] >= lo[None, :]) & (nm[:, None] < hi[None, :])).astype(float)
    if (m.sum(axis=0) == 0).any():
        raise ValueError("the truth has no wavelengths in some bands (%.0f..%.0f nm)" % (nm[0], nm[-1]))
    return m / m.sum(axis=0, keepdims=True)


def pure_pixels(weights, h, width, margin_px=EDGE_MARGIN_PX, level=0.999, void=0):
    """[width] True where everything the pixel saw, and margin_px either side, was one material
    (and not the void past the scene)."""
    w = np.asarray(weights, float)
    total = w.sum(axis=1)
    dom = w.argmax(axis=1)
    ok = (total > 0) & (w.max(axis=1) >= level * total) & (dom != void)
    edges = np.linspace(-1.0, 1.0, width + 1)
    pad = margin_px * 2.0 / width
    a = np.searchsorted(h, edges[:-1] - pad, side="left")
    b = np.searchsorted(h, edges[1:] + pad, side="left")
    good = (a > 0) & (b < len(h)) & (b > a)   # the margin must stay on the sampled slit
    cum = np.concatenate([[0], np.cumsum(ok)])
    out = np.zeros(width, bool)
    for p in np.flatnonzero(good):
        if cum[b[p]] - cum[a[p]] == b[p] - a[p]:
            d = dom[a[p]:b[p]]
            out[p] = d.min() == d.max()
    return out


def profile_shift(a, b, max_lag=6.0):
    """Sub-pixel shift (px) that best lines profile b up with profile a: the s that minimises
    the squared difference between b and a moved by s (positive: b sits at higher pixels)."""
    x = np.arange(len(a), dtype=float)
    inner = slice(int(math.ceil(max_lag)) + 1, len(a) - int(math.ceil(max_lag)) - 1)

    def cost(s):
        return float(np.sum((np.interp(x[inner] - s, x, a) - b[inner]) ** 2))

    best = min(np.arange(-max_lag, max_lag + 0.5, 0.5), key=cost)
    return float(min(np.arange(best - 0.5, best + 0.5 + 1e-9, 0.01), key=cost))


def correlation(a, b):
    a = a - a.mean()
    b = b - b.mean()
    d = math.sqrt(float(np.dot(a, a) * np.dot(b, b)))
    return float(np.dot(a, b) / d) if d > 0 else float("nan")


def rotation_angle(r):
    return math.acos(max(-1.0, min(1.0, (np.trace(r) - 1.0) / 2.0)))


def write_truth(session, dataset, log=print):
    """Writes DATASET/gt/ from the hsisim session it was converted from, and returns the checks'
    numbers as a dict."""
    from so101_scan_description.kinematics import Robot

    doc = json.load(open(os.path.join(dataset, "dataset.json")))
    meta = doc.get("metadata") or {}
    files = doc["files"]
    line_sweep = np.load(os.path.join(dataset, files["line_sweep"]))
    line_angle = np.load(os.path.join(dataset, files["line_mirror_angle"]))
    head_pose = np.load(os.path.join(dataset, files["sweep_head_pose"]))
    lines = np.load(os.path.join(dataset, files["lines"]), mmap_mode="r")
    sweep_ids = meta.get("sweep_ids")
    if not sweep_ids or len(sweep_ids) != len(head_pose):
        raise ValueError("%s doesn't say which scan sweeps its sweeps are (metadata.sweep_ids)" % dataset)
    L, W, B = lines.shape

    logged = read_rows(os.path.join(session, "lines.csv"))
    true_rows = read_rows(os.path.join(session, "truth", "lines_true.csv"))
    truth_line = {(int(r["sweep_id"]), int(r["index"])): i for i, r in enumerate(true_rows)}

    # match each dataset line to its row of lines.csv: same sweep, same logged mirror angle
    by_sweep = {}
    for r in logged:
        by_sweep.setdefault(int(r["sweep_id"]), []).append(r)
    matched = []
    for i in range(L):
        sid = int(sweep_ids[int(line_sweep[i])])
        rows = by_sweep.get(sid) or []
        if not rows:
            raise ValueError("dataset line %d is from sweep %d, which lines.csv doesn't have" % (i, sid))
        d = [abs(float(r["mirror_angle"]) - line_angle[i]) for r in rows]
        j = int(np.argmin(d))
        if d[j] > 1e-8:
            raise ValueError("dataset line %d matches no line of sweep %d in lines.csv" % (i, sid))
        matched.append(rows[j])
    t_index = np.array([truth_line[(int(r["sweep_id"]), int(r["index"]))] for r in matched])

    # the dataset's world: base_link moved so the table top is z = 0; recover the move from sweep 0
    first = [k for k in range(L) if line_sweep[k] == 0]
    logged_mean, _, _ = mean_pose([csv_pose(matched[k], "head_") for k in first])
    world = ls.pose_matrix(head_pose[0]) @ np.linalg.inv(logged_mean)

    # the truth's line cameras must follow the head model the trainer uses (robot.urdf's)
    with open(os.path.join(session, "robot.urdf")) as f:
        head = ls.head_from_urdf(Robot(f.read()))
    worst_pos = worst_ang = 0.0
    true_angle = np.zeros(L)
    for k in range(L):
        t = true_rows[t_index[k]]
        true_angle[k] = float(t["mirror_angle"])
        v = np.linalg.inv(csv_pose(t, "head_")) @ csv_pose(t, "cam_")
        want = ls.virtual_camera_in_head(head, true_angle[k])
        worst_pos = max(worst_pos, float(np.abs(v[:3, 3] - want[:3, 3]).max()))
        worst_ang = max(worst_ang, float(np.abs(v[:3, :3] - want[:3, :3]).max()))
    if worst_pos > 1e-5 or worst_ang > 1e-4:
        raise ValueError("the truth's line cameras are up to %.3f mm and %.4f rad off the head model in robot.urdf"
                         % (worst_pos * 1e3, worst_ang))

    gt_pose, pose_err = [], []
    for s in range(len(head_pose)):
        ks = [k for k in range(L) if line_sweep[k] == s]
        p, spread, _ = mean_pose([world @ csv_pose(true_rows[t_index[k]], "head_") for k in ks])
        if spread > 1e-6:
            log("  ! sweep %s: the true head moved %.4f mm during the sweep" % (sweep_ids[s], spread * 1e3))
        gt_pose.append(ls.pose_array(p))
        rec = ls.pose_matrix(head_pose[s])
        pose_err.append((float(np.linalg.norm(rec[:3, 3] - p[:3, 3])),
                         math.degrees(rotation_angle(rec[:3, :3].T @ p[:3, :3]))))

    # what each pixel really saw, band by band
    tdir = os.path.join(session, "truth")
    weights = np.load(os.path.join(tdir, "weights.npy"), mmap_mode="r")
    h = np.load(os.path.join(tdir, "h.npy")).astype(float)
    nm = np.load(os.path.join(tdir, "nm.npy")).astype(float)
    spectra = np.load(os.path.join(tdir, "materials.npy")).astype(float)
    centres = np.asarray(doc["wavelengths_nm"], float)
    step = float(centres[1] - centres[0]) if len(centres) > 1 else 10.0
    px = pixel_matrix(h, W)
    band_spectra = spectra @ band_matrix(nm, centres, step)          # [material, band]
    clean = np.empty((L, W, B), np.float32)
    ratios, all_err, corr, corr_mirror, shifts = [], [], [], [], []
    for k in range(L):
        w = np.asarray(weights[t_index[k]], float)
        clean[k] = (px @ w) @ band_spectra
        got = np.asarray(lines[k], float)
        all_err.append(got - clean[k])
        pure = pure_pixels(w, h, W)
        if pure.any():
            c = clean[k][pure]
            bright = c > REFLECTANCE_FLOOR
            ratios.append(got[pure][bright] / c[bright])
        tp, gp = clean[k].mean(axis=1), got.mean(axis=1)
        if tp.std() > PROFILE_CONTRAST:
            corr.append(correlation(gp, tp))
            corr_mirror.append(correlation(gp, tp[::-1]))
            shifts.append(profile_shift(tp, gp))

    gt = os.path.join(dataset, "gt")
    os.makedirs(gt, exist_ok=True)
    np.save(os.path.join(gt, "sweep_head_pose.npy"), np.array(gt_pose, dtype="<f8").reshape(-1, 7))
    np.save(os.path.join(gt, "line_mirror_angle.npy"), true_angle.astype("<f8"))
    np.save(os.path.join(gt, "lines_clean.npy"), clean)

    r = np.concatenate(ratios) if ratios else np.array([np.nan])
    e = np.concatenate([a.ravel() for a in all_err])
    out = dict(
        lines=int(L), pure_values=int(r.size),
        reflectance_ratio_median=float(abs(np.median(r) - 1.0)),
        reflectance_ratio_rms=float(np.sqrt(np.mean((r - 1.0) ** 2))),
        reflectance_rms_all=float(np.sqrt(np.mean(e ** 2))),
        profile_lines=len(corr),
        profile_correlation=float(np.median(corr)) if corr else float("nan"),
        profile_correlation_mirrored=float(np.median(corr_mirror)) if corr else float("nan"),
        profile_shift_px=float(np.median(np.abs(shifts))) if shifts else float("nan"),
        logged_pose_error_mm=max(p[0] for p in pose_err) * 1e3,
        logged_pose_error_deg=max(p[1] for p in pose_err),
        truth_head_model_mm=worst_pos * 1e3,
    )
    meta_gt = dict(source="hsisim truth", session=os.path.abspath(session),
                   files=dict(sweep_head_pose="gt/sweep_head_pose.npy", line_mirror_angle="gt/line_mirror_angle.npy",
                              clean_lines="gt/lines_clean.npy"),
                   checks=out)
    with open(os.path.join(gt, "truth.json"), "w") as f:
        json.dump(meta_gt, f, indent=2)
        f.write("\n")
    log("truth     %d lines: reflectance %.2f%% median, %.2f%% rms off on %d pure values (%.4f rms on all); "
        "profiles correlate %.3f (mirrored %.3f), %.2f px apart; logged poses up to %.2f mm, %.2f deg off"
        % (L, 100 * out["reflectance_ratio_median"], 100 * out["reflectance_ratio_rms"], out["pure_values"],
           out["reflectance_rms_all"], out["profile_correlation"], out["profile_correlation_mirrored"],
           out["profile_shift_px"], out["logged_pose_error_mm"], out["logged_pose_error_deg"]))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="Write a converted hsisim session's truth into its dataset's gt/")
    ap.add_argument("session", help="the hsisim scan session (with truth/)")
    ap.add_argument("dataset", help="the dataset scan_to_dataset made from it")
    a = ap.parse_args(argv)
    write_truth(a.session, a.dataset)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
