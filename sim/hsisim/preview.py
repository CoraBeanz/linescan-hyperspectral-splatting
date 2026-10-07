"""Pictures of a simulated session (needs matplotlib).

    python -m hsisim preview sim_out          # writes sim_out/preview/*.png

  scene.png        the scene from straight above, in true colour and in colour infrared
                   (the word hidden in the NIR shows there), with every scan line's
                   in-focus footprint, one colour per sweep
  frame.png        one raw frame as the camera gives it: the spectrum across, the slit
                   down, with the NoIR sensor's colour mosaic still in it
  sweep_<id>.png   each sweep as an image, scan lines across and the slit down: what the
                   lines really saw, and, once `hsical apply` has turned the sweep into
                   reflectance (session/spectra/, as the self-test leaves it), what came out
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from . import spectra
from .scenes import make as make_scene
from .truth import ScanTruth


def _plt():
    try:
        import matplotlib
    except ImportError:
        raise RuntimeError("the preview needs matplotlib: pip install matplotlib") from None
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def top_view(scene, half=0.04, step=0.0002, nm=None):
    """(true colour, colour infrared) linear RGB images [y, x] of the scene from straight
    above, and the extent (x0, x1, y0, y1) in metres."""
    nm = spectra.wavelength_grid(4.0) if nm is None else nm
    tx, ty, tz = scene.target
    xs = np.arange(tx - half, tx + half, step) + step / 2
    ys = np.arange(ty - half, ty + half, step) + step / 2
    X, Y = np.meshgrid(xs, ys[::-1])
    o = np.stack([X.ravel(), Y.ravel(), np.full(X.size, tz + 1.0)], -1)
    d = np.broadcast_to([0.0, 0.0, -1.0], o.shape)
    hits = scene.cast(o, d)
    table = spectra.table(nm)           # both colours are linear in the spectrum: one per material
    shade = hits.shading.reshape(X.shape + (1,))
    return (spectra.true_color(table, nm)[hits.material].reshape(X.shape + (3,)) * shade,
            spectra.color_infrared(table, nm)[hits.material].reshape(X.shape + (3,)) * shade,
            (xs[0] - step / 2, xs[-1] + step / 2, ys[0] - step / 2, ys[-1] + step / 2))


def footprints(truth, half_line, focus):
    """[lines, 2, 3] the ends of every line's in-focus scan line (true poses), in base_link."""
    out = []
    for i in range(len(truth)):
        T = truth.pose(i, "cam")
        ends = np.array([[-half_line, 0.0, focus, 1.0], [half_line, 0.0, focus, 1.0]])
        out.append((ends @ T.T)[:, :3])
    return np.array(out)


def _image(rgb):
    return spectra.to_srgb8(np.clip(rgb, 0, None) / max(np.percentile(rgb, 99.5), 1e-6))


def preview(session, out_dir=None, log=print):
    plt = _plt()
    session = Path(session)
    out = Path(out_dir or session / "preview")
    out.mkdir(parents=True, exist_ok=True)
    info = json.loads((session / "scan.json").read_text())
    sim = info["simulated"]
    truth = ScanTruth(session)
    target = json.loads((session / "truth" / "scene.json").read_text())["target"]
    scene = make_scene(sim["scene"], tuple(target), sim["lighting"]["type"])
    obj = sim["objective"]
    written = []

    # the scene from above, with the scan lines on it
    rgb, cir, ext = top_view(scene)
    ends = footprints(truth, obj["half_line_m"], obj["focus_mm"] * 1e-3)
    fig, axes = plt.subplots(1, 2, figsize=(12, 6.2))
    colours = plt.cm.tab10(np.arange(10))
    for ax, img, title in zip(axes, (rgb, cir), ("true colour", "colour infrared (800-900, 620-680, 520-580 nm)")):
        ax.imshow(_image(img), extent=[1e3 * e for e in ext])
        for i, (a, b) in enumerate(ends):
            sid = int(truth.lines["sweep_id"][i])
            ax.plot(1e3 * np.array([a[0], b[0]]), 1e3 * np.array([a[1], b[1]]), lw=0.6,
                    color=colours[(sid - 1) % 10], alpha=0.8)
        ax.set_xlim(1e3 * ext[0], 1e3 * ext[1])
        ax.set_ylim(1e3 * ext[2], 1e3 * ext[3])
        ax.set_xlabel("base_link x (mm)")
        ax.set_ylabel("base_link y (mm)")
        ax.set_title(title)
    fig.suptitle(f"{scene.name}: {scene.description}\nscan lines in focus, one colour per sweep", fontsize=9)
    fig.tight_layout()
    written.append(out / "scene.png")
    fig.savefig(written[-1], dpi=110)
    plt.close(fig)

    # one raw frame, from the middle of the first sweep
    sweeps = sorted(p for p in (session / "frames").iterdir() if p.is_dir())
    frames = sorted(sweeps[0].glob("frame_*.npy"))
    frame = np.load(frames[len(frames) // 2])
    fig, ax = plt.subplots(figsize=(10, 10 * frame.shape[0] / frame.shape[1] + 0.8))
    ax.imshow(np.sqrt(np.clip(frame.astype(float) - 64, 0, None)), cmap="gray", interpolation="nearest")
    ax.set_title(f"{sweeps[0].name}/{frames[len(frames) // 2].name}: raw 10-bit frame (square root of the "
                 "signal over the black level)", fontsize=9)
    ax.set_xlabel("column (the spectrum)")
    ax.set_ylabel("row (along the slit)")
    fig.tight_layout()
    written.append(out / "frame.png")
    fig.savefig(written[-1], dpi=110)
    plt.close(fig)

    # each sweep: what it saw, and what came out of the calibration
    nm = truth.nm
    for d in sweeps:
        sid = int(json.loads((d / "meta.json").read_text())["sweep_id"])
        lines = [truth.line_number(sid, k) for k in range(len(list(d.glob("frame_*.npy"))))]
        inside = np.abs(truth.h) <= 1.0
        seen = np.stack([truth.reflectance(i)[inside] for i in lines], 1)  # [h, lines, nm]
        panels = [("what the slit saw (truth)", spectra.true_color(seen, nm), (-1.0, 1.0),
                   "slit position h (-1: the line camera's -x end)")]
        cube_path = session / "spectra" / f"{d.name}_reflectance.npy"
        meta_path = session / "spectra" / f"{d.name}_reflectance.json"
        if cube_path.exists() and meta_path.exists():
            cube = np.load(cube_path)                                    # [lines, slit rows, nm]
            meta = json.loads(meta_path.read_text())
            rows = meta["slit_rows"]
            panels.append(("after hsical calibrate and apply", spectra.true_color(
                np.nan_to_num(cube.transpose(1, 0, 2)), np.asarray(meta["nm"], float)),
                (rows[0] - 0.5, rows[-1] + 0.5), "rectified slit row (the first one is h = -1)"))
        fig, axes = plt.subplots(1, len(panels), figsize=(4.2 * len(panels), 6), squeeze=False)
        for ax, (title, img, (top, bottom), ylabel) in zip(axes[0], panels):
            n = img.shape[1]
            ax.imshow(spectra.to_srgb8(img), aspect="auto", interpolation="nearest",
                      extent=(-0.5, n - 0.5, bottom, top))
            ax.set_title(title, fontsize=9)
            ax.set_xlabel("scan line")
            ax.set_ylabel(ylabel)
        fig.suptitle(f"sweep {sid}", fontsize=10)
        fig.tight_layout()
        written.append(out / f"sweep_{sid:03d}.png")
        fig.savefig(written[-1], dpi=110)
        plt.close(fig)
    for p in written:
        log(f"  {p}")
    return written
