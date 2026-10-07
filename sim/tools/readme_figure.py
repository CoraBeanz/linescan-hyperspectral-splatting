"""Draw img/simulator.png for the README from a scan of the relief target.

    python -m hsisim scan ring_scan --plan ring
    python -m hsical calibrate ring_scan/calibration -o ring_cal       (from calibration/)
    python -m hsisim evaluate ring_scan ring_cal                       (writes ring_scan/spectra/)
    python tools/readme_figure.py ring_scan
"""

import io
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

from hsisim import spectra  # noqa: E402
from hsisim.preview import _image, _plt, footprints, top_view  # noqa: E402
from hsisim.scenes import make  # noqa: E402
from hsisim.truth import ScanTruth  # noqa: E402


def main(session, out=HERE / "img" / "simulator.png", sweep=1):
    plt = _plt()
    session = Path(session)
    info = json.loads((session / "scan.json").read_text())
    sim = info["simulated"]
    truth = ScanTruth(session)
    target = json.loads((session / "truth" / "scene.json").read_text())["target"]
    scene = make(sim["scene"], tuple(target), sim["lighting"]["type"])
    rgb, _, ext = top_view(scene, half=0.034, step=0.0001)
    ends = footprints(truth, sim["objective"]["half_line_m"], sim["objective"]["focus_mm"] * 1e-3)
    mine = truth.lines["sweep_id"] == sweep
    name = f"sweep_{sweep:03d}"
    files = sorted((session / "frames" / name).glob("frame_*.npy"))
    k = len(files) // 2
    frame = np.load(files[k]).astype(float)
    lines = [truth.line_number(sweep, i) for i in range(len(files))]
    inside = np.abs(truth.h) <= 1.0
    seen = spectra.true_color(np.stack([truth.reflectance(i)[inside] for i in lines], 1), truth.nm)
    cube = np.load(session / "spectra" / f"{name}_reflectance.npy")
    nm = np.asarray(json.loads((session / "spectra" / f"{name}_reflectance.json").read_text())["nm"])
    got = spectra.true_color(np.nan_to_num(cube.transpose(1, 0, 2)), nm)

    fig, ax = plt.subplots(1, 4, figsize=(15, 4.6), gridspec_kw=dict(width_ratios=[1.0, 1.35, 0.62, 0.62]))
    ax[0].imshow(_image(rgb), extent=[1e3 * e for e in ext])
    for i in np.flatnonzero(mine)[::2]:
        a, b = ends[i]
        ax[0].plot(1e3 * np.array([a[0], b[0]]), 1e3 * np.array([a[1], b[1]]), color="#00e5ff", lw=0.5, alpha=0.7)
    a, b = ends[np.flatnonzero(mine)[k]]
    ax[0].plot(1e3 * np.array([a[0], b[0]]), 1e3 * np.array([a[1], b[1]]), color="#ff2d55", lw=1.6)
    ax[0].set_title("Relief target from above, with every\nsecond scan line of the first sweep", fontsize=9)
    ax[0].set_xlabel("base_link x (mm)", fontsize=8)
    ax[0].set_ylabel("base_link y (mm)", fontsize=8)
    shown = np.sqrt(np.clip(frame - 64, 0, None))
    ax[1].imshow(shown, cmap="gray", interpolation="antialiased", aspect="auto")
    ax[1].set_title("Raw 10-bit frame of the red line: spectrum across,\nslit down (inset: the NoIR sensor's mosaic)",
                    fontsize=9)
    ax[1].set_xlabel("sensor column", fontsize=8)
    ax[1].set_ylabel("sensor row", fontsize=8)
    r0, c0, n = 300 if frame.shape[0] > 400 else frame.shape[0] // 2, frame.shape[1] // 2, 24
    inset = ax[1].inset_axes([0.66, 0.6, 0.32, 0.37])
    inset.imshow(shown[r0:r0 + n, c0:c0 + n], cmap="gray", interpolation="nearest")
    inset.set_xticks([])
    inset.set_yticks([])
    ax[1].add_patch(plt.Rectangle((c0, r0), n, n, fill=False, color="#ff2d55", lw=1))
    for a_, img, title in ((ax[2], seen, "What the slit saw\n(truth)"),
                           (ax[3], got, "After hsical calibrate\nand apply")):
        a_.imshow(spectra.to_srgb8(img), aspect="auto", interpolation="nearest")
        a_.set_title(title, fontsize=9)
        a_.set_xlabel("scan line", fontsize=8)
        a_.set_yticks([])
    ax[2].set_ylabel("along the slit", fontsize=8)
    for a_ in ax:
        a_.tick_params(labelsize=7)
    fig.tight_layout()
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    buf = io.BytesIO()
    fig.savefig(buf, dpi=100)
    plt.close(fig)
    from PIL import Image  # comes with matplotlib
    Image.open(buf).convert("RGB").quantize(256).save(out, optimize=True)  # a palette keeps it small
    print(out, f"{out.stat().st_size // 1024} kB")


if __name__ == "__main__":
    main(*sys.argv[1:])
