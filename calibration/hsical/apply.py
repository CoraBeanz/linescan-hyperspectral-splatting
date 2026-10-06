"""Apply a calibration to raw frames: rectified radiance or reflectance, saved as .npy."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .frames import FrameSet, read_frames
from .model import Calibration


def _load(path):
    """(frames (N, H, W), exposure_us, gain) from a frame-set folder or a single frame file."""
    p = Path(path)
    if p.is_dir():
        meta_path = p / "meta.json"
        meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
        fs = FrameSet(p.name, p, meta)
        stack = np.concatenate([read_frames(f, meta) for f in fs.files()]).astype(float)
        return stack, fs.exposure_us, fs.gain
    return read_frames(p).astype(float), None, 1.0


def _dark(spec):
    if spec is None:
        return 64.0
    try:
        return float(spec)
    except ValueError:
        frames, _, _ = _load(spec)
        return frames.mean(0)


def apply_frames(cal_dir, frames, dark=None, out="spectra", white=None, white_dark=None,
                 white_reflectance=0.98, log=print):
    cal = Calibration.load(cal_dir)
    stack, exp, gain = _load(frames)
    if exp is None:
        log("  ! no exposure known for these frames (no meta.json): radiance is per 1 s")
        exp = 1e6
    d = _dark(dark)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    name = Path(frames).stem
    if white:
        wstack, wexp, wgain = _load(white)
        wmean = wstack.mean(0)
        wd = _dark(white_dark) if white_dark else d
        cube = np.stack([cal.reflectance(f, d, exp, wmean, wd, wexp or exp, gain, wgain, white_reflectance)
                         for f in stack])
        what, unit = "reflectance", f"fraction (white reference = {white_reflectance})"
    else:
        cube = np.stack([cal.radiance(f, d, exp, gain) for f in stack])
        what, unit = "radiance", f"relative to a {cal.temp_k:.0f} K blackbody, 1 at 700 nm"
    np.save(out / f"{name}_{what}.npy", cube.astype(np.float32))
    (out / f"{name}_{what}.json").write_text(json.dumps(dict(
        quantity=what, unit=unit, shape=["frame", "slit row", "wavelength"],
        nm=cal.nm_grid.tolist(), slit_rows=cal.s_grid.tolist(), source=str(frames)), indent=1))
    log(f"  {stack.shape[0]} frame(s) -> {out / (name + '_' + what + '.npy')} {cube.shape}")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 5))
        v = np.nanpercentile(cube[0], 99.5)
        ax.imshow(cube[0], aspect="auto", cmap="magma", vmin=0, vmax=v,
                  extent=[cal.nm_grid[0], cal.nm_grid[-1], cal.s_grid[-1], cal.s_grid[0]])
        ax.set_xlabel("wavelength (nm)")
        ax.set_ylabel("slit row")
        ax.set_title(f"{name}: {what}, frame 0")
        fig.tight_layout()
        fig.savefig(out / f"{name}_{what}.png", dpi=100)
        plt.close(fig)
    except ImportError:
        pass
    return cube
