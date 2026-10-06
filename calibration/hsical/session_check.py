"""A quick look at a session before calibrating: what is there, and is it exposed right."""

from __future__ import annotations

import numpy as np

from .frames import load_session, match_dark


def inspect_session(path, log=print):
    """Print a table of the frame sets and any problems; True if nothing blocks a calibration."""
    sets, cfg = load_session(path)
    errors, warnings = [], []
    if not sets:
        log(f"no frame sets in {path}")
        return False
    darks = [s for s in sets.values() if s.kind == "dark"]
    sizes = set()
    log(f"{'set':14s} {'kind':6s} {'source':13s} {'n':>3s} {'exposure':>10s} {'gain':>5s} "
        f"{'peak':>6s} {'sat %':>7s}  dark")
    for name, s in sets.items():
        mean, sat, n = s.summary()
        sizes.add(mean.shape)
        full = s.max_dn
        peak = float(np.percentile(mean, 99.95))
        frac = (peak - 64.0) / (full - 64.0)
        satp = 100.0 * float(sat.mean())
        exp = "?" if s.exposure_us is None else f"{s.exposure_us / 1000:.1f} ms"
        d = match_dark(s, darks) if s.kind != "dark" else None
        dark = "" if s.kind == "dark" else (d.name if d else "none at this exposure")
        log(f"{name:14s} {s.kind:6s} {str(s.source or ''):13s} {n:3d} {exp:>10s} {s.gain:5.2g} "
            f"{100 * frac:5.0f}% {satp:7.3f}  {dark}")
        long_set = "long" in name
        if s.kind != "dark" and d is None:
            warnings.append(f"{name}: no dark at {exp}, gain {s.gain:g}; the calibration will scale "
                            "one from other exposures")
        if s.kind == "flat" and satp > 0.01:
            errors.append(f"{name}: {satp:.2f}% of pixels saturated; shorten the exposure")
        if s.kind in ("lamp", "laser") and not long_set and satp > 0.05:
            warnings.append(f"{name}: {satp:.2f}% saturated; those lines will be skipped "
                            "(fine for a *_long set, not for the main one)")
        if s.kind in ("lamp", "flat", "laser") and frac < 0.2:
            warnings.append(f"{name}: brightest pixels only reach {100 * frac:.0f}% of full scale; "
                            "lengthen the exposure")
        if s.kind == "dark" and frac > 0.3:
            warnings.append(f"{name}: a dark frame reaching {100 * frac:.0f}% of full scale; is the lens capped?")
        if s.meta.get("guessed"):
            warnings.append(f"{name}: no meta.json; kind guessed from the folder name, exposure unknown")
    kinds = {s.kind for s in sets.values()}
    lamps = [s for s in sets.values() if s.kind == "lamp"]
    if not lamps:
        errors.append("no lamp set (kind 'lamp', source 'cfl' or 'neon'): nothing to fit wavelengths to")
    if "flat" not in kinds:
        warnings.append("no flat (halogen on PTFE): no keystone, mosaic correction or spectral response")
    if "wires" not in kinds:
        warnings.append("no wires set: keystone from the slit ends only")
    if "laser" not in kinds:
        warnings.append("no laser set: one fewer cross-check")
    if not any("long" in s.name for s in lamps):
        warnings.append("no *_long lamp set: few lines past 900 nm, so the red end is extrapolated")
    if not darks:
        warnings.append("no darks: a flat black level of 64 is subtracted instead")
    if len(sizes) > 1:
        errors.append(f"frame sets differ in size: {sorted(sizes)}")
    for w in warnings:
        log(f"  note: {w}")
    for e in errors:
        log(f"  PROBLEM: {e}")
    if not errors:
        log("Ready to calibrate.")
    return not errors
