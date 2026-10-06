"""Render a synthetic session, calibrate it, and compare with the answer it was built from."""

from __future__ import annotations

import shutil
import tempfile
from pathlib import Path

from .pipeline import calibrate
from .synth import Truth, compare, write_session

# How close a calibration of a synthetic session must come to the truth.
LIMITS = dict(
    wavelength_rms_nm=0.10,        # whole slit, 500-1000 nm
    wavelength_max_nm=0.30,
    grid_wavelength_max_nm=0.30,   # every rectified cell sees its own wavelength
    grid_row_spread_px=0.30,       # every rectified row sees one point of the slit
    laser_error_nm=0.20,
    reflectance_ratio_rms=0.03,
)


def evaluate(errs):
    """[(name, value, limit, ok)] for the errors that have a limit."""
    rows = []
    for k, lim in LIMITS.items():
        if k in errs:
            rows.append((k, errs[k], lim, abs(errs[k]) <= lim))
    if "reflectance_ratio_median" in errs:
        v = errs["reflectance_ratio_median"]
        rows.append(("reflectance_ratio_median", v, 0.01, abs(v - 1) <= 0.01))
    return rows


def run(scale=4.0, keep=None, flip_x=False, transpose=False, log=print):
    base = Path(keep) if keep else Path(tempfile.mkdtemp(prefix="hsical_selftest_"))
    try:
        session, out = base / "session", base / "cal"
        log(f"Rendering a synthetic session (binning {scale:g}) ...")
        write_session(session, Truth(scale=scale, flip_x=flip_x, transpose=transpose),
                      log=lambda *a: None)
        log("Calibrating ...")
        r = calibrate(session, out, log=lambda m: log("  " + m))
        rows = evaluate(compare(r.calibration, session))
        log("\nAgainst the truth:")
        for name, v, lim, ok in rows:
            log(f"  {name:26s} {v:9.4f}   limit {lim:g}   {'ok' if ok else 'FAIL'}")
        good = all(ok for *_, ok in rows) and r.ok
        log("\nSelf-test " + ("passed." if good else "FAILED."))
        if keep:
            log(f"Session and calibration kept in {base}")
        return good
    finally:
        if not keep:
            shutil.rmtree(base, ignore_errors=True)
