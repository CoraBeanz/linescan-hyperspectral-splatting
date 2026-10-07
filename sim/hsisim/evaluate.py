"""End to end: calibrate the simulated instrument, turn its scan into reflectance with hsical,
and compare what comes out with what went in.

    errs = evaluate("sim_out", "sim_out/cal")      # after python -m hsical calibrate sim_out/calibration -o sim_out/cal

It checks the three things a real scan depends on:

  reflectance   every rectified cell that saw one material only, against that material's
                reflectance (the cell's slit position and wavelength come from the instrument's
                true maps, so a calibration error shows up here as a reflectance error)
  wavelengths   the rare-earth tile's absorption bands, fitted in the scan's own spectra,
                against the same fit to the true spectrum blurred to the calibrated resolution
  geometry      where each rectified slit row really looks along the slit, against where its
                row number puts it (h = -1 at the first row, +1 at the last); and each line's
                profile along the slit against the truth's: the scan line lands where its
                pose says, and slit row 0 is the line camera's -x end
"""

from __future__ import annotations

import json
import warnings
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter1d, map_coordinates
from scipy.optimize import OptimizeWarning, curve_fit

from . import repo  # noqa: F401
from . import spectra
from .truth import ScanTruth
from hsical.apply import apply_frames  # noqa: E402
from hsical.model import Calibration  # noqa: E402

LIMITS = dict(
    reflectance_ratio_median=0.01,   # |median - 1|, cells with reflectance over 0.15
    reflectance_ratio_rms=0.05,      # one frame per line, so noisier than a lamp set's average
    reflectance_abs_rms=0.02,
    band_centre_max_nm=0.30,
    slit_row_error_max_px=0.30,      # as hsical's limit for a rectified row's spread along the slit
    slit_shift_px=0.5,
)
BAND_NM = (520.0, 960.0)  # past the filter's edge and short of where the sensor fades out


def _cell_truth(cal, truth):
    """(h, nm) the instrument really sees at every rectified cell."""
    mx, my = cal.maps()
    h_t = cal.orient(truth.h_map).astype(float)
    nm_t = cal.orient(truth.nm_map).astype(float)
    return (map_coordinates(h_t, [my, mx], order=1, cval=np.nan),
            map_coordinates(nm_t, [my, mx], order=1, cval=np.nan))


def slit_row_errors(cal, h_at, band):
    """[slit rows] how far (rows) each rectified row really looks from where its row number
    says along the slit, over the wavelengths in `band`."""
    k = cal.keystone
    half = 0.5 * (k.s_bottom - k.s_top)
    h_nominal = (cal.s_grid - 0.5 * (k.s_bottom + k.s_top)) / half
    with np.errstate(invalid="ignore"):
        return (np.nanmedian(np.where(band[None, :], h_at, np.nan), axis=1) - h_nominal) * half


def apply_sweeps(session, cal_dir, out_dir, sweeps=None, log=print):
    """Run `hsical apply` (reflectance against the session's white reference) on each sweep.

    Returns {sweep_id: (cube [lines, slit rows, nm], meta from its .json)}."""
    session = Path(session)
    white = session / "reference" / "white"
    white_meta = json.loads((white / "meta.json").read_text())
    out = {}
    for d in sorted((session / "frames").iterdir()):
        meta = json.loads((d / "meta.json").read_text())
        sid = int(meta["sweep_id"])
        if sweeps is not None and sid not in sweeps:
            continue
        # strings, as from hsical's command line (its --dark also takes a number)
        cube = apply_frames(str(cal_dir), str(d), dark=str(session / "reference" / "dark"), out=str(out_dir),
                            white=str(white), white_reflectance=white_meta.get("reflectance", 0.98),
                            log=lambda *a: None)
        info = json.loads((Path(out_dir) / f"{d.name}_reflectance.json").read_text())
        out[sid] = (cube, info)
        log(f"  sweep {sid}: {cube.shape[0]} lines -> {cube.shape[1]} slit rows x {cube.shape[2]} wavelengths")
    return out


def _band_fit(nm, y, centre, sigma):
    """Centre (nm) of one absorption band: a Gaussian dip on a sloping continuum."""
    half = max(3.0 * sigma, 12.0)
    m = (nm > centre - half) & (nm < centre + half) & np.isfinite(y)
    x, v = nm[m] - centre, y[m]

    def model(x, a, b, depth, mu, s):
        return (a + b * x) * (1 - depth * np.exp(-0.5 * ((x - mu) / s) ** 2))

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", OptimizeWarning)  # only the centre is used, not its covariance
        p, _ = curve_fit(model, x, v, p0=(v.max(), 0.0, 0.4, 0.0, sigma), maxfev=20000)
    return centre + p[3]


def _profile_shift(meas, true, max_shift=6.0, step=0.05):
    """Shift (rows) that best lines up a measured profile with a true one, and the correlation."""
    r = np.arange(len(true), dtype=float)
    ok = np.isfinite(meas) & np.isfinite(true)
    best = (-np.inf, 0.0)
    for s in np.arange(-max_shift, max_shift + 1e-9, step):
        t = np.interp(r + s, r[ok], true[ok], left=np.nan, right=np.nan)
        m = ok & np.isfinite(t)
        if m.sum() < 20:
            continue
        c = np.corrcoef(meas[m], t[m])[0, 1]
        if c > best[0]:
            best = (c, s)
    return best[1], best[0]


def evaluate(session, cal_dir, spectra_dir=None, sweeps=None, log=print):
    session = Path(session)
    cal = Calibration.load(cal_dir)
    truth = ScanTruth(session)
    spectra_dir = Path(spectra_dir or session / "spectra")
    cubes = apply_sweeps(session, cal_dir, spectra_dir, sweeps, log)
    h_at, nm_at = _cell_truth(cal, truth)
    nm_g = cal.nm_grid
    S, B = h_at.shape
    rows_per_h = 0.5 * (cal.keystone.s_bottom - cal.keystone.s_top)
    # keep 3 sigma of the camera lens blur along the slit, plus the rectifier's smoothing and
    # resampling, away from any edge between materials
    inst = json.loads((session / "scan.json").read_text())["simulated"]["instrument"]
    margin_h = (3.0 * inst["blur_y_px"] / inst["scale"] + 2.5) / rows_per_h
    h_idx = (h_at - truth.h[0]) / (truth.h[1] - truth.h[0])
    cols = np.broadcast_to(np.arange(B, dtype=float), (S, B))
    band = (nm_g > BAND_NM[0]) & (nm_g < BAND_NM[1])
    meas_all, true_all, mat_all = [], [], []
    re_id = truth.materials.index("rare_earth")
    re_sum, re_n = np.zeros(B), np.zeros(B)
    shifts, corrs, flips = [], [], []
    nir = (nm_g > 780) & (nm_g < 920)
    for sid, (cube, _) in cubes.items():
        for k, meas in enumerate(cube):
            line = truth.line_number(sid, k)
            rho = truth.reflectance(line, nm_g)                                  # [h, B]
            want = map_coordinates(rho, [h_idx, cols], order=1, cval=np.nan)    # [S, B]
            pure_h = truth.pure(line, margin_h)
            dom = truth.dominant(line)
            hi = np.clip(np.round(h_idx).astype(int), 0, truth.h.size - 1)
            pure = pure_h[hi] & np.isfinite(h_idx) & (np.abs(h_at) < 0.97)
            sel = pure & band[None, :] & np.isfinite(meas) & np.isfinite(want)
            meas_all.append(meas[sel])
            true_all.append(want[sel])
            mat_all.append(dom[hi][sel])
            re = pure & (dom[hi] == re_id) & np.isfinite(meas)
            re_sum += np.where(re, meas, 0.0).sum(0)
            re_n += re.sum(0)
            # profile along the slit in the near infrared, where the materials differ most
            with np.errstate(invalid="ignore"):
                mp = np.nanmean(np.where(nir[None, :], meas, np.nan), axis=1)
                tp = np.nanmean(np.where(nir[None, :], want, np.nan), axis=1)
            inside = np.abs(np.nanmedian(h_at, axis=1)) < 0.95
            mp, tp = np.where(inside, mp, np.nan), np.where(inside, tp, np.nan)
            if np.nanstd(tp) > 0.02:
                s, c = _profile_shift(mp, tp)
                shifts.append(s)
                corrs.append(c)
                flips.append(_profile_shift(mp, tp[::-1])[1])
    meas, want, mat = (np.concatenate(a) for a in (meas_all, true_all, mat_all))
    bright = want > 0.15
    ratio = meas[bright] / want[bright]
    row_err = slit_row_errors(cal, h_at, band)
    row_err = row_err[np.isfinite(row_err) & (np.abs(np.nanmedian(h_at, axis=1)) < 0.95)]
    errs = dict(
        lines=int(sum(c.shape[0] for c, _ in cubes.values())),
        cells_compared=int(meas.size),
        reflectance_ratio_median=float(np.median(ratio) - 1.0),
        reflectance_ratio_rms=float(np.sqrt(np.mean((ratio - 1.0) ** 2))),
        reflectance_abs_rms=float(np.sqrt(np.mean((meas - want) ** 2))),
        per_material={truth.materials[m]: dict(cells=int((mat == m).sum()),
                                               median_error=float(np.median(meas[mat == m] - want[mat == m])))
                      for m in np.unique(mat)},
        slit_row_error_max_px=float(np.abs(row_err).max()),
        slit_row_error_rms_px=float(np.sqrt(np.mean(row_err ** 2))),
    )
    if re_n.max() > 0:
        avg = np.where(re_n > 0, re_sum / np.maximum(re_n, 1), np.nan)
        lsf = cal.info.get("lsf", {})
        res = float(lsf.get("resolution_nm") or lsf.get("design_resolution_nm") or 4.0)
        fine = np.arange(nm_g[0] - 20, nm_g[-1] + 20, 0.1)
        blurred = gaussian_filter1d(spectra.reflectance("rare_earth", fine), res / 2.3548 / 0.1)
        expect = np.interp(nm_g, fine, blurred)
        bands = []
        for c, sg, _ in spectra.RARE_EARTH_BANDS:
            if not (nm_g[0] + 15 < c < nm_g[-1] - 15) or np.isnan(avg[np.abs(nm_g - c) < 10]).any():
                continue
            got, exp_c = _band_fit(nm_g, avg, c, sg), _band_fit(nm_g, expect, c, sg)
            bands.append(dict(nm=c, measured=float(got), expected=float(exp_c), error=float(got - exp_c)))
        errs["rare_earth_bands"] = bands
        if bands:
            errs["band_centre_max_nm"] = float(max(abs(b["error"]) for b in bands))
    if shifts:
        errs["slit_shift_px"] = float(np.median(np.abs(shifts)))
        errs["slit_profile_correlation"] = float(np.median(corrs))
        errs["slit_profile_correlation_flipped"] = float(np.median(flips))
    return errs


def check(errs):
    """[(name, value, limit, ok)] for the errors that have a limit."""
    rows = []
    for k, lim in LIMITS.items():
        if k in errs:
            rows.append((k, errs[k], lim, abs(errs[k]) <= lim))
    if "slit_profile_correlation" in errs:
        c, f = errs["slit_profile_correlation"], errs.get("slit_profile_correlation_flipped", -1.0)
        rows.append(("slit_profile_correlation", c, 0.9, c >= 0.9 and c > f + 0.2))
    return rows
