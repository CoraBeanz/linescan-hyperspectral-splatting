"""Has the spectrograph moved since it was calibrated? One lamp frame against a saved calibration.

    python -m hsical check cal/ session/cfl              # dark: a matching set next to it, or 64
    python -m hsical check cal/ today/cfl --dark today/dark_60000us --json drift.json

A printed housing shifts a little with temperature, and a knock moves the slit or the
camera, so a CFL (or neon) frame at the start of a scanning session shows whether the
lines are still where the calibration put them. The lines the calibration fitted are
found again, in a few bands along the slit, with the same line-group fit and line shape
the calibration used; each one's shift from where the calibration predicts it (smile
included) gives how far the wavelength scale has moved. A plane through those shifts
separates a plain offset from a tilt along the slit (the camera turned) and a change
across the spectrum (the grating or the camera lens moved). The slit's ends, measured
along each line, give how far the image moved along the slit.

Exits with 0 when everything is within the limits and 2 when not, like calibrate.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation, map_coordinates

from . import geometry as G
from . import lines as L
from . import sensor as SE
from . import wavecal as WC
from .frames import FrameSet, load_session, match_dark, read_frames
from .model import Calibration
from .profiles import binomial_smooth

MAX_NM = 0.2         # worst wavelength error anywhere on the sensor: twice the self-test's rms limit
MAX_SLIT = 0.002     # slit image shift as a share of its length: half of one of the dataset's 256 bins
MIN_SNR = 20.0


@dataclass
class LineShift:
    label: str
    nm: float
    band: int        # 0 .. n_bands - 1, from the slit's top end
    s: float         # slit position of the band, -1 (top end) .. +1
    x_cal: float     # column where the calibration puts the line in this band
    shift_px: float  # measured column minus x_cal: positive toward the red end
    shift_nm: float  # the same in nm: how much too long the calibration now reads wavelengths
    sigma_nm: float  # its uncertainty, from the fit
    snr: float
    used: bool = True  # False: an outlier the plane fit left out


@dataclass
class Drift:
    calibration: str
    frames: str
    source: str
    dark: str
    lines: list = field(default_factory=list)
    offset_nm: float = float("nan")         # the plane's value at the slit's middle and the band's
    along_slit_nm: float = float("nan")     # end-to-end change along the slit (top to bottom)
    across_nm: float = float("nan")         # end-to-end change across the spectrum (blue to red)
    worst_nm: float = float("nan")          # the plane's largest absolute value at the corners
    rms_nm: float = float("nan")            # scatter of the lines about the plane
    offset_px: float = float("nan")
    slit_top_px: float = float("nan")       # slit ends' shift along the slit (rows), median over lines
    slit_bottom_px: float = float("nan")
    slit_shift: float = float("nan")        # mean of the two, as a share of the slit's length
    max_nm: float = MAX_NM
    max_slit: float = MAX_SLIT
    notes: list = field(default_factory=list)

    @property
    def spectral_ok(self):
        return np.isfinite(self.worst_nm) and abs(self.worst_nm) <= self.max_nm

    @property
    def slit_ok(self):
        return not np.isfinite(self.slit_shift) or abs(self.slit_shift) <= self.max_slit

    @property
    def ok(self):
        return self.spectral_ok and self.slit_ok

    def to_dict(self):
        d = asdict(self)
        d["lines"] = [asdict(x) for x in self.lines]
        d.update(ok=self.ok, spectral_ok=self.spectral_ok, slit_ok=self.slit_ok)
        return _finite(d)


def _finite(o):
    if isinstance(o, dict):
        return {k: _finite(v) for k, v in o.items()}
    if isinstance(o, list):
        return [_finite(v) for v in o]
    if isinstance(o, float) and not np.isfinite(o):
        return None
    return o


def _frames(path):
    """(mean frame, saturated pixels, FrameSet or None) of a frame-set folder or one file."""
    p = Path(path)
    if p.is_dir():
        meta_path = p / "meta.json"
        fs = FrameSet(p.name, p, json.loads(meta_path.read_text()) if meta_path.exists() else {})
        mean, sat, _ = fs.summary()
        return mean, sat, fs
    stack = read_frames(p)
    return stack.astype(float).mean(0), (stack >= 1021).any(0), None


def _dark(spec, fs):
    """The dark to subtract and what it was: --dark, else a matching set beside the frames, else 64."""
    if spec is not None:
        try:
            return float(spec), f"black level {float(spec):g}"
        except ValueError:
            mean, _, _ = _frames(spec)
            return mean, f"dark {Path(spec).name}"
    if fs is not None and fs.path.parent.is_dir():
        sets, _ = load_session(fs.path.parent)
        d = match_dark(fs, [s for s in sets.values() if s.kind == "dark"])
        if d is not None:
            return d.summary()[0], f"dark {d.name}"
    return 64.0, "no dark given or found beside the frames: subtracted a black level of 64"


def _lsf(info):
    d = info.get("lsf") or {}
    if "w0" not in d:
        raise ValueError("this calibration has no line shape (lsf) in calibration.json; recalibrate it")
    return WC.LsfModel(**{k: float(d[k]) for k in ("w0", "w1", "s0", "s1", "x0", "xs")})


def _x_on_row(cal, nm, y, xc=None, iters=4):
    """Column at which wavelength nm crosses row y: lambda_c(x - delta(x, y)) = nm.

    xc: the line's column on the middle row, when known better than the wavelength
    table puts it (the calibration's own fit of that line)."""
    nm, y = np.broadcast_arrays(np.asarray(nm, float), np.asarray(y, float))
    xc = np.asarray(cal.wavelength.inverse(nm) if xc is None else np.broadcast_to(xc, nm.shape), float)
    x = xc.copy()
    for _ in range(iters):
        x = xc + cal.smile(x, y)
    return x


def _row_of_s(cal, x, s, iters=4):
    """Row at which slit coordinate s crosses column x: y + Delta(x, y) = s."""
    y = np.full_like(np.asarray(x, float), s)
    for _ in range(iters):
        y = s - cal.keystone.delta(x, y)
    return y


def _end_row(cal, nm, xc, s):
    """Row where the calibration puts the slit end s along the line at nm (its smile and keystone)."""
    y = float(_row_of_s(cal, np.array([xc]), s)[0])
    for _ in range(3):
        y = float(_row_of_s(cal, _x_on_row(cal, np.array([nm]), y, xc), s)[0])
    return y


def _coarse_lag(spec, fwhm, xs, weights, reach):
    """Shift (px) that best lines the spectrum's peaks up with the expected columns xs."""
    p = WC.pattern(spec, fwhm)
    cols = np.arange(p.size, dtype=float)
    lags = np.arange(-reach, reach + 1e-9, 0.1)
    score = [float(np.sum(weights * np.interp(xs + lag, cols, p))) for lag in lags]
    k = int(np.argmax(score))
    return float(lags[k]), k in (0, len(lags) - 1)


def check(cal_dir, frames, dark=None, source=None, n_bands=5, max_nm=MAX_NM, max_slit=MAX_SLIT, log=print):
    """Measure how far a lamp frame's lines sit from where the calibration puts them. Returns a Drift."""
    cal = Calibration.load(cal_dir)
    info = cal.info
    mean, sat, fs = _frames(frames)
    source = source or (fs.source if fs is not None else None)
    if source not in L.LAMPS:
        raise ValueError(f"the frames' lamp is {source!r}; give --source cfl or --source neon")
    d, dark_note = _dark(dark, fs)
    img = cal.orient(mean - d)
    sat = cal.orient(sat)
    H, W = img.shape
    if (W, H) != (cal.width, cal.height):
        raise ValueError(f"the frames are {W} x {H} (spectrum x slit) but the calibration is for "
                         f"{cal.width} x {cal.height}: take them in the same sensor mode")
    out = Drift(calibration=str(cal_dir), frames=str(frames), source=source, dark=dark_note,
                max_nm=max_nm, max_slit=max_slit)
    if cal.bad is not None:
        img = SE.fill_bad(img, cal.bad)
        sat = sat & ~cal.bad
    if cal.response is not None:
        # take out the colour mosaic the way calibrate did, so the lines are measured on the
        # same footing: the response map carries the flat's pixel-to-pixel pattern
        resp = np.nan_to_num(np.asarray(cal.response, float))
        img = img / SE.cfa_gain(SE.fill_bad(resp, cal.bad) if cal.bad is not None else resp)
    img = binomial_smooth(img)

    lsf = _lsf(info)
    fwhm_px = float(info["lsf"].get("fwhm_px") or np.median(lsf.fwhm(np.array([0.25, 0.5, 0.75]) * W)))
    fwhm_nm = float(info["lsf"].get("resolution_nm") or fwhm_px * abs(float(cal.wavelength.deriv(W / 2))))
    tbl = cal.wavelength
    nm_lo, nm_hi = sorted([float(tbl.values[0]), float(tbl.values[-1])])
    groups = L.groups(L.catalog(source), fwhm_nm, nm_range=(max(nm_lo, 470.0), min(nm_hi, 1030.0)))
    # The lines the calibration itself fitted and kept: where it found each one on the middle
    # row (from the lamp set of the same name if it has one, as `cfl` rather than `cfl_long`)
    # and how bright. Measuring against those positions rather than the smooth wavelength
    # table leaves out each line's own small misfit (a blend, a catalog error).
    fitted = {}
    for f in info.get("lines") or []:
        lamp = str(f.get("lamp", ""))
        if f.get("used") and lamp.startswith(source) and f.get("x") is not None:
            prev = fitted.get(f["label"])
            if prev is None or (lamp == source and prev[2] != source):
                fitted[f["label"]] = (float(f["x"]), float(f.get("amp") or 0.0), lamp)
    if fitted:
        groups = [g for g in groups if g.label in fitted]
    if len(groups) < 3:
        raise RuntimeError(f"only {len(groups)} {source} lines to look for in this calibration; "
                           "was it made with this lamp?")
    x_fit = {g.label: (fitted[g.label][0] if g.label in fitted else float(tbl.inverse(g.ref.nm))) for g in groups}
    weights = np.array([fitted[g.label][1] if g.label in fitted else 1.0 for g in groups])
    weights = np.sqrt(np.clip(weights, 0, None) / max(weights.max(), 1e-12))

    top, bot = (float(v) for v in info.get("slit_rows") or (cal.keystone.s_top, cal.keystone.s_bottom))
    edges = np.linspace(top + 3.0, bot - 3.0, n_bands + 1)
    y_mid = 0.5 * (edges[:-1] + edges[1:])
    s_mid = (y_mid - 0.5 * (top + bot)) / (0.5 * (bot - top))

    # one shift for the whole frame first, from the middle band, so a moved instrument
    # still gets each line fitted around where it really is
    lo, hi = edges[n_bands // 2], edges[n_bands // 2 + 1]
    spec, _ = WC.band_spectrum(img, lo, hi)
    x_pred = np.array([_x_on_row(cal, g.ref.nm, y_mid[n_bands // 2], x_fit[g.label]) for g in groups], float)
    lag, at_edge = _coarse_lag(spec, fwhm_px, x_pred, weights, reach=2.0 * fwhm_px)
    if at_edge:
        out.notes.append(f"the lines moved more than {2 * fwhm_px:.0f} px (two line widths), "
                         "too far to follow: recalibrate")
    dxdnm = (lambda nm: 1.0 / tbl.deriv(tbl.inverse(nm)))
    for b in range(n_bands):
        lo, hi = edges[b], edges[b + 1]
        spec, noise = WC.band_spectrum(img, lo, hi)
        satc = binary_dilation(sat[int(lo):int(np.ceil(hi)) + 1].any(0), iterations=3)
        yb = float(y_mid[b])

        def x_of_nm(nm, yb=yb):
            return _x_on_row(cal, nm, yb) + lag

        for g in groups:
            f = WC.fit_group(spec, noise, g, x_of_nm, dxdnm, lsf, max_shift=max(0.6 * fwhm_px, 1.5), sat=satc)
            if f is None or f.snr < MIN_SNR:
                continue
            x_cal = float(_x_on_row(cal, g.ref.nm, yb, x_fit[g.label]))
            shift = f.x - x_cal
            nm_per_px = float(tbl.deriv(f.x - cal.smile(f.x, yb)))
            out.lines.append(LineShift(label=g.label, nm=g.ref.nm, band=b, s=float(s_mid[b]), x_cal=x_cal,
                                       shift_px=float(shift), shift_nm=float(shift * nm_per_px),
                                       sigma_nm=float(max(f.x_sigma, 0.01) * abs(nm_per_px)), snr=float(f.snr)))
    if len(out.lines) < 6:
        out.notes.append(f"only {len(out.lines)} line measurements: check the lamp and its exposure")
        if not out.lines:
            return out
    _fit_plane(out, nm_lo, nm_hi)
    _slit_ends(out, cal, img, fwhm_px, top, bot, n_bands // 2, x_fit)
    return out


def _fit_plane(out, nm_lo, nm_hi):
    """shift_nm = c0 + c1 s + c2 v, with s along the slit and v across the band, both -1..1.

    Weighted by each line's own uncertainty: a bright line pins its position to a few
    hundredths of a pixel, a faint one only to a few tenths."""
    s = np.array([x.s for x in out.lines])
    nm = np.array([x.nm for x in out.lines])
    d = np.array([x.shift_nm for x in out.lines])
    sd = np.array([x.sigma_nm for x in out.lines])
    c_nm, h_nm = 0.5 * (nm_lo + nm_hi), 0.5 * (nm_hi - nm_lo)
    v = (nm - c_nm) / h_nm
    A = np.column_stack([np.ones_like(s), s, v])
    keep = np.ones(d.size, bool)
    for _ in range(4):  # drop lines a blend or a faint neighbour threw off
        c, *_ = np.linalg.lstsq(A[keep] / sd[keep, None], d[keep] / sd[keep], rcond=None)
        r = d - A @ c
        chi = np.abs(r) / sd
        scale = max(1.4826 * float(np.median(chi[keep])), 1.0)
        new = chi < 4 * scale
        if new.sum() < 4 or (new == keep).all():
            break
        keep = new
    for x, k in zip(out.lines, keep):
        x.used = bool(k)
    # the band's extremes: the lines actually measured, not the whole table
    v_lo, v_hi = float(v[keep].min()), float(v[keep].max())
    corners = [c[0] + c[1] * a + c[2] * b for a in (-1.0, 1.0) for b in (v_lo, v_hi)]
    out.offset_nm = float(c[0])
    out.along_slit_nm = float(2 * c[1])
    out.across_nm = float(c[2] * (v_hi - v_lo))
    out.worst_nm = float(max(corners, key=abs))
    w = 1.0 / sd[keep] ** 2
    out.rms_nm = float(np.sqrt(np.sum(w * r[keep] ** 2) / np.sum(w)))
    px = np.array([x.shift_px for x in out.lines])
    out.offset_px = float(np.sum(w * px[keep]) / np.sum(w))


def _slit_ends(out, cal, img, fwhm_px, top, bot, mid_band, x_fit):
    """Where the slit's ends are along each strong line, against the calibration's keystone."""
    H, W = img.shape
    a = max(6, int(round(12 * H / 2464.0)))
    rows = np.arange(H, dtype=float)
    half = max(1, int(round(0.4 * fwhm_px)))
    mid = [x for x in out.lines if x.band == mid_band]
    tops, bots = [], []
    for ln in sorted(mid, key=lambda x: -x.snr)[:12]:
        x_line = _x_on_row(cal, ln.nm, rows, x_fit[ln.label]) + ln.shift_px   # follow its smile
        cols = x_line[:, None] + np.arange(-half, half + 1)[None, :]
        prof = map_coordinates(img, [np.repeat(rows[:, None], cols.shape[1], 1), cols], order=1,
                               mode="nearest").mean(1)
        t, b, _ = G.slit_edges(prof[:, None], plateau_px=(a, 5 * a))
        if np.isfinite(t[0]):
            tops.append(t[0] - _end_row(cal, ln.nm, x_fit[ln.label], cal.keystone.s_top))
        if np.isfinite(b[0]):
            bots.append(b[0] - _end_row(cal, ln.nm, x_fit[ln.label], cal.keystone.s_bottom))
    if len(tops) >= 3 and len(bots) >= 3:
        out.slit_top_px, out.slit_bottom_px = float(np.median(tops)), float(np.median(bots))
        out.slit_shift = 0.5 * (out.slit_top_px + out.slit_bottom_px) / (bot - top)
    else:
        out.notes.append("the slit's ends didn't show clearly along the lines: slit shift not measured")


def report(r: Drift, log=print):
    log(f"{r.source} frames {r.frames} against {r.calibration} ({r.dark})")
    if not r.lines:
        for n in r.notes:
            log(f"  ! {n}")
        log("No lamp lines measured.")
        return
    bands = sorted({x.band for x in r.lines})
    log(f"  {len(r.lines)} line measurements in {len(bands)} bands along the slit")
    for b in bands:
        xs = [x for x in r.lines if x.band == b]
        w = np.array([1.0 / x.sigma_nm ** 2 for x in xs])
        mean_px = float(np.sum(w * [x.shift_px for x in xs]) / w.sum())
        mean_nm = float(np.sum(w * [x.shift_nm for x in xs]) / w.sum())
        log(f"    slit {xs[0].s:+.2f}: {len(xs):2d} lines, shift {mean_px:+.2f} px = {mean_nm:+.3f} nm "
            f"(weighted by each line's precision)")
    log(f"  wavelength offset    {r.offset_nm:+.3f} nm ({r.offset_px:+.2f} px; + means the lines moved toward red)")
    log(f"  along the slit       {r.along_slit_nm:+.3f} nm end to end (camera or slit turned)")
    log(f"  across the spectrum  {r.across_nm:+.3f} nm blue to red (grating or camera lens moved)")
    log(f"  worst anywhere       {r.worst_nm:+.3f} nm, limit {r.max_nm:g}   {'ok' if r.spectral_ok else 'MOVED'}")
    log(f"  lines about the fit  {r.rms_nm:.3f} nm rms (weighted), "
        f"{sum(not x.used for x in r.lines)} outlier(s) left out")
    if np.isfinite(r.slit_shift):
        log(f"  slit ends            {r.slit_top_px:+.2f} / {r.slit_bottom_px:+.2f} rows (top / bottom), "
            f"{100 * r.slit_shift:+.3f}% of the slit, limit {100 * r.max_slit:g}%   "
            f"{'ok' if r.slit_ok else 'MOVED'}")
    for n in r.notes:
        log(f"  ! {n}")
    if r.ok:
        log("Still calibrated: the lines are where the calibration puts them.")
    else:
        log("The instrument has moved since it was calibrated: run a new calibration session before scanning.")
