"""The whole calibration, from a session folder of raw frames to a saved Calibration.

    python -m hsical calibrate session/ -o cal/

Steps, each logged as it runs:

 1. Read every frame set, average its frames, subtract the matching dark.
 2. Work out which way the camera is mounted (spectrum along x or y, blue
    on the left or right) so everything after sees blue-left, slit vertical.
 3. Hot and dead pixels from the long dark and the flat; the colour mosaic's
    pattern from the flat.
 4. The slit's ends and middle row from the flat.
 5. Wavelengths along the middle row from the lamps (wavecal.py).
 6. Smile: lamp lines followed along the slit.
 7. Keystone: slit ends and wire shadows followed across the spectrum.
 8. Spectral response from the flat and the halogen's temperature.
 9. The laser, as a check: one wavelength everywhere along the slit, and the
    line width, which is the resolution.
10. Pass/fail checks, then calibration.json, maps.npz and a report.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation

from . import geometry as G
from . import response as RS
from . import sensor as SE
from . import wavecal as WC
from .design import FULL_SENSOR, DesignMap
from .frames import Orientation, load_session, spectrum_is_vertical
from .model import Calibration
from .profiles import binomial_smooth, fit_slit_profile

DEFAULT_TEMP_K = 2850.0


@dataclass
class Check:
    name: str
    ok: bool
    value: str
    target: str
    advice: str = ""
    critical: bool = True   # False: worth knowing, but the calibration is still good

    @property
    def status(self):
        return "pass" if self.ok else ("FAIL" if self.critical else "note")

    def to_dict(self):
        return dict(name=self.name, status=self.status, value=self.value, target=self.target,
                    advice=self.advice)


@dataclass
class Result:
    calibration: Calibration
    checks: list
    plots: dict = field(default_factory=dict)   # arrays the report draws
    log: list = field(default_factory=list)

    @property
    def ok(self):
        return all(c.ok or not c.critical for c in self.checks)


def guess_scale(width):
    """Sensor binning from the frame width (3280 or 3264 = full, 1640 = 2x2, 820 = 4x4)."""
    for s in (1, 2, 4):
        if abs(FULL_SENSOR[0] / s - width) / width < 0.03:
            return s
    return FULL_SENSOR[0] / float(width)


class _Frames:
    """Oriented, dark-subtracted means of every frame set, kept as float32."""

    def __init__(self, sets, orientation, log):
        self.sets = sets
        self.o = orientation
        self.darks = [s for s in sets.values() if s.kind == "dark"]
        self.img, self.sat, self.notes = {}, {}, {}
        for name, s in sets.items():
            if s.kind == "dark":
                continue
            mean, sat, n = s.summary()
            dark, note = SE.dark_for(s, self.darks)
            a = mean - (dark if dark is not None else 64.0)
            self.img[name] = orientation.apply(a).astype(np.float32)
            self.sat[name] = orientation.apply(sat)
            self.notes[name] = note
            frac = float(sat.mean())
            log(f"  {name:12s} {s.kind:6s} {n:2d} frames, {self._exp(s)}, {note}"
                + (f", {100 * frac:.2f}% pixels saturated" if frac > 1e-5 else ""))

    @staticmethod
    def _exp(s):
        e = s.exposure_us
        return ("exposure ?" if e is None else f"{e / 1000:.1f} ms") + \
            ("" if s.gain == 1.0 else f" gain {s.gain:g}")

    def flip_x(self):
        for d in (self.img, self.sat):
            for k in d:
                d[k] = np.ascontiguousarray(d[k][:, ::-1])


def _pick(sets, kind, prefer=None):
    c = [s for s in sets.values() if s.kind == kind]
    if prefer:
        c.sort(key=lambda s: (s.name != prefer, s.name))
    return c


def _slit_rows(flat_s, lamp_s, W):
    """Top and bottom rows of the slit near the middle columns."""
    lo, hi = int(0.3 * W), int(0.7 * W)
    src = flat_s if flat_s is not None else lamp_s
    prof = src[:, lo:hi].mean(1, keepdims=True)
    H = src.shape[0]
    a = max(6, int(round(12 * H / 2464.0)))
    top, bot, _ = G.slit_edges(prof, plateau_px=(a, 5 * a))
    if not (np.isfinite(top[0]) and np.isfinite(bot[0])):
        raise RuntimeError("could not find the slit's ends in the flat; is the halogen on and the "
                           "exposure below saturation?")
    return float(top[0]), float(bot[0])


def calibrate(session, out_dir=None, temp_k=None, nm_step=2.0, scale=None, flip_y=None,
              transpose=None, lamp_names=None, report=True, log=print):
    """Calibrate from a session folder. Returns a Result; saves it when out_dir is given."""
    t0 = time.time()
    lines = []

    def say(msg):
        lines.append(msg)
        log(msg)

    sets, cfg = load_session(session)
    if not sets:
        raise RuntimeError(f"no frame sets in {session} (folders with frames and a meta.json)")
    temp_k = float(temp_k or cfg.get("halogen_k") or DEFAULT_TEMP_K)
    lamps = [s for s in _pick(sets, "lamp") if s.source in ("cfl", "neon")
             and (lamp_names is None or s.name in lamp_names)]
    if not lamps:
        raise RuntimeError("no lamp frames: the wavelength fit needs a CFL and/or neon set "
                           "(meta.json kind 'lamp', source 'cfl' or 'neon')")
    flats = _pick(sets, "flat", "flat")
    wires = _pick(sets, "wires", "wires")
    lasers = _pick(sets, "laser", "laser")
    flat = flats[0] if flats else None
    checks = []

    # 1-2. frames and orientation ----------------------------------------
    say("Reading frames")
    first = lamps[0]
    mean, _, _ = first.summary()
    d, _ = SE.dark_for(first, [s for s in sets.values() if s.kind == "dark"])
    if transpose is None:
        transpose = cfg.get("transpose")
    if transpose is None:
        transpose = bool(spectrum_is_vertical(mean - (d if d is not None else 64.0)))
    if flip_y is None:
        flip_y = bool(cfg.get("flip_y", False))
    orient = Orientation(transpose=bool(transpose), flip_x=bool(cfg.get("flip_x", False)), flip_y=flip_y)
    if transpose:
        say("  the spectrum runs along the camera's y axis; frames are turned 90 degrees")
    F = _Frames(sets, orient, say)
    H, W = F.img[first.name].shape
    scale = float(scale or cfg.get("binning") or guess_scale(W))
    design = DesignMap(width=W, height=H, scale=scale)
    say(f"  frames are {W} x {H} (spectrum x slit), treated as binning {scale:g}")

    # 3. sensor ------------------------------------------------------------
    say("Sensor")
    long_dark = max(F.darks, key=lambda s: (s.exposure_us or 0) * s.gain, default=None)
    ld = orient.apply(long_dark.summary()[0]) if long_dark is not None else None
    flat_img = F.img[flat.name] if flat else None
    bad = SE.find_bad_pixels(ld, flat_img, shape=(H, W))
    say(f"  {bad.hot} hot and {bad.odd} odd pixels masked")

    def saturated(name):
        # bad pixels get filled from their neighbours, so a hot pixel stuck at full
        # scale in a long exposure must not knock out its column
        return F.sat[name] & ~bad.mask

    gain_map = None
    if flat is not None:
        flat_filled = SE.fill_bad(flat_img, bad.mask)
        gain_map = SE.cfa_gain(flat_filled, saturated(flat.name))
        sat_frac = float(saturated(flat.name).mean())
        checks.append(Check("flat exposure", sat_frac < 1e-4, f"{100 * sat_frac:.3f}% saturated",
                            "no saturated pixels", "shorten the flat's exposure"))
    else:
        say("  ! no flat: no mosaic correction, keystone or spectral response")

    def clean(name, equalize=True):
        a = SE.fill_bad(F.img[name], bad.mask)
        if equalize and gain_map is not None:
            a = a / gain_map
        return binomial_smooth(a)

    lamp_s = {s.name: clean(s.name) for s in lamps}
    flat_s = clean(flat.name, equalize=False) if flat else None

    # 4-5. slit rows and wavelengths ---------------------------------------
    def wavelengths():
        top, bot = _slit_rows(flat_s, sum(lamp_s.values()), W)
        y0, half_len = 0.5 * (top + bot), 0.5 * (bot - top)
        hb = max(4.0, 0.04 * (bot - top))
        spectra, satc = {}, {}
        for s in lamps:
            spectra[s.name] = WC.band_spectrum(lamp_s[s.name], y0 - hb, y0 + hb)
            sc = saturated(s.name)[max(int(y0 - hb) - 2, 0):int(y0 + hb) + 3].any(0)
            satc[s.name] = binary_dilation(sc, iterations=3)
        hints = cfg.get("hints", {})
        disp = WC.solve_dispersion(spectra, design, hints=hints, sat=satc, log=say)
        return top, bot, y0, half_len, hb, spectra, satc, disp

    say("Slit and wavelengths")
    top, bot, y0, half_len, hb, spectra, satc, disp = wavelengths()
    say(f"  slit ends at rows {top:.1f} and {bot:.1f} (design {design.slit_rows()[0]:.1f}, "
        f"{design.slit_rows()[1]:.1f})")
    if disp.flipped:
        say("  blue is on the right: mirroring the frames and fitting again")
        orient.flip_x = not orient.flip_x
        F.flip_x()
        bad.mask = np.ascontiguousarray(bad.mask[:, ::-1])
        if gain_map is not None:
            gain_map = np.ascontiguousarray(gain_map[:, ::-1])
        lamp_s = {k: np.ascontiguousarray(v[:, ::-1]) for k, v in lamp_s.items()}
        flat_s = None if flat_s is None else np.ascontiguousarray(flat_s[:, ::-1])
        top, bot, y0, half_len, hb, spectra, satc, disp = wavelengths()
        if disp.flipped:
            raise RuntimeError("the lamp spectrum matched the catalog only when mirrored, twice; "
                               "check the lamp set's 'source' in meta.json")
    tbl = disp.table
    fwhm_px = float(np.median(disp.lsf.fwhm(np.array([0.25, 0.5, 0.75]) * W)))
    disp_750 = 1.0 / float(tbl.deriv(tbl.inverse(750.0)))  # px per nm
    res_nm = fwhm_px / disp_750
    say(f"  {sum(f.used for f in disp.fits)} lines from {disp.nm_covered[0]:.0f} to "
        f"{disp.nm_covered[1]:.0f} nm, fit rms {disp.rms_nm:.3f} nm; line width {fwhm_px:.1f} px "
        f"= {res_nm:.1f} nm; {disp_750:.2f} px/nm at 750 nm")
    sig = tbl.uncertainty(np.arange(W))
    checks.append(Check("wavelength fit", disp.rms_nm < 0.3 and len(disp.fits) >= 8,
                        f"{disp.rms_nm:.3f} nm rms over {sum(f.used for f in disp.fits)} lines",
                        "under 0.3 nm, 8+ lines", "check focus and lamp exposure; see the line table"))
    lo_nm, hi_nm = disp.nm_covered
    checks.append(Check("lines cover the band", lo_nm < 560 and hi_nm > 850,
                        f"{lo_nm:.0f}-{hi_nm:.0f} nm", "from below 560 to above 850 nm",
                        "add a long neon exposure for the near infrared lines"))

    # 6. smile -----------------------------------------------------------------
    say("Smile")
    y_lo, y_hi = top + 3, bot - 3
    n_bins = int(round((y_hi - y_lo) / max(4.0, 0.02 * (y_hi - y_lo))))
    Xs, Fs = [], []
    for s in lamps:
        bands, yb = WC.bin_rows(lamp_s[s.name], y_lo, y_hi, n_bins)
        sb, _ = WC.bin_rows(saturated(s.name).astype(float), y_lo, y_hi, n_bins)
        bands[binary_dilation(sb > 0, structure=np.ones((1, 7)))] = np.nan
        b0 = int(np.argmin(np.abs(yb - y0)))
        xs = WC.pick_trace_lines(*spectra[s.name], fwhm_px, sat=satc[s.name])
        X, Fl = WC.trace_lines(bands, xs, b0, 0.9 * fwhm_px, 0.6 * fwhm_px)
        Xs.append(X)
        Fs.append(Fl)
    smile = WC.fit_smile(np.vstack(Xs), np.vstack(Fs), yb, W, y0, half_len, max_rms=0.1 * fwhm_px)
    if smile.dropped:
        say(f"  {smile.dropped} trace(s) dropped: too ragged to be a single line")
    end_bow = float(smile.delta(tbl.inverse(900.0), top) if np.isfinite(top) else np.nan)
    say(f"  {len(smile.traces)} lines followed, fit rms {smile.rms_px:.2f} px; lines bow "
        f"{end_bow:+.1f} px at the slit's end (900 nm)")
    checks.append(Check("smile fit", smile.rms_px < 0.3 * fwhm_px and len(smile.traces) >= 6,
                        f"{smile.rms_px:.2f} px rms, {len(smile.traces)} lines",
                        f"under {0.3 * fwhm_px:.1f} px, 6+ lines", "lamp lines too faint near the slit ends?"))

    # 7. keystone --------------------------------------------------------------
    say("Keystone")
    x_ref = float(tbl.inverse(750.0))
    good_cols = np.flatnonzero((tbl.values > 500) & (tbl.values < 1000))
    x_lo, x_hi = int(good_cols[0]), int(good_cols[-1]) + 1
    if flat_s is not None:
        wires_s = clean(wires[0].name, equalize=False) if wires else None
        ks, edge_info = G.measure_keystone(flat_s, wires_s, x_lo=x_lo, x_hi=x_hi, x_ref=x_ref,
                                           bin_px=max(4, 16 / scale), log=say)
        n_w = sum(t["kind"] == "wire" for t in ks.tracks)
        checks.append(Check("keystone fit", ks.rms_px < 0.5, f"{ks.rms_px:.2f} px rms, {n_w} wire(s)",
                            "under 0.5 px", "uneven flat? check the slit ends in the report"))
        if not wires:
            say("  no wires frame: keystone from the slit ends only (straight-line correction)")
    else:
        poly = G.Poly2D([(1, 0)], x0=x_ref, xs=(W - 1) / 2.0, y0=y0, ys=half_len, coef=[0.0])
        ks = G.Keystone(poly, x_ref, top, bot, float("nan"))
        edge_info = {}

    # 8. response ------------------------------------------------------------------
    cal = Calibration(width=W, height=H, orientation=orient, y0=y0, wavelength=tbl, smile=smile.poly,
                      keystone=ks, nm_grid=np.zeros(1), s_grid=np.zeros(1), bad=bad.mask, temp_k=temp_k)
    nm_map = cal.wavelength_map()
    plots = dict(spectra={k: v[0].tolist() for k, v in spectra.items()}, fits=[f.to_dict() for f in disp.fits],
                 table=tbl.values.tolist(), sigma=sig.tolist(), edge_info=edge_info,
                 traces=smile.traces, lsf=disp.lsf.to_dict(), band=(y0 - hb, y0 + hb))
    info = {}
    if flat is not None:
        say("Spectral response")
        h_map = cal.keystone.h(*np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float)))
        inside = (np.abs(h_map) <= 1.0)
        flat_raw = SE.fill_bad(F.img[flat.name], bad.mask)
        R = RS.response_map(flat_raw, nm_map, flat.exposure_us or 1e6, temp_k, flat.gain, inside)
        rows = np.arange(int(y0 - 2 * hb), int(y0 + 2 * hb) + 1)
        curves, red = RS.channel_curves(R, rows, tbl.values)
        peak = float(np.nanmax(curves["all"]))
        usable = np.flatnonzero(curves["all"] > 0.05 * peak)
        lo_u, hi_u = float(tbl.values[usable[0]]), float(tbl.values[usable[-1]])
        say(f"  halogen at {temp_k:.0f} K; response above 5% of peak from {lo_u:.0f} to {hi_u:.0f} nm; "
            f"red pixels at (row, column) parity {red}")
        cal.response = R
        plots["response"] = {k: v.tolist() for k, v in curves.items()}
        info["response_range_nm"] = [lo_u, hi_u]
    else:
        lo_u, hi_u = max(500.0, disp.nm_covered[0] - 40), min(1000.0, disp.nm_covered[1] + 40)

    # rectified grid -------------------------------------------------------------------
    nm0 = np.ceil(max(lo_u, float(tbl.values[0])) / nm_step) * nm_step
    nm1 = np.floor(min(hi_u, float(tbl.values[-1])) / nm_step) * nm_step
    cal.nm_grid = np.arange(nm0, nm1 + 1e-9, nm_step)
    cal.s_grid = np.arange(np.ceil(ks.s_top), np.floor(ks.s_bottom) + 1.0)
    say(f"Rectified grid: {cal.nm_grid.size} wavelengths {nm0:.0f}-{nm1:.0f} nm every {nm_step:g} nm, "
        f"{cal.s_grid.size} slit rows")

    first_lamp = max(lamps, key=lambda s: (s.source == "cfl", "long" not in s.name))
    plots["rectified_lamp"] = dict(name=first_lamp.name, img=cal.rectify(lamp_s[first_lamp.name]))
    if flat is not None:
        flat_rect = cal.rectify(SE.fill_bad(F.img[flat.name], bad.mask))
        prof = RS.slit_profile(flat_rect, cal.nm_grid)
        dust = RS.find_dust(prof, width=max(5, int(round(25 / scale))))
        n = len(prof)
        a, b = int(0.05 * n), int(0.95 * n)
        taper = float(np.polyfit(np.linspace(-1, 1, b - a), prof[a:b], 1)[0] * 2) if b - a > 10 else 0.0
        say(f"  slit evenness: {len(dust)} dip(s) deeper than 3% (dust?), "
            f"{100 * taper:+.1f}% brightness change end to end")
        plots["slit_profile"] = prof.tolist()
        info["slit"] = dict(dust=[dict(s=float(cal.s_grid[i]), depth=d) for i, d in dust], taper=taper)
        checks.append(Check("slit evenness", len(dust) == 0 and abs(taper) < 0.15,
                            f"{len(dust)} dip(s), {100 * taper:+.1f}% end to end",
                            "no dips over 3%, under 15% end to end",
                            "blow the slit clean; the response corrects what remains", critical=False))

    # 9. laser ---------------------------------------------------------------------------
    if lasers:
        say("Laser")
        L_s = clean(lasers[0].name)
        info["laser"] = laser_check(L_s, cal, top, bot, y0, hb, fwhm_px, disp.lsf.w0, say)
        if info["laser"]:
            lz = info["laser"]
            checks.append(Check("laser straight along the slit", lz["spread_nm"] < 0.15,
                                f"{lz['spread_nm']:.3f} nm rms along the slit", "under 0.15 nm",
                                "the smile correction is off; check the lamp traces"))
            plots["laser"] = lz.pop("trace")

    info.update(
        created=time.strftime("%Y-%m-%d %H:%M:%S"),
        session=str(Path(session)),
        design_scale=scale,
        dispersion=dict(rms_nm=disp.rms_nm, extra_nm=disp.extra_nm, chi2=disp.chi2,
                        nm_covered=list(disp.nm_covered), px_per_nm_750=disp_750, coarse=disp.coarse,
                        notes=disp.notes),
        lines=[f.to_dict() for f in disp.fits],
        lsf=dict(disp.lsf.to_dict(), fwhm_px=fwhm_px, resolution_nm=res_nm,
                 design_resolution_nm=float(design.resolution_nm)),
        smile_fit=dict(rms_px=smile.rms_px, n_lines=len(smile.traces), bow_at_end_px=end_bow),
        slit_rows=[top, bot],
        frames={name: dict(kind=s.kind, source=s.source, exposure_us=s.exposure_us, gain=s.gain,
                           dark=F.notes.get(name)) for name, s in sets.items() if s.kind != "dark"},
        checks=[c.to_dict() for c in checks],
        log=lines,
    )
    cal.info = info
    result = Result(calibration=cal, checks=checks, plots=plots, log=lines)
    failed = [c.name for c in checks if not c.ok and c.critical]
    notes = [c.name for c in checks if not c.ok and not c.critical]
    say(f"Done in {time.time() - t0:.0f} s: " + ("checks failed: " + ", ".join(failed) if failed
                                                  else "all checks passed")
        + (f" (see the report about: {', '.join(notes)})" if notes else ""))
    if out_dir is not None:
        out = cal.save(out_dir)
        if report:
            from .report import write_report
            write_report(result, out)
        say(f"Saved to {out}")
    return result


def laser_check(img, cal, top, bot, y0, hb, fwhm_px, slit_px, log=print):
    """Laser line: its wavelength along the slit and its width."""
    W = img.shape[1]
    spec, noise = WC.band_spectrum(img, y0 - hb, y0 + hb)
    k = int(np.argmax(spec))
    if spec[k] < 20 * np.median(noise):
        log("  ! no laser line found")
        return {}
    half = int(np.ceil(1.5 * fwhm_px)) + 3
    lo, hi = max(k - half, 0), min(k + half + 1, W)
    r = fit_slit_profile(np.arange(lo, hi, dtype=float), spec[lo:hi], float(k), slit_px, 0.25 * fwhm_px)
    n_bins = int(round((bot - top - 6) / max(4.0, 0.02 * (bot - top))))
    bands, yb = WC.bin_rows(img, top + 3, bot - 3, n_bins)
    b0 = int(np.argmin(np.abs(yb - y0)))
    X, Fl = WC.trace_lines(bands, np.array([r.get("center", float(k))]), b0, 0.9 * fwhm_px, 0.6 * fwhm_px)
    good = np.isfinite(X[0]) & (Fl[0] > 0)
    lam = cal.wavelength_at(X[0][good], yb[good])
    wts = Fl[0][good]
    mean = float(np.sum(lam * wts) / np.sum(wts))
    spread = float(np.sqrt(np.sum(wts * (lam - mean) ** 2) / np.sum(wts)))
    nm_per_px = abs(float(cal.wavelength.deriv(r.get("center", k))))
    fwhm_nm = r["fwhm"] * nm_per_px if r.get("ok") else float("nan")
    log(f"  laser at {mean:.2f} nm (+- {spread:.3f} nm along the slit), line width "
        f"{r.get('fwhm', float('nan')):.1f} px = {fwhm_nm:.2f} nm")
    return dict(nm=mean, spread_nm=spread, fwhm_px=float(r.get("fwhm", np.nan)), fwhm_nm=fwhm_nm,
                trace=dict(y=yb[good].tolist(), x=X[0][good].tolist(), nm=lam.tolist()))
