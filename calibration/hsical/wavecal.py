"""Wavelength calibration: which wavelength lands on which pixel.

For each lamp frame:

1. Average a band of rows across the middle of the slit into one spectrum.
2. Coarse match: slide and stretch the catalog's synthetic spectrum (placed
   where the design's dispersion puts it) along the measured one, mirrored and
   not, and keep the best normalised correlation. This finds the spectrum
   even when it sits hundreds of pixels from where the design says.
3. Line shape: fit a slit profile to bright isolated lines, giving the slit
   image width and the lens blur.
4. Group fits: fit each reference line together with its neighbours
   (spacings fixed by the catalog, brightnesses free), one precise position
   per reference line.
5. Dispersion: a cubic lambda_c(x) through those positions, weighted, with
   outliers rejected. Steps 4 and 5 run again with the better dispersion.

Then the smile: bright lines are followed from the middle of the slit to its
ends. How far each one bends gives a displacement field delta(x, y), and

    lambda(x, y) = lambda_c(x - delta(x, y)).

Tracing needs no line identification at all, so any bright line helps.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.ndimage import minimum_filter1d, uniform_filter1d
from scipy.optimize import brentq, least_squares, lsq_linear, nnls
from scipy.signal import correlate, find_peaks

from . import lines as L
from .design import interp_extrap
from .poly import Poly1D, Poly2D
from .profiles import centroid, fit_slit_profile, half_max_width, slit_profile, slit_profile_fwhm

# ---------------------------------------------------------------------------
# 1-D spectra
# ---------------------------------------------------------------------------


def band_spectrum(img, y_lo, y_hi):
    """Mean of rows y_lo..y_hi of a (smoothed) frame, and its noise per column."""
    rows = np.asarray(img[int(y_lo):int(y_hi)], float)
    spec = rows.mean(0)
    level = rows.sum(1) / max(rows.sum(1).mean(), 1e-12)  # remove a slow brightness drift
    resid = rows - spec[None, :] * level[:, None]
    # neighbouring rows of a binomial-smoothed frame are correlated, hence n / 2
    noise = resid.std(0) / np.sqrt(max(rows.shape[0] / 2.0, 1.0))
    floor = np.median(noise) * 0.5 + 1e-9
    return spec, np.maximum(noise, floor)


def baseline(p, size):
    """Running minimum, then smoothed: the floor under the lines."""
    size = max(3, int(size) | 1)
    return uniform_filter1d(minimum_filter1d(p, size, mode="nearest"), size, mode="nearest")


def pattern(p, fwhm):
    """Line-emphasising version of a spectrum for pattern matching.

    The square root keeps one bright line from dominating, and the high-pass
    removes phosphor glow and the slow fall of the sensor's response."""
    q = np.sqrt(np.clip(p - baseline(p, 3 * fwhm), 0.0, None))
    return q - uniform_filter1d(q, max(3, int(4 * fwhm) | 1), mode="nearest")


def estimate_fwhm(spec, noise, guess):
    """Typical width (px) of the narrowest bright lines in a spectrum."""
    s = spec - baseline(spec, 6 * guess)
    peaks, props = find_peaks(s, prominence=10 * np.median(noise), distance=max(2, int(guess / 3)))
    if len(peaks) == 0:
        return None
    top = peaks[np.argsort(props["prominences"])[::-1][:15]]
    w = half_max_width(np.repeat(s[None], top.size, 0), top.astype(float), 1.5 * guess)
    w = w[np.isfinite(w) & (w > 0.3 * guess) & (w < 4 * guess)]
    return float(np.percentile(w, 30)) if w.size else None


def blur_for_width(fwhm, w, floor=0.3):
    """Blur sigma (px) that widens a w-px slit image to a line width of fwhm px."""
    if not fwhm or not np.isfinite(fwhm) or slit_profile_fwhm(w, floor) >= fwhm:
        return floor
    return float(brentq(lambda s: slit_profile_fwhm(w, s) - fwhm, floor, float(fwhm)))


# ---------------------------------------------------------------------------
# Coarse match against the design
# ---------------------------------------------------------------------------


@dataclass
class Coarse:
    """Design dispersion stretched by k and slid by lag: x_design = xc + k (x + lag - xc)."""
    k: float
    lag: float
    score: float
    runner_up: float
    xc: float
    design: object = field(repr=False)

    def x_of_nm(self, nm):
        return (self.design.col(np.zeros_like(np.asarray(nm, float)), nm) - self.xc) / self.k \
            + self.xc - self.lag

    def nm_of_x(self, x):
        return self.design.wavelength_at(self.xc + self.k * (np.asarray(x, float) + self.lag - self.xc))

    @property
    def flipped(self):
        return self.k < 0


def coarse_register(spec, fwhm_px, design, lines, hints=(), k_range=(0.9, 1.1), k_step=0.004,
                    allow_flip=True):
    """Find the stretch, mirror and offset that best line the catalog up with ``spec``.

    hints: (nm, x) pairs that must be honoured (within 2 line widths)."""
    W = spec.size
    xc = (W - 1) / 2.0
    O = pattern(spec, fwhm_px)
    O = O / (np.linalg.norm(O) + 1e-12)
    fwhm_nm = fwhm_px / float(design.dispersion(750.0))
    lsf = L.Lsf(0.85 * fwhm_nm, 0.25 * fwhm_nm)
    nm = np.arange(design.nm_range[0], design.nm_range[1], 0.05)
    cols = design.col(np.zeros_like(nm), nm)
    E_nm = L.spectrum(lines, nm, lsf)
    ks = np.arange(k_range[0], k_range[1] + 1e-9, k_step)
    if allow_flip:
        ks = np.concatenate([ks, -ks])
    cands = []
    for k in ks:
        u_of_nm = xc + (cols - xc) / k
        order = np.argsort(u_of_nm)
        u_lo = np.floor(u_of_nm.min())
        u = np.arange(u_lo, np.ceil(u_of_nm.max()) + 1)
        e = pattern(np.interp(u, u_of_nm[order], E_nm[order]), fwhm_px)
        ep = np.concatenate([np.zeros(W), e, np.zeros(W)])
        num = correlate(ep, O, mode="valid", method="fft")
        c2 = np.concatenate([[0.0], np.cumsum(ep ** 2)])
        den = np.sqrt(np.maximum(c2[W:] - c2[:-W], 0.0))
        lag = u_lo - W + np.arange(num.size)
        ncc = np.where(den > 0.3 * den.max(), num / np.maximum(den, 1e-12), -1.0)
        for nm_h, x_h in hints:
            x_pred = (np.interp(nm_h, nm, cols) - xc) / k + xc - lag
            ncc = np.where(np.abs(x_pred - x_h) < 2 * fwhm_px, ncc, -1.0)
        # keep the best few local maxima per stretch, for the runner-up check
        pk, _ = find_peaks(ncc, distance=max(2, int(2 * fwhm_px)))
        for i in pk[np.argsort(ncc[pk])[::-1][:3]]:
            cands.append((float(ncc[i]), float(k), float(lag[i])))
    if not cands:
        raise RuntimeError("no match between the lamp spectrum and the catalog")
    cands.sort(reverse=True)
    best = cands[0]
    test = np.array([550.0, 750.0, 950.0])

    def where(c):
        return (np.interp(test, nm, cols) - xc) / c[1] + xc - c[2]

    xb = where(best)
    runner = max((c[0] for c in cands[1:] if np.max(np.abs(where(c) - xb)) > 2 * fwhm_px),
                 default=-1.0)
    return Coarse(k=best[1], lag=best[2], score=best[0], runner_up=runner, xc=xc, design=design)


# ---------------------------------------------------------------------------
# Line shape and group fits
# ---------------------------------------------------------------------------


@dataclass
class LsfModel:
    """Slit image width and blur (px), each a straight line in normalised x.

    The width follows the design (the grating stretches the slit image toward
    the red), scaled by what bright isolated lines measure."""
    w0: float
    s0: float
    w1: float = 0.0
    s1: float = 0.0
    x0: float = 0.0
    xs: float = 1.0
    samples: list = field(default_factory=list)

    def at(self, x):
        u = (np.asarray(x, float) - self.x0) / self.xs
        return np.maximum(self.w0 + self.w1 * u, 0.5), np.maximum(self.s0 + self.s1 * u, 0.3)

    def fwhm(self, x):
        from .profiles import slit_profile_fwhm
        w, s = self.at(x)
        return np.vectorize(slit_profile_fwhm)(w, s)

    def to_dict(self):
        return dict(w0=self.w0, w1=self.w1, s0=self.s0, s1=self.s1, x0=self.x0, xs=self.xs,
                    samples=self.samples)


def measure_lsf(spectra, groups, x_of_nm, w_design, s_guess, W, min_snr=30.0, min_purity=0.97):
    """Fit slit profiles to bright, isolated lines; return an LsfModel.

    w_design(x): the design's slit image width at column x."""
    xc = (W - 1) / 2.0
    pts = []
    for (spec, noise), gs in zip(spectra, groups):
        for g in gs:
            if g.purity < min_purity:
                continue
            x0 = float(x_of_nm(g.ref.nm))
            w_guess = float(w_design(x0))
            half = int(np.ceil(0.5 * w_guess + 3.5 * s_guess)) + 1
            lo, hi = int(round(x0)) - half, int(round(x0)) + half + 1
            if lo < 1 or hi > W - 1:
                continue
            xs = np.arange(lo, hi, dtype=float)
            r = fit_slit_profile(xs, spec[lo:hi], x0, w_guess, s_guess)
            nz = np.median(noise[lo:hi])
            band = g.ref.width * abs(float(x_of_nm(g.ref.nm + 0.5)) - float(x_of_nm(g.ref.nm - 0.5))) \
                / L.FWHM_PER_SIGMA
            if (r.get("ok") and r["amp"] > min_snr * nz and 0.5 * w_guess < r["width"] < 2.0 * w_guess
                    and r["sigma"] < 2 * w_guess and abs(r["center"] - x0) < 0.5 * w_guess):
                s = float(np.sqrt(max(r["sigma"] ** 2 - band ** 2, 0.09)))
                pts.append((r["center"], r["width"], s, r["width"] / w_guess))
    wl, wr = float(w_design(0.0)), float(w_design(W - 1.0))
    m = LsfModel(w0=0.5 * (wl + wr), w1=0.5 * (wr - wl), s0=s_guess, x0=xc, xs=xc)
    if not pts:
        return m
    p = np.array(pts)
    m.samples = [dict(x=float(a), width=float(b), sigma=float(c)) for a, b, c, _ in p]
    a = float(np.median(p[:, 3]))
    m.w0, m.w1 = m.w0 * a, m.w1 * a
    m.s0 = float(np.median(p[:, 2]))
    return m


def _parabola_min(xs, cs):
    """Where a parabola through the lowest point and its neighbours bottoms out."""
    i = int(np.argmin(cs))
    if 0 < i < len(xs) - 1:
        a, b, c = xs[i - 1:i + 2]
        fa, fb, fc = cs[i - 1:i + 2]
        den = (b - a) * (fb - fc) - (b - c) * (fb - fa)
        if den != 0:
            return float(np.clip(b - 0.5 * ((b - a) ** 2 * (fb - fc) - (b - c) ** 2 * (fb - fa)) / den, a, c))
    return float(xs[i])


def refine_lsf(spectra, groups, x_of_nm, dxdnm, lsf, max_shift, sat=None, n_groups=20, min_snr=40.0):
    """The line width and shape under which the strongest line groups fit best.

    Isolated lines give a first width, but few lines are truly alone at this
    resolution: a neighbour's wing under a line reads as background, which
    makes the line look narrower and flatter-topped than it is. And on its own
    a line can't tell a wide slit image with little blur from a narrow one with
    a lot of blur; where lines overlap, the two add up differently. So fit the
    strongest groups, all their lines together, under a range of widths and of
    splits between slit image and blur, and keep the best (lowest chi-square).
    Returns (LsfModel, n_groups used)."""
    sat = sat or {}
    f0 = float(lsf.fwhm(np.array([lsf.x0]))[0])
    cand = []
    for name, (spec, noise) in spectra.items():
        for g in groups[name]:
            f = fit_group(spec, noise, g, x_of_nm, dxdnm, lsf, max_shift, sat=sat.get(name))
            if f is not None and f.snr >= min_snr:
                cand.append((f.snr, name, g))
    cand = sorted(cand, key=lambda t: -t[0])[:n_groups]
    if len(cand) < 5:
        return lsf, 0

    def model(k, q):
        fw = k * f0          # width at half maximum, at the centre column
        w0 = q * fw          # the slit image's share of it; blur makes up the rest
        return LsfModel(w0=w0, w1=lsf.w1 * w0 / lsf.w0, s0=blur_for_width(fw, w0), s1=0.0,
                        x0=lsf.x0, xs=lsf.xs, samples=lsf.samples)

    seen = {}

    def cost(k, q):
        key = (round(k, 4), round(q, 4))
        if key not in seen:
            m = model(k, q)
            tot = 0.0
            for _, name, g in cand:
                spec, noise = spectra[name]
                f = fit_group(spec, noise, g, x_of_nm, dxdnm, m, max_shift, sat=sat.get(name))
                tot += f.chi2 if f is not None else 100.0
            seen[key] = tot
        return seen[key]

    def search(vals, fn, step, grow=2):
        vals = list(vals)
        for _ in range(grow + 1):   # widen the range while the best is at an end
            cs = [fn(v) for v in vals]
            i = int(np.argmin(cs))
            if 0 < i < len(vals) - 1 or step is None:
                break
            vals = [vals[0] - step] + vals if i == 0 else vals + [vals[-1] + step]
            if vals[0] <= 0.5:
                vals = vals[1:]
                break
        cs = [fn(v) for v in vals]
        return _parabola_min(np.array(vals), np.array(cs))

    q0 = float(np.clip(lsf.w0 / f0, 0.25, 0.97))
    k = search([0.9, 1.0, 1.1, 1.2, 1.3], lambda v: cost(v, q0), 0.1)
    q = search([0.25, 0.4, 0.55, 0.7, 0.8, 0.9, 0.97], lambda v: cost(k, v), None)
    k = search([0.96 * k, k, 1.04 * k], lambda v: cost(v, q), 0.04 * k)
    return model(k, q), len(cand)


@dataclass
class GroupFit:
    lamp: str
    label: str
    nm: float           # reference wavelength (air)
    nm_sigma: float     # catalog uncertainty of that wavelength
    x: float            # fitted position of the reference line (px)
    x_sigma: float
    amp: float
    snr: float
    chi2: float
    n_members: int
    blend_nm: float = 0.0
    used: bool = True
    resid_nm: float = float("nan")

    def to_dict(self):
        return {k: (float(v) if isinstance(v, (float, np.floating)) else v)
                for k, v in self.__dict__.items()}


def fit_group(spec, noise, group, x_of_nm, dxdnm, lsf, max_shift, max_band_nm=1.5, sat=None):
    """Fit one reference line and its neighbours; return a GroupFit or None.

    sat: optional boolean array, True where the spectrum is saturated."""
    W = spec.size
    a, b = sorted([float(x_of_nm(group.lo_nm)), float(x_of_nm(group.hi_nm))])
    lo, hi = int(np.floor(a)), int(np.ceil(b)) + 1
    if lo < 2 or hi > W - 2 or hi - lo < 6:
        return None
    if sat is not None and np.any(sat[lo:hi]):
        return None
    xs = np.arange(lo, hi, dtype=float)
    y = spec[lo:hi]
    sd = noise[lo:hi]
    mem = group.members
    pos0 = np.array([float(x_of_nm(m.nm)) for m in mem])
    disp = np.abs(np.array([float(dxdnm(m.nm)) for m in mem]))
    band = np.array([m.width for m in mem]) * disp / L.FWHM_PER_SIGMA
    free = [i for i, m in enumerate(mem) if m.quality == "C"]
    iref = mem.index(group.ref)
    span = max(hi - lo, 1)
    xm = xs.mean()

    def design_matrix(p):
        pos = pos0 + p[0]
        for n, i in enumerate(free):
            pos[i] += p[1 + n]
        w, s = lsf.at(pos)
        cols = [slit_profile(xs, pos[i], 1.0, w[i], np.hypot(s[i], band[i])) for i in range(len(mem))]
        return np.column_stack(cols + [np.ones_like(xs), (xs - xm) / span])

    def solve(A):
        # brightnesses >= 0, background free. nnls is many times faster than lsq_linear
        # here; giving each background term a mirrored column lets it go negative.
        Aw, yw = A / sd[:, None], y / sd
        k = len(mem)
        try:
            z, _ = nnls(np.column_stack([Aw, -Aw[:, k:]]), yw)
            return np.r_[z[:k], z[k:k + 2] - z[k + 2:]]
        except RuntimeError:
            lb = np.r_[np.zeros(k), -np.inf, -np.inf]
            return lsq_linear(Aw, yw, bounds=(lb, np.inf)).x

    def resid(p):
        A = design_matrix(p)
        return (A @ solve(A) - y) / sd

    band_px = max_band_nm * np.median(disp)
    lb = np.r_[-max_shift, -band_px * np.ones(len(free))]
    ub = np.r_[max_shift, band_px * np.ones(len(free))]
    try:
        r = least_squares(resid, np.zeros(1 + len(free)), bounds=(lb, ub), diff_step=1e-3,
                          x_scale=1.0)
    except (ValueError, np.linalg.LinAlgError):
        return None
    shift = float(r.x[0])
    if abs(shift) > 0.95 * max_shift:
        return None
    A = design_matrix(r.x)
    amp = solve(A)
    dof = max(xs.size - len(amp) - len(r.x), 1)
    chi2 = float(np.sum(r.fun ** 2) / dof)
    try:
        cov = np.linalg.inv(r.jac.T @ r.jac) * max(chi2, 1.0)
        x_sigma = float(np.sqrt(cov[0, 0]))
        Aw = A / sd[:, None]
        amp_sigma = float(np.sqrt(np.linalg.pinv(Aw.T @ Aw)[iref, iref]))
    except np.linalg.LinAlgError:
        return None
    return GroupFit(lamp=group.source, label=group.label, nm=group.ref.nm,
                    nm_sigma=L.QUALITY_NM[group.ref.quality], x=float(pos0[iref] + shift),
                    x_sigma=x_sigma, amp=float(amp[iref]), snr=float(amp[iref] / max(amp_sigma, 1e-12)),
                    chi2=chi2, n_members=len(mem), blend_nm=group.blend_nm)


# ---------------------------------------------------------------------------
# The dispersion relation lambda_c(x)
# ---------------------------------------------------------------------------


class Table1D:
    """A monotonic function of x given at every column, linear in between and beyond.

    The calibrated wavelength of each column is stored this way: one number
    per column is the simplest thing to load in C++ or CUDA."""

    def __init__(self, values, sigma=None):
        self.values = np.asarray(values, float)
        self.sigma = None if sigma is None else np.asarray(sigma, float)
        self.cols = np.arange(self.values.size, dtype=float)
        self.slope = np.gradient(self.values)

    def __call__(self, x):
        return interp_extrap(x, self.cols, self.values)

    def deriv(self, x):
        return interp_extrap(x, self.cols, self.slope)

    def inverse(self, v):
        if self.values[-1] >= self.values[0]:
            return interp_extrap(v, self.values, self.cols)
        return interp_extrap(v, self.values[::-1], self.cols[::-1])

    def uncertainty(self, x):
        return None if self.sigma is None else np.interp(x, self.cols, self.sigma)


@dataclass
class Dispersion:
    table: Table1D = None
    lsf: LsfModel = None
    fits: list = field(default_factory=list)
    coarse: dict = field(default_factory=dict)
    coef: list = field(default_factory=list)
    rms_nm: float = float("nan")
    chi2: float = float("nan")
    extra_nm: float = 0.0
    nm_covered: tuple = (float("nan"), float("nan"))
    flipped: bool = False
    notes: list = field(default_factory=list)

    def nm(self, x):
        return self.table(x)

    def x(self, nm):
        return self.table.inverse(nm)


def fit_dispersion(fits, base, degree=3, prior_nm=1.0, floor_nm=0.02, clip=4.0, iters=6):
    """lambda_c(x) = base(x) + sum_k c_k u^k with u = (x - xc) / xc.

    base is the design's curve as the coarse match placed it. The constant and
    linear terms are free; the curvature terms (k >= 2) are held near the
    design's by a prior of ``prior_nm`` at the ends of the sensor, which only
    matters where no lamp line pins the curve down.

    Each line is weighted by its own uncertainty (noise, catalog, blend) plus a
    shared extra scatter, chosen so the residuals match the uncertainties:
    line shapes that differ a little from the model move blended lines by a
    tenth of a nanometre or so, and noise alone would not admit that.

    Returns a dict: table (Table1D with uncertainty), coef, keep, rms, chi2, extra_nm."""
    W = base.size
    xc = (W - 1) / 2.0
    ok = [f for f in fits if np.isfinite(f.x) and np.isfinite(f.x_sigma)]
    x = np.array([f.x for f in ok])
    nm = np.array([f.nm for f in ok])
    cols = np.arange(W, dtype=float)
    b = interp_extrap(x, cols, base)
    slope = np.abs(interp_extrap(x, cols, np.gradient(base)))
    sig0 = np.sqrt((np.array([f.x_sigma for f in ok]) * slope) ** 2
                   + np.array([f.nm_sigma for f in ok]) ** 2
                   + np.array([f.blend_nm for f in ok]) ** 2 + floor_nm ** 2)
    n_ref = len({round(f.nm, 1) for f in ok})
    deg = int(min(degree, max(1, n_ref - 1)))
    powers = np.arange(deg + 1)
    A = ((x - xc) / xc)[:, None] ** powers[None, :]
    z = nm - b
    prior = np.zeros((deg + 1, deg + 1))
    for k in range(2, deg + 1):
        prior[k, k] = 1.0 / prior_nm ** 2

    def solve(keep, extra):
        sig = np.hypot(sig0, extra)
        w = 1.0 / sig[keep] ** 2
        N = (A[keep] * w[:, None]).T @ A[keep] + prior
        c = np.linalg.solve(N, (A[keep] * w[:, None]).T @ z[keep])
        r = (z - A @ c) / sig
        dof = max(int(keep.sum()) - (deg + 1), 1)
        return c, N, r, float(np.sum(r[keep] ** 2) / dof)

    def balance(keep):
        """Extra scatter that brings the reduced chi-square to 1 (0 if already below)."""
        if solve(keep, 0.0)[3] <= 1.0:
            return 0.0
        lo, hi = 0.0, 2.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if solve(keep, mid)[3] > 1.0 else (lo, mid)
        return hi

    keep = np.ones(len(ok), bool)
    extra = 0.0
    for _ in range(iters):
        extra = balance(keep)
        c, N, r, chi2 = solve(keep, extra)
        new = np.abs(r) <= clip
        if np.array_equal(new, keep):
            break
        keep = new
    extra = balance(keep)
    c, N, r, chi2 = solve(keep, extra)
    resid = z - A @ c
    cov = np.linalg.inv(N) * max(chi2, 1.0)
    Ac = ((cols - xc) / xc)[:, None] ** powers[None, :]
    values = base + Ac @ c
    sigma = np.sqrt(np.maximum(np.einsum("ij,jk,ik->i", Ac, cov, Ac), 0.0))
    for f, k, rr in zip(ok, keep, resid):
        f.used = bool(k)
        f.resid_nm = float(rr)
    for f in fits:
        if f not in ok:
            f.used = False
    rms = float(np.sqrt(np.mean(resid[keep] ** 2))) if keep.any() else float("nan")
    return dict(table=Table1D(values, sigma), coef=[float(v) for v in c], keep=keep, rms=rms,
                chi2=chi2, extra_nm=float(extra))


def solve_dispersion(spectra, design, hints=(), allow_flip=True, min_snr=8.0, sat=None, log=print):
    """Wavelength vs column along the reference band, from one or more lamps.

    spectra: {name: (spectrum, noise)}, where name is a lamp ("cfl", "neon") or
    starts with one ("neon_long"). sat: {name: boolean array} of saturated
    columns. Returns a Dispersion; if the spectrum runs red-to-blue along x it
    returns early with flipped=True so the caller can mirror the frames."""
    out = Dispersion()
    sat = sat or {}
    W = next(iter(spectra.values()))[0].size
    xc = (W - 1) / 2.0
    w_design0 = float(design.slit_image_px(0.0, 750.0))
    fwhms = [estimate_fwhm(s, n, w_design0) for s, n in spectra.values()]
    fwhm_px = float(np.median([f for f in fwhms if f])) if any(fwhms) else 1.2 * w_design0
    lamp_of = {name: next(l for l in L.LAMPS if name.startswith(l)) for name in spectra}
    catalogs = {name: L.catalog(lamp_of[name]) for name in spectra}
    coarse = {}
    for name, (spec, _) in spectra.items():
        h = hints.get(lamp_of[name], ()) if isinstance(hints, dict) else hints
        c = coarse_register(spec, fwhm_px, design, catalogs[name], hints=h, allow_flip=allow_flip)
        coarse[name] = c
        out.coarse[name] = dict(k=c.k, lag=c.lag, score=c.score, runner_up=c.runner_up)
        log(f"  {name}: pattern match {c.score:.2f} (next best {c.runner_up:.2f}), "
            f"stretch {c.k:+.3f}, offset {c.lag:+.0f} px")
    best = max(coarse, key=lambda s: coarse[s].score - max(coarse[s].runner_up, 0))
    if coarse[best].flipped:
        out.flipped = True
        return out
    ref = coarse[best]
    for name, c in coarse.items():
        if name == best:
            continue
        d = np.abs(c.x_of_nm(np.array([600.0, 800.0])) - ref.x_of_nm(np.array([600.0, 800.0])))
        if c.flipped or d.max() > 2 * fwhm_px:
            out.notes.append(f"{name} frames' pattern match disagreed with {best}'s; used {best}'s")
            log(f"  ! {out.notes[-1]}")
    cols = np.arange(W, dtype=float)
    base = ref.nm_of_x(cols)
    table = Table1D(base)
    nm_lo, nm_hi = sorted([float(base[0]), float(base[-1])])
    nm_range = (max(nm_lo, 470.0), min(nm_hi, 1030.0))

    def w_design(x):
        return design.slit_image_px(0.0, np.clip(table(x), *design.nm_range))

    # the blur that widens the design's slit image to the width the lines show: a soft
    # lens can double it, and the line shape fit needs a window that wide
    s_guess = blur_for_width(fwhm_px, w_design0)
    lsf = LsfModel(w0=w_design0, s0=s_guess, x0=xc, xs=xc)
    max_shift = 1.0 * fwhm_px
    for it in range(3):
        x_of_nm = table.inverse
        dxdnm = (lambda t: (lambda nm: 1.0 / t.deriv(t.inverse(nm))))(table)
        fwhm_nm = float(np.median(lsf.fwhm(np.array([0.25, 0.5, 0.75]) * W))) / abs(float(dxdnm(750.0)))
        groups = {name: L.groups(catalogs[name], fwhm_nm, nm_range=nm_range) for name in spectra}
        if it == 0:
            lsf = measure_lsf([spectra[s] for s in spectra], [groups[s] for s in spectra], x_of_nm,
                              w_design, s_guess, W)
            alone = float(lsf.fwhm(np.array([xc]))[0])
            lsf, n_g = refine_lsf(spectra, groups, x_of_nm, dxdnm, lsf, max_shift, sat)
            log(f"  line shape: {float(lsf.fwhm(np.array([xc]))[0]):.1f} px wide at half maximum "
                f"({alone:.1f} from {len(lsf.samples)} isolated lines alone); slit image "
                f"{lsf.w0:.1f} px (design {w_design(xc):.1f}) and blur sigma {lsf.s0:.2f} px, "
                f"from the {n_g} strongest line groups")
            fwhm_nm = float(np.median(lsf.fwhm(np.array([0.25, 0.5, 0.75]) * W))) \
                / abs(float(dxdnm(750.0)))
            groups = {name: L.groups(catalogs[name], fwhm_nm, nm_range=nm_range) for name in spectra}
        fits = []
        for name, (spec, noise) in spectra.items():
            for g in groups[name]:
                f = fit_group(spec, noise, g, x_of_nm, dxdnm, lsf, max_shift, sat=sat.get(name))
                if f is not None and f.snr >= min_snr:
                    f.lamp = name
                    fits.append(f)
        if len({round(f.nm, 1) for f in fits}) < 3:
            raise RuntimeError(f"only {len(fits)} lamp lines found; check the exposure and the lamp frames")
        res = fit_dispersion(fits, base)
        table = res["table"]
        disp = abs(float(table.deriv(xc)))
        max_shift = max(0.25 * fwhm_px, 3 * res["rms"] * disp + 1.0)
        log(f"  pass {it + 1}: {int(res['keep'].sum())} of {len(fits)} lines used, "
            f"rms {res['rms']:.3f} nm")
    used = [f.nm for f in fits if f.used]
    out.table, out.lsf, out.fits, out.coef = table, lsf, fits, res["coef"]
    out.rms_nm, out.chi2, out.extra_nm = res["rms"], res["chi2"], res["extra_nm"]
    out.nm_covered = (float(min(used)), float(max(used)))
    return out


# ---------------------------------------------------------------------------
# Smile: follow lines along the slit
# ---------------------------------------------------------------------------


def pick_trace_lines(spec, noise, fwhm, min_snr=25.0, min_sep=1.6, sat=None):
    """Columns of bright lines worth following along the slit.

    Any line works (no identification needed) as long as nothing else of
    similar brightness sits within ``min_sep`` line widths of it."""
    s = spec - baseline(spec, 4 * fwhm)
    pk, props = find_peaks(s, prominence=min_snr * np.median(noise), distance=max(2, int(0.5 * fwhm)))
    keep = []
    for i, p in enumerate(pk):
        if sat is not None and sat[max(p - int(fwhm), 0):p + int(fwhm) + 1].any():
            continue
        near = np.abs(pk - p) < min_sep * fwhm
        near[i] = False
        if np.any(s[pk[near]] > 0.15 * s[p]):
            continue
        keep.append(p)
    if not keep:
        return np.zeros(0)
    keep = np.array(keep)
    x, _, ok = centroid(np.repeat(s[None], keep.size, 0), keep.astype(float), 0.9 * fwhm)
    return x[ok]


def bin_rows(img, y_lo, y_hi, n_bins):
    """Average rows into n_bins bands between y_lo and y_hi; return (bands, centre rows)."""
    edges = np.unique(np.linspace(y_lo, y_hi, n_bins + 1).round().astype(int))
    block = np.asarray(img[edges[0]:edges[-1]], float)
    sums = np.add.reduceat(block, edges[:-1] - edges[0], axis=0)
    return sums / np.diff(edges)[:, None], 0.5 * (edges[:-1] + edges[1:] - 1)


def trace_lines(bands, x_start, b_start, half, max_step):
    """Follow each line from band b_start outward in both directions.

    Returns (x, flux), each (n_lines, n_bands), NaN where a line was lost."""
    n, nb = len(x_start), bands.shape[0]
    X = np.full((n, nb), np.nan)
    F = np.zeros((n, nb))
    x, f, ok = centroid(np.repeat(bands[b_start][None], n, 0), np.asarray(x_start, float), half)
    X[:, b_start] = np.where(ok, x, np.nan)
    F[:, b_start] = f
    for step in (1, -1):
        last = X[:, b_start].copy()
        slope = np.zeros(n)
        b = b_start + step
        while 0 <= b < nb:
            pred = last + slope
            x, f, ok = centroid(np.repeat(bands[b][None], n, 0), np.nan_to_num(pred), half)
            good = ok & np.isfinite(pred) & (np.abs(x - pred) < max_step)
            X[:, b] = np.where(good, x, np.nan)
            F[:, b] = np.where(good, f, 0.0)
            slope = np.where(good, 0.5 * slope + 0.5 * (x - last), slope)
            last = np.where(good, x, pred)
            b += step
    return X, F


@dataclass
class Smile:
    poly: Poly2D
    rms_px: float
    traces: list   # per line: dict(x0, y, x, kept)
    dropped: int = 0  # traces too ragged to be a single line

    def delta(self, x, y):
        return self.poly(x, y)


def fit_smile(X, F, yb, W, y0, half_len, x_deg=2, y_deg=3, min_cover=0.5, max_rms=None):
    """Displacement field delta(x, y) (px along x) from traced lines; delta = 0 at y0.

    max_rms: drop a trace that scatters more than this (px) about its own smooth
    curve; a real line stays within a fraction of a pixel, while a trace that
    hops between a faint feature and its neighbours wanders by many."""
    xs, ys, ds, sig, traces = [], [], [], [], []
    dropped = 0
    for i in range(X.shape[0]):
        good = np.isfinite(X[i]) & (F[i] > 0)
        if good.sum() < 6 or np.ptp(yb[good]) < min_cover * 2 * half_len:
            continue
        yy, xx, ff = yb[good], X[i, good], F[i, good]
        rel = np.sqrt(np.median(ff) / ff)
        p = Poly1D(4, x0=y0, xs=half_len)
        keep = p.fit(yy, xx, rel, clip=4.0)
        res = (xx - p(yy))[keep]
        s = max(float(np.std(res)), 0.01)
        if max_rms is not None and s > max_rms:
            dropped += 1
            continue
        x0 = float(p(y0))
        traces.append(dict(x0=x0, y=yy.tolist(), x=xx.tolist(), kept=keep.tolist(), rms=s))
        xs.append(xx[keep])
        ys.append(yy[keep])
        ds.append(xx[keep] - x0)
        sig.append(s * rel[keep])
    if not xs:
        raise RuntimeError("no line could be followed along the slit")
    terms = [(i, j) for i in range(x_deg + 1) for j in range(1, y_deg + 1)]
    poly = Poly2D(terms, x0=(W - 1) / 2.0, xs=(W - 1) / 2.0, y0=y0, ys=half_len)
    x, y, d, s = (np.concatenate(a) for a in (xs, ys, ds, sig))
    keep = poly.fit(x, y, d, s, clip=4.0)
    rms = float(np.sqrt(np.mean((poly(x, y) - d)[keep] ** 2)))
    return Smile(poly=poly, rms_px=rms, traces=traces, dropped=dropped)
