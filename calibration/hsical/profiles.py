"""Line profiles: the image of the slit, and how to find a line's centre and width.

A narrow emission line fills the slit evenly, so on the sensor it is a picture
of the slit: a box as wide as the slit image (about 23 px here), softened by
the lens blur. That box blurred by a Gaussian is the model used for fits.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import brentq, least_squares
from scipy.special import erf

SQRT2 = np.sqrt(2.0)


def slit_profile(x, center, amp, width, sigma):
    """A box ``width`` px wide centred on ``center``, blurred by a Gaussian ``sigma``.

    ``amp`` is the height the box would have without blur."""
    sigma = np.maximum(sigma, 1e-6)
    a = (x - center + 0.5 * width) / (SQRT2 * sigma)
    b = (x - center - 0.5 * width) / (SQRT2 * sigma)
    return 0.5 * amp * (erf(a) - erf(b))


def slit_profile_fwhm(width, sigma):
    """Full width at half maximum of :func:`slit_profile`."""
    width, sigma = float(abs(width)), float(max(sigma, 1e-6))
    peak = slit_profile(0.0, 0.0, 1.0, width, sigma)
    f = lambda x: slit_profile(x, 0.0, 1.0, width, sigma) - 0.5 * peak  # noqa: E731
    return 2.0 * brentq(f, 0.0, 0.5 * width + 10.0 * sigma)


def fit_slit_profile(x, y, center, width=20.0, sigma=3.0, fit_width=True):
    """Least-squares fit of :func:`slit_profile` plus a straight background.

    Returns a dict with center, amp, width, sigma, fwhm, residual rms and ok."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    span = max(np.ptp(x), 1.0)
    xm = x.mean()

    def model(p):
        c, a, w, s, b0, b1 = p
        return slit_profile(x, c, a, w, s) + b0 + b1 * (x - xm) / span

    def resid(p):
        if not fit_width:
            p = np.concatenate([p[:2], [width], p[2:]])
        return model(p) - y

    edge = np.concatenate([y[:3], y[-3:]])
    b0 = float(np.median(edge))
    a0 = float(max(y.max() - b0, 1e-9))
    p0 = [center, a0, width, sigma, b0, 0.0]
    lo = [x.min(), 0.0, 1.0, 0.3, -np.inf, -np.inf]
    hi = [x.max(), np.inf, span, span, np.inf, np.inf]
    if not fit_width:
        p0 = p0[:2] + p0[3:]
        lo = lo[:2] + lo[3:]
        hi = hi[:2] + hi[3:]
    try:
        r = least_squares(resid, p0, bounds=(lo, hi), x_scale="jac")
    except ValueError:
        return dict(ok=False)
    p = r.x if fit_width else np.concatenate([r.x[:2], [width], r.x[2:]])
    c, a, w, s, b0, b1 = (float(v) for v in p)
    return dict(ok=bool(r.success), center=c, amp=a, width=w, sigma=s,
                fwhm=slit_profile_fwhm(w, s), bg=b0, rms=float(np.sqrt(np.mean(r.fun ** 2))))


def _windows(profiles, x, m):
    """Gather columns floor(x)-m .. floor(x)+m of each row; positions and values."""
    n, w = profiles.shape
    base = np.floor(np.nan_to_num(x, nan=-10 * w)).astype(int)
    idx = base[:, None] + np.arange(-m, m + 1)[None, :]
    inside = (idx >= 0) & (idx < w)
    vals = np.take_along_axis(profiles, np.clip(idx, 0, w - 1), axis=1)
    return idx.astype(float), vals, inside


def _overlap(pos, lo, hi):
    """How much of each pixel [pos - 0.5, pos + 0.5] lies inside [lo, hi]."""
    return np.clip(np.minimum(pos + 0.5, hi[:, None]) - np.maximum(pos - 0.5, lo[:, None]),
                   0.0, 1.0)


def _background(pos, vals, lo, hi, bg_px):
    """Straight line through the mean of the bg_px pixels just outside each side.

    The side windows use fractional pixel weights, so they sit symmetrically
    about the line however the window falls on the pixel grid; otherwise a
    line's wings bias the background slope and with it the centre."""
    left = _overlap(pos, lo - bg_px, lo)
    right = _overlap(pos, hi, hi + bg_px)
    nl = np.maximum(left.sum(1), 1e-9)
    nr = np.maximum(right.sum(1), 1e-9)
    bl = (vals * left).sum(1) / nl
    br = (vals * right).sum(1) / nr
    xl = (pos * left).sum(1) / nl
    xr = (pos * right).sum(1) / nr
    slope = (br - bl) / np.where(xr > xl, xr - xl, 1.0)
    return bl[:, None] + (pos - xl[:, None]) * slope[:, None]


def centroid(profiles, x0, half, iters=12, tol=1e-4, bg_px=3):
    """Windowed, re-centred centroid of one line in each row of ``profiles``.

    Each pass puts a window ``half`` px either side of the last estimate,
    subtracts a straight background through the pixels just outside it, and
    takes the intensity-weighted mean position. For a symmetric line the
    window ends up centred on the line, so the answer does not depend on the
    line's shape. The same estimator is used on the catalog's synthetic
    spectra, so blended lines are measured the same way on both sides.

    profiles: (n, W) or (W,) array. x0: start position(s).
    Returns (x, flux, ok) arrays of length n.
    """
    p = np.atleast_2d(np.asarray(profiles, float))
    n, w = p.shape
    x = np.broadcast_to(np.asarray(x0, float), (n,)).copy()
    half = float(half)
    m = int(np.ceil(half)) + bg_px + 2
    ok = np.isfinite(x)
    flux = np.zeros(n)
    for _ in range(iters):
        pos, vals, inside = _windows(p, x, m)
        lo, hi = x - half, x + half
        sig = (vals - _background(pos, vals, lo, hi, bg_px)) * _overlap(pos, lo, hi)
        flux = sig.sum(1)
        good = ok & (flux > 0) & inside.all(1)
        xn = np.where(good, (sig * pos).sum(1) / np.where(flux > 0, flux, 1.0), x)
        step = np.abs(xn - x)
        x = np.where(good, xn, x)
        ok = good
        if np.all(step[ok] < tol):
            break
    return x, flux, ok


def half_max_width(profiles, x0, half):
    """FWHM of the line near x0 in each row (half-maximum crossings, background removed)."""
    p = np.atleast_2d(np.asarray(profiles, float))
    n = p.shape[0]
    x = np.broadcast_to(np.asarray(x0, float), (n,))
    m = int(np.ceil(half)) + 6
    pos, vals, inside = _windows(p, x, m)
    sig = vals - _background(pos, vals, x - half, x + half, 3)
    out = np.full(n, np.nan)
    for i in range(n):
        if not inside[i].all():
            continue
        s = sig[i]
        core = np.abs(pos[i] - x[i]) <= half
        if not core.any():
            continue
        k = int(np.argmax(np.where(core, s, -np.inf)))
        peak = s[k]
        if peak <= 0:
            continue
        hm = 0.5 * peak
        left = k
        while left > 0 and s[left] > hm:
            left -= 1
        right = k
        while right < s.size - 1 and s[right] > hm:
            right += 1
        if s[left] > hm or s[right] > hm:
            continue
        xl = pos[i, left] + (hm - s[left]) / (s[left + 1] - s[left])
        xr = pos[i, right] - (hm - s[right]) / (s[right - 1] - s[right])
        out[i] = xr - xl
    return out


def binomial_smooth(img):
    """[1, 2, 1]/4 smoothing along both axes.

    Its response is exactly zero at a 2-pixel period, so it erases the
    sensor's 2 x 2 colour-filter pattern while blurring lines by only
    0.7 px."""
    a = np.asarray(img, float)
    out = a.copy()
    for axis in range(a.ndim):
        lo = np.take(out, [0], axis=axis)
        hi = np.take(out, [-1], axis=axis)
        pad = np.concatenate([lo, out, hi], axis=axis)
        n = out.shape[axis]
        sl = lambda s, e: tuple(slice(s, e) if ax == axis else slice(None)  # noqa: E731
                                for ax in range(a.ndim))
        out = 0.25 * pad[sl(0, n)] + 0.5 * pad[sl(1, n + 1)] + 0.25 * pad[sl(2, n + 2)]
    return out
