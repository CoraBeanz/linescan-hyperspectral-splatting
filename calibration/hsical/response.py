"""Spectral response: how many counts a pixel gives for a known spectrum.

A halogen bulb is close to a blackbody at its filament temperature (about
2800-3000 K for a 12 V halogen, so pick the number on the bulb's datasheet or
leave the default). Bounced off PTFE, which reflects 97-99% of everything from
400 to 1100 nm, it makes an even, known spectrum along the whole slit. Each
pixel's response is then

    R(x, y) = flat counts per second / blackbody(lambda(x, y), T),

which holds the camera's quantum efficiency, the colour filter in front of that
pixel, the grating's efficiency, the long-pass filter, vignetting and any dust
on the slit, all at once. Dividing a scan frame by R gives relative spectral
radiance; dividing by a white-reference frame instead gives reflectance, and
then T drops out entirely.
"""

from __future__ import annotations

import numpy as np

C2 = 1.4388e7  # second radiation constant, nm K


def planck_rel(nm, temp_k, ref_nm=700.0):
    """Blackbody spectral radiance at temp_k, relative to its value at ref_nm."""
    nm = np.asarray(nm, float)
    return (ref_nm / nm) ** 5 * np.expm1(C2 / (ref_nm * temp_k)) / np.expm1(C2 / (nm * temp_k))


def response_map(flat, nm_map, exposure_us, temp_k, gain=1.0, inside=None):
    """Counts per second (at gain 1) per unit of relative blackbody radiance, per pixel.

    flat: dark-subtracted halogen-on-PTFE frame (not mosaic-equalised).
    inside: optional mask of pixels inside the slit image; others are set to 0."""
    rate = np.asarray(flat, float) / (exposure_us * 1e-6 * gain)
    with np.errstate(invalid="ignore", over="ignore"):
        r = rate / planck_rel(np.clip(nm_map, 300.0, 1200.0), temp_k)
    r = np.where(np.isfinite(r), r, 0.0)
    if inside is not None:
        r = np.where(inside, r, 0.0)
    return np.maximum(r, 0.0).astype(np.float32)


def channel_curves(R, rows, nm_of_col):
    """Response of each colour channel along x, averaged over ``rows``.

    The mosaic's layout depends on the sensor's readout and on how the frame
    was turned, so it is read off the data: the red pixels are the ones that
    respond most at 650-750 nm, the blue ones sit diagonally from them.
    Returns {"R", "G", "B", "all": (n_cols,) arrays} and the red pixel's
    (row, column) parity."""
    rows = np.asarray(rows, int)
    R = np.asarray(R, float)
    W = R.shape[1]
    nm = np.asarray(nm_of_col, float)
    red_cols = (nm > 650) & (nm < 750)
    sub = {}
    for rp in (0, 1):
        rr = rows[rows % 2 == rp]
        for cp in (0, 1):
            m = np.full(W, np.nan)
            m[cp::2] = R[rr][:, cp::2].mean(0) if rr.size else np.nan
            sub[(rp, cp)] = m
    if red_cols.any():
        red = max(sub, key=lambda k: np.nanmean(sub[k][red_cols]))
    else:
        red = (0, 0)
    blue = (1 - red[0], 1 - red[1])
    greens = [k for k in sub if k not in (red, blue)]

    def smooth(m):
        good = np.isfinite(m)
        return np.interp(np.arange(W), np.flatnonzero(good), m[good]) if good.any() else m

    out = {"R": smooth(sub[red]), "B": smooth(sub[blue]),
           "G": 0.5 * (smooth(sub[greens[0]]) + smooth(sub[greens[1]])),
           "all": R[rows].mean(0)}
    return out, red


def slit_profile(flat_rect, nm_grid, nm_lo=550.0, nm_hi=900.0):
    """Relative brightness along the slit, from a rectified flat (rows = slit, columns = nm).

    Each wavelength column is divided by its median along the slit, then the
    median over wavelengths is taken: what is left is the slit's own
    unevenness (dust, a slit narrower at one end), the same at every wavelength."""
    sel = (np.asarray(nm_grid) >= nm_lo) & (np.asarray(nm_grid) <= nm_hi)
    a = np.asarray(flat_rect, float)[:, sel]
    with np.errstate(invalid="ignore"):
        a = a / np.nanmedian(a, axis=0, keepdims=True)
    return np.nanmedian(a, axis=1)


def find_dust(profile, min_depth=0.03, width=25):
    """Dips in a slit profile deeper than ``min_depth``: (row, depth) pairs."""
    from scipy.ndimage import median_filter
    from scipy.signal import find_peaks
    p = np.asarray(profile, float)
    good = np.isfinite(p)
    if good.sum() < 3 * width:
        return []
    q = np.interp(np.arange(p.size), np.flatnonzero(good), p[good])
    trend = median_filter(q, size=4 * width + 1, mode="nearest")
    d = 1.0 - q / np.maximum(trend, 1e-9)
    pk, props = find_peaks(d, height=min_depth, distance=width)
    return [(int(i), float(h)) for i, h in zip(pk, props["peak_heights"])]
