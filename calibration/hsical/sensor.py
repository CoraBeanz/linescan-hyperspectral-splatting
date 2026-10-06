"""Sensor-level clean-up: dark frames, bad pixels and the colour filter mosaic.

The IMX219 in a NoIR camera still has its Bayer colour filters. Between about
580 and 780 nm the red pixels pass ten times more light than the green and
blue ones, so a raw frame of a spectrum is striped with a 2 x 2 pattern. A
halogen flat sees the same stripes at the same wavelengths, so dividing a frame
by the flat's pixel-to-pixel pattern (not by the whole flat, which would also
divide out the lamp's colour) leaves a frame in which every pixel answers like
the average of the four filters. Line fits then see clean line shapes.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import median_filter

from .profiles import binomial_smooth


def dark_for(light, darks):
    """Mean dark frame for a light FrameSet, and a note on how it was made.

    Uses the dark set at the same exposure and gain when there is one.
    Otherwise scales the dark current between the two nearest exposures (the
    black level stays put), and failing that subtracts a flat black level."""
    from .frames import match_dark
    d = match_dark(light, darks)
    if d is not None:
        return d.summary()[0], f"dark {d.name}"
    usable = [x for x in darks if x.exposure_us and abs(x.gain / light.gain - 1) <= 0.02]
    if len(usable) >= 2 and light.exposure_us:
        usable.sort(key=lambda x: abs(x.exposure_us - light.exposure_us))
        a, b = usable[0], usable[1]
        ma, mb = a.summary()[0], b.summary()[0]
        t = (light.exposure_us - a.exposure_us) / (b.exposure_us - a.exposure_us)
        return ma + t * (mb - ma), f"dark interpolated from {a.name} and {b.name}"
    if usable:
        return usable[0].summary()[0], f"dark {usable[0].name} (different exposure)"
    return None, "no dark frames: subtracted a black level of 64"


@dataclass
class BadPixels:
    """Pixels to ignore: hot (bright in the dark), dead or odd (wrong in the flat)."""
    mask: np.ndarray
    hot: int = 0
    odd: int = 0

    def to_dict(self):
        return dict(hot=self.hot, odd=self.odd, total=int(self.mask.sum()))


def find_bad_pixels(dark=None, flat=None, dark_sigma=8.0, flat_tol=0.25, shape=None):
    """Bad pixel mask from a long dark (hot pixels) and a flat (dead or odd pixels).

    Each pixel is compared with its same-colour neighbours (2 px away), so the
    colour mosaic does not count as a defect."""
    shape = shape or (dark.shape if dark is not None else flat.shape)
    mask = np.zeros(shape, bool)
    hot = odd = 0
    if dark is not None:
        d = np.asarray(dark, float)
        ref = _same_colour_median(d)
        r = d - ref
        mad = 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-6
        m = r > dark_sigma * mad + 2.0
        hot = int(m.sum())
        mask |= m
    if flat is not None:
        f = np.asarray(flat, float)
        ref = _same_colour_median(f)
        lit = ref > 0.05 * np.percentile(ref, 99.5)
        m = lit & (np.abs(f / np.where(lit, ref, 1.0) - 1.0) > flat_tol)
        odd = int((m & ~mask).sum())
        mask |= m
    return BadPixels(mask=mask, hot=hot, odd=odd)


def _same_colour_median(a):
    """Median of each pixel's 8 same-colour neighbours (a 5 x 5 grid, step 2)."""
    fp = np.zeros((5, 5), bool)
    fp[::2, ::2] = True
    fp[2, 2] = False
    return median_filter(a, footprint=fp, mode="mirror")


def fill_bad(img, mask):
    """Replace masked pixels with the median of their same-colour neighbours."""
    if mask is None or not mask.any():
        return img
    out = np.array(img, float)
    out[mask] = _same_colour_median(out)[mask]
    return out


def cfa_gain(flat, sat_mask=None, min_frac=0.03, rows=9):
    """Pixel-to-pixel gain pattern of a dark-subtracted flat: flat / smoothed flat.

    Divide light frames by it to take out the colour mosaic. Where the flat is
    too dim to say (blue of the long-pass filter, outside the slit), the gain
    is 1. The mosaic's effect depends on wavelength, so it changes along x but
    hardly along the slit; the ratio is therefore median-filtered over ``rows``
    rows of the same colour, which keeps the flat's own noise out of the gain."""
    f = np.asarray(flat, float)
    s = binomial_smooth(f)
    lit = s > min_frac * np.percentile(s, 99.5)
    if sat_mask is not None:
        lit &= ~sat_mask
    r = np.where(lit, f / np.where(lit, s, 1.0), np.nan)
    g = np.ones_like(f)
    for p in (0, 1):  # even rows, odd rows: same colours within each
        sub = r[p::2]
        filled = np.where(np.isnan(sub), 1.0, sub)
        med = median_filter(filled, size=(rows, 1), mode="nearest")
        g[p::2] = np.where(np.isnan(sub), 1.0, med)
    return np.clip(g, 0.02, 50.0)


def prepare(raw_mean, dark, gain=None, bad=None, black=64.0):
    """Dark-subtracted, mosaic-equalised frame with bad pixels filled in."""
    img = np.asarray(raw_mean, float) - (dark if dark is not None else black)
    img = fill_bad(img, None if bad is None else bad.mask)
    if gain is not None:
        img = img / gain
    return img
