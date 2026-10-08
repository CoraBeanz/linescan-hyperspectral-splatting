"""Spectra to preview colours, the same maths as the splat renderer and the web viewer.

True colour and colour infrared are both linear in the spectrum, so each is a (bands, 3) matrix
of weights and a whole line's colours are one matrix product. The maths is a port of
splat/src/spectra.cpp (true_color, color_infrared) by way of viewer/js/spectral.js, so the live
waterfall, the trainer's previews and the viewer show a material in the same colour; the tests
check a few spectra against numbers from the viewer's code.

  true colour      the CIE 1931 observer under equal-energy light, white balanced so that
                   reflectance 1 is white. Below the first band (the GG-495 filter cuts there)
                   the first band's value is held, so blues are only approximate.
  colour infrared  the classic false colour: red shows 800-900 nm, green 620-680 nm and blue
                   520-580 nm, so living leaves come out red and green paint doesn't.
"""

import numpy as np

CIR_BANDS = ((800.0, 900.0), (620.0, 680.0), (520.0, 580.0))


def _add_sample_weights(wl, nm, out, scale=1.0):
    """Adds `scale` times the weights that interpolate a spectrum at nm: linear between band
    centres, held at the end values past either end."""
    n = len(wl)
    if nm <= wl[0]:
        out[0] += scale
    elif nm >= wl[n - 1]:
        out[n - 1] += scale
    else:
        i = int(np.searchsorted(wl, nm, side="right"))   # the first centre above nm
        t = (nm - wl[i - 1]) / (wl[i] - wl[i - 1])
        out[i - 1] += scale * (1.0 - t)
        out[i] += scale * t
    return out


def band_mean_weights(wl, lo, hi):
    """The mean over [lo, hi], sampled every nanometre."""
    wl = np.asarray(wl, float)
    out = np.zeros(len(wl))
    samples = np.arange(lo, hi + 1e-9, 1.0)
    for nm in samples:
        _add_sample_weights(wl, nm, out, 1.0 / len(samples))
    return out


def _lobe(x, mu, s1, s2):
    s = s1 if x < mu else s2
    return np.exp(-0.5 * (x - mu) ** 2 / (s * s))


def cie_xyz(nm):
    """The CIE 1931 colour matching functions, as the analytic fit of Wyman, Sloan and Shirley,
    "Simple Analytic Approximations to the CIE XYZ Color Matching Functions" (JCGT 2013)."""
    return (1.056 * _lobe(nm, 599.8, 37.9, 31.0) + 0.362 * _lobe(nm, 442.0, 16.0, 26.7)
            - 0.065 * _lobe(nm, 501.1, 20.4, 26.2),
            0.821 * _lobe(nm, 568.8, 46.9, 40.5) + 0.286 * _lobe(nm, 530.9, 16.3, 31.1),
            1.217 * _lobe(nm, 437.0, 11.8, 36.0) + 0.681 * _lobe(nm, 459.0, 26.0, 13.8))


XYZ_TO_LINEAR_SRGB = np.array([[3.2406, -1.5372, -0.4986],
                               [-0.9689, 1.8758, 0.0415],
                               [0.0557, -0.2040, 1.0570]])


def true_color_weights(wl):
    """(bands, 3) weights for linear sRGB R, G and B."""
    wl = np.asarray(wl, float)
    xyz = np.zeros((len(wl), 3))
    white = np.zeros(3)
    for nm in np.arange(380.0, 780.0 + 1e-9, 2.0):
        c = cie_xyz(nm)
        for k in range(3):
            _add_sample_weights(wl, nm, xyz[:, k], c[k])
        white += c
    xyz /= white[1]
    balance = XYZ_TO_LINEAR_SRGB @ (white / white[1])
    return (xyz @ XYZ_TO_LINEAR_SRGB.T) / balance


def color_infrared_weights(wl):
    """(bands, 3) weights: each channel the mean over its CIR band."""
    return np.stack([band_mean_weights(wl, lo, hi) for lo, hi in CIR_BANDS], axis=1)


MODES = {"true_color": true_color_weights, "cir": color_infrared_weights}


def weights(mode, wl):
    if mode not in MODES:
        raise ValueError("colour mode %r; use one of %s" % (mode, ", ".join(MODES)))
    return MODES[mode](wl)


def srgb8(linear):
    """Linear values (clipped to 0..1) to 8-bit sRGB."""
    x = np.clip(np.nan_to_num(np.asarray(linear, float)), 0.0, 1.0)
    s = np.where(x <= 0.0031308, 12.92 * x, 1.055 * np.power(x, 1.0 / 2.4) - 0.055)
    return np.round(255.0 * s).astype(np.uint8)
