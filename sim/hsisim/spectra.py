"""The spectral library: reflectance of the scene materials, the lamp, and preview colours.

The materials are the splat's synthetic ones (splat/src/spectra.cpp), curve for curve, so
a scene simulated here and a splat trained on it talk about the same spectra. Two are
added for checking the instrument: PTFE, the white reference, and a rare-earth tile whose
narrow absorption bands sit at known wavelengths, shaped after didymium glass (the
neodymium and praseodymium filter glass of welding goggles), so a scan shows whether
wavelengths come out right without a lamp in the scene.
"""

from __future__ import annotations

import numpy as np

from . import repo  # noqa: F401  (makes hsical importable)
from hsical.synth import planck  # noqa: E402

NM_RANGE = (440.0, 1060.0)  # covers the design map's 450-1050 nm with a margin


def wavelength_grid(step=2.0, lo=NM_RANGE[0], hi=NM_RANGE[1]):
    return np.arange(lo, hi + 1e-9, step)


def _sig(x):
    return 1.0 / (1.0 + np.exp(-x))


def _bump(x, mu, s):
    return np.exp(-0.5 * ((x - mu) / s) ** 2)


# Absorption bands of the rare-earth tile: centre (nm), sigma (nm), depth (0..1).
RARE_EARTH_BANDS = ((528.0, 4.0, 0.30), (584.0, 5.5, 0.60), (683.0, 4.5, 0.25), (742.0, 5.0, 0.45),
                    (803.0, 6.0, 0.50), (874.0, 7.0, 0.30))
PTFE_REFLECTANCE = 0.98  # what `hsical apply --white` assumes by default


def _rare_earth(nm):
    out = np.full_like(nm, 0.86)
    for c, s, d in RARE_EARTH_BANDS:
        out = out * (1.0 - d * _bump(nm, c, s))
    return out


MATERIALS = {
    # name: (reflectance(nm), what it stands for)
    "void": (lambda nm: 0.0 * nm, "nothing: a ray that leaves the scene"),
    "white_paper": (lambda nm: 0.86 - 0.05 * (nm - 500.0) / 450.0, "white paper"),
    "carbon_black": (lambda nm: 0.04 + 0.0 * nm, "carbon black, dark in the visible and the near infrared"),
    "ir_black_dye": (lambda nm: 0.04 + 0.74 * _sig((nm - 740.0) / 16.0),
                     "a dye that is black to the eye and clear past 740 nm"),
    "leaf": (lambda nm: 0.045 + 0.085 * _bump(nm, 553.0, 32.0) - 0.015 * _bump(nm, 676.0, 16.0)
             + 0.46 * _sig((nm - 714.0) / 13.0) - 0.05 * _bump(nm, 970.0, 22.0),
             "a leaf: green bump, red absorption, the red edge at 715 nm"),
    "red_paint": (lambda nm: 0.05 + 0.72 * _sig((nm - 603.0) / 11.0), "red paint"),
    "orange_plastic": (lambda nm: 0.06 + 0.74 * _sig((nm - 572.0) / 11.0), "orange plastic"),
    "yellow_paint": (lambda nm: 0.08 + 0.74 * _sig((nm - 518.0) / 10.0), "yellow paint"),
    "green_paint": (lambda nm: 0.05 + 0.26 * _bump(nm, 535.0, 32.0) + 0.40 * _sig((nm - 775.0) / 20.0),
                    "green paint: like the leaf to the eye, but no red edge"),
    "blue_paint": (lambda nm: 0.06 + 0.28 * (1.0 - _sig((nm - 535.0) / 18.0)) + 0.48 * _sig((nm - 745.0) / 22.0),
                   "blue paint"),
    "gray18": (lambda nm: 0.18 + 0.0 * nm, "18% gray card"),
    "wood": (lambda nm: 0.10 + 0.30 * _sig((nm - 610.0) / 45.0), "wood, the table"),
    "ptfe": (lambda nm: PTFE_REFLECTANCE + 0.0 * nm, "PTFE sheet, the white reference"),
    "rare_earth": (_rare_earth, "rare-earth tile: absorption bands shaped after didymium glass"),
}
NAMES = list(MATERIALS)


def material_id(name):
    try:
        return NAMES.index(name)
    except ValueError:
        raise ValueError(f"unknown material '{name}'; known: {', '.join(NAMES)}") from None


def reflectance(name, nm):
    return np.asarray(MATERIALS[name][0](np.asarray(nm, float)), float)


def table(nm, names=NAMES):
    """[materials, wavelengths] reflectance of every material, in id order."""
    nm = np.asarray(nm, float)
    return np.stack([reflectance(n, nm) for n in names])


def halogen(nm, temp_k=2850.0):
    """A halogen filament's spectrum (a blackbody), 1 at 700 nm."""
    return planck(nm, temp_k) / planck(700.0, temp_k)


# -- preview colours ------------------------------------------------------------------


def _lobe(x, mu, s1, s2):
    s = np.where(x < mu, s1, s2)
    return np.exp(-0.5 * ((x - mu) / s) ** 2)


def _cie_xyz(nm):
    """CIE 1931 colour matching functions, the analytic fit of Wyman, Sloan and Shirley (2013)."""
    x = 1.056 * _lobe(nm, 599.8, 37.9, 31.0) + 0.362 * _lobe(nm, 442.0, 16.0, 26.7) \
        - 0.065 * _lobe(nm, 501.1, 20.4, 26.2)
    y = 0.821 * _lobe(nm, 568.8, 46.9, 40.5) + 0.286 * _lobe(nm, 530.9, 16.3, 31.1)
    z = 1.217 * _lobe(nm, 437.0, 11.8, 36.0) + 0.681 * _lobe(nm, 459.0, 26.0, 13.8)
    return np.stack([x, y, z])


_XYZ_TO_RGB = np.array([[3.2406, -1.5372, -0.4986], [-0.9689, 1.8758, 0.0415], [0.0557, -0.2040, 1.0570]])


def true_color(spectra, nm):
    """Linear sRGB of reflectance spectra [..., wavelengths] under equal-energy light,
    white balanced so reflectance 1 is white. Below the first wavelength the first value
    is held, as the splat's previews do."""
    vis = np.arange(380.0, 781.0, 2.0)
    cmf = _cie_xyz(vis)                                  # [3, n]
    idx = np.interp(vis, nm, np.arange(len(nm)))         # fractional index into nm
    lo = np.floor(idx).astype(int)
    hi = np.minimum(lo + 1, len(nm) - 1)
    f = idx - lo
    s = np.asarray(spectra, float)
    r = s[..., lo] * (1 - f) + s[..., hi] * f            # [..., n]
    xyz = r @ cmf.T / cmf[1].sum()
    white = _XYZ_TO_RGB @ (cmf.sum(1) / cmf[1].sum())
    return (xyz @ _XYZ_TO_RGB.T) / white


def color_infrared(spectra, nm):
    """The classic CIR false colour: R = 800-900 nm, G = 620-680 nm, B = 520-580 nm."""
    s = np.asarray(spectra, float)
    bands = [(800.0, 900.0), (620.0, 680.0), (520.0, 580.0)]
    return np.stack([s[..., (nm >= a) & (nm <= b)].mean(-1) for a, b in bands], -1)


def to_srgb8(linear):
    x = np.clip(np.asarray(linear, float), 0.0, 1.0)
    s = np.where(x <= 0.0031308, 12.92 * x, 1.055 * np.power(x, 1 / 2.4) - 0.055)
    return np.round(255 * s).astype(np.uint8)
