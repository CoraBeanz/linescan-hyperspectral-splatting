"""Lamp line catalogs, and what the lamps look like at this spectrograph's resolution.

With a slit image about 23 px wide and lens blur on top, lines closer than about
5 nm merge into one bump, and the centre of a merged bump depends on how bright
each line is, which no catalog knows for your lamp. So each reference line is
fit together with its neighbours (see Group): the catalog fixes where the lines
sit relative to each other, the fit finds how bright each one is.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .design import DATA
from .profiles import slit_profile, slit_profile_fwhm

QUALITY_NM = {"A": 0.005, "B": 0.3, "C": 1.0}
FWHM_PER_SIGMA = 2.0 * np.sqrt(2.0 * np.log(2.0))
LAMPS = ("cfl", "neon")


@dataclass(frozen=True)
class Line:
    source: str
    species: str
    nm: float
    rel: float
    quality: str
    width: float = 0.0
    note: str = ""

    @property
    def label(self):
        return f"{self.species} {self.nm:.2f}"


def load_catalog(path=None):
    """All lines in data/lines.csv (or another CSV with the same columns)."""
    path = Path(path or DATA / "lines.csv")
    rows = [r for r in path.read_text().splitlines() if r.strip() and not r.startswith("#")]
    out = []
    for r in csv.DictReader(rows):
        out.append(Line(source=r["source"].strip(), species=r["species"].strip(),
                        nm=float(r["nm"]), rel=float(r["rel"]), quality=r["quality"].strip(),
                        width=float(r["width"] or 0.0), note=(r.get("note") or "").strip()))
    return out


def catalog(source, path=None):
    lines = [ln for ln in load_catalog(path) if ln.source == source]
    if not lines:
        raise ValueError(f"no lines for source {source!r} in the catalog")
    return lines


# Rough instrument response, used only to predict which lines show up and how
# blends weigh: a 495 nm long-pass edge times a generic silicon sensor with no
# IR-cut filter. The real response is measured from the halogen flat.
_QE_NM = np.array([450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950, 1000, 1050.0])
_QE = np.array([0.50, 0.60, 0.66, 0.62, 0.56, 0.48, 0.40, 0.32, 0.24, 0.15, 0.08, 0.035, 0.012])


def generic_response(nm):
    nm = np.asarray(nm, float)
    edge = 1.0 / (1.0 + np.exp(-(nm - 495.0) / 4.0))
    return edge * np.interp(nm, _QE_NM, _QE)


@dataclass
class Lsf:
    """Instrument line shape in nm: a box (the slit image) blurred by a Gaussian."""
    box_nm: float
    sigma_nm: float

    @property
    def fwhm(self):
        return slit_profile_fwhm(self.box_nm, self.sigma_nm)


def line_profiles(lines, grid, lsf, response=generic_response):
    """Each line's contribution to the spectrum on ``grid`` (n_lines x n_grid)."""
    out = np.zeros((len(lines), grid.size))
    for i, ln in enumerate(lines):
        flux = ln.rel * float(response(ln.nm))
        s = np.hypot(lsf.sigma_nm, ln.width / FWHM_PER_SIGMA)
        reach = 0.5 * lsf.box_nm + 8.0 * s
        sel = np.abs(grid - ln.nm) < reach
        out[i, sel] = slit_profile(grid[sel], ln.nm, flux / lsf.box_nm, lsf.box_nm, s)
    return out


def spectrum(lines, grid, lsf, response=generic_response):
    """Synthetic lamp spectrum on ``grid`` (nm) at this line shape."""
    return line_profiles(lines, grid, lsf, response).sum(0)


@dataclass
class Group:
    """A reference line plus every catalog line whose light lands near it.

    At this resolution neighbouring lines blend, so the camera's spectrum
    around a reference line is fit with all of them at once: their spacings
    come from the catalog (exact for atomic lines), their brightnesses are
    free. That makes the fit insensitive to the catalog's brightness guesses.
    """
    source: str
    ref: Line                  # the line whose position the group reports
    members: list              # all lines in the fit window, ref included
    lo_nm: float               # fit window
    hi_nm: float
    blend_nm: float = 0.0      # how far an unknown brightness ratio could move the fit
    purity: float = 1.0        # share of the light near the reference line that is its own

    @property
    def label(self):
        return f"{self.ref.species} {self.ref.nm:.2f}"


def groups(lines, fwhm_nm, nm_range=(480.0, 1020.0), response=generic_response,
           half=1.25, reach=1.2, min_share=0.5, min_strength=2e-4, merge=0.75, ratio_sigma=0.7):
    """One fit group per reference line that dominates its own neighbourhood.

    A reference is an atomic line (quality A) or a sharp phosphor peak (B)
    whose own light is at least ``min_share`` of the synthetic spectrum at its
    position. Lines closer than ``merge`` x FWHM fuse into one reference (the
    brighter one reports). The window spans ``half`` x FWHM each side; every
    line within ``reach`` x FWHM beyond the window joins the fit.

    Lines much closer than the FWHM cannot be told apart by shape, so their
    brightness ratio (known to a factor of about 2, ``ratio_sigma`` in log
    terms) moves the fitted position, and so does a phosphor band under the
    line, whose own peak is only known to about 1 nm. ``blend_nm`` estimates
    by how much, and the dispersion fit weights the group accordingly.
    """
    lo, hi = nm_range
    strength = np.array([ln.rel * float(response(ln.nm)) for ln in lines])
    keep = strength > min_strength * strength.max()
    lines = [ln for ln, k in zip(lines, keep) if k]
    strength = strength[keep]
    lsf = Lsf(box_nm=fwhm_nm, sigma_nm=0.25 * fwhm_nm)  # shape only matters loosely here
    nm = np.array([ln.nm for ln in lines])
    prof = line_profiles(lines, nm, lsf, response)       # prof[i, j]: line i at line j's position
    share = np.diag(prof) / np.maximum(prof.sum(0), 1e-30)
    out = []
    order = np.argsort(strength)[::-1]
    for i in order:
        ln = lines[i]
        if ln.quality not in "AB" or not lo <= ln.nm <= hi or share[i] < min_share:
            continue
        if any(abs(ln.nm - g.ref.nm) < merge * fwhm_nm for g in out):
            continue
        w_lo, w_hi = ln.nm - half * fwhm_nm, ln.nm + half * fwhm_nm
        members = [m for m in lines if w_lo - reach * fwhm_nm <= m.nm <= w_hi + reach * fwhm_nm]
        tot = prof[:, i].sum()
        terms = []
        for k, m in enumerate(lines):
            if m is ln or m not in members:
                continue
            f = prof[k, i] / tot
            if abs(m.nm - ln.nm) < 0.7 * fwhm_nm:  # too close to separate by shape
                terms.append((m.nm - ln.nm) * f * (1 - f) * ratio_sigma)
            if m.quality == "C":  # a phosphor band whose own peak is only known to ~1 nm
                terms.append(f * QUALITY_NM["C"])
        blend = float(np.sqrt(np.sum(np.square(terms)))) if terms else 0.0
        core = np.linspace(ln.nm - 0.75 * fwhm_nm, ln.nm + 0.75 * fwhm_nm, 31)
        pc = line_profiles(members, core, lsf, response)
        purity = float(pc[members.index(ln)].sum() / max(pc.sum(), 1e-30))
        out.append(Group(source=ln.source, ref=ln, members=members, lo_nm=w_lo, hi_nm=w_hi,
                         blend_nm=float(blend), purity=purity))
    return sorted(out, key=lambda g: g.ref.nm)
