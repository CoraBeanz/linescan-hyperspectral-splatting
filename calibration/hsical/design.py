"""The design's slit-to-sensor map, traced in Optiland.

tools/export_optiland_map.py traces chief rays through optics/spectrograph_model.py
and stores where each (slit position, wavelength) pair lands on the sensor.
The calibration uses it as a starting guess (which column should see which
wavelength) and the synthetic frame renderer uses it as ground truth.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from scipy.interpolate import RectBivariateSpline

DATA = Path(__file__).resolve().parent / "data"
FULL_SENSOR = (3280, 2464)  # IMX219 active pixels: columns (spectrum), rows (slit)


def interp_extrap(x, xp, fp):
    """np.interp, but continuing the end segments' slopes past the ends."""
    x0 = np.asarray(x, float)
    x = np.atleast_1d(x0)
    y = np.interp(x, xp, fp)
    lo, hi = x < xp[0], x > xp[-1]
    if np.any(lo):
        y[lo] = fp[0] + (x[lo] - xp[0]) * (fp[1] - fp[0]) / (xp[1] - xp[0])
    if np.any(hi):
        y[hi] = fp[-1] + (x[hi] - xp[-1]) * (fp[-1] - fp[-2]) / (xp[-1] - xp[-2])
    return y.reshape(x0.shape)


class DesignMap:
    """Where the design puts slit position ``h`` and wavelength on the sensor.

    h runs from -1 to +1 between the two ends of the slit. Pixel coordinates
    follow numpy and OpenCV: column x, row y, pixel centres on integers. The
    spectrum runs along x, with wavelength increasing with x; the slit runs
    along y.

    ``scale`` > 1 describes a binned sensor (scale 2 = 1640 x 1232 pixels for
    the same field of view).
    """

    def __init__(self, width=None, height=None, scale=1.0, path=None):
        d = json.loads(Path(path or DATA / "optiland_map.json").read_text())
        self.meta = d
        self.config = d["config"]
        self.scale = float(scale)
        self.width = int(width or round(FULL_SENSOR[0] / self.scale))
        self.height = int(height or round(FULL_SENSOR[1] / self.scale))
        self.pixel_mm = d["pixel_mm"] * self.scale
        h = np.asarray(d["h"], float)
        nm = np.asarray(d["nm"], float)
        rows = np.asarray(d["u_mm"]) / self.pixel_mm + (self.height - 1) / 2.0
        cols = np.asarray(d["v_mm"]) / self.pixel_mm + (self.width - 1) / 2.0
        width_px = np.asarray(d["slit_width_mm"]) / self.pixel_mm
        self.h_range = (float(h[0]), float(h[-1]))
        self.nm_range = (float(nm[0]), float(nm[-1]))
        self._row = RectBivariateSpline(h, nm, rows, kx=3, ky=3)
        self._col = RectBivariateSpline(h, nm, cols, kx=3, ky=3)
        self._w = RectBivariateSpline(h, nm, width_px, kx=3, ky=3)
        vh = np.asarray(d["vignetting_h"], float)
        vnm = np.asarray(d["vignetting_nm"], float)
        self._vig = RectBivariateSpline(vh, vnm, np.asarray(d["vignetting"], float), kx=1, ky=3)
        nm_fine = np.linspace(self.nm_range[0], self.nm_range[1], 1201)
        self._nm_fine = nm_fine

    # -- forward map -------------------------------------------------------
    def _ev(self, spline, h, nm):
        """Spline value, continued linearly past the traced grid (splines clamp there)."""
        h, nm = np.broadcast_arrays(np.asarray(h, float), np.asarray(nm, float))
        hc = np.clip(h, *self.h_range)
        nc = np.clip(nm, *self.nm_range)
        out = spline.ev(hc, nc)
        dh, dn = h - hc, nm - nc
        if np.any(dh):
            out = out + dh * spline.ev(hc, nc, dx=1)
        if np.any(dn):
            out = out + dn * spline.ev(hc, nc, dy=1)
        return out

    def row(self, h, nm):
        return self._ev(self._row, h, nm)

    def col(self, h, nm):
        return self._ev(self._col, h, nm)

    def slit_image_px(self, h, nm):
        """Width of the slit's image along the dispersion, in pixels."""
        return self._ev(self._w, h, nm)

    def vignetting(self, h, nm):
        """Share of the light that reaches the sensor (1 = nothing lost)."""
        return np.clip(self._vig.ev(np.minimum(np.abs(h), 1.0), nm), 0.0, 1.0)

    def dispersion(self, nm, h=0.0):
        """Pixels per nm along x, at slit position h."""
        nm = np.clip(np.asarray(nm, float), *self.nm_range)
        return self._col.ev(np.full_like(nm, h), nm, dy=1)

    # -- handy derived numbers --------------------------------------------
    def wavelength_at(self, x, h=0.0):
        """Wavelength (nm) the design puts on column(s) x at slit position h."""
        cols = self.col(np.full_like(self._nm_fine, h), self._nm_fine)
        return interp_extrap(x, cols, self._nm_fine)

    def slit_rows(self, nm=750.0):
        """Rows of the two slit ends at one wavelength."""
        return float(self.row(-1.0, nm)), float(self.row(1.0, nm))

    def smile_px(self, nm, h=1.0):
        """How far (px, along x) a line bows at slit position h versus the centre."""
        return float(self.col(h, nm) - self.col(0.0, nm))

    @property
    def resolution_nm(self):
        """Slit-limited bandpass: slit width x grating period / collimator focal length.

        The grating widens the slit image toward the red (anamorphic
        magnification) at the same rate as it widens the dispersion, so in nm
        the bandpass is the same at every wavelength."""
        c = self.config
        return c["slit_width"] * (1e6 / c["lines_per_mm"]) / c["f_coll"]
