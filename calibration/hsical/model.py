"""A finished calibration: load it, and turn raw scan frames into spectra.

    cal = Calibration.load("cal/")
    cube_line = cal.radiance(raw, dark, exposure_us=30000)   # (slit rows, wavelengths)
    cal.nm_grid, cal.s_grid                                   # the axes of that array

Saved as two files: calibration.json (every number and fit result, readable
by anything) and maps.npz (per-pixel maps: wavelength, slit position,
response, bad pixels, and the rectifying warp). numpy.load reads the npz; from
C++, cnpy does, or re-save the arrays as raw float32.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.ndimage import uniform_filter1d

from .frames import Orientation
from .geometry import Keystone, rectify_maps, remap
from .poly import Poly2D
from .profiles import binomial_smooth
from .wavecal import Table1D

FORMAT = "hsical-1"


@dataclass
class Calibration:
    width: int                      # frame size after orientation (x = spectrum)
    height: int
    orientation: Orientation
    y0: float                       # row where the wavelength table holds exactly
    wavelength: Table1D             # lambda_c(x) along row y0, one value per column
    smile: Poly2D                   # delta(x, y): lambda(x, y) = lambda_c(x - delta)
    keystone: Keystone              # s(x, y) = y + Delta(x, y)
    nm_grid: np.ndarray             # wavelengths of the rectified output
    s_grid: np.ndarray              # slit rows of the rectified output
    response: np.ndarray = None     # R(x, y): counts/s per unit relative radiance
    bad: np.ndarray = None          # pixels to ignore
    temp_k: float = 2850.0
    info: dict = field(default_factory=dict)
    _maps: tuple = field(default=None, repr=False)

    # -- per-pixel coordinates --------------------------------------------
    def wavelength_at(self, x, y):
        x = np.asarray(x, float)
        return self.wavelength(x - self.smile(x, y))

    def slit_at(self, x, y):
        return self.keystone.s(x, y)

    def _per_pixel(self, f, block=256):
        out = np.empty((self.height, self.width), np.float32)
        xs = np.arange(self.width, dtype=float)
        for r in range(0, self.height, block):
            X, Y = np.meshgrid(xs, np.arange(r, min(r + block, self.height), dtype=float))
            out[r:r + X.shape[0]] = f(X, Y)
        return out

    def wavelength_map(self):
        """Wavelength (nm) seen by every pixel."""
        return self._per_pixel(self.wavelength_at)

    def slit_map(self):
        """Slit coordinate (rows at the reference column) seen by every pixel."""
        return self._per_pixel(self.slit_at)

    # -- rectification ----------------------------------------------------
    def maps(self):
        """(map_x, map_y): source pixel of each output cell (slit row, wavelength)."""
        if self._maps is None:
            self._maps = rectify_maps(self.wavelength.inverse, self.smile, self.keystone.delta,
                                      self.nm_grid, self.s_grid)
        return self._maps

    @property
    def px_per_cell(self):
        """Columns covered by one output wavelength step."""
        step = float(np.median(np.diff(self.nm_grid))) if self.nm_grid.size > 1 else 1.0
        x = self.wavelength.inverse(np.array([750.0 - step / 2, 750.0 + step / 2]))
        return float(abs(x[1] - x[0]))

    def _smooth(self, img):
        """Average over one output cell: [1 2 1] along the slit, a box of the wavelength step along x."""
        a = binomial_smooth(np.where(np.isfinite(img), img, 0.0))
        n = int(round(self.px_per_cell))
        return uniform_filter1d(a, n, axis=1, mode="nearest") if n >= 2 else a

    def rectify(self, img):
        """Resample a per-pixel image onto (s_grid, nm_grid), averaging over each cell."""
        mx, my = self.maps()
        return remap(self._smooth(img), mx, my)

    # -- radiometry -------------------------------------------------------
    def orient(self, raw):
        return self.orientation.apply(np.asarray(raw))

    def counts(self, raw, dark, exposure_us, gain=1.0):
        """Dark-subtracted counts per second at gain 1, oriented, bad pixels zeroed.

        raw, dark: frames as the camera gives them (dark may be a number, the
        black level)."""
        c = self.orient(np.asarray(raw, float)) - (self.orient(np.asarray(dark, float))
                                                    if np.ndim(dark) else float(dark))
        c = c / (exposure_us * 1e-6 * gain)
        if self.bad is not None:
            c = np.where(self.bad, 0.0, c)
        return c

    def radiance(self, raw, dark, exposure_us, gain=1.0, min_response=0.02):
        """Relative spectral radiance on (s_grid, nm_grid).

        Units: the halogen flat's blackbody at temp_k, normalised to 1 at 700 nm.
        Within each output cell the counts and the response are summed
        separately and then divided, which weights every pixel by how much
        light it actually gets (the mosaic's dim pixels count for less)."""
        if self.response is None:
            raise ValueError("this calibration has no spectral response (no halogen flat)")
        c = self.counts(raw, dark, exposure_us, gain)
        R = np.where(self.bad, 0.0, self.response) if self.bad is not None else self.response
        num, den = self.rectify(c), self.rectify(R)
        with np.errstate(invalid="ignore", divide="ignore"):
            out = num / den
        return np.where(den > min_response * np.nanmax(den), out, np.nan).astype(np.float32)

    def reflectance(self, raw, dark, exposure_us, white, white_dark, white_exposure_us, gain=1.0,
                    white_gain=1.0, white_reflectance=0.98, min_frac=0.02):
        """Reflectance on (s_grid, nm_grid) against a white reference (PTFE, 0.98 by default).

        The lamp's spectrum and the instrument's response cancel; no
        temperature is needed."""
        a = self.rectify(self.counts(raw, dark, exposure_us, gain))
        b = self.rectify(self.counts(white, white_dark, white_exposure_us, white_gain))
        with np.errstate(invalid="ignore", divide="ignore"):
            out = white_reflectance * a / b
        return np.where(b > min_frac * np.nanmax(b), out, np.nan).astype(np.float32)

    # -- files ------------------------------------------------------------
    def to_dict(self):
        d = dict(
            format=FORMAT,
            sensor=dict(width=self.width, height=self.height, orientation=self.orientation.to_dict()),
            wavelength=dict(y0=self.y0, table_nm=[round(float(v), 5) for v in self.wavelength.values],
                            sigma_nm=None if self.wavelength.sigma is None
                            else [round(float(v), 5) for v in self.wavelength.sigma]),
            smile=self.smile.to_dict(),
            keystone=self.keystone.to_dict(),
            rectified=dict(nm=[float(self.nm_grid[0]), float(self.nm_grid[-1]), len(self.nm_grid)],
                           s=[float(self.s_grid[0]), float(self.s_grid[-1]), len(self.s_grid)]),
            response=dict(temp_k=self.temp_k, has_map=self.response is not None),
        )
        d.update({k: v for k, v in self.info.items() if k not in d})
        return d

    def save(self, out_dir, include_pixel_maps=True):
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        (out / "calibration.json").write_text(json.dumps(_jsonable(self.to_dict()), indent=1) + "\n")
        mx, my = self.maps()
        arrays = dict(map_x=mx, map_y=my, nm_grid=self.nm_grid.astype(np.float32),
                      s_grid=self.s_grid.astype(np.float32))
        if self.response is not None:
            arrays["response"] = self.response.astype(np.float32)
        if self.bad is not None:
            arrays["bad"] = self.bad.astype(bool)
        if include_pixel_maps:
            arrays["wavelength"] = self.wavelength_map()
            arrays["slit"] = self.slit_map()
        np.savez_compressed(out / "maps.npz", **arrays)
        return out

    @classmethod
    def load(cls, path):
        path = Path(path)
        if path.is_file():
            path = path.parent
        d = json.loads((path / "calibration.json").read_text())
        if d.get("format") != FORMAT:
            raise ValueError(f"{path} is not an {FORMAT} calibration")
        z = np.load(path / "maps.npz")
        sens = d["sensor"]
        wl = d["wavelength"]
        cal = cls(width=sens["width"], height=sens["height"],
                  orientation=Orientation(**sens["orientation"]), y0=wl["y0"],
                  wavelength=Table1D(wl["table_nm"], wl.get("sigma_nm")),
                  smile=Poly2D.from_dict(d["smile"]), keystone=Keystone.from_dict(d["keystone"]),
                  nm_grid=np.asarray(z["nm_grid"], float), s_grid=np.asarray(z["s_grid"], float),
                  response=z["response"] if "response" in z else None,
                  bad=z["bad"] if "bad" in z else None, temp_k=d["response"]["temp_k"],
                  info={k: v for k, v in d.items() if k not in ("format", "sensor", "wavelength", "smile",
                                                                 "keystone", "rectified", "response")})
        cal._maps = (z["map_x"], z["map_y"])
        return cal


def _jsonable(o):
    if isinstance(o, dict):
        return {str(k): _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    if isinstance(o, np.ndarray):
        return _jsonable(o.tolist())
    if isinstance(o, (np.floating, float)):
        return None if not np.isfinite(o) else float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o
