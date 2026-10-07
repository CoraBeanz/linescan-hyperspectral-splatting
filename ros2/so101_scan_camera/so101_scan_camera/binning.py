"""Turning a raw spectrograph frame into one scan line: slit positions x wavelengths.

A calibration from the kit in calibration/ (python -m hsical calibrate) knows the wavelength
and the slit position every sensor pixel sees: maps.npz holds them as `wavelength` and `slit`
(and `bad`, `response`), in the calibration's turned frame, and calibration.json says how to
turn a raw frame into that frame and where the slit's two ends are. LineBinner cuts the
slit into `slit_bins` equal pieces and the band into `nm_step` wide bins, and every output cell
is the mean of the raw pixels whose wavelength and slit position fall in it. Bad pixels are
left out, and a cell with a saturated pixel comes out NaN.

That is the same estimate hsical's Calibration.radiance makes with its warp, done as a sum:
summing a cell's counts and dividing by the cell's summed response weights each pixel by how
much light it gets, so the colour mosaic's dim pixels don't add noise. The mean raw counts are
kept (not radiance), so darks, whites and the response can be applied later, exactly, by the
converter (scan_to_dataset.py), with the per-cell response and pixel counts saved alongside.

Binning is linear, so mean(raw - dark) = mean(raw) - mean(dark) over the same pixels, and one
frame costs one bincount over the rows the slit's image covers (a few ms for a full frame).
"""

import hashlib
import json
import os
from dataclasses import dataclass

import numpy as np

from so101_scan_camera.sources import FULL_SCALE

SATURATED = FULL_SCALE - 2    # hsical counts a pixel within 2 counts of full scale as clipped


@dataclass
class Orientation:
    """How hsical turns a camera frame so the spectrum runs along x, blue on the left:
    transpose, then mirror x, then mirror y (hsical.frames.Orientation)."""
    transpose: bool = False
    flip_x: bool = False
    flip_y: bool = False

    def apply(self, a):
        if self.transpose:
            a = np.swapaxes(a, -1, -2)
        if self.flip_x:
            a = a[..., ::-1]
        if self.flip_y:
            a = a[..., ::-1, :]
        return np.ascontiguousarray(a)

    def unapply(self, a):
        """The camera frame a turned frame came from."""
        if self.flip_y:
            a = a[..., ::-1, :]
        if self.flip_x:
            a = a[..., ::-1]
        if self.transpose:
            a = np.swapaxes(a, -1, -2)
        return np.ascontiguousarray(a)

    def raw_shape(self, oriented_shape):
        h, w = oriented_shape
        return (w, h) if self.transpose else (h, w)

    def to_dict(self):
        return dict(transpose=self.transpose, flip_x=self.flip_x, flip_y=self.flip_y)


@dataclass
class CalibrationMaps:
    """What the binner needs from an hsical calibration folder."""
    path: str
    orientation: Orientation
    wavelength: np.ndarray       # (H', W') nm, turned frame
    slit: np.ndarray             # (H', W') slit coordinate s (rows at the reference column)
    s_top: float                 # the slit's ends in s
    s_bottom: float
    nm_grid: np.ndarray          # the calibration's own output wavelengths
    response: np.ndarray = None  # (H', W') counts/s per unit radiance at gain 1
    bad: np.ndarray = None       # (H', W') pixels to leave out
    temp_k: float = 2850.0
    sha256: str = ""

    @property
    def raw_shape(self):
        return self.orientation.raw_shape(self.wavelength.shape)


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_calibration(path):
    """Read calibration.json and maps.npz from an hsical calibration folder."""
    if os.path.isfile(path):
        path = os.path.dirname(path)
    with open(os.path.join(path, "calibration.json")) as f:
        d = json.load(f)
    if d.get("format") != "hsical-1":
        raise ValueError("%s is not an hsical calibration (format %r)" % (path, d.get("format")))
    maps_path = os.path.join(path, "maps.npz")
    z = np.load(maps_path)
    if "wavelength" not in z or "slit" not in z:
        raise ValueError("%s has no per-pixel wavelength and slit maps; recalibrate with a current hsical"
                         % maps_path)
    o = d["sensor"]["orientation"]
    ks = d["keystone"]
    return CalibrationMaps(
        path=os.path.abspath(path),
        orientation=Orientation(bool(o.get("transpose")), bool(o.get("flip_x")), bool(o.get("flip_y"))),
        wavelength=np.asarray(z["wavelength"], np.float32), slit=np.asarray(z["slit"], np.float32),
        s_top=float(ks["s_top"]), s_bottom=float(ks["s_bottom"]), nm_grid=np.asarray(z["nm_grid"], float),
        response=np.asarray(z["response"], np.float32) if "response" in z else None,
        bad=np.asarray(z["bad"], bool) if "bad" in z else None,
        temp_k=float(d.get("response", {}).get("temp_k", 2850.0)), sha256=file_sha256(maps_path))


def nm_edges_for(maps, nm_step=None, nm_range=None):
    """Bin edges along the spectrum: the calibration's own grid unless told otherwise."""
    grid = maps.nm_grid
    step = float(nm_step) if nm_step else (float(np.median(np.diff(grid))) if grid.size > 1 else 2.0)
    lo, hi = (float(nm_range[0]), float(nm_range[1])) if nm_range else (grid[0] - step / 2, grid[-1] + step / 2)
    n = max(1, int(round((hi - lo) / step)))
    return np.linspace(lo, lo + n * step, n + 1)


class LineBinner:
    def __init__(self, maps, slit_bins=256, nm_step=None, nm_range=None):
        self.maps = maps
        self.raw_shape = maps.raw_shape
        self.slit_edges = np.linspace(maps.s_top, maps.s_bottom, int(slit_bins) + 1)
        self.nm_edges = nm_edges_for(maps, nm_step, nm_range)
        self.shape = (len(self.slit_edges) - 1, len(self.nm_edges) - 1)
        self.n_cells = n_cells = self.shape[0] * self.shape[1]

        s, nm = maps.slit, maps.wavelength
        i_s = np.searchsorted(self.slit_edges, s, side="right") - 1
        i_nm = np.searchsorted(self.nm_edges, nm, side="right") - 1
        ok = (i_s >= 0) & (i_s < self.shape[0]) & (i_nm >= 0) & (i_nm < self.shape[1]) & np.isfinite(s) \
            & np.isfinite(nm)
        if maps.bad is not None:
            ok &= ~maps.bad
        # every raw pixel's cell, n_cells for the ones left out, over the raw rows that have any
        # (so a frame is binned by one bincount over those rows, with no gather)
        cell = maps.orientation.unapply(np.where(ok, i_s * self.shape[1] + i_nm, n_cells))
        used = np.flatnonzero((cell < n_cells).any(axis=1))
        self.rows = (int(used[0]), int(used[-1])) if used.size else (0, self.raw_shape[0] - 1)
        self.dst = np.ascontiguousarray(cell[self.rows[0]:self.rows[1] + 1].reshape(-1), dtype=np.intp)
        self.pixels = np.bincount(self.dst, minlength=n_cells + 1)[:n_cells].reshape(self.shape)
        self.empty = self.pixels == 0
        self.response = None
        if maps.response is not None:
            r = maps.orientation.unapply(np.asarray(maps.response, np.float64))[self.rows[0]:self.rows[1] + 1]
            with np.errstate(invalid="ignore", divide="ignore"):
                self.response = (np.bincount(self.dst, weights=r.reshape(-1), minlength=n_cells + 1)[:n_cells]
                                 .reshape(self.shape) / self.pixels).astype(np.float32)

    @property
    def slit_centres(self):
        return 0.5 * (self.slit_edges[:-1] + self.slit_edges[1:])

    @property
    def nm_centres(self):
        return 0.5 * (self.nm_edges[:-1] + self.nm_edges[1:])

    def check(self, image):
        if tuple(image.shape) != tuple(self.raw_shape):
            raise ValueError("frames are %dx%d but the calibration is for %dx%d frames: calibrate in the sensor "
                             "mode you scan in (python3 -m hsical plan ... --width %d --height %d)"
                             % (image.shape[1], image.shape[0], self.raw_shape[1], self.raw_shape[0],
                                image.shape[1], image.shape[0]))

    def bin(self, image):
        """(mean raw counts per cell as float32 [slit_bins, bands], saturated pixel count)."""
        self.check(image)
        v = np.asarray(image)[self.rows[0]:self.rows[1] + 1].reshape(-1)
        n = self.n_cells
        sums = np.bincount(self.dst, weights=v, minlength=n + 1)[:n]
        with np.errstate(invalid="ignore", divide="ignore"):
            mean = (sums.reshape(self.shape) / self.pixels).astype(np.float32)
        mean[self.empty] = np.nan
        hot = self.dst[np.flatnonzero(v >= SATURATED)]
        hot = hot[hot < n]                  # hot pixels the calibration marked bad don't count
        if hot.size:
            mean.reshape(-1)[np.unique(hot)] = np.nan
        return mean, int(hot.size)

    def save(self, path, **extra):
        """binning.npz: the grid, so a session's binned lines can be read without the calibration."""
        np.savez(path, slit_edges=self.slit_edges, nm_edges=self.nm_edges, pixels=self.pixels.astype(np.int32),
                 response=self.response if self.response is not None else np.zeros(0, np.float32),
                 raw_shape=np.array(self.raw_shape, np.int32), rows=np.array(self.rows, np.int32),
                 orientation=np.array([self.maps.orientation.transpose, self.maps.orientation.flip_x,
                                       self.maps.orientation.flip_y]),
                 s_ends=np.array([self.maps.s_top, self.maps.s_bottom]), **extra)
