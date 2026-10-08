"""Colour waterfalls of a sweep: a row per scan line, the slit across, coloured from the spectra.

line_camera publishes them while a sweep runs (~/scan_preview_true_color and ~/scan_preview_cir),
so the scene shows up in colour as the mirror steps across it, before any splat is trained.
Each binned line (mean raw counts, slit bins x the calibration's fine bands) goes through the
same steps scan_to_dataset takes, so the waterfall shows what the dataset will hold:

  1. counts per second: (line - dark) / (exposure x gain), with a dark taken at the line's
     exposure and gain if there is one, else the sensor's black level;
  2. the dataset's bands: 10 nm wide from 500 to 950 nm (as far as the calibration reaches),
     each the pixel-weighted sum of the fine cells in it;
  3. reflectance: divided by the white reference's counts per second over its reflectance (PTFE,
     0.98, unless its meta.json says). Until there is a white, by the calibration's response
     times a halogen lamp at the calibration lamp's temperature (so a white under a halogen lamp
     comes out grey), scaled so the sweep's brightest is white: the colours are close but not
     exact. A calibration without a response map gives no colours until a white is taken, as
     the sensor's colour mosaic makes every slit bin's raw counts a different colour. Values the
     white or the response barely reach are filled from their neighbours along the spectrum, as
     in the dataset;
  4. colour: true colour or colour infrared, with spectral.py's weights, which are the
     renderer's and the web viewer's.

A slit bin with a saturated pixel shows magenta, so clipping is easy to spot and the exposure
can come down. Each line is worked out once, when it is first drawn, and kept with its sweep.
"""

import json
import os
import warnings

import numpy as np

from so101_scan_camera import spectral
from so101_scan_camera.scan_to_dataset import Grid, band_sums, fill_missing
from so101_scan_camera.sources import BLACK_LEVEL
from so101_scan_camera.synthetic import halogen

WHITE_REFLECTANCE = 0.98    # PTFE, as scan_to_dataset assumes when a white's meta.json doesn't say
DIM = 0.02                  # bands the white (or response) lights to less than this of its best are left out
DARK_TOLERANCE = 0.02       # a dark matches a line within 2% in exposure and gain
SATURATED_COLOR = (1.0, 0.0, 1.0)
NM_MIN, NM_MAX, NM_STEP = 500.0, 950.0, 10.0     # the dataset's bands (scan_to_dataset's defaults)


def dataset_bands(nm_edges, nm_min=NM_MIN, nm_max=NM_MAX, step=NM_STEP):
    """The dataset's band centres that the binned cells cover."""
    centres = np.arange(nm_min, nm_max + 1e-9, step)
    return centres[(centres - step / 2 >= nm_edges[0] - 1e-9) & (centres + step / 2 <= nm_edges[-1] + 1e-9)]


class Waterfall:
    def __init__(self, binner, slit_reversed=False):
        self.grid = Grid.of(binner)
        self.shape = binner.shape
        self.slit_reversed = bool(slit_reversed)
        self.nm = dataset_bands(self.grid.nm_edges)
        c = self.grid.nm_centres
        self.m = ((c[:, None] >= self.nm[None, :] - NM_STEP / 2)
                  & (c[:, None] < self.nm[None, :] + NM_STEP / 2)).astype(np.float64)
        self.pixels = self.grid.pixels.astype(np.float64)
        self.weights = {mode: spectral.weights(mode, self.nm) for mode in spectral.MODES} if len(self.nm) else {}
        self.lamp_k = float(getattr(binner.maps, "temp_k", 2850.0) or 2850.0)
        self.darks = []             # (exposure_us, gain, binned)
        self.white = None           # (name, binned, exposure_us, gain, reflectance)
        self.version = 0            # bumped when the references change, so lines are worked out again
        self._set_denominator()

    @property
    def usable(self):
        """Whether it can draw colours: the bands reach far enough, and there is a white or a response."""
        return len(self.nm) >= 3 and self.basis is not None

    @property
    def basis(self):
        """What the colours are relative to: white, response or None (nothing to go on)."""
        return "white" if self.white else ("response" if self.grid.response is not None else None)

    # --- references ---------------------------------------------------------------------------

    def add_dark(self, binned, exposure_us, gain):
        binned = self._check(binned, "dark")
        self.darks = [d for d in self.darks if not self._same(d[0], d[1], exposure_us, gain)]
        self.darks.append((float(exposure_us), float(gain), binned))
        self._set_denominator()     # the white may be at this exposure
        self.version += 1

    def set_white(self, binned, exposure_us, gain, reflectance=None, name="white"):
        binned = self._check(binned, "white")
        w = self.per_second(binned, exposure_us, gain)
        if not np.nanmax(np.where(np.isfinite(w), w, -np.inf)) > 0:
            raise ValueError("the white has no signal")
        self.white = (name, binned, float(exposure_us), float(gain), float(reflectance or WHITE_REFLECTANCE))
        self._set_denominator()
        self.version += 1

    def _check(self, binned, what):
        binned = np.asarray(binned, np.float64)
        if binned.shape != self.shape:
            raise ValueError("the %s is binned %s, the lines %s" % (what, binned.shape, self.shape))
        return binned

    @staticmethod
    def _same(e0, g0, e1, g1):
        return abs(e0 / e1 - 1) <= DARK_TOLERANCE and abs(g0 / g1 - 1) <= DARK_TOLERANCE

    def dark_for(self, exposure_us, gain):
        for e, g, binned in self.darks:
            if self._same(e, g, exposure_us, gain):
                return binned
        return BLACK_LEVEL

    def per_second(self, binned, exposure_us, gain):
        return (np.asarray(binned, np.float64) - self.dark_for(exposure_us, gain)) / (exposure_us * 1e-6 * gain)

    def _set_denominator(self):
        """[slit bins, bands]: what a line's band sums are divided by (as scan_to_dataset)."""
        if self.white:
            _, binned, exposure_us, gain, reflectance = self.white
            den = band_sums(self.per_second(binned, exposure_us, gain), self.pixels, self.m) / reflectance
        elif self.grid.response is not None:
            den = band_sums(self.grid.response.astype(np.float64), self.pixels, self.m) * halogen(self.nm, self.lamp_k)
        else:
            self.den = None
            return
        level = den / np.maximum(self.pixels @ self.m, 1.0)
        with np.errstate(invalid="ignore"):
            self.den = np.where(level > DIM * np.nanmax(np.where(np.isfinite(level), level, 0.0)), den, np.nan)

    def load_references(self, folder):
        """The darks and the white capture_reference left in folder (reference/<name>/meta.json
        and binned/reference_<name>.npy), if they were binned on this grid. Returns what was
        loaded, as text."""
        refs = os.path.join(folder, "reference")
        grid = os.path.join(folder, "binned", "binning.npz")
        if not os.path.isdir(refs):
            return "no references in %s" % folder
        if os.path.exists(grid) and not Grid.load(grid).same(self.grid):
            return "the references in %s were binned on another grid" % folder
        found, whites = [], []
        for name in sorted(os.listdir(refs)):
            meta_path = os.path.join(refs, name, "meta.json")
            binned_path = os.path.join(folder, "binned", "reference_%s.npy" % name)
            if not (os.path.exists(meta_path) and os.path.exists(binned_path)):
                continue
            with open(meta_path) as f:
                meta = json.load(f)
            binned = np.load(binned_path)
            if binned.shape != self.shape or not meta.get("exposure_us"):
                continue
            if meta.get("kind") == "dark":
                self.add_dark(binned, meta["exposure_us"], meta.get("gain", 1.0))
                found.append("dark %s" % name)
            elif meta.get("kind") == "white":
                whites.append((name != "white", name, binned, meta))
        for _, name, binned, meta in sorted(whites, key=lambda w: w[:2])[:1]:   # "white" first, as the converter
            try:
                self.set_white(binned, meta["exposure_us"], meta.get("gain", 1.0), meta.get("reflectance"), name)
                found.append("white %s" % name)
            except ValueError:
                pass
        return ("%s from %s" % (", ".join(found), folder)) if found else "no usable references in %s" % folder

    # --- lines and images ---------------------------------------------------------------------

    def line_values(self, binned, exposure_us, gain):
        """A line in the dataset's bands, [slit bins, bands], missing values filled, and which
        slit bins had a saturated pixel."""
        c = self.per_second(binned, exposure_us, gain)
        with np.errstate(invalid="ignore", divide="ignore"):
            v = band_sums(c, self.pixels, self.m) / self.den
        saturated = (np.isnan(v) & np.isfinite(self.den)).any(axis=1)
        if np.isnan(v).all():
            v[:] = 0.0
        else:
            fill_missing(v)
        return v.astype(np.float32), saturated

    def image(self, lines, mode="true_color", cache=None, mark_saturated=True):
        """The waterfall as uint8 RGB [lines, slit bins, 3]. lines: {index: (binned, exposure_us,
        gain)}, binned None for a line without frames (a black row). cache: a dict kept with the
        sweep, so each line is worked out once."""
        if not lines or not self.usable:
            return None
        cache = {} if cache is None else cache
        n = max(lines) + 1
        idx = []
        for i, (binned, e, g) in sorted(lines.items()):
            if binned is None:
                continue
            got = cache.get(i)
            if got is None or got[0] != self.version:
                got = cache[i] = (self.version,) + self.line_values(binned, e, g)
            idx.append(i)
        rgb = np.zeros((n, self.shape[0], 3))
        if idx:
            v = np.stack([cache[i][1] for i in idx])               # [lines, slit bins, bands]
            saturated = np.stack([cache[i][2] for i in idx])
            rows = v @ self.weights[mode]
            if self.basis == "response":       # relative: the sweep's brightest is white
                rows = rows / self._top(rows[~saturated].max(axis=-1))
            if mark_saturated:
                rows[saturated] = SATURATED_COLOR
            rgb[idx] = rows
        if self.slit_reversed:
            rgb = rgb[:, ::-1]
        return spectral.srgb8(rgb)

    @staticmethod
    def _top(values):
        if values.size == 0:
            return 1.0
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            top = np.nanpercentile(values, 99)
        return top if np.isfinite(top) and top > 0 else 1.0


def grey(lines, width, slit_reversed=False):
    """The plain waterfall: each cell's mean over the spectrum above the black level, scaled to
    the sweep's brightest (mono8 [lines, width slit bins])."""
    if not lines:
        return None
    img = np.zeros((max(lines) + 1, width))
    for i, (v, _, _) in lines.items():
        if v is not None:
            ok = np.isfinite(v)
            img[i] = np.where(ok, v - BLACK_LEVEL, 0.0).sum(axis=1) / np.maximum(ok.sum(axis=1), 1)
    top = float(np.percentile(img, 99.5)) or 1.0
    if slit_reversed:
        img = img[:, ::-1]
    return np.clip(img / top * 255.0, 0, 255).astype(np.uint8)
