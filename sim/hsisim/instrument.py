"""The spectrograph and its IMX219: from what a scan line sees to a raw 10-bit frame.

This is hsical's synthetic instrument (hsical.synth.Instrument), the one its self-test
calibrates, rendering scene light instead of lamps. So everything the calibration has to
measure is in the scan frames too: the spectrum shifted, rotated and stretched against the
design, barrel distortion, smile and keystone from the Optiland map, the slit's width,
taper and dust, the camera lens blur, the filter, grating and sensor response, the NoIR
sensor's colour mosaic, and shot, read and fixed-pattern noise, hot pixels and 10-bit
clipping. A scan made with a Truth calibrates with the hsical session made with the
same Truth.

Rendering a line: every pixel sees one slit position h and one wavelength (the
instrument's maps), so its light is the line's spectral radiance at (h, nm), looked up
from the radiance computed along the slit. Light from across the slit's width spreads
into the slit's image along the spectrum; each slice of the slit lands at its own place
within that image, so a line whose scene changes across the slit's width (an edge
running along it) shifts its spectrum a little, as a real spectrograph's does.
"""

from __future__ import annotations

import numpy as np
from scipy.ndimage import map_coordinates

from . import repo  # noqa: F401
from . import spectra
from hsical.synth import Instrument, Truth, box_filter_var, cfa_transmission, qe  # noqa: E402

FULL_DN = 1023


class Spectrograph:
    """hsical's synthetic instrument, rendering the lines of a scan.

    nm_step: spacing (nm) of the radiance computed along the slit. h_step: spacing of slit
    positions, as a fraction of a sensor row (0.5: two per row)."""

    def __init__(self, truth: Truth, temp_k=2850.0, nm_step=2.0, h_step=0.5, h_max=1.06):
        self.truth = truth
        self.inst = Instrument(truth)
        inst = self.inst
        self.H, self.W = inst.H, inst.W
        self.temp_k = float(temp_k)
        self.nm = spectra.wavelength_grid(nm_step)
        n_h = int(np.ceil(2 * h_max * inst.half_len / h_step)) + 1
        self.h = np.linspace(-h_max, h_max, n_h)
        # what stays the same from line to line
        self.base = (spectra.halogen(inst.nm_map, temp_k) * inst.jac
                     * inst.slit(inst.h_map, "scene", pixels=True) * inst.optics(inst.nm_map, inst.h_map))
        self.response = qe(inst.nm_map) * cfa_transmission(inst.nm_map, inst.cfa) * inst.prnu
        self.widths = inst.slit_image_widths()
        self.coords = np.stack([(inst.h_map - self.h[0]) / (self.h[1] - self.h[0]),
                                (inst.nm_map - self.nm[0]) / (self.nm[1] - self.nm[0])]).astype(np.float32)

    @property
    def full_electrons(self):
        return (FULL_DN - self.truth.black_dn) * self.truth.e_per_dn

    def signal(self, reflectance, slice_centres):
        """Expected signal (arbitrary units) on every pixel.

        reflectance: [slices, h, nm] the light at each slit position over the lamp's,
        per slice across the slit; slice_centres: each slice's place across the slit, in
        slit widths (-0.5 .. 0.5)."""
        K = len(reflectance)
        acc = np.zeros((self.H, self.W))
        for r, u in zip(reflectance, slice_centres):
            light = map_coordinates(np.asarray(r, float), self.coords, order=1, mode="nearest") * self.base
            acc += box_filter_var(light, self.widths / K, offset=-u * self.widths)
        return self.inst.blur(acc / K) * self.response

    def uniform_signal(self, reflectance=spectra.PTFE_REFLECTANCE):
        """Signal of a scene that is the same everywhere along and across the slit (a white sheet)."""
        r = np.broadcast_to(np.asarray(reflectance, float), self.nm.shape)
        return self.signal(np.broadcast_to(r, (1, self.h.size, self.nm.size)), [0.0])

    def expose(self, signal, level, exposure_us, n=1, gain=1.0):
        """n raw frames, as the camera gives them (after its mounting's flips)."""
        frames = self.inst.expose(signal * level, exposure_us, n, gain)
        return [self.inst.orient(f) for f in frames]

    def dark(self, exposure_us, n=1, gain=1.0):
        return self.expose(np.zeros((self.H, self.W)), 0.0, exposure_us, n, gain)

    def level_for(self, signal, exposure_us, gain=1.0, fill=0.8):
        """Electrons per microsecond per unit of signal that puts signal's peak at `fill`
        of full scale."""
        return fill * self.full_electrons / (exposure_us * gain) / float(signal.max())

    def pixel_maps(self):
        """(nm, h) of every pixel, in the frames' orientation."""
        return (self.inst.orient(self.inst.nm_map).astype(np.float32),
                self.inst.orient(self.inst.h_map).astype(np.float32))
