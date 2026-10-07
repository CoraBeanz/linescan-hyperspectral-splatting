"""A made-up spectrograph, for the tests and for running line_camera without a camera.

    ros2 run so101_scan_camera fake_calibration /tmp/fake_cal      # write one to try things with

Instrument is a small version of the real thing as the calibration kit sees it. In hsical's
turned frame (spectrum along x, blue on the left, the slit along y) every pixel sees one
wavelength and one point on the slit, with some smile (a wavelength's line bends along the
slit) and keystone (the slit's image tilts with wavelength), behind the IMX219's RGGB colour
mosaic and a 500 nm long-pass filter, with a few bad pixels. save_calibration() writes what the
kit would have measured, in its format: calibration.json and maps.npz with the per-pixel maps
line_camera reads (wavelength, slit, response, bad). It isn't a full hsical calibration: hsical
itself can't load it, since it has no fitted polynomials or rectifying warp.

render() makes the raw frame the sensor would give for a radiance L(t, nm) along the slit, t
running from 0 at the slit's first end (s_top, binned pixel 0) to 1 at the other, with shot and
read noise. Radiance is relative, like hsical's (a white under the scan's lamp is about 1);
response is counts per second per unit radiance at gain 1.
"""

import argparse
import json
import os

import numpy as np

from so101_scan_camera.binning import Orientation
from so101_scan_camera.sources import BLACK_LEVEL, FULL_SCALE


def _sig(x):
    return 1.0 / (1.0 + np.exp(-x))


def optics_response(nm):
    """Silicon's response behind a 500 nm long-pass filter, peak 1."""
    nm = np.asarray(nm, float)
    return np.exp(-((nm - 680.0) / 260.0) ** 2) * _sig((nm - 505.0) / 5.0)


def mosaic_transmission(nm, channel):
    """The colour mosaic: channel 0 red, 1 and 2 green, 3 blue. All of them pass the near IR."""
    nm = np.asarray(nm, float)
    red = 0.04 + 0.9 * _sig((nm - 585.0) / 12.0)
    green = 0.04 + 0.75 * np.exp(-((nm - 535.0) / 40.0) ** 2) + 0.85 * _sig((nm - 790.0) / 20.0)
    blue = 0.04 + 0.7 * np.exp(-((nm - 460.0) / 35.0) ** 2) + 0.85 * _sig((nm - 805.0) / 20.0)
    return np.where(channel == 0, red, np.where(channel == 3, blue, green))


def halogen(nm, temp_k=2850.0):
    """A halogen lamp's spectrum relative to 700 nm (Planck)."""
    nm = np.asarray(nm, float)
    c2 = 1.4388e7
    return (700.0 / nm) ** 5 * np.expm1(c2 / (700.0 * temp_k)) / np.expm1(c2 / (nm * temp_k))


class Instrument:
    def __init__(self, width=164, height=124, orientation=None, nm_range=(470.0, 1010.0), slit_rows=(0.1, 0.9),
                 smile_px=1.5, keystone_px=1.2, peak_rate=110000.0, bad_fraction=0.001, read_noise=1.5,
                 electrons_per_count=1.0, seed=0):
        self.width, self.height = int(width), int(height)       # raw frame
        self.orientation = orientation or Orientation()
        oh, ow = (self.width, self.height) if self.orientation.transpose else (self.height, self.width)
        self.turned_shape = (oh, ow)
        y, x = np.mgrid[0:oh, 0:ow].astype(float)
        u = (y - (oh - 1) / 2.0) / ((oh - 1) / 2.0)             # -1..1 along the slit
        v = (x - (ow - 1) / 2.0) / ((ow - 1) / 2.0)             # -1..1 along the spectrum
        lo, hi = nm_range
        self.nm = (lo + (x - smile_px * u ** 2) / (ow - 1) * (hi - lo)).astype(np.float32)
        self.s = (y + keystone_px * u * v).astype(np.float32)   # slit position, rows at the middle column
        self.s_top, self.s_bottom = slit_rows[0] * (oh - 1), slit_rows[1] * (oh - 1)
        self.t = (self.s - self.s_top) / (self.s_bottom - self.s_top)
        self.inside = (self.t >= 0.0) & (self.t <= 1.0)
        raw_y, raw_x = np.mgrid[0:self.height, 0:self.width]
        channel = self.orientation.apply((raw_y % 2) * 2 + (raw_x % 2))   # RGGB, in the turned frame
        self.response = (peak_rate * optics_response(self.nm) * mosaic_transmission(self.nm, channel)
                         ).astype(np.float32)
        rng = np.random.default_rng(seed)
        self.bad = rng.random((oh, ow)) < bad_fraction
        self.hot = self.bad & (rng.random((oh, ow)) < 0.5)
        self.read_noise = read_noise
        self.electrons_per_count = electrons_per_count
        self.rng = np.random.default_rng(seed + 1)
        self.nm_range = (lo, hi)
        self.params = dict(width=self.width, height=self.height, orientation=self.orientation.to_dict(),
                           nm_range=list(nm_range), slit_rows=list(slit_rows), smile_px=smile_px,
                           keystone_px=keystone_px, peak_rate=peak_rate, bad_fraction=bad_fraction, seed=seed)

    @property
    def raw_slit_rows(self):
        """The raw rows that see the slit."""
        rows = self.orientation.unapply(np.broadcast_to(self.inside, self.turned_shape))
        r = np.flatnonzero(rows.any(axis=1))
        return int(r[0]), int(r[-1])

    def radiance_image(self, radiance):
        """radiance(t, nm) at every pixel of the turned frame inside the slit's image; 0 outside."""
        out = np.zeros(self.turned_shape)
        out[self.inside] = radiance(self.t[self.inside], self.nm[self.inside])
        return out

    def render(self, radiance, exposure_us, gain=1.0, noise=True, black=BLACK_LEVEL):
        """The raw (height, width) uint16 frame for radiance(t, nm), an array function, or a
        number for a uniform one."""
        rad = radiance if callable(radiance) else (lambda t, nm, c=float(radiance): np.full(np.shape(t), c))
        signal = self.response * self.radiance_image(rad) * (exposure_us * 1e-6) * gain
        img = black + signal
        if noise:
            electrons = np.maximum(signal, 0.0) * self.electrons_per_count / max(gain, 1e-6)
            img = img + self.rng.normal(0.0, 1.0, signal.shape) * np.sqrt(
                electrons * (gain / self.electrons_per_count) ** 2 + self.read_noise ** 2)
        img = np.where(self.hot, FULL_SCALE, np.where(self.bad, black, img))
        raw = self.orientation.unapply(np.clip(np.round(img), 0, FULL_SCALE).astype(np.uint16))
        return raw

    def save_calibration(self, out_dir, nm_step=None, temp_k=2850.0):
        """calibration.json and maps.npz as the calibration kit writes them (the parts line_camera
        reads). Its wavelength grid is 2 nm, or two columns a step on a small sensor. Returns the
        folder."""
        os.makedirs(out_dir, exist_ok=True)
        oh, ow = self.turned_shape
        lo, hi = self.nm_range
        if nm_step is None:
            nm_step = max(2.0, float(np.ceil(2.0 * (hi - lo) / (ow - 1))))
        nm_grid = np.arange(np.ceil((lo + 10) / nm_step) * nm_step, hi - 10 + 1e-9, nm_step)
        s_grid = np.arange(np.ceil(self.s_top), np.floor(self.s_bottom) + 1.0)
        y0 = (oh - 1) / 2.0
        d = dict(
            format="hsical-1",
            sensor=dict(width=ow, height=oh, orientation=self.orientation.to_dict()),
            wavelength=dict(y0=y0, table_nm=[round(float(v), 5) for v in self.nm[int(round(y0))]], sigma_nm=None),
            keystone=dict(x_ref=(ow - 1) / 2.0, s_top=float(self.s_top), s_bottom=float(self.s_bottom), rms_px=0.0),
            rectified=dict(nm=[float(nm_grid[0]), float(nm_grid[-1]), len(nm_grid)],
                           s=[float(s_grid[0]), float(s_grid[-1]), len(s_grid)]),
            response=dict(temp_k=temp_k, has_map=True),
            synthetic=self.params,
        )
        with open(os.path.join(out_dir, "calibration.json"), "w") as f:
            json.dump(d, f, indent=1)
            f.write("\n")
        np.savez_compressed(os.path.join(out_dir, "maps.npz"), wavelength=self.nm, slit=self.s,
                            response=self.response, bad=self.bad, nm_grid=nm_grid.astype(np.float32),
                            s_grid=s_grid.astype(np.float32))
        return out_dir


def scene_radiance(t, nm, line=0.0):
    """A test pattern under a halogen lamp: stripes along the slit that change from line to line
    and colour that changes along the spectrum, smooth enough to bin without bias."""
    reflectance = (0.55 + 0.3 * np.sin(2 * np.pi * (1.5 * t + 0.13 * line))
                   * (0.6 + 0.4 * np.cos((np.asarray(nm) - 500.0) / 150.0)))
    return reflectance * halogen(nm)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Write a made-up spectrograph calibration (hsical format) to try "
                                             "line_camera with source:=fake")
    ap.add_argument("out", help="folder to write calibration.json and maps.npz to")
    ap.add_argument("--width", type=int, default=164)
    ap.add_argument("--height", type=int, default=124)
    ap.add_argument("--seed", type=int, default=0)
    args, _ = ap.parse_known_args(argv)
    inst = Instrument(args.width, args.height, seed=args.seed)
    print("wrote %s" % inst.save_calibration(args.out))


if __name__ == "__main__":
    main()
