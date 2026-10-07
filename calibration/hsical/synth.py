"""Synthetic calibration sessions, rendered through the Optiland design.

Renders the frames a first-light session captures: darks, a halogen lamp on
PTFE (the flat), the same with thin wires across the slit, a CFL, a neon lamp,
a red laser through a diffuser, and a test scene. Light goes through the
design's slit-to-sensor map, a slit-image line shape, a guessed IMX219 response
with its colour filter mosaic, and sensor noise.

The rendered instrument is deliberately *not* the design: the spectrum is
shifted and rotated, the grating has a slightly different line density, the
lens has some barrel distortion, and lamp brightness ratios differ from the
catalog. The calibration has to measure all of that, which is the point.
Everything that went in is written to truth.json / truth.npz next to the frames.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter1d, map_coordinates

from .design import DesignMap
from .lines import FWHM_PER_SIGMA, catalog

# ---------------------------------------------------------------------------
# The "real" instrument's spectral pieces. They are guesses at the parts on the
# shopping list, and on purpose not the curves the pipeline itself assumes.
# ---------------------------------------------------------------------------


def planck(nm, temp_k):
    """Blackbody spectral radiance, arbitrary units, nm in nanometres."""
    nm = np.asarray(nm, float)
    return 1e15 * nm ** -5.0 / np.expm1(1.4388e7 / (nm * temp_k))


def gg495(nm):
    """Schott GG495 long-pass (3 mm): 50% at 495 nm, ~92% plateau with surface losses."""
    return 0.92 / (1.0 + np.exp(-(np.asarray(nm, float) - 495.0) / 4.5))


_NM = np.array([450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950, 1000, 1050.0])
QE = np.array([0.55, 0.66, 0.68, 0.62, 0.54, 0.45, 0.36, 0.27, 0.19, 0.12, 0.065, 0.03, 0.012])
GRATING = np.array([0.28, 0.31, 0.32, 0.31, 0.30, 0.28, 0.25, 0.23, 0.20, 0.18, 0.16, 0.14, 0.12])
# Colour filter dyes of a camera with no IR-cut filter: R, G, B. Past ~800 nm
# all three go clear, which is why a NoIR sensor works for this at all.
CFA = np.array([
    [0.04, 0.03, 0.06, 0.70, 0.92, 0.93, 0.94, 0.95, 0.95, 0.96, 0.96, 0.96, 0.96],
    [0.15, 0.70, 0.90, 0.35, 0.07, 0.08, 0.20, 0.55, 0.85, 0.93, 0.94, 0.95, 0.95],
    [0.90, 0.55, 0.12, 0.05, 0.04, 0.05, 0.08, 0.30, 0.70, 0.88, 0.91, 0.92, 0.92],
])


def qe(nm):
    return np.interp(nm, _NM, QE)


def grating_efficiency(nm):
    return np.interp(nm, _NM, GRATING)


def cfa_transmission(nm, channel):
    """channel: 0 = R, 1 = G, 2 = B (array or int)."""
    nm = np.asarray(nm, float)
    t = np.stack([np.interp(nm, _NM, CFA[k]) for k in range(3)])
    ch = np.broadcast_to(np.asarray(channel), nm.shape)
    return np.take_along_axis(t, ch[None], axis=0)[0]


def bayer_channels(height, width, pattern="RGGB"):
    """Colour of each pixel (0 R, 1 G, 2 B) for a 2 x 2 mosaic like 'RGGB'."""
    code = {"R": 0, "G": 1, "B": 2}
    tile = np.array([[code[pattern[0]], code[pattern[1]]], [code[pattern[2]], code[pattern[3]]]])
    return np.tile(tile, (height // 2 + 1, width // 2 + 1))[:height, :width]


def leaf_reflectance(nm):
    """A leaf: low in the visible with a green bump, then the red edge near 700 nm."""
    nm = np.asarray(nm, float)
    green = 0.08 * np.exp(-0.5 * ((nm - 550.0) / 25.0) ** 2)
    edge = 0.48 / (1.0 + np.exp(-(nm - 715.0) / 12.0))
    return 0.05 + green + edge


def red_paint_reflectance(nm):
    nm = np.asarray(nm, float)
    return 0.06 + 0.78 / (1.0 + np.exp(-(nm - 600.0) / 10.0)) - 0.10 * (nm > 800) * (nm - 800) / 200


# ---------------------------------------------------------------------------


@dataclass
class Truth:
    """Knobs of the synthetic instrument. Lengths are full-resolution pixels."""
    scale: float = 2.0             # 1 = full 3280 x 2464 sensor, 2 = 1640 x 1232, ...
    shift_px: tuple = (55.0, -18.0)  # spectrum offset from the design (x, y)
    rotation_deg: float = 0.35     # sensor rotation about the optical axis
    grating_scale: float = 1.012   # real lines/mm over the design's 500
    distortion: float = -0.004     # barrel distortion at the sensor corner (fraction)
    blur_px: float = 6.0           # lens blur sigma along the spectrum
    blur_y_px: float = 3.0         # lens blur sigma along the slit
    slit_taper: float = 0.03       # throughput difference end to end (jaws not parallel)
    halogen_k: float = 2850.0
    laser_nm: float = 652.6
    laser_fwhm_nm: float = 0.8
    wires_h: tuple = (-0.62, -0.21, 0.24, 0.66)
    line_scatter: float = 0.35     # lognormal scatter of lamp line brightness vs catalog
    black_dn: float = 64.0
    e_per_dn: float = 4.0
    read_e: float = 2.5
    prnu: float = 0.01
    hot_fraction: float = 5e-4
    frames: int = 4
    transpose: bool = False        # camera mounted with the spectrum vertical
    flip_x: bool = False           # wavelength decreasing along x
    flip_y: bool = False
    seed: int = 7


# name: (kind, source, lamp, exposure in us, analogue gain). Darks are taken at
# each distinct exposure and gain. The first set lit by each lamp is exposed to
# 80% of full scale, and every later set under the same lamp is exactly as
# bright per microsecond: the "_long" lamp sets, which saturate the bright lines
# to bring up the faint near-infrared ones, and the wires and scene frames,
# which share the halogen with the flat (so the flat is their white reference).
PLAN = {
    "flat": ("flat", "halogen+ptfe", "halogen", 30000, 1.0),
    "wires": ("wires", "halogen+ptfe", "halogen", 30000, 1.0),
    "cfl": ("lamp", "cfl", "cfl", 60000, 1.0),
    "cfl_long": ("lamp", "cfl", "cfl", 600000, 1.0),
    "neon": ("lamp", "neon", "neon", 250000, 1.0),
    "neon_long": ("lamp", "neon", "neon", 600000, 4.0),
    "laser": ("laser", "laser", "laser", 3000, 1.0),
    "scene": ("scene", "halogen", "halogen", 30000, 1.0),
}
LOOKS = {"cfl_long": "cfl", "neon_long": "neon"}  # renders like another set


class Instrument:
    """A synthetic spectrograph + IMX219 built from :class:`Truth`."""

    def __init__(self, truth: Truth):
        self.t = truth
        self.design = DesignMap(scale=truth.scale)
        self.W, self.H = self.design.width, self.design.height
        self.rng = np.random.default_rng(truth.seed)
        rng = self.rng
        H, W = self.H, self.W
        self.cfa = bayer_channels(H, W)
        self.prnu = 1.0 + truth.prnu * rng.standard_normal((H, W))
        hot = rng.random((H, W)) < truth.hot_fraction
        self.hot_rate = np.where(hot, rng.uniform(20.0, 2000.0, (H, W)), 0.0)  # DN/s
        self.col_fpn = 0.4 * rng.standard_normal(W)
        self.dust = [(rng.uniform(-0.9, 0.9), rng.uniform(0.03, 0.15), rng.uniform(0.004, 0.012))
                     for _ in range(4)]
        self.lamp_lines = {}
        for src in ("cfl", "neon"):
            lines = catalog(src)
            gain = np.exp(truth.line_scatter * rng.standard_normal(len(lines)))
            self.lamp_lines[src] = [(ln, ln.rel * g) for ln, g in zip(lines, gain)]
        speckle = gaussian_filter1d(rng.standard_normal(4096), 12.0)
        self.speckle = 1.0 + 0.3 * speckle / speckle.std()
        self.half_len = 0.5 * abs(self.design.row(1.0, 750.0) - self.design.row(-1.0, 750.0))
        self._maps()

    # -- geometry ------------------------------------------------------------
    def forward(self, h, nm):
        """Sensor (row, col) of slit position h and wavelength nm, before any flip."""
        t = self.t
        nm_d = np.asarray(nm, float) * t.grating_scale
        r = self.design.row(h, nm_d)
        c = self.design.col(h, nm_d)
        cx, cy = (self.W - 1) / 2.0, (self.H - 1) / 2.0
        dx, dy = c - cx, r - cy
        th = np.radians(t.rotation_deg)
        dx, dy = dx * np.cos(th) - dy * np.sin(th), dx * np.sin(th) + dy * np.cos(th)
        f = 1.0 + t.distortion * (dx * dx + dy * dy) / ((self.W / 2.0) ** 2 + (self.H / 2.0) ** 2)
        return cy + dy * f + t.shift_px[1] / t.scale, cx + dx * f + t.shift_px[0] / t.scale

    def _maps(self):
        """Slit position, wavelength and their density for every pixel (Newton on forward)."""
        step = max(2, int(round(8 / self.t.scale)))
        ys = np.arange(-2 * step, self.H + 3 * step, step, dtype=float)
        xs = np.arange(-2 * step, self.W + 3 * step, step, dtype=float)
        Y, X = np.meshgrid(ys, xs, indexing="ij")
        cy = (self.H - 1) / 2.0
        h = (Y - cy) / self.half_len
        nm = self.design.wavelength_at(X) / self.t.grating_scale
        for _ in range(12):
            r, c = self.forward(h, nm)
            rh, ch = self.forward(h + 1e-4, nm)
            rl, cl = self.forward(h, nm + 1e-2)
            a, b = (rh - r) / 1e-4, (rl - r) / 1e-2
            cc, d = (ch - c) / 1e-4, (cl - c) / 1e-2
            det = a * d - b * cc
            er, ec = Y - r, X - c
            h = h + (d * er - b * ec) / det
            nm = nm + (a * ec - cc * er) / det
        dh_dy, dh_dx = np.gradient(h, step, step)
        dl_dy, dl_dx = np.gradient(nm, step, step)
        jac = np.abs(dh_dy * dl_dx - dh_dx * dl_dy)
        py, px = np.mgrid[0:self.H, 0:self.W].astype(float)
        coords = [(py - ys[0]) / step, (px - xs[0]) / step]
        up = lambda a: map_coordinates(a, coords, order=1)  # noqa: E731
        self.h_map, self.nm_map, self.jac = up(h), up(nm), up(jac)
        self.dh_px = up(np.abs(dh_dy))

    # -- light ---------------------------------------------------------------
    def slit(self, h, kind, pixels=False):
        """Slit transmission at h: open between the ends, a taper, dust, optional wires."""
        h = np.asarray(h, float)
        if pixels:  # share of each pixel's slit span inside the slit (anti-aliased ends)
            cover = np.clip((1.0 - np.abs(h)) / np.maximum(self.dh_px, 1e-9) + 0.5, 0.0, 1.0)
        else:
            cover = (np.abs(h) <= 1.0).astype(float)
        out = cover * (1.0 + 0.5 * self.t.slit_taper * h)
        for h0, depth, width in self.dust:
            out = out * (1.0 - depth * np.exp(-0.5 * ((h - h0) / width) ** 2))
        if kind == "wires":
            for h0 in self.t.wires_h:
                out = out * (1.0 - 0.95 * np.exp(-0.5 * ((h - h0) / 0.012) ** 2))
        if kind == "laser":
            idx = np.clip(((h + 1.2) / 2.4 * (self.speckle.size - 1)).astype(int), 0,
                          self.speckle.size - 1)
            out = out * np.clip(self.speckle[idx], 0.0, None)
        return out

    def optics(self, nm, h):
        """Wavelength-dependent throughput before the sensor: filter, grating, vignetting."""
        return gg495(nm) * grating_efficiency(nm) * self.design.vignetting(
            h, np.asarray(nm) * self.t.grating_scale)

    def continuum(self, kind, nm):
        t = self.t
        if kind in ("flat", "wires"):
            return 0.985 * planck(nm, t.halogen_k)
        if kind == "scene":
            return planck(nm, t.halogen_k) * np.where(self.h_map < 0, leaf_reflectance(nm),
                                                      red_paint_reflectance(nm))
        if kind == "cfl":  # a faint smooth glow under the lines and phosphor peaks
            return 0.004 * np.exp(-0.5 * ((nm - 600.0) / 80.0) ** 2)
        return None

    def lines(self, kind):
        """(nm, brightness) of the narrow features, drawn as exact sub-pixel points.

        Phosphor bands are split into 25 narrow pieces with Gaussian weights,
        so even a binned sensor gets their flux right."""
        if kind == "laser":
            return [(self.t.laser_nm, 1.0)]
        out = []
        if kind in ("cfl", "neon"):
            for ln, rel in self.lamp_lines[kind]:
                if ln.width == 0:
                    out.append((ln.nm, rel))
                    continue
                s = ln.width / FWHM_PER_SIGMA
                k = np.linspace(-3.0, 3.0, 25)
                w = np.exp(-0.5 * k ** 2)
                out += [(ln.nm + s * kk, rel * ww / w.sum()) for kk, ww in zip(k, w)]
        return out

    def slit_image_widths(self):
        """Width (px) of the slit's image along x at every column, at the middle row's wavelengths."""
        return self.design.slit_image_px(0.0, self.nm_map[int(self.H // 2)] * self.t.grating_scale)

    def blur(self, img):
        """The camera lens blur, along x and along the slit."""
        img = gaussian_filter1d(img, self.t.blur_px / self.t.scale, axis=1)
        return gaussian_filter1d(img, self.t.blur_y_px / self.t.scale, axis=0)

    def render(self, kind):
        """Expected signal (arbitrary units) on every pixel for one kind of frame."""
        H, W = self.H, self.W
        widths = self.slit_image_widths()

        def spread(img):  # slit image width, then lens blur
            return self.blur(box_filter_var(img, widths))

        out = np.zeros((H, W))
        cont = self.continuum(kind, self.nm_map)
        if cont is not None:
            # a smooth continuum: the sensor's response at each pixel's own wavelength will do
            c = cont * self.jac * self.slit(self.h_map, kind, pixels=True) * self.optics(self.nm_map, self.h_map)
            out += spread(c) * qe(self.nm_map) * cfa_transmission(self.nm_map, self.cfa)
        feats = self.lines(kind)
        if feats:
            # A line keeps its own wavelength as the lens spreads it over neighbouring
            # pixels, so every pixel it reaches responds at that wavelength, through
            # its own colour filter: render each filter colour separately.
            n = int(8 * self.half_len)
            hs = np.linspace(-1.0, 1.0, n)
            placed = []
            for nm, rel in feats:
                r, c = self.forward(hs, np.full(n, nm))
                placed.append((nm, r, c, rel * self.slit(hs, kind) * self.optics(nm, hs) * (2.0 / n) * qe(nm)))
            for ch in range(3):
                img = np.zeros((H, W))
                for nm, r, c, w in placed:
                    _splat(img, r, c, w * float(cfa_transmission(nm, ch)))
                out += np.where(self.cfa == ch, spread(img), 0.0)
        return out * self.prnu

    # -- sensor --------------------------------------------------------------
    def expose(self, e_rate, exposure_us, n, gain=1.0):
        """n raw 10-bit frames of a signal in electrons per microsecond."""
        t = self.t
        out = []
        for _ in range(n):
            e = self.rng.poisson(np.clip(e_rate, 0, None) * exposure_us).astype(float)
            e += t.read_e * self.rng.standard_normal(e.shape)
            dn = (e / t.e_per_dn + self.hot_rate * exposure_us * 1e-6) * gain + t.black_dn \
                + self.col_fpn[None, :]
            out.append(np.clip(np.round(dn), 0, 1023).astype(np.uint16))
        return out

    def orient(self, a):
        """Apply the camera mounting (flips, transpose) to a frame-shaped array."""
        if self.t.flip_x:
            a = a[..., :, ::-1]
        if self.t.flip_y:
            a = a[..., ::-1, :]
        if self.t.transpose:
            a = np.swapaxes(a, -1, -2)
        return np.ascontiguousarray(a)


def _splat(img, rows, cols, w):
    """Add weights w at fractional (rows, cols), shared bilinearly between 4 pixels."""
    H, W = img.shape
    r0, c0 = np.floor(rows).astype(int), np.floor(cols).astype(int)
    fr, fc = rows - r0, cols - c0
    flat = img.ravel()
    for dr, wr in ((0, 1 - fr), (1, fr)):
        for dc, wc in ((0, 1 - fc), (1, fc)):
            rr, cc = r0 + dr, c0 + dc
            ok = (rr >= 0) & (rr < H) & (cc >= 0) & (cc < W)
            flat += np.bincount(rr[ok] * W + cc[ok], weights=(w * wr * wc)[ok], minlength=H * W)


def box_filter_var(img, widths, offset=0.0):
    """Mean over a box of ``widths[x]`` pixels (fractional allowed) centred on each column.

    Uses the running integral, so a 23.4 px slit image is exactly that wide.
    ``offset`` (px, per column allowed) moves each box's centre to the right of
    its column, which shifts the result left by as much."""
    H, W = img.shape
    F = np.zeros((H, W + 1))
    np.cumsum(img, axis=1, out=F[:, 1:])
    x = np.arange(W, dtype=float)
    w = np.broadcast_to(np.asarray(widths, float), (W,))
    c = x + 0.5 + np.broadcast_to(np.asarray(offset, float), (W,))

    def at(t):  # F at fractional boundary index t (boundary j sits at x = j - 0.5)
        t = np.clip(t, 0.0, W)
        i = np.minimum(np.floor(t).astype(int), W - 1)
        f = t - i
        return F[:, i] * (1 - f) + F[:, i + 1] * f

    return (at(c + w / 2) - at(c - w / 2)) / w


def write_session(out_dir, truth: Truth | None = None, kinds=None, log=print):
    """Render a whole calibration session into out_dir (one folder per frame set)."""
    truth = truth or Truth()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    inst = Instrument(truth)
    kinds = kinds or list(PLAN)
    full_e = (1023 - truth.black_dn) * truth.e_per_dn
    settings = set()
    level, renders = {}, {}
    for name in kinds:
        kind, source, lamp, exp_us, gain = PLAN[name]
        look = LOOKS.get(name, name)
        if look not in renders:
            renders[look] = inst.render(look)
        sig = renders[look]
        if lamp not in level:  # the first set under each lamp fills 80% of full scale
            level[lamp] = 0.8 * full_e / (exp_us * gain) / sig.max()
        frames = inst.expose(sig * level[lamp], exp_us, truth.frames, gain)
        _write_set(out / name, [inst.orient(f) for f in frames], dict(
            kind=kind, source=source, exposure_us=exp_us, gain=gain, synthetic=True,
            design_scale=truth.scale))
        settings.add((exp_us, gain))
        log(f"  {name:9s} {exp_us / 1000:7.1f} ms  gain {gain:g}  x{truth.frames}")
    for exp_us, gain in sorted(settings):
        frames = inst.expose(np.zeros((inst.H, inst.W)), exp_us, truth.frames, gain)
        tag = f"dark_{exp_us}us" + ("" if gain == 1.0 else f"_gain{gain:g}")
        _write_set(out / tag, [inst.orient(f) for f in frames], dict(
            kind="dark", source="capped", exposure_us=exp_us, gain=gain, synthetic=True,
            design_scale=truth.scale))
        log(f"  dark      {exp_us / 1000:7.1f} ms  gain {gain:g}  x{truth.frames}")
    _write_truth(out, inst)
    (out / "session.json").write_text(json.dumps(dict(
        note="Synthetic session rendered by `python -m hsical synth` from the Optiland design.",
        halogen_k=truth.halogen_k), indent=2) + "\n")
    return inst


def _write_set(path, frames, meta):
    path.mkdir(parents=True, exist_ok=True)
    for i, f in enumerate(frames):
        np.save(path / f"frame_{i:03d}.npy", f)
    h, w = frames[0].shape
    meta = dict(meta, frames=len(frames), width=w, height=h, format="npy", bits=10)
    (path / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")


def _write_truth(out, inst: Instrument):
    t = inst.t
    d = inst.design
    hs = np.array([-1.0, -0.5, 0.0, 0.5, 1.0])
    rows750, cols750 = inst.forward(hs, np.full(5, 750.0))
    nm = np.arange(480.0, 1021.0, 5.0)
    truth = dict(
        params=asdict(t),
        sensor=dict(width=inst.W, height=inst.H),
        laser_nm=t.laser_nm,
        slit_image_px_750=float(d.slit_image_px(0.0, 750.0 * t.grating_scale)),
        blur_px=t.blur_px / t.scale,
        resolution_nm=float(d.resolution_nm),
        slit_rows_750=[float(rows750[0]), float(rows750[-1])],
        smile_px={str(int(n)): float(inst.forward(1.0, n)[1] - inst.forward(0.0, n)[1])
                  for n in (550, 650, 750, 850, 950)},
        keystone_px={str(int(n)): float(inst.forward(1.0, n)[0] - inst.forward(-1.0, n)[0])
                     for n in (550, 650, 750, 850, 950)},
        dust=inst.dust,
        response=dict(nm=nm.tolist(), optics=inst.optics(nm, 0.0).tolist(), qe=qe(nm).tolist(),
                      cfa_mean=((cfa_transmission(nm, 0) + 2 * cfa_transmission(nm, 1)
                                 + cfa_transmission(nm, 2)) / 4).tolist()),
    )
    (out / "truth.json").write_text(json.dumps(truth, indent=1) + "\n")
    np.savez_compressed(out / "truth.npz", nm=inst.orient(inst.nm_map).astype(np.float32),
                        h=inst.orient(inst.h_map).astype(np.float32))


def compare(cal, session_dir):
    """How far a Calibration made from a synthetic session is from that session's truth.

    Returns a dict of errors: wavelength map (nm), slit position (rows), the
    rectified grid, the laser's wavelength, and the scene's reflectance against
    the flat (the halogen frames share one lamp, so the flat is a white
    reference with the PTFE's 0.985)."""
    from scipy.ndimage import map_coordinates

    from .frames import load_session
    from .sensor import dark_for

    d = Path(session_dir)
    tj = json.loads((d / "truth.json").read_text())
    z = np.load(d / "truth.npz")
    nm_t = cal.orient(z["nm"]).astype(float)
    h_t = cal.orient(z["h"]).astype(float)
    inside = (np.abs(h_t) < 0.97) & (nm_t > 500) & (nm_t < 1000)
    e_nm = (cal.wavelength_map() - nm_t)[inside]
    mx, my = cal.maps()
    nm_at = map_coordinates(nm_t, [my, mx], order=1)
    h_at = map_coordinates(h_t, [my, mx], order=1)
    ok = np.abs(h_at) < 0.97
    half = 0.5 * (cal.keystone.s_bottom - cal.keystone.s_top)
    full = ok.sum(1) > 1          # rectified rows with slit light in them
    row_spread = np.nanstd(np.where(ok, h_at, np.nan)[full], axis=1) * half
    out = dict(
        wavelength_rms_nm=float(np.sqrt(np.mean(e_nm ** 2))),
        wavelength_max_nm=float(np.abs(e_nm).max()),
        grid_wavelength_max_nm=float(np.abs(nm_at - cal.nm_grid[None, :])[ok].max()),
        grid_row_spread_px=float(row_spread.max()),
    )
    laser = cal.info.get("laser") or {}
    if "nm" in laser:
        out["laser_error_nm"] = float(laser["nm"] - tj["laser_nm"])
    sets, _ = load_session(d)
    if "scene" in sets and "flat" in sets and cal.response is not None:
        darks = [s for s in sets.values() if s.kind == "dark"]

        def raw(name):
            s = sets[name]
            dk, _ = dark_for(s, darks)
            return s.summary()[0], dk, s.exposure_us

        refl = cal.reflectance(*raw("scene"), *raw("flat"), white_reflectance=0.985)
        nm_g = cal.nm_grid[None, :]
        want = np.where(h_at < 0, leaf_reflectance(nm_g), red_paint_reflectance(nm_g))
        sel = ok & (np.abs(h_at) > 0.05) & np.isfinite(refl) & (nm_g > 520) & (nm_g < 960)
        r = (refl / want)[sel]
        out["reflectance_ratio_median"] = float(np.median(r))
        out["reflectance_ratio_rms"] = float(np.sqrt(np.mean((r - 1) ** 2)))
    return out
