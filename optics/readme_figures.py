"""
README figures, traced from the Optiland model in spectrograph_model.py.

Writes three PNGs to docs/img/:

    optiland_dispersion.png   config C in the dispersion plane, rays colored by wavelength
    optiland_field_lens.png   configs A, B and C in the slit plane: where light from
                              the ends of the line goes, and where it gets clipped
    sensor_frame.png          simulated IMX219 frames: a white target under halogen
                              light, a neon lamp, and a zoom on one neon line's smile

Geometry is traced, not drawn: ray paths, vignetting, where each wavelength lands
on the sensor, smile and keystone all come from the model. Brightness in
sensor_frame.png is illustrative only: a 2850 K halogen spectrum times a generic
silicon response, and neon line strengths rounded to three levels.

Run:  python3 optics/readme_figures.py   (from any directory)
Needs: pip install optiland   (tested with 0.6.2)
"""

import warnings
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Ellipse, Rectangle
from scipy.interpolate import RegularGridInterpolator

from spectrograph_model import CONFIGS, PIXEL_MM, analyse, build, transmission

warnings.filterwarnings("ignore")

OUT_DIR = Path(__file__).resolve().parent.parent / "docs" / "img"

# Dark "lab" palette shared with the SVG diagrams in docs/img/
BG = "#0b1020"
FG = "#e6e9f2"
MUTED = "#8b93b0"
FAINT = "#2a3358"
LENS = "#7dd3fc"
SLIT = "#e2e8f0"
GRATING = "#c4b5fd"
SENSOR = "#34d399"
CLIP = "#f87171"
CENTER = "#38bdf8"
EDGE = "#fbbf24"

# Display color for each wavelength. 500-700 nm follow the visible spectrum.
# Past 700 nm the light is invisible, so the NIR end is false color (crimson to purple).
SPECTRUM = [(500, "#00f5a0"), (530, "#5cff3a"), (560, "#c8ff00"), (580, "#ffe600"),
            (600, "#ffa600"), (630, "#ff5e00"), (660, "#ff2a1f"), (700, "#e8173a"),
            (800, "#d41a55"), (900, "#a61a6e"), (1000, "#7a1c80")]


def _rgb(hex_color):
    h = hex_color.lstrip("#")
    return np.array([int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)])


_SPEC_NM = np.array([s[0] for s in SPECTRUM], float)
_SPEC_RGB = np.array([_rgb(s[1]) for s in SPECTRUM])


def wl_rgb(nm):
    """RGB (0-1) display color for wavelength(s) in nm; returns shape (..., 3)."""
    nm = np.clip(np.asarray(nm, float), _SPEC_NM[0], _SPEC_NM[-1])
    return np.stack([np.interp(nm, _SPEC_NM, _SPEC_RGB[:, k]) for k in range(3)], -1)


def style_axes(ax):
    ax.set_facecolor(BG)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)


def surface_index(o, comment):
    return [s.comment for s in o.surface_group.surfaces].index(comment)


def surface_pose(o, comment):
    """(z, y, x, tilt) of a surface's center; tilt in radians about x."""
    cs = o.surface_group.surfaces[surface_index(o, comment)].geometry.cs
    return float(cs.z), float(cs.y), float(cs.x), float(cs.rx)


def trace_paths(o, Hx, wl, n, axis):
    """Trace a fan of rays; return (z, t, alive) with t = y or x, shape (surfaces, rays).

    alive[k, r] is False from the first surface where ray r was clipped onward."""
    distribution = "line_y" if axis == "y" else "line_x"
    o.trace(Hx, 0.0, wl, num_rays=n, distribution=distribution)
    sg = o.surface_group
    z = np.asarray(sg.z, float)
    t = np.asarray(sg.y if axis == "y" else sg.x, float)
    alive = np.asarray(sg.intensity, float) > 0
    return z, t, alive


def glow_lines(ax, segs, color, lw=1.1, alpha=0.95, glow=True):
    if glow:
        ax.add_collection(LineCollection(segs, colors=[color], linewidths=lw * 4.5,
                                         alpha=0.10, capstyle="round"))
    ax.add_collection(LineCollection(segs, colors=[color], linewidths=lw,
                                     alpha=alpha, capstyle="round"))


def draw_lens(ax, z, t, height, tilt=0.0, width=0.9, color=LENS):
    ax.add_patch(Ellipse((z, t), width, height, angle=np.degrees(tilt),
                         facecolor=color, alpha=0.16, edgecolor="none", zorder=3))
    ax.add_patch(Ellipse((z, t), width, height, angle=np.degrees(tilt),
                         facecolor="none", edgecolor=color, lw=1.4, zorder=4))


def draw_bar(ax, z, t, half, tilt=0.0, color=SENSOR, lw=5.0):
    dz, dt = -np.sin(tilt) * half, np.cos(tilt) * half
    ax.plot([z - dz, z + dz], [t - dt, t + dt], color=color, lw=lw,
            solid_capstyle="butt", zorder=5)


def draw_train(ax, o, cfg, axis):
    """Lenses, slit, grating and sensor of one config, in the y (dispersion) or x (slit) plane."""
    z_obj, *_ = surface_pose(o, "objective")
    draw_lens(ax, z_obj, 0.0, 4.6)
    for s in (1, -1):  # printed washer = aperture stop
        ax.plot([z_obj, z_obj], [s * 2.05, s * 3.0], color=MUTED, lw=3, zorder=5)
    z_slit, *_ = surface_pose(o, "slit")
    if axis == "y":  # slit seen across its 50 um width
        for s in (1, -1):
            ax.plot([z_slit, z_slit], [s * 0.18, s * 3.2], color=SLIT, lw=2.6, zorder=5)
    else:  # slit seen along its 5 mm length
        for s in (1, -1):
            ax.plot([z_slit, z_slit], [s * cfg.slit_len / 2, s * 3.6], color=SLIT, lw=2.6,
                    zorder=5)
    if cfg.f_field:
        z_fl, *_ = surface_pose(o, "field lens")
        draw_lens(ax, z_fl + 0.25, 0.0, 9.0, width=0.7, color=SENSOR)
    z_c, *_ = surface_pose(o, "collimator")
    draw_lens(ax, z_c, 0.0, cfg.d_coll)
    z_g, *_ = surface_pose(o, "grating")
    ax.add_patch(Rectangle((z_g - 0.25, -5.0), 0.5, 10.0, facecolor=GRATING, alpha=0.35,
                           edgecolor=GRATING, lw=1.0, hatch="////", zorder=4))
    z_l, y_l, _, rx = surface_pose(o, "camera lens")
    z_s, y_s, _, _ = surface_pose(o, "sensor")
    if axis == "y":
        draw_lens(ax, z_l, y_l, cfg.d_cam, tilt=-rx, width=0.9)
        draw_bar(ax, z_s, y_s, 3.68 / 2, tilt=-rx)
    else:  # the tilt is about x, so in the slit plane the camera looks straight on
        draw_lens(ax, z_l, 0.0, cfg.d_cam, width=0.9)
        draw_bar(ax, z_s, 0.0, 2.76 / 2)


def colorbar(fig, rect, title=True):
    cax = fig.add_axes(rect)
    nm = np.linspace(500, 1000, 600)
    cax.imshow(wl_rgb(nm)[None, :, :], aspect="auto", extent=(500, 1000, 0, 1))
    cax.set_yticks([])
    cax.set_xticks([500, 600, 700, 800, 900, 1000])
    cax.tick_params(colors=MUTED, labelsize=10.5, length=3)
    for s in cax.spines.values():
        s.set_visible(False)
    cax.axvline(700, color=BG, lw=1.5)
    if title:
        cax.set_title("wavelength, nm    (past 700 nm: NIR, false color)", color=MUTED,
                      fontsize=10.5, loc="left", pad=4)
    return cax


# ---------------------------------------------------------------------------
# Figure 1: dispersion plane
# ---------------------------------------------------------------------------

def fig_dispersion(path):
    cfg = CONFIGS[2]
    o, res = analyse(cfg)
    g = surface_index(o, "grating")
    wls = np.round(np.arange(1.0, 0.4999, -0.05), 3)  # long first, so visible ends on top
    period_um = 1000.0 / cfg.lines_per_mm
    angle = lambda wl: np.degrees(np.arcsin(wl / period_um))

    fig = plt.figure(figsize=(12, 7.4), facecolor=BG)
    top = fig.add_axes([0.02, 0.585, 0.96, 0.29])
    zoom = fig.add_axes([0.02, 0.02, 0.62, 0.55])
    for ax in (top, zoom):
        style_axes(ax)

    def rays(ax, nray, glow):
        z, y, _ = trace_paths(o, 0.0, 0.75, nray, "y")
        white = [np.column_stack([z[:g + 1, r], y[:g + 1, r]]) for r in range(nray)]
        glow_lines(ax, white, "#f8fafc", lw=0.9, alpha=0.75, glow=glow)
        for wl in wls:
            z, y, alive = trace_paths(o, 0.0, wl, nray, "y")
            segs = [np.column_stack([z[g:, r], y[g:, r]]) for r in range(nray)
                    if alive[-1, r]]
            glow_lines(ax, segs, wl_rgb(wl * 1000), lw=1.0, alpha=0.9, glow=glow)

    # whole train, objective to sensor
    rays(top, 9, glow=False)
    draw_train(top, o, cfg, "y")
    top.set_xlim(-13, 86)
    top.set_ylim(-6.2, 9.4)
    top.text(-12.8, -2.6, "from the scene,\n150 mm away", color=MUTED, fontsize=10.5,
             va="top", linespacing=1.3)
    for z, name, sub in [(0, "objective", f"{cfg.f_obj:.0f} mm f/{cfg.fno_obj:.0f}"),
                         (18.2, "slit + field lens",
                          f"{cfg.slit_width * 1000:.0f} µm · f {cfg.f_field:.0f} mm"),
                         (42.9, "collimator", f"{cfg.f_coll:.0f} mm, reversed")]:
        top.text(z, 6.6, name, color=FG, ha="center", va="bottom", fontsize=11.5,
                 fontweight="bold")
        top.text(z, 6.5, sub, color=MUTED, ha="center", va="top", fontsize=10)
    zx0, zx1, zy0, zy1 = 61.5, 81.5, -5.4, 8.3
    top.add_patch(Rectangle((zx0, zy0), zx1 - zx0, zy1 - zy0, fill=False, edgecolor=MUTED,
                            lw=1.0, ls=(0, (4, 3)), zorder=9))
    top.text(zx0 - 0.8, zy1, "grating → camera\n(zoomed below)", color=MUTED, fontsize=10,
             ha="right", va="top", linespacing=1.3)
    fig.text(0.02, 0.965, "Optiland ray trace  ·  dispersion plane", color=FG, fontsize=17,
             fontweight="bold", va="top")
    fig.text(0.02, 0.918, "Recommended design (config C), ideal lenses. White light from the slit is collimated, "
             "fanned out by the grating and refocused onto the sensor.", color=MUTED,
             fontsize=11.5, va="top")

    # zoom on grating -> sensor
    rays(zoom, 7, glow=True)
    draw_train(zoom, o, cfg, "y")
    zoom.set_xlim(zx0, zx1)
    zoom.set_ylim(zy0, zy1)
    zoom.add_patch(Rectangle((zx0, zy0), zx1 - zx0, zy1 - zy0, fill=False, edgecolor=MUTED,
                             lw=1.0, ls=(0, (4, 3))))
    th = o._theta
    z_g, *_ = surface_pose(o, "grating")
    z_s, y_s, _, _ = surface_pose(o, "sensor")
    zoom.plot([z_g, z_g + 11], [0, 0], color=MUTED, lw=0.8, ls=(0, (2, 3)))
    zoom.text(z_g + 11.2, 0, "grating normal", color=MUTED, fontsize=10, va="center")
    zoom.plot([z_g, z_s], [0, y_s], color=MUTED, lw=0.8, ls=(0, (2, 3)), zorder=6)
    arc = np.linspace(0, th, 30)
    zoom.plot(z_g + 9.5 * np.cos(arc), 9.5 * np.sin(arc), color=MUTED, lw=0.9)
    zoom.text(z_g + 9.9 * np.cos(th / 2), 9.9 * np.sin(th / 2), f"{np.degrees(th):.0f}°",
              color=FG, fontsize=11, va="center")
    z_l, y_l, _, rx = surface_pose(o, "camera lens")
    zoom.text(z_l + 1.6, -2.4, "camera lens", color=FG, fontsize=11, ha="left", va="top",
              fontweight="bold")
    zoom.text(z_l + 1.6, -3.2, f"{cfg.f_cam:.0f} mm f/2, tilted {np.degrees(th):.0f}°",
              color=MUTED, fontsize=10, ha="left", va="top")
    zoom.text(z_s + 0.9, y_s - 2.6, "IMX219", color=SENSOR, fontsize=11.5, ha="center",
              va="top", fontweight="bold")
    zoom.text(z_g - 0.5, -5.0, f"grating\n{cfg.lines_per_mm:.0f} l/mm", color=FG,
              fontsize=11, ha="right", va="bottom", fontweight="bold")
    zoom.text(zx0 + 0.3, zy1 - 0.3, "zoom: grating → sensor, 500–1000 nm in 50 nm steps",
              color=MUTED, fontsize=10.5, va="top")

    # side panel: the numbers, computed from the model
    x = 0.675
    rows = [(0.535, "First order at the grating", FG, 13, "bold"),
            (0.495, f"500 nm leaves at {angle(0.5):.1f}°, 1000 nm at {angle(1.0):.0f}°.\n"
                    f"The camera looks at 750 nm ({np.degrees(th):.0f}°).", MUTED, 11),
            (0.395, "On the sensor", FG, 13, "bold"),
            (0.355, f"500–1000 nm spans {res['spectrum_mm']:.2f} mm, about\n"
                    f"{res['disp_px_nm']:.1f} px per nm on the IMX219.", MUTED, 11),
            (0.255, "Resolution", FG, 13, "bold"),
            (0.215, f"The {cfg.slit_width * 1000:.0f} µm slit images to "
                    f"{res['slit_img_um']:.0f} µm "
                    f"({res['slit_img_um'] / 1000 / PIXEL_MM:.0f} px),\n"
                    f"so about {res['res_nm']:.1f} nm with perfect lenses.", MUTED, 11)]
    for y, text, color, size, *weight in rows:
        fig.text(x, y, text, color=color, fontsize=size, va="top",
                 fontweight=weight[0] if weight else "normal", linespacing=1.4)
    colorbar(fig, [x, 0.065, 0.30, 0.035])
    fig.savefig(path, dpi=150, facecolor=BG)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 2: slit plane, what the field lens fixes
# ---------------------------------------------------------------------------

def _percent_range(values):
    lo, hi = (round(100 * v) for v in (min(values), max(values)))
    return f"{lo}%" if lo == hi else f"{lo}–{hi}%"


def fig_field_lens(path):
    names = ["Stock Pi NoIR v2 lens", "M12 IMX219 + 12 mm f/2 lens", "Same + 18 mm field lens"]
    fig = plt.figure(figsize=(12, 6.6), facecolor=BG)
    fig.text(0.02, 0.965, "Why the camera lens and the field lens changed", color=FG,
             fontsize=17, fontweight="bold", va="top")
    fig.text(0.02, 0.905, "Slit plane: light from the center (blue) and both ends (amber) "
             "of the scan line, ✕ where rays get clipped.\nPercentages are the share of the "
             "light that reaches the sensor, 500–1000 nm.", color=MUTED, fontsize=11.5,
             va="top", linespacing=1.5)
    row_h = 0.245
    for k, (cfg, name) in enumerate(zip(CONFIGS, names)):
        o = build(cfg)
        bottom = 0.80 - (k + 1) * row_h
        center = [transmission(o, 0.0, wl) for wl in (0.5, 0.75, 1.0)]
        ends = [transmission(o, 1.0, wl) for wl in (0.5, 0.75, 1.0)]
        tag = cfg.name.split(":")[0]
        fig.text(0.02, bottom + row_h - 0.035, f"{tag}", color=FG, fontsize=22,
                 fontweight="bold", va="top")
        fig.text(0.06, bottom + row_h - 0.04, name, color=FG, fontsize=12.5,
                 fontweight="bold", va="top")
        for j, (what, vals, color) in enumerate([("line center", center, CENTER),
                                                 ("line ends", ends, EDGE)]):
            ok = min(vals) > 0.9
            fig.text(0.06 + j * 0.12, bottom + row_h - 0.1, _percent_range(vals),
                     color=color if ok else CLIP, fontsize=18, fontweight="bold", va="top")
            fig.text(0.06 + j * 0.12, bottom + row_h - 0.165, what, color=MUTED,
                     fontsize=10.5, va="top")
        ax = fig.add_axes([0.30, bottom + 0.012, 0.69, row_h - 0.024])
        style_axes(ax)
        for Hx, color in [(-1.0, EDGE), (1.0, EDGE), (0.0, CENTER)]:
            z, x, alive = trace_paths(o, Hx, 0.75, 7, "x")
            segs, hits = [], []
            for r in range(z.shape[1]):
                last = np.argmin(alive[:, r]) if not alive[:, r].all() else z.shape[0]
                segs.append(np.column_stack([z[:last, r], x[:last, r]]))
                if last < z.shape[0]:  # clipped at surface `last`: end the ray there
                    segs[-1] = np.vstack([segs[-1], [z[last, r], x[last, r]]])
                    hits.append((z[last, r], x[last, r]))
            glow_lines(ax, segs, color, lw=0.9, alpha=0.85)
            for zh, xh in hits:
                ax.plot(zh, xh, marker="x", color=CLIP, ms=6, mew=1.8, zorder=12)
        draw_train(ax, o, cfg, "x")
        ax.set_xlim(-12, 86)
        ax.set_ylim(-9.8, 9.8)
    for comment, name, ha, dz in [("objective", "objective", "center", 0),
                                  ("slit", "slit + field lens", "center", 0),
                                  ("collimator", "collimator", "center", 0),
                                  ("grating", "grating", "right", 0.6),
                                  ("camera lens", "camera lens", "left", -0.6),
                                  ("sensor", "sensor", "left", 2.2)]:
        z = surface_pose(o, comment)[0] + dz  # o is config C, the last row
        ax.text(z, -10.4, name, color=MUTED, fontsize=10, ha=ha, va="top")
    fig.savefig(path, dpi=150, facecolor=BG)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 3: simulated sensor frames
# ---------------------------------------------------------------------------

SENSOR_PX = (3280, 2464)  # IMX219: dispersion axis, slit axis
SHOW = 4  # display pixel = SHOW x SHOW sensor pixels

# Approximate response of a silicon CMOS sensor with no IR-cut filter (illustrative)
SILICON_QE = [(500, 0.60), (550, 0.66), (600, 0.62), (650, 0.56), (700, 0.48), (750, 0.40),
              (800, 0.32), (850, 0.24), (900, 0.15), (950, 0.08), (1000, 0.035)]

# Ne I lines between 500 and 1000 nm (nm, air). Strengths are rounded to three levels
# for the picture; real ratios depend on the lamp and current.
NEON = {1.0: [585.25, 640.22, 692.95, 703.24, 724.52, 837.76, 849.54, 865.44, 878.06],
        0.5: [588.19, 594.48, 597.55, 603.00, 607.43, 609.62, 614.31, 616.36, 621.73,
              626.65, 630.48, 633.44, 638.30, 650.65, 653.29, 659.90, 667.83, 671.70,
              717.39, 743.89, 747.24, 748.89, 753.58, 754.40, 811.85, 813.64, 830.03,
              859.13, 863.46, 885.39],
        0.25: [794.32, 808.25, 825.94, 826.61, 836.57, 841.84, 846.34, 848.44, 854.47,
               857.14, 868.19, 870.41, 877.17, 878.38, 886.58, 891.95, 920.18, 922.01,
               930.09, 932.65, 942.54, 948.67, 953.42, 954.74]}


def qe(nm):
    return np.interp(nm, *zip(*SILICON_QE))


def planck(nm, T=2850.0):
    return nm ** -5.0 / np.expm1(1.4388e7 / (nm * T))


def sensor_map(o, n_h=41, n_wl=51):
    """Interpolators (Hx, nm) -> sensor pixel (row along the slit, col along the spectrum)."""
    th = o._theta
    cs = o.surface_group.surfaces[-1].geometry.cs
    h = np.linspace(-1, 1, n_h)
    nm = np.linspace(500, 1000, n_wl)
    rows, cols = np.zeros((n_h, n_wl)), np.zeros((n_h, n_wl))
    for j, w in enumerate(nm):
        zeros = np.zeros_like(h)
        o.trace_generic(h, zeros, zeros, zeros, w / 1000)
        sg = o.surface_group
        x, y, z = (np.asarray(a[-1], float) for a in (sg.x, sg.y, sg.z))
        v = (y - float(cs.y)) * np.cos(th) - (z - float(cs.z)) * np.sin(th)
        rows[:, j] = x / PIXEL_MM + SENSOR_PX[1] / 2
        cols[:, j] = v / PIXEL_MM + SENSOR_PX[0] / 2
    grid = dict(bounds_error=False, fill_value=None)  # extrapolate linearly past the edges
    return (RegularGridInterpolator((h, nm), rows, **grid),
            RegularGridInterpolator((h, nm), cols, **grid))


def invert_map(to_row, to_col, rows, cols, iters=6):
    """Slit position Hx and wavelength (nm) that land on sensor pixel (rows, cols).

    Newton steps on the traced map: rows depend mostly on Hx, cols mostly on wavelength."""
    h = np.zeros_like(rows)
    nm = np.full_like(rows, 750.0)
    for _ in range(iters):
        p = np.column_stack([h.ravel(), nm.ravel()])
        dr = (to_row(p + [1e-3, 0]) - to_row(p - [1e-3, 0])) / 2e-3
        h = h - ((to_row(p) - rows.ravel()) / dr).reshape(h.shape)
        p = np.column_stack([h.ravel(), nm.ravel()])
        dc = (to_col(p + [0, 0.5]) - to_col(p - [0, 0.5])) / 1.0
        nm = nm - ((to_col(p) - cols.ravel()) / dc).reshape(nm.shape)
    return h, nm


def splat(img, rows, cols, rgb):
    """Add rgb at fractional (rows, cols), split linearly between neighbouring columns."""
    r = np.clip(np.round(rows).astype(int), 0, img.shape[0] - 1)
    c0 = np.floor(cols).astype(int)
    f = cols - c0
    for c, wgt in ((c0, 1 - f), (c0 + 1, f)):
        ok = (c >= 0) & (c < img.shape[1])
        np.add.at(img, (r[ok], c[ok]), rgb[ok] * wgt[ok, None])


def box_blur_cols(img, width):
    """Convolve along the dispersion axis with the slit image (a box `width` px wide)."""
    n = int(np.ceil(width)) | 1
    k = np.ones(n)
    k[[0, -1]] = (width - (n - 2)) / 2  # fractional edges
    k /= k.sum()
    out = np.empty_like(img)
    for ch in range(img.shape[2]):
        out[..., ch] = np.apply_along_axis(lambda a: np.convolve(a, k, "same"), 1, img[..., ch])
    return out


def gauss_blur(img, sigma):
    from scipy.ndimage import gaussian_filter
    return gaussian_filter(img, sigma=(sigma, sigma, 0))


def tone(img, white):
    return np.clip(1 - np.exp(-2.2 * img / white), 0, 1)


def fig_sensor_frame(path):
    cfg = CONFIGS[2]
    o, res = analyse(cfg)
    to_row, to_col = sensor_map(o)
    H, W = SENSOR_PX[1] // SHOW, SENSOR_PX[0] // SHOW
    slit_px = res["slit_img_um"] / 1000 / PIXEL_MM  # slit image width, sensor px

    # white PTFE under a halogen lamp: invert the traced map, so every display pixel
    # gets the slit position and wavelength that land on it (2x2 supersampled)
    rr, cc = np.mgrid[0:2 * H, 0:2 * W] * (SHOW / 2) + SHOW / 4
    h_at, wl_at = invert_map(to_row, to_col, rr, cc)
    lit = (np.abs(h_at) <= 1) & (wl_at >= 500) & (wl_at <= 1000)
    weight = np.where(lit, planck(np.clip(wl_at, 500, 1000)) * qe(np.clip(wl_at, 500, 1000)), 0)
    weight /= weight.max()
    cont = wl_rgb(wl_at) * (1 - np.exp(-2.6 * weight))[..., None]
    cont = cont.reshape(H, 2, W, 2, 3).mean(axis=(1, 3))
    cont = gauss_blur(box_blur_cols(cont, slit_px / SHOW), 0.5)

    # neon lamp: discrete lines, each bent by smile
    h = np.linspace(-1, 1, 4000)
    neon = np.zeros((H, W, 3))
    for strength, lines in NEON.items():
        for line in lines:
            pts = np.column_stack([h, np.full_like(h, line)])
            rgb = np.tile(wl_rgb(line) * strength * qe(line) / qe(500), (len(h), 1))
            splat(neon, to_row(pts) / SHOW, to_col(pts) / SHOW, rgb)
    neon = box_blur_cols(neon, slit_px / SHOW)
    neon = neon + 0.6 * gauss_blur(neon, 4.0)
    per_row = len(h) / (np.ptp(to_row(np.column_stack([h, np.full_like(h, 750.0)]))) / SHOW)
    neon = tone(neon, 0.3 * per_row)

    # one neon line at full sensor resolution, for the smile zoom
    line = 703.24
    pts = np.column_stack([h, np.full_like(h, line)])
    l_rows, l_cols = to_row(pts), to_col(pts)
    c_mid = float(to_col([[0.0, line]])[0])
    smile_px = float(to_col([[1.0, line]])[0] - c_mid)
    half = 48
    strip = np.zeros((SENSOR_PX[1] // SHOW, 2 * half, 3))
    rgb = np.tile(wl_rgb(line), (len(h), 1))
    splat(strip, l_rows / SHOW, l_cols - (c_mid - half), rgb)
    strip = box_blur_cols(strip, slit_px)
    strip = tone(strip, 0.45 * per_row)

    fig = plt.figure(figsize=(12, 5.6), facecolor=BG)
    fig.text(0.02, 0.965, "What the camera should see", color=FG, fontsize=17,
             fontweight="bold", va="top")
    fig.text(0.02, 0.895, "Simulated IMX219 frames from the Optiland model (ideal lenses). "
             "Each frame is one line of the scene (vertical) by its spectrum (horizontal).\n"
             "Where each wavelength lands, smile and keystone are traced; brightness is "
             "illustrative. Colors past 700 nm are false color.", color=MUTED, fontsize=11,
             va="top", linespacing=1.5)
    frame_w, frame_h, y0 = 0.36, 0.36 * 12 / 5.6 * H / W, 0.11
    tick_nm = [500, 600, 700, 800, 900, 1000]
    tick_cols = to_col(np.column_stack([np.zeros(6), tick_nm])) / SHOW
    for k, (img, title) in enumerate([(cont, "White target under a halogen lamp"),
                                      (neon, "Neon lamp, for wavelength calibration")]):
        ax = fig.add_axes([0.05 + k * 0.385, y0, frame_w, frame_h])
        ax.imshow(img, interpolation="bilinear", extent=(0, W, H, 0))
        ax.set_xticks(tick_cols, [f"{t}" for t in tick_nm])
        ax.set_yticks([])
        ax.tick_params(colors=MUTED, labelsize=10, length=3)
        for sp in ax.spines.values():
            sp.set_color(FAINT)
        ax.set_title(title, color=FG, fontsize=12.5, fontweight="bold", loc="left", pad=7)
        ax.set_xlabel("wavelength, nm", color=MUTED, fontsize=10.5, labelpad=2)
        if k == 0:
            ax.set_ylabel("along the scan line", color=MUTED, fontsize=10.5)
        if k == 1:
            ax.axvline(c_mid / SHOW, color=FG, lw=0.8, ls=(0, (3, 3)), alpha=0.8)
            ax.text(c_mid / SHOW + 6, 18, f"{line:.0f} nm", color=FG, fontsize=9.5,
                    va="top")
    ax = fig.add_axes([0.83, y0, 0.15, frame_h])
    ax.imshow(strip, interpolation="bilinear", aspect="auto",
              extent=(-half, half, SENSOR_PX[1], 0))
    ax.plot(l_cols - c_mid, l_rows, color=FG, lw=1.4)
    ax.axvline(0, color=FG, lw=0.9, ls=(0, (3, 3)), alpha=0.7)
    ax.set_xlim(-half, half)
    ax.set_ylim(SENSOR_PX[1], 0)
    ax.set_xticks([-40, 0, 40], ["−40", "0", "+40"])
    ax.set_yticks([])
    ax.tick_params(colors=MUTED, labelsize=10, length=3)
    for sp in ax.spines.values():
        sp.set_color(FAINT)
    fig.canvas.draw()
    bb = ax.get_window_extent()
    stretch = (bb.width / (2 * half)) / (bb.height / SENSOR_PX[1])
    ax.set_title(f"Smile, {line:.0f} nm line", color=FG, fontsize=12.5, fontweight="bold",
                 loc="left", pad=7)
    ax.set_xlabel(f"sensor px, ×{stretch:.0f} sideways", color=MUTED, fontsize=10.5,
                  labelpad=2)
    top_row = float(l_rows.min())
    ax.annotate("", xy=(smile_px, top_row + 40), xytext=(0, top_row + 40),
                arrowprops=dict(arrowstyle="<->", color=FG, lw=0.9, shrinkA=0, shrinkB=0))
    ax.text(-3, top_row + 40, f"{smile_px:.0f} px", color=FG, fontsize=10.5, ha="right",
            va="center")
    ax.text(-half + 4, SENSOR_PX[1] / 2, f"slit image\n{slit_px:.0f} px wide", color=MUTED,
            fontsize=9.5, va="center")
    fig.savefig(path, dpi=150, facecolor=BG)
    plt.close(fig)


if __name__ == "__main__":
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fig_dispersion(OUT_DIR / "optiland_dispersion.png")
    fig_field_lens(OUT_DIR / "optiland_field_lens.png")
    fig_sensor_frame(OUT_DIR / "sensor_frame.png")
