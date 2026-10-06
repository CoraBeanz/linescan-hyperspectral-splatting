"""Where along the slit each pixel looks: slit ends, wire shadows and keystone.

The slit's image is not quite a rectangle. Its length changes a little with
wavelength (keystone) and the lens bends straight lines (distortion), so the
row that sees a given point of the slit drifts across the spectrum. Two kinds
of features fix the slit coordinate:

* the slit's two ends, the top and bottom edges of the lit band in a halogen
  flat, and
* optionally a few thin wires across the slit (or across the target in front
  of the objective), which print dark lines at fixed slit positions.

Each feature is followed across the spectrum. Its row at the reference column
x_ref defines its slit coordinate s, and

    s(x, y) = y + Delta(x, y),  Delta(x_ref, y) = 0,

so slit coordinates are "rows at the reference column". With only the two
ends, Delta is linear in y (a magnification change); wires add curvature.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.signal import find_peaks

from .poly import Poly1D, Poly2D
from .profiles import centroid, half_max_width


def column_bins(x_lo, x_hi, width):
    """Edges of column bins about ``width`` px wide covering [x_lo, x_hi)."""
    n = max(1, int(round((x_hi - x_lo) / width)))
    return np.linspace(x_lo, x_hi, n + 1).round().astype(int)


def bin_columns(img, edges):
    """Mean over each column bin: (H, n_bins) profiles, and bin centres."""
    a = np.asarray(img, float)
    sums = np.add.reduceat(a[:, edges[0]:edges[-1]], edges[:-1] - edges[0], axis=1)
    return sums / np.diff(edges)[None, :], 0.5 * (edges[:-1] + edges[1:] - 1)


def _crossing(p, i_out, i_in, level):
    """Fractional index where p crosses ``level`` between i_out (below) and i_in (above)."""
    step = 1 if i_in > i_out else -1
    i = i_out
    while i != i_in and p[i + step] < level:
        i += step
    if i == i_in:
        return np.nan
    a, b = p[i], p[i + step]
    return i + step * (level - a) / (b - a) if b != a else np.nan


def slit_edges(profiles, plateau_px=(6, 30), min_level=0.05):
    """Top and bottom edge rows (50% of the local plateau) of each column profile.

    profiles: (H, n) array, the flat averaged over column bins. plateau_px:
    where to read the plateau level, as (start, end) rows in from the edge.
    Returns (top, bottom, level) arrays of length n; NaN where a column is too dim."""
    H, n = profiles.shape
    top, bot, lev = (np.full(n, np.nan) for _ in range(3))
    peak = np.nanpercentile(profiles, 99.5)
    a, b = plateau_px
    for k in range(n):
        p = profiles[:, k]
        pk = np.percentile(p, 98)
        if not pk > min_level * peak:
            continue
        lit = np.flatnonzero(p > 0.25 * pk)
        if lit.size < 3 * b:
            continue
        i0, i1 = lit[0], lit[-1]
        pt = np.median(p[i0 + a:i0 + b])
        pb = np.median(p[i1 - b + 1:i1 - a + 1])
        top[k] = _crossing(p, max(i0 - 2 * a, 0), i0 + a, 0.5 * pt)
        bot[k] = _crossing(p, min(i1 + 2 * a, H - 1), i1 - a, 0.5 * pb)
        lev[k] = np.median(p[i0 + a:i1 - a])
    return top, bot, lev


def wire_notches(ratio_profiles, top, bot, margin=8, min_depth=0.12):
    """Rows of the dark lines that wires print on each column profile.

    ratio_profiles: (H, n) wires frame / flat, binned like the flat. Returns a
    list (one per column bin) of arrays of notch rows, and the typical notch
    width (FWHM, px)."""
    H, n = ratio_profiles.shape
    out, widths = [], []
    for k in range(n):
        if not (np.isfinite(top[k]) and np.isfinite(bot[k])):
            out.append(np.zeros(0))
            continue
        lo, hi = int(np.ceil(top[k])) + margin, int(np.floor(bot[k])) - margin
        d = 1.0 - ratio_profiles[lo:hi, k]
        d = np.where(np.isfinite(d), d, 0.0)
        pk, _ = find_peaks(d, height=min_depth, distance=max(3, margin))
        if pk.size == 0:
            out.append(np.zeros(0))
            continue
        w = half_max_width(np.repeat(d[None], pk.size, 0), pk.astype(float), 3.0 * margin)
        w = np.where(np.isfinite(w), w, 2.0 * margin)
        widths.extend(w.tolist())
        x, _, ok = centroid(np.repeat(d[None], pk.size, 0), pk.astype(float), np.median(w), bg_px=3)
        out.append(x[ok] + lo)
    return out, (float(np.median(widths)) if widths else float("nan"))


def link_tracks(points, xb, k0, max_step=3.0, min_cover=0.5):
    """Join per-bin detections into tracks running across the spectrum.

    points: list of arrays (rows found in each bin); k0: the bin to start from.
    Returns a list of (x, y) arrays, one per track."""
    tracks = []
    for y0 in points[k0]:
        ys = np.full(len(points), np.nan)
        ys[k0] = y0
        for step in (1, -1):
            last, slope = y0, 0.0
            k = k0 + step
            while 0 <= k < len(points):
                pred = last + slope
                cand = points[k]
                if cand.size:
                    j = int(np.argmin(np.abs(cand - pred)))
                    if abs(cand[j] - pred) < max_step:
                        slope = 0.7 * slope + 0.3 * (cand[j] - last)
                        last = ys[k] = cand[j]
                k += step
        good = np.isfinite(ys)
        if good.mean() >= min_cover:
            tracks.append((xb[good], ys[good]))
    return tracks


@dataclass
class Keystone:
    """s(x, y) = y + Delta(x, y): slit coordinate as rows at the reference column."""
    poly: Poly2D
    x_ref: float
    s_top: float            # slit ends at x_ref
    s_bottom: float
    rms_px: float
    tracks: list = field(default_factory=list)   # per track: dict(kind, s, x, y, rms)

    def delta(self, x, y):
        return self.poly(x, y)

    def s(self, x, y):
        return np.asarray(y, float) + self.poly(x, y)

    def h(self, x, y):
        """Normalised slit position: -1 at the top end, +1 at the bottom end."""
        c = 0.5 * (self.s_top + self.s_bottom)
        return (self.s(x, y) - c) / (0.5 * (self.s_bottom - self.s_top))

    def to_dict(self):
        return dict(poly=self.poly.to_dict(), x_ref=self.x_ref, s_top=self.s_top,
                    s_bottom=self.s_bottom, rms_px=self.rms_px,
                    tracks=[{k: v for k, v in t.items() if k in ("kind", "s", "rms")} for t in self.tracks])

    @classmethod
    def from_dict(cls, d):
        return cls(Poly2D.from_dict(d["poly"]), d["x_ref"], d["s_top"], d["s_bottom"], d["rms_px"],
                   d.get("tracks", []))


def fit_keystone(tracks, x_ref, W, H, x_deg=3):
    """Fit Delta(x, y) to tracks of constant slit position.

    tracks: list of dict(kind, x, y). Each track's own smooth fit in x gives its
    row at x_ref, which is its slit coordinate s."""
    xs, ys, ds, sig, kept = [], [], [], [], []
    for t in tracks:
        x, y = np.asarray(t["x"], float), np.asarray(t["y"], float)
        if x.size < 6:
            continue
        p = Poly1D(min(x_deg, x.size - 3), x0=(W - 1) / 2.0, xs=(W - 1) / 2.0)
        keep = p.fit(x, y, clip=3.5)
        res = (y - p(x))[keep]
        rms = float(max(np.std(res), 0.02))
        s = float(p(x_ref))
        t.update(s=s, rms=rms)
        xs.append(x[keep])
        ys.append(y[keep])
        ds.append(s - y[keep])
        sig.append(np.full(int(keep.sum()), rms))
        kept.append(t)
    if len(kept) < 2:
        raise RuntimeError("could not follow both ends of the slit across the spectrum")
    n_rows = len({round(t["s"]) for t in kept})
    y_deg = min(3, n_rows - 1)
    terms = [(i, j) for i in range(1, x_deg + 1) for j in range(y_deg + 1)]
    ends = sorted(t["s"] for t in kept if t["kind"] == "end")
    s_top, s_bot = (ends[0], ends[-1]) if len(ends) >= 2 else (min(t["s"] for t in kept),
                                                                max(t["s"] for t in kept))
    poly = Poly2D(terms, x0=x_ref, xs=(W - 1) / 2.0, y0=0.5 * (s_top + s_bot),
                  ys=0.5 * (s_bot - s_top))
    x, y, d, sg = (np.concatenate(a) for a in (xs, ys, ds, sig))
    keep = poly.fit(x, y, d, sg, clip=4.0)
    rms = float(np.sqrt(np.mean((poly(x, y) - d)[keep] ** 2)))
    return Keystone(poly=poly, x_ref=float(x_ref), s_top=float(s_top), s_bottom=float(s_bot),
                    rms_px=rms, tracks=kept)


def measure_keystone(flat_s, wires_s=None, x_lo=0, x_hi=None, x_ref=None, bin_px=16, log=print):
    """Slit ends from a smoothed flat, wire lines from a smoothed wires frame; fit the keystone.

    Returns (Keystone, info dict with the per-bin edge rows for plotting)."""
    H, W = flat_s.shape
    x_hi = W if x_hi is None else x_hi
    edges = column_bins(x_lo, x_hi, bin_px)
    prof, xb = bin_columns(flat_s, edges)
    a = max(6, int(round(12 * H / 2464.0)))
    top, bot, lev = slit_edges(prof, plateau_px=(a, 5 * a))
    good = np.isfinite(top) & np.isfinite(bot)
    if good.sum() < 6:
        raise RuntimeError("the flat does not show the slit's two ends clearly")
    tracks = [dict(kind="end", x=xb[good], y=top[good]), dict(kind="end", x=xb[good], y=bot[good])]
    n_wires = 0
    if wires_s is not None:
        wp, _ = bin_columns(wires_s, edges)
        with np.errstate(divide="ignore", invalid="ignore"):
            ratio = np.where(prof > 0.1 * np.nanmax(prof), wp / prof, np.nan)
        notches, wire_fwhm = wire_notches(ratio, top, bot, margin=a)
        # start from the bin nearest x_ref that shows the usual number of wires
        counts = np.array([len(n) for n in notches])
        usual = np.bincount(counts[counts > 0]).argmax() if np.any(counts > 0) else 0
        xr = x_ref if x_ref is not None else 0.5 * (x_lo + x_hi)
        k0 = int(np.argmin(np.where(counts == usual, np.abs(xb - xr), np.inf)))
        for xw, yw in link_tracks(notches, xb, k0):
            tracks.append(dict(kind="wire", x=xw, y=yw))
            n_wires += 1
    x_ref = 0.5 * (x_lo + x_hi - 1) if x_ref is None else x_ref
    ks = fit_keystone(tracks, x_ref, W, H)
    log(f"  slit ends at rows {ks.s_top:.1f} and {ks.s_bottom:.1f} (column {x_ref:.0f}), "
        f"{n_wires} wire line(s), fit rms {ks.rms_px:.2f} px")
    return ks, dict(x=xb.tolist(), top=top.tolist(), bottom=bot.tolist(), level=lev.tolist())


# ---------------------------------------------------------------------------
# The rectifying warp
# ---------------------------------------------------------------------------


def rectify_maps(nm_to_xc, smile_delta, keystone_delta, nm_grid, s_grid, iters=8):
    """Source pixel (x, y) for every output cell (slit row s, wavelength nm).

    Solves  x - delta(x, y) = x_c(nm)  and  y + Delta(x, y) = s  by fixed-point
    iteration (both corrections change slowly, so it converges in a few steps).
    smile_delta and keystone_delta are functions of (x, y). Returns float32
    (map_x, map_y), shaped (len(s_grid), len(nm_grid)), ready for cv2.remap or
    scipy's map_coordinates."""
    xc = np.asarray(nm_to_xc(np.asarray(nm_grid, float)), float)
    S, XC = np.meshgrid(np.asarray(s_grid, float), xc, indexing="ij")
    x, y = XC.copy(), S.copy()
    for _ in range(iters):
        x_new = XC + smile_delta(x, y)
        y_new = S - keystone_delta(x_new, y)
        done = max(np.max(np.abs(x_new - x)), np.max(np.abs(y_new - y))) < 1e-4
        x, y = x_new, y_new
        if done:
            break
    return x.astype(np.float32), y.astype(np.float32)


def remap(img, map_x, map_y, order=1, cval=np.nan):
    """Sample img at (map_x, map_y). Uses OpenCV when installed, else scipy."""
    try:
        import cv2
        border = cv2.BORDER_CONSTANT
        interp = cv2.INTER_LINEAR if order == 1 else cv2.INTER_CUBIC
        a = np.asarray(img, np.float32)
        return cv2.remap(a, map_x, map_y, interp, borderMode=border,
                         borderValue=float(cval) if np.isfinite(cval) else np.nan)
    except ImportError:
        from scipy.ndimage import map_coordinates
        return map_coordinates(np.asarray(img, float), [map_y, map_x], order=order, mode="constant",
                               cval=cval).astype(np.float32)
