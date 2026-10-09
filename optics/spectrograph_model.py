"""
Line-scan (pushbroom) spectrograph: Optiland model of the full optical train.

    scene --(150 mm)--> objective (16 mm, f/4, aperture stop)
          --> slit (50 um x 5 mm, slit long axis = x)
          --> [optional field lens right behind the slit]
          --> collimator (25 mm M12 lens used backwards)
          --> 500 l/mm transmission grating (grooves along x, disperses in y)
          --> camera lens tilted to the 1st order at 750 nm
          --> IMX219 sensor (1.12 um pixels)

Every M12 lens is a *paraxial* (ideal thin) lens. M12 board lenses ship without
a published prescription, so this model answers first-order questions honestly
(dispersion, slit image size, line length, smile, keystone, vignetting) but
says nothing about the real lenses' aberrations. Swap in a catalog lens
(e.g. an Edmund/Thorlabs achromat from Optiland's database) when you have one.

Configs A to C are the design study. D and E use the parts that were bought,
where the vendor publishes enough to model them: the Edmund #49-840 field lens
as real N-BK7 surfaces, the 3 mm GG-495 filter as a glass plate, the printed
stop and the 20 x 20 mm scan mirror as apertures, and the CIL161's 15.6 mm focal
length. D places them as cad/ first drew them; E flips the field lens and moves
the stop, the two changes the trace argues for, and is what cad/ builds now. For
D and E the script also traces the slit image width, the band that lands on the
sensor and the spot sizes.

Conventions: z is the optical axis, y is the dispersion direction, x is along
the slit (the spatial / scan-line direction). Units are mm; wavelengths in um.

Run:  python3 spectrograph_model.py   (from any directory; prints the tables and
      writes results.txt + layout PNGs to model_output/ next to this script)
Needs: pip install optiland  (tested with 0.6.x)
"""

import warnings
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from optiland.materials import IdealMaterial, Material
from optiland.optic import Optic
from optiland.physical_apertures import RadialAperture, RectangularAperture

warnings.filterwarnings("ignore")

PIXEL_MM = 1.12e-3  # IMX219 pixel pitch
SENSOR_MM = (3.68, 2.76)  # IMX219 active area (w, h)
WAVELENGTHS = [0.50, 0.60, 0.70, 0.75, 0.80, 0.90, 1.00]  # um
CENTER_WL = 0.75
# SCHOTT GG495, 3 mm (Edmund #54-652): internal transmittance from SCHOTT's
# datasheet; times the reflection factor 0.917 for the two bare faces.
GG495_TAU = {480: 0.0028, 490: 0.218, 500: 0.732, 510: 0.918, 520: 0.962,
             550: 0.986, 600: 0.989, 700: 0.976, 750: 0.968, 800: 0.959,
             900: 0.944, 1000: 0.937}
GG495_PD = 0.917


@dataclass
class Config:
    name: str
    f_cam: float  # camera lens focal length
    d_cam: float  # camera lens clear aperture (entrance pupil) diameter
    f_field: float | None = None  # field lens at the slit (None = no field lens)
    coll_to_grating: float = 10.0
    grating_to_cam: float = 5.0
    f_obj: float = 16.0
    fno_obj: float = 4.0
    f_coll: float = 25.0
    d_coll: float = 10.0  # collimator clear aperture (assumed for a 25 mm M12 lens)
    scene_dist: float = 150.0
    slit_len: float = 5.0
    slit_width: float = 0.050
    lines_per_mm: float = 500.0


@dataclass
class Bought(Config):
    """A layout with some parts modelled as they really are (config D).

    The defaults model nothing extra, so Bought(**asdict(c)) traces exactly like c.
    A separate class keeps Config's fields, which other modules export, as they were."""
    field_r: float | None = None  # real plano-convex field lens: convex radius (None = ideal lens)
    field_ct: float = 5.25  # its centre thickness
    field_flat_first: bool = True  # flat face toward the slit
    field_gap: float = 0.5  # slit to the field lens's first face
    field_clear_r: float = 5.0  # hole in the slit block behind the lens, radius
    stop_ahead: float = 0.0  # printed stop this far in front of the objective (0 = stop at the lens)
    stop_t: float = 0.0  # thickness of the plate the stop hole is cut in
    d_obj: float | None = None  # objective's own clear aperture (None = unlimited)
    filter_t: float = 0.0  # long-pass glass thickness (0 = none)
    filter_ahead: float = 0.0  # its front face, this far in front of the objective
    filter_n: float = 1.52  # GG-495: n_d 1.524, 1.516 at 852 nm
    mirror_ahead: float = 0.0  # scan mirror centre in front of the objective (0 = not modelled)
    mirror_w: float = 20.0  # square mirror's side, at 45 degrees
    refocus: bool = False  # collimator focused on the slit as seen through the field lens


def field_lens_image(c: Config, wl: float = CENTER_WL):
    """Where a real plano-convex field lens shows the slit to the collimator.

    Paraxial ray-transfer matrices from the slit through both faces of the lens.
    Returns (distance of the slit's virtual image behind the slit, its
    magnification). A thin lens right at the slit would give (0, 1); a thick lens
    whose principal plane sits millimetres behind the slit magnifies it."""
    n = float(np.ravel(Material("N-BK7").n(wl))[0])
    r1, r2 = (np.inf, -c.field_r) if c.field_flat_first else (c.field_r, np.inf)

    def face(r, n1, n2):
        return np.array([[1.0, 0.0], [-(n2 - n1) / r, 1.0]])

    def gap(d, n_):
        return np.array([[1.0, d / n_], [0.0, 1.0]])

    m = face(r2, n, 1.0) @ gap(c.field_ct, n) @ face(r1, 1.0, n) @ gap(c.field_gap, 1.0)
    s_back = -m[0, 1] / m[1, 1]  # image distance from the lens's back face (< 0: virtual)
    return c.field_gap + c.field_ct + s_back, 1.0 / m[1, 1]


def build(c: Config) -> Optic:
    if not isinstance(c, Bought):
        c = Bought(**asdict(c))
    period_um = 1000.0 / c.lines_per_mm
    theta = np.arcsin(CENTER_WL / period_um)  # 1st-order angle at 750 nm
    s_img = 1 / (1 / c.f_obj - 1 / c.scene_dist)  # objective -> slit distance

    o = Optic()
    o.add_surface(index=0, thickness=c.scene_dist, comment="scene")
    # What sits between the mirror and the objective, in the order the light meets it.
    front = []
    if c.mirror_ahead:
        # The fold mirror, unfolded: full width along the shaft (x), foreshortened
        # by cos 45 across it (y).
        hx, hy = c.mirror_w / 2, c.mirror_w / 2 * np.cos(np.pi / 4)
        front.append((-c.mirror_ahead, dict(aperture=RectangularAperture(-hx, hx, -hy, hy),
                                            comment="scan mirror")))
    if c.stop_ahead:
        # A printed stop: a hole in a plate, so both of its edges can clip a beam.
        r_stop = 0.5 * c.f_obj / c.fno_obj
        front.append((-c.stop_ahead, dict(is_stop=True, aperture=RadialAperture(r_stop), comment="stop")))
        if c.stop_t:
            front.append((-c.stop_ahead + c.stop_t, dict(aperture=RadialAperture(r_stop),
                                                         comment="stop, back of the hole")))
    if c.filter_t:
        if c.stop_ahead and c.stop_t and (min(c.stop_ahead, c.filter_ahead)
                                          > max(c.stop_ahead - c.stop_t, c.filter_ahead - c.filter_t)):
            raise ValueError(f"{c.name}: the stop plate overlaps the filter glass")
        front.append((-c.filter_ahead, dict(material=IdealMaterial(c.filter_n), comment="long-pass filter")))
        front.append((-c.filter_ahead + c.filter_t, dict(comment="long-pass filter, back")))
    i = 1
    for z, kw in sorted(front, key=lambda t: t[0]):
        o.add_surface(index=i, z=z, **kw); i += 1
    o.add_surface(index=i, surface_type="paraxial", f=c.f_obj, z=0.0,
                  is_stop=not c.stop_ahead, comment="objective",
                  **({"aperture": RadialAperture(c.d_obj / 2)} if c.d_obj else {})); i += 1
    o.add_surface(index=i, z=s_img, comment="slit"); i += 1
    z_focus = s_img  # what the collimator has to focus on
    if c.field_r:
        # The real lens: two spherical/flat faces of N-BK7, then the slit block's hole.
        z1 = s_img + c.field_gap
        r1, r2 = (np.inf, -c.field_r) if c.field_flat_first else (c.field_r, np.inf)
        o.add_surface(index=i, z=z1, radius=r1, material="N-BK7", comment="field lens"); i += 1
        o.add_surface(index=i, z=z1 + c.field_ct, radius=r2, aperture=RadialAperture(c.field_clear_r),
                      comment="field lens back"); i += 1
        if c.refocus:
            z_focus = s_img + field_lens_image(c)[0]
    elif c.f_field:
        # A thin field lens right behind the slit. With f_field = objective->slit
        # distance, chief rays leave the slit parallel to the axis, so after the
        # collimator every field point's bundle crosses at the collimator's back
        # focal plane: put the grating + camera there and nothing walks off.
        o.add_surface(index=i, surface_type="paraxial", f=c.f_field,
                      z=s_img + 0.5, comment="field lens")
        i += 1
    z_coll = z_focus + c.f_coll  # slit at the collimator's front focal plane
    o.add_surface(index=i, surface_type="paraxial", f=c.f_coll, z=z_coll,
                  aperture=RadialAperture(c.d_coll / 2),
                  comment="collimator"); i += 1
    z_g = z_coll + c.coll_to_grating
    o.add_surface(index=i, surface_type="grating", grating_order=1,
                  grating_period=period_um, z=z_g, comment="grating"); i += 1
    L = c.grating_to_cam
    o.add_surface(index=i, surface_type="paraxial", f=c.f_cam,
                  y=L * np.sin(theta), z=z_g + L * np.cos(theta), rx=-theta,
                  aperture=RadialAperture(c.d_cam / 2), comment="camera lens"); i += 1
    o.add_surface(index=i, y=(L + c.f_cam) * np.sin(theta),
                  z=z_g + (L + c.f_cam) * np.cos(theta), rx=-theta,
                  comment="sensor")

    o.set_aperture("EPD", c.f_obj / c.fno_obj)
    o.set_field_type("object_height")
    half_line = 0.5 * c.slit_len * c.scene_dist / s_img  # scene half-length
    o.add_field(y=0.0)
    o.add_field(y=0.0, x=0.5 * half_line)
    o.add_field(y=0.0, x=half_line)
    for w in WAVELENGTHS:
        o.add_wavelength(w, is_primary=(w == CENTER_WL))
    o._theta, o._half_line, o._s_img = theta, half_line, s_img
    o._z_coll = z_coll
    return o


def sensor_xy(o, Hx, wl, Px=0.0, Py=0.0):
    """Trace one ray, return (x, y) in the sensor's own frame (mm)."""
    o.trace_generic(Hx, 0.0, Px, Py, wl)
    sg = o.surface_group
    x = float(np.ravel(sg.x[-1])[0])
    y = float(np.ravel(sg.y[-1])[0])
    z = float(np.ravel(sg.z[-1])[0])
    s = sg.surfaces[-1].geometry.cs
    th = o._theta
    # local "up" axis of a plane tilted by rx=-theta about x
    v = (y - float(s.y)) * np.cos(th) - (z - float(s.z)) * np.sin(th)
    return x, v


def transmission(o, Hx, wl, n=40):
    """Fraction of the objective's light (uniform pupil grid) reaching the sensor."""
    rays = o.trace(Hx, 0.0, wl, num_rays=n, distribution="uniform")
    return float(np.sum(np.asarray(rays.i) > 0)) / np.asarray(rays.i).size


def through_slit(o, Hx, wl, n=40):
    """Fraction of the objective's light that gets through the slit (same grid)."""
    o.trace(Hx, 0.0, wl, num_rays=n, distribution="uniform")
    sg = o.surface_group
    k = [s.comment for s in sg.surfaces].index("slit")
    return float(np.mean(np.asarray(sg.intensity)[k] > 0))


def analyse(c: Config):
    o = build(c)
    # chief-ray positions on the sensor
    pos = {(h, w): sensor_xy(o, h, w) for h in (0.0, 1.0) for w in WAVELENGTHS}
    v_lo, v_hi = pos[(0.0, 0.50)][1], pos[(0.0, 1.00)][1]
    disp_px_per_nm = abs(v_hi - v_lo) / PIXEL_MM / 500.0
    # local dispersion at 750 nm
    dv = abs(sensor_xy(o, 0, 0.76)[1] - sensor_xy(o, 0, 0.74)[1]) / 20.0
    slit_img = c.slit_width * c.f_cam / c.f_coll  # paraxial slit image width
    res_nm = slit_img / dv  # geometric spectral resolution (FWHM-ish)
    line_px = 2 * abs(pos[(1.0, CENTER_WL)][0]) / PIXEL_MM
    smile_px = max(abs(pos[(1.0, w)][1] - pos[(0.0, w)][1]) for w in WAVELENGTHS) / PIXEL_MM
    keystone_px = (max(abs(pos[(1.0, w)][0]) for w in WAVELENGTHS)
                   - min(abs(pos[(1.0, w)][0]) for w in WAVELENGTHS)) / PIXEL_MM
    trans = {(h, w): transmission(o, h, w) for h in (0.0, 1.0) for w in (0.5, 0.75, 1.0)}
    return o, dict(
        spectrum_mm=abs(v_hi - v_lo), disp_px_nm=disp_px_per_nm,
        slit_img_um=slit_img * 1e3, res_nm=res_nm, line_px=line_px,
        line_mm_scene=2 * o._half_line, smile_px=smile_px, keystone_px=keystone_px,
        trans=trans,
    )


def _sensor_frame(o, x, y, z):
    s = o.surface_group.surfaces[-1].geometry.cs
    th = o._theta
    return x, (y - float(s.y)) * np.cos(th) - (z - float(s.z)) * np.sin(th)


def traced(c: Config, o=None):
    """First-order numbers traced through the whole train rather than estimated.

    The slit image width comes from chief rays through the two slit edges, so it
    includes the grating's anamorphic stretch (1 / cos 22 deg = 1.08, since the
    beam leaves the grating at an angle) and any field-lens magnification, which
    analyse()'s paraxial estimate f_cam / f_coll leaves out. When the line is
    longer than the sensor, "end" means the last point of it on the sensor."""
    o = o or build(c)
    hy = c.slit_width / c.slit_len  # a slit edge, in units of the scene half-line

    def chief(hx, hy_, wl):
        o.trace_generic(hx, hy_, 0.0, 0.0, wl)
        sg = o.surface_group
        return _sensor_frame(o, *(float(np.ravel(a[-1])[0]) for a in (sg.x, sg.y, sg.z)))

    slit_mm = abs(chief(0.0, hy, CENTER_WL)[1] - chief(0.0, -hy, CENTER_WL)[1])
    dv = abs(chief(0.0, 0.0, 0.76)[1] - chief(0.0, 0.0, 0.74)[1]) / 20.0
    line_mm = abs(chief(1.0, 0.0, CENTER_WL)[0] - chief(-1.0, 0.0, CENTER_WL)[0])

    # The line end: the slit's end, or the last point of the line that still
    # lands on the sensor at 500, 750 and 1000 nm when the line overfills it.
    def off(h):
        return max(abs(chief(h, 0.0, w)[0]) for w in (0.5, 0.75, 1.0)) - SENSOR_MM[1] / 2
    h_end = 1.0
    if off(1.0) > 0:
        lo, hi = 0.0, 1.0
        for _ in range(40):
            lo, hi = ((0.5 * (lo + hi), hi) if off(0.5 * (lo + hi)) <= 0 else (lo, 0.5 * (lo + hi)))
        h_end = lo
    pos = {(h, w): chief(h * h_end, 0.0, w) for h in (0.0, 1.0) for w in WAVELENGTHS}
    smile_px = max(abs(pos[(1.0, w)][1] - pos[(0.0, w)][1]) for w in WAVELENGTHS) / PIXEL_MM
    keystone_px = (max(abs(pos[(1.0, w)][0]) for w in WAVELENGTHS)
                   - min(abs(pos[(1.0, w)][0]) for w in WAVELENGTHS)) / PIXEL_MM

    def wl_at(v):  # wavelength (um) whose chief ray lands at v on the sensor
        lo, hi = 0.40, 1.10
        sign = np.sign(chief(0.0, 0.0, lo)[1] - v)
        if np.sign(chief(0.0, 0.0, hi)[1] - v) == sign:
            return float("nan")  # the sensor edge is outside 400-1100 nm
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            if np.sign(chief(0.0, 0.0, mid)[1] - v) == sign:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    band = sorted(1000 * wl_at(v) for v in (-SENSOR_MM[0] / 2, SENSOR_MM[0] / 2))
    spots = {}
    for h in (0.0, 1.0):
        for w in (0.5, 0.75, 1.0):
            rays = o.trace(h * h_end, 0.0, w, num_rays=20, distribution="uniform")
            sg = o.surface_group
            ok = np.asarray(rays.i) > 0
            x, v = _sensor_frame(o, *(np.asarray(a[-1], float)[ok] for a in (sg.x, sg.y, sg.z)))
            spots[(h, w)] = (np.sqrt(np.mean((x - x.mean()) ** 2)) * 1e3,
                             np.sqrt(np.mean((v - v.mean()) ** 2)) * 1e3)
    return dict(slit_img_um=slit_mm * 1e3, res_nm=slit_mm / dv, line_mm=line_mm,
                band_nm=band, spots=spots, h_end=h_end, smile_px=smile_px, keystone_px=keystone_px,
                trans={(h, w): transmission(o, h * h_end, w)
                       for h in (0.0, 0.5, 1.0) for w in (0.5, 0.75, 1.0)})


CONFIGS = [
    Config("A: stock Pi NoIR v2 lens (3.04 mm f/2)", f_cam=3.04, d_cam=1.52),
    Config("B: M12-mount IMX219 + 12 mm f/2 lens", f_cam=12.0, d_cam=6.0),
    Config("C: as B + 18 mm field lens at slit", f_cam=12.0, d_cam=6.0,
           f_field=17.9, coll_to_grating=22.0, grating_to_cam=3.0),
    # The parts Ryan bought, at the spacings cad/ builds (cad/build_report.json):
    # Commonlands CIL161 objective (15.6 mm EFL) behind a 4 mm printed stop and the
    # 3 mm GG-495 in the filter cap, the 20 x 20 mm mirror 21 mm ahead, Edmund
    # #49-840 field lens (N-BK7, R 7.75, CT 5.25) behind 0.38 mm blades, LN016
    # collimator (12.5 mm aperture) focused on the slit, CIL122 camera lens.
    # Stop and lens positions are cad/ estimates until the lenses are measured.
    Bought("D: bought parts, as cad/ first drew them", f_cam=12.0, d_cam=6.0, f_field=15.0,
           coll_to_grating=6.0, grating_to_cam=16.03, f_obj=15.6, fno_obj=3.9, d_coll=12.5,
           field_r=7.75, field_ct=5.25, field_flat_first=True, field_gap=0.43, refocus=True,
           stop_ahead=11.7, stop_t=1.2, d_obj=7.8, filter_t=3.0, filter_ahead=10.5,
           mirror_ahead=21.0, mirror_w=20.0),
    # D with two changes: the field lens turned round (convex face toward the
    # slit), and the stop moved from the front of the cap to a 0.6 mm washer on the
    # lens's front face, with the filter in front of it. cad/ builds this one:
    # cad/rig/optics_link.py reads its stations from here by the tag "E".
    Bought("E: D with the field lens flipped and the stop on the lens", f_cam=12.0, d_cam=6.0,
           f_field=15.0, coll_to_grating=6.0, grating_to_cam=16.03, f_obj=15.6, fno_obj=3.9,
           d_coll=12.5, field_r=7.75, field_ct=5.25, field_flat_first=False, field_gap=0.43,
           refocus=True, stop_ahead=8.1, stop_t=0.6, d_obj=7.8, filter_t=3.0, filter_ahead=11.1,
           mirror_ahead=21.0, mirror_w=20.0),
]


OUT_DIR = Path(__file__).resolve().parent / "model_output"


def _pct(t):
    return "  ".join(f"{('center', 'mid', 'end')[int(2 * h)]}@{int(w * 1000)}nm={v:4.0%}"
                     for (h, w), v in t.items())


def main():
    OUT_DIR.mkdir(exist_ok=True)
    lines = []

    def out(msg=""):
        print(msg)
        lines.append(msg)

    for c in CONFIGS:
        o, r = analyse(c)
        t = r["trans"]
        real = isinstance(c, Bought)
        if real:  # the real field lens magnifies the slit: trace its width
            x = traced(c, o)
            r.update(slit_img_um=x["slit_img_um"], res_nm=x["res_nm"],
                     smile_px=x["smile_px"], keystone_px=x["keystone_px"])
        out(f"\n== {c.name}")
        out(f"  scene line length       : {r['line_mm_scene']:.1f} mm at {c.scene_dist:.0f} mm")
        out(f"  spectrum 500-1000 nm    : {r['spectrum_mm']:.2f} mm on sensor "
              f"(sensor is {SENSOR_MM[0]} x {SENSOR_MM[1]} mm)")
        out(f"  dispersion              : {r['disp_px_nm']:.2f} px/nm")
        out(f"  slit image width        : {r['slit_img_um']:.1f} um "
              f"({r['slit_img_um'] / 1e3 / PIXEL_MM:.1f} px)" + (", traced" if real else ""))
        out(f"  spectral resolution     : {r['res_nm']:.1f} nm (geometric, ideal lenses"
            + (", traced)" if real else ")"))
        out(f"  pixels along the line   : {r['line_px']:.0f}"
            + (f" (the sensor has {SENSOR_MM[1] / PIXEL_MM:.0f})" if real else ""))
        edge = ", at the last point on the sensor" if real and x["h_end"] < 1 else ""
        out(f"  smile at line end       : {r['smile_px']:.1f} px{edge}")
        out(f"  keystone                : {r['keystone_px']:.1f} px{edge}")
        if not real:
            out("  light reaching sensor   :  " + "  ".join(
                f"{'center' if h == 0 else 'edge'}@{int(w*1000)}nm={v:4.0%}" for (h, w), v in t.items()))
        else:
            gap, mag = field_lens_image(c)
            on = min(1.0, SENSOR_MM[1] / x["line_mm"])
            out(f"  field lens              : the slit looks {mag:.2f}x as big, {gap:.2f} mm further back; "
                f"collimator {o._z_coll - o._s_img:.2f} mm behind the slit")
            out(f"  line on the sensor      : {x['line_mm']:.2f} mm, {on:.0%} of it on the sensor "
                f"({on * r['line_mm_scene']:.1f} mm of the scene)")
            if x["h_end"] < 1:
                out(f"  line end                : the rest falls off the sensor, so 'end' here and "
                    f"below is {x['h_end']:.0%} of the way to the slit's end")
            out(f"  band on the sensor      : {x['band_nm'][0]:.0f}-{x['band_nm'][1]:.0f} nm "
                f"at the line centre")
            out("  light reaching sensor   :  " + _pct(x["trans"]))
            out("  rms spot, along/across  :  " + "  ".join(
                f"{'center' if h == 0 else 'end'}@{int(w * 1000)}nm={a:.1f}/{b:.1f} um"
                for (h, w), (a, b) in x["spots"].items()))
            out("  GG-495 transmits        :  " + "  ".join(
                f"{nm}nm={GG495_PD * GG495_TAU[nm]:3.0%}" for nm in (490, 500, 520, 750, 1000)))
        tag = c.name.split(":")[0]
        fig, ax = o.draw(fields="all", wavelengths=[0.5, 0.75, 1.0], num_rays=5,
                         projection="YZ", figsize=(12, 4),
                         title=f"{c.name} - dispersion plane (YZ)")
        fig.savefig(OUT_DIR / f"layout_{tag}_YZ.png", dpi=130, bbox_inches="tight")
        fig, ax = o.draw(fields="all", wavelengths=[0.75], num_rays=5,
                         projection="XZ", figsize=(12, 4),
                         title=f"{c.name} - slit plane (XZ)")
        fig.savefig(OUT_DIR / f"layout_{tag}_XZ.png", dpi=130, bbox_inches="tight")
        plt.close("all")

    # The same traced numbers for C, D and E side by side. Traced, C's slit image
    # is 1.11x its paraxial width above: 1.08x from the grating, 1.03x from the
    # thin field lens sitting 0.5 mm behind the slit.
    tagged = {c.name.split(":")[0]: c for c in CONFIGS}
    rows = {k: traced(tagged[k]) for k in ("C", "D", "E")}
    out("\n== C, D and E traced the same way")
    out(f"  {'':24s}" + "".join(f"{k:>16s}" for k in rows))
    for label, fmt in (
        ("slit image width", lambda x: f"{x['slit_img_um']:.1f} um"),
        ("spectral resolution", lambda x: f"{x['res_nm']:.1f} nm"),
        ("line on the sensor", lambda x: f"{x['line_mm']:.2f} mm"),
        ("line end on the sensor", lambda x: f"{x['h_end']:.0%} of the way"),
        ("smile / keystone there", lambda x: f"{x['smile_px']:.0f} / {x['keystone_px']:.0f} px"),
        ("light at line end 750", lambda x: f"{x['trans'][(1.0, 0.75)]:.0%}"),
        ("light at line end 500", lambda x: f"{x['trans'][(1.0, 0.5)]:.0%}"),
    ):
        out(f"  {label:24s}" + "".join(f"{fmt(x):>16s}" for x in rows.values()))

    # One change at a time around E. Signal is the light reaching the sensor at
    # 750 nm, counting what a bigger stop lets in (f/2 admits 3.8x f/3.9);
    # "lost" is light that gets through the slit but misses the sensor, so it
    # ends up on the housing walls as stray light. Both relative to E's signal at
    # the line centre; "end" is the last point of the line on the sensor.
    E = tagged["E"]
    ref = transmission(build(E), 0.0, 0.75)
    out("\n== E with one thing changed (750 nm, line centre / line end, relative to E's centre)")
    for label, c in (
        ("E as it is", E),
        ("field lens flat side to slit", replace(E, field_flat_first=True)),
        ("stop at the cap front, as D", replace(E, stop_ahead=11.7, stop_t=1.2, filter_ahead=10.5)),
        ("stop inside the lens", replace(E, stop_ahead=0.0, stop_t=0.0)),
        ("5 mm stop on the lens", replace(E, fno_obj=3.12)),
        ("no printed stop, f/2", replace(E, stop_ahead=0.0, stop_t=0.0, fno_obj=2.0)),
        ("40 um slit", replace(E, slit_width=0.040)),
    ):
        o = build(c)
        x = traced(c, o)
        t = x["trans"]
        gain = (E.fno_obj / c.fno_obj) ** 2 * c.slit_width / E.slit_width / ref
        slit = {h: through_slit(o, h * x["h_end"], 0.75) for h in (0.0, 1.0)}
        out(f"  {label:30s}: {x['res_nm']:.1f} nm, line {x['line_mm']:.2f} mm, signal "
            + " / ".join(f"{gain * t[(h, 0.75)]:.2f}" for h in (0.0, 1.0))
            + ", lost " + " / ".join(f"{gain * (slit[h] - t[(h, 0.75)]):.2f}" for h in (0.0, 1.0)))
    (OUT_DIR / "results.txt").write_text("\n".join(lines).lstrip("\n") + "\n")


if __name__ == "__main__":
    main()
