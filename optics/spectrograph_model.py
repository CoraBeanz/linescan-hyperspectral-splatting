"""
Line-scan (pushbroom) spectrograph: Optiland model of the full optical train.

    scene --(150 mm)--> objective (16 mm, f/4, aperture stop)
          --> slit (50 um x 5 mm, slit long axis = x)
          --> [optional field lens right behind the slit]
          --> collimator (25 mm M12 lens used backwards)
          --> 500 l/mm transmission grating (grooves along x, disperses in y)
          --> camera lens tilted to the 1st order at 750 nm
          --> IMX219 sensor (1.12 um pixels)

Every lens is a *paraxial* (ideal thin) lens. M12 board lenses ship without a
published prescription, so this model answers first-order questions honestly
(dispersion, slit image size, line length, smile, keystone, vignetting) but
says nothing about the real lenses' aberrations. Swap in a catalog lens
(e.g. an Edmund/Thorlabs achromat from Optiland's database) when you have one.

Conventions: z is the optical axis, y is the dispersion direction, x is along
the slit (the spatial / scan-line direction). Units are mm; wavelengths in um.

Run:  python3 spectrograph_model.py   (from any directory; prints the tables and
      writes results.txt + layout PNGs to model_output/ next to this script)
Needs: pip install optiland  (tested with 0.6.x)
"""

import warnings
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from optiland.optic import Optic
from optiland.physical_apertures import RadialAperture

warnings.filterwarnings("ignore")

PIXEL_MM = 1.12e-3  # IMX219 pixel pitch
SENSOR_MM = (3.68, 2.76)  # IMX219 active area (w, h)
WAVELENGTHS = [0.50, 0.60, 0.70, 0.75, 0.80, 0.90, 1.00]  # um
CENTER_WL = 0.75


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


def build(c: Config) -> Optic:
    period_um = 1000.0 / c.lines_per_mm
    theta = np.arcsin(CENTER_WL / period_um)  # 1st-order angle at 750 nm
    s_img = 1 / (1 / c.f_obj - 1 / c.scene_dist)  # objective -> slit distance

    o = Optic()
    o.add_surface(index=0, thickness=c.scene_dist, comment="scene")
    o.add_surface(index=1, surface_type="paraxial", f=c.f_obj, z=0.0,
                  is_stop=True, comment="objective")
    o.add_surface(index=2, z=s_img, comment="slit")
    i = 3
    if c.f_field:
        # A thin field lens right behind the slit. With f_field = objective->slit
        # distance, chief rays leave the slit parallel to the axis, so after the
        # collimator every field point's bundle crosses at the collimator's back
        # focal plane: put the grating + camera there and nothing walks off.
        o.add_surface(index=i, surface_type="paraxial", f=c.f_field,
                      z=s_img + 0.5, comment="field lens")
        i += 1
    z_coll = s_img + c.f_coll  # slit at the collimator's front focal plane
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


CONFIGS = [
    Config("A: stock Pi NoIR v2 lens (3.04 mm f/2)", f_cam=3.04, d_cam=1.52),
    Config("B: M12-mount IMX219 + 12 mm f/2 lens", f_cam=12.0, d_cam=6.0),
    Config("C: as B + 18 mm field lens at slit", f_cam=12.0, d_cam=6.0,
           f_field=17.9, coll_to_grating=22.0, grating_to_cam=3.0),
]


OUT_DIR = Path(__file__).resolve().parent / "model_output"


def main():
    OUT_DIR.mkdir(exist_ok=True)
    lines = []

    def out(msg=""):
        print(msg)
        lines.append(msg)

    for c in CONFIGS:
        o, r = analyse(c)
        t = r["trans"]
        out(f"\n== {c.name}")
        out(f"  scene line length       : {r['line_mm_scene']:.1f} mm at {c.scene_dist:.0f} mm")
        out(f"  spectrum 500-1000 nm    : {r['spectrum_mm']:.2f} mm on sensor "
              f"(sensor is {SENSOR_MM[0]} x {SENSOR_MM[1]} mm)")
        out(f"  dispersion              : {r['disp_px_nm']:.2f} px/nm")
        out(f"  slit image width        : {r['slit_img_um']:.1f} um "
              f"({r['slit_img_um'] / 1e3 / PIXEL_MM:.1f} px)")
        out(f"  spectral resolution     : {r['res_nm']:.1f} nm (geometric, ideal lenses)")
        out(f"  pixels along the line   : {r['line_px']:.0f}")
        out(f"  smile at line end       : {r['smile_px']:.1f} px")
        out(f"  keystone                : {r['keystone_px']:.1f} px")
        out("  light reaching sensor   :  "
              + "  ".join(f"{'center' if h == 0 else 'edge'}@{int(w*1000)}nm={v:4.0%}"
                          for (h, w), v in t.items()))
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
    (OUT_DIR / "results.txt").write_text("\n".join(lines).lstrip("\n") + "\n")


if __name__ == "__main__":
    main()
