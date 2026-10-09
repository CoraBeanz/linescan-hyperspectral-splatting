"""Export the spectrograph's slit-to-sensor map from the Optiland model.

Traces chief rays from points along the slit at many wavelengths through
optics/spectrograph_model.py and writes where each one lands on the IMX219,
in millimetres on the sensor, to calibration/hsical/data/optiland_map.json.
The calibration kit uses that table twice:

  * as the first guess of which column sees which wavelength, so line
    identification on a real lamp frame has somewhere to start, and
  * to render synthetic lamp, flat and dark frames (hsical synth), so the
    whole pipeline can be tested before the parts arrive.

Default layout is the as-built head with ideal thin lenses: config C's
train with the focal lengths and stop of the parts that were bought (the
15 mm Edmund field lens, the 15.6 mm CIL161 at f/3.9), at the spacings the
CAD actually builds (cad/build_report.json). Layout E, which cad/optics_check.py
traces, has the real field lens, and that also stretches the slit's image
unevenly along its length, by up to about 12 px; this map leaves it out.
--layout design uses config C from the model instead.

    python calibration/tools/export_optiland_map.py [--layout as-built|design]

Needs optiland (tested with 0.6.2), like the model itself.
"""

import argparse
import datetime
import json
import sys
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "optics"))
import spectrograph_model as sm  # noqa: E402

OUT = ROOT / "calibration" / "hsical" / "data" / "optiland_map.json"

# Slit position h: -1 and +1 are the slit ends. The grid runs a little past them so
# pixels just beyond the ends can be mapped back too (they get no light).
H_GRID = np.round(np.arange(-1.2, 1.2 + 1e-9, 0.05), 6)
H_MAX = 1.25  # widest field traced, in slit half-lengths
NM_GRID = np.arange(450.0, 1050.0 + 1, 10.0)  # wavelength, nm
H_VIG = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
NM_VIG = np.arange(450.0, 1050.0 + 1, 20.0)


def config_for(layout):
    design = next(c for c in sm.CONFIGS if c.name.startswith("C:"))
    if layout == "design":
        return design
    bought = next(c for c in sm.CONFIGS if c.name.startswith("E:"))
    st = json.loads((ROOT / "cad" / "build_report.json").read_text())["stations_mm"]
    return replace(design, name="as built, ideal lenses", f_field=bought.f_field, f_obj=bought.f_obj,
                   fno_obj=bought.fno_obj, coll_to_grating=round(st["z_film"] - st["z_coll"], 2),
                   grating_to_cam=round(st["g2c"], 2))


def chief_rays(o, hx, hy, nm):
    """Sensor-frame (u along the slit, v along the dispersion) of chief rays, mm.

    hx, hy are in units of the slit half-length (hx = +-1 at the slit ends)."""
    th = o._theta
    cs = o.surface_group.surfaces[-1].geometry.cs
    zeros = np.zeros_like(hx)
    # Optiland wants normalised fields inside (-1, 1); the extra field added in
    # main() makes 1 mean H_MAX slit half-lengths.
    o.trace_generic(hx / H_MAX, hy / H_MAX, zeros, zeros, nm / 1000.0)
    sg = o.surface_group
    x, y, z = (np.asarray(a[-1], float) for a in (sg.x, sg.y, sg.z))
    v = (y - float(cs.y)) * np.cos(th) - (z - float(cs.z)) * np.sin(th)
    return x, v


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--layout", choices=["as-built", "design"], default="as-built")
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()

    c = config_for(args.layout)
    o = sm.build(c)
    o.add_field(y=0.0, x=H_MAX * o._half_line)  # trace a little past the slit ends
    half_slit = 0.5 * c.slit_len
    # Field coordinates are normalised to the scene half-line, which the
    # objective images onto the slit end; so Hy = d / half_slit moves a point
    # d mm across the slit (the dispersion direction).
    dy = 0.5 * c.slit_width / half_slit

    u = np.zeros((H_GRID.size, NM_GRID.size))
    v = np.zeros_like(u)
    width = np.zeros_like(u)
    for j, nm in enumerate(NM_GRID):
        u[:, j], v[:, j] = chief_rays(o, H_GRID, np.zeros_like(H_GRID), nm)
        _, v_hi = chief_rays(o, H_GRID, np.full_like(H_GRID, dy), nm)
        _, v_lo = chief_rays(o, H_GRID, np.full_like(H_GRID, -dy), nm)
        width[:, j] = np.abs(v_hi - v_lo)

    vig = np.array([[sm.transmission(o, h / H_MAX, nm / 1000.0) for nm in NM_VIG]
                    for h in H_VIG])

    cfg = {k: val for k, val in asdict(c).items()}
    data = {
        "description": "Chief-ray map of the spectrograph from optics/spectrograph_model.py "
                       "(Optiland, ideal thin lenses). u_mm: sensor coordinate along the slit; "
                       "v_mm: sensor coordinate along the dispersion; both from the sensor "
                       "centre. slit_width_mm: width of the slit image along v. vignetting: "
                       "fraction of the objective's light that reaches the sensor.",
        "layout": args.layout,
        "config": cfg,
        "generated": datetime.date.today().isoformat(),
        "pixel_mm": sm.PIXEL_MM,
        "sensor_mm": list(sm.SENSOR_MM),
        "h": [round(float(h), 6) for h in H_GRID],
        "nm": [float(n) for n in NM_GRID],
        "u_mm": np.round(u, 6).tolist(),
        "v_mm": np.round(v, 6).tolist(),
        "slit_width_mm": np.round(width, 6).tolist(),
        "vignetting_h": H_VIG.tolist(),
        "vignetting_nm": NM_VIG.tolist(),
        "vignetting": np.round(vig, 4).tolist(),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(data, indent=1) + "\n")

    px = sm.PIXEL_MM
    j750 = int(np.argmin(np.abs(NM_GRID - 750)))
    i0, i1, i_lo = H_GRID.size // 2, int(np.argmin(np.abs(H_GRID - 1))), int(np.argmin(np.abs(H_GRID + 1)))
    smile = np.abs(v[i1] - v[i0]).max() / px
    print(f"{c.name}: wrote {args.out}")
    print(f"  v(500..1000 nm) at slit centre: {v[i0, np.searchsorted(NM_GRID, 500)]:.3f} .. "
          f"{v[i0, np.searchsorted(NM_GRID, 1000)]:.3f} mm")
    print(f"  line length at 750 nm: {abs(u[i1, j750] - u[i_lo, j750]) / px:.0f} px")
    print(f"  smile at the slit end: {smile:.1f} px")
    print(f"  slit image width: {width[i0].min() * 1000:.1f} .. {width[i0].max() * 1000:.1f} um")


if __name__ == "__main__":
    main()
