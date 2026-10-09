"""Trace the optical train at the spacings the CAD actually builds.

Layout E in optics/spectrograph_model.py is the bought parts as the head
holds them: the real field lens, curved side toward the slit, the stop
washer on the objective's front face with the filter in front of it, and the
grating and camera lens where their barrels allow (6 mm and 16 mm). Its
lens and stop positions were cad/ estimates when it was written.
build_rig.py writes the as-built stations to cad/build_report.json; this
script feeds them back into the Optiland model and prints layout E next to
the as-built train, so a change to the CAD (or a measured lens) shows up here.

    python3 cad/optics_check.py      (needs optiland, like the model)
"""

import json
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "optics"))
import spectrograph_model as sm  # noqa: E402

OUT = ROOT / "cad" / "optics_as_built.txt"


def main():
    st = json.loads((ROOT / "cad" / "build_report.json").read_text())["stations_mm"]
    design = next(c for c in sm.CONFIGS if c.name.startswith("E:"))
    built = replace(design, coll_to_grating=round(st["z_film"] - st["z_coll"], 2),
                    grating_to_cam=round(st["g2c"], 2), stop_ahead=st["stop_ahead"],
                    filter_ahead=st["filter_ahead"])
    lines = []
    for name, c in (("layout E (design)", design), ("as built", built)):
        o, r = sm.analyse(c)
        x = sm.traced(c, o)     # through the real field lens and the tilted grating, as the model reports E
        coll = o._z_coll - o._s_img
        lines += [
            name,
            "  stop / filter front     : %.2f / %.2f mm ahead of the objective" % (c.stop_ahead, c.filter_ahead),
            "  collimator              : %.2f mm behind the slit" % coll + (
                " (the CAD: %.2f)" % (st["z_coll"] - st["z_slit"]) if c is built else ""),
            "  collimator -> grating   : %.2f mm" % c.coll_to_grating,
            "  grating -> camera lens  : %.2f mm" % c.grating_to_cam,
            "  slit image, resolution  : %.1f um, %.1f nm" % (x["slit_img_um"], x["res_nm"]),
            "  spectrum 500-1000 nm    : %.2f mm on sensor" % r["spectrum_mm"],
            "  line on the sensor      : %.2f mm" % x["line_mm"],
            "  smile / keystone at end : %.1f / %.1f px" % (x["smile_px"], x["keystone_px"]),
            "  light reaching sensor   : " + sm._pct(x["trans"]),
            "",
        ]
    # The paraxial camera lens has its 6 mm stop at its principal plane, which
    # for the estimated CIL122 barrel is 12 mm behind the front glass. A real
    # M12 lens usually has its entrance pupil a few mm behind the front glass,
    # so the true grating -> stop distance is likely shorter than modelled.
    lines.append("light at 500 / 750 / 1000 nm (centre field) vs grating -> camera stop distance")
    for d in (4, 8, 12, round(built.grating_to_cam)):
        _, r = sm.analyse(replace(built, grating_to_cam=d))
        t = r["trans"]
        lines.append("  %2d mm : %3.0f%% / %3.0f%% / %3.0f%%" % (
            d, 100 * t[(0.0, 0.5)], 100 * t[(0.0, 0.75)], 100 * t[(0.0, 1.0)]))
    text = "\n".join(lines) + "\n"
    print(text)
    OUT.write_text(text)


if __name__ == "__main__":
    main()
