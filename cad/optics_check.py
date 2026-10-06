"""Trace the optical train at the spacings the CAD actually builds.

Layout C in optics/spectrograph_model.py puts the grating 22 mm behind the
collimator and the camera lens 3 mm behind the grating. Real lens barrels
don't fit there: the collimator barrel ends 2 mm past its principal plane and
the camera lens barrel reaches 12 mm in front of its own, so the housing puts
the grating right after the collimator and the camera lens where its barrel
clears the grating. build_rig.py writes those as-built stations to
cad/build_report.json; this script feeds them back into the Optiland model
and prints layout C next to the as-built train.

    python3 cad/optics_check.py      (needs optiland, like the model)
"""

import json
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "optics"))
import spectrograph_model as sm  # noqa: E402

F_FIELD_BOUGHT = 15.0   # the 15 mm plano-convex lens on the parts list
OUT = ROOT / "cad" / "optics_as_built.txt"


def chief_ray_crossing(c):
    """Distance behind the collimator where the chief rays cross (the image
    of the objective's stop through the field lens and collimator), mm."""
    s_img = 1 / (1 / c.f_obj - 1 / c.scene_dist)
    gap = 0.5   # field lens sits 0.5 mm behind the slit, as in the model
    s = -(s_img + gap)
    v = 1 / (1 / c.f_field + 1 / s) if c.f_field else s
    s = v - (c.f_coll - gap)
    return 1 / (1 / c.f_coll + 1 / s)


def main():
    st = json.loads((ROOT / "cad" / "build_report.json").read_text())["stations_mm"]
    design = next(c for c in sm.CONFIGS if c.name.startswith("C:"))
    built = replace(design, coll_to_grating=round(st["z_film"] - st["z_coll"], 2),
                    grating_to_cam=round(st["g2c"], 2))
    rows = [
        ("layout C (design)", design),
        ("as built, 17.9 mm field lens", built),
        ("as built, 15 mm field lens", replace(built, f_field=F_FIELD_BOUGHT)),
    ]
    lines = []
    for name, c in rows:
        _, r = sm.analyse(c)
        t = r["trans"]
        lines += [
            name,
            "  collimator -> grating   : %.2f mm" % c.coll_to_grating,
            "  grating -> camera lens  : %.2f mm" % c.grating_to_cam,
            "  chief rays cross at     : %.1f mm behind the collimator" % chief_ray_crossing(c),
            "  spectrum 500-1000 nm    : %.2f mm on sensor" % r["spectrum_mm"],
            "  smile / keystone        : %.1f / %.1f px" % (r["smile_px"], r["keystone_px"]),
            "  light reaching sensor   : " + "  ".join(
                "%s@%dnm=%3.0f%%" % ("center" if h == 0 else "edge", int(w * 1000), 100 * v)
                for (h, w), v in t.items()),
            "",
        ]
    # The paraxial camera lens has its 6 mm stop at its principal plane, which
    # for the estimated CIL122 barrel is 12 mm behind the front glass. A real
    # M12 lens usually has its entrance pupil a few mm behind the front glass,
    # so the true grating -> stop distance is likely shorter than modelled.
    lines.append("light at 500 / 750 / 1000 nm (centre field) vs grating -> camera stop distance")
    for d in (4, 8, 12, round(built.grating_to_cam)):
        _, r = sm.analyse(replace(built, f_field=F_FIELD_BOUGHT, grating_to_cam=d))
        t = r["trans"]
        lines.append("  %2d mm : %3.0f%% / %3.0f%% / %3.0f%%" % (
            d, 100 * t[(0.0, 0.5)], 100 * t[(0.0, 0.75)], 100 * t[(0.0, 1.0)]))
    text = "\n".join(lines) + "\n"
    print(text)
    OUT.write_text(text)


if __name__ == "__main__":
    main()
