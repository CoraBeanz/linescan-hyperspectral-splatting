"""Self-test: render a synthetic calibration scan with a known head, calibrate it, compare.

    python -m headcal selftest            # about a minute
    python -m headcal selftest --keep out # keep the scan, the stills and the results

The synthetic head is off its CAD numbers by about what a printed head is (a millimetre or so
and a degree or two per part, the mirror's home by a degree or two), the pose camera has a real
lens's distortion, the arm's joint readings carry 0.03 deg of noise, and both cameras see noise,
vignetting and defocus. Passing means the solver puts the line camera's rays within a few tenths
of a millimetre of the true ones on the board, which is well under one 0.3 mm scan line.
"""

import os
import shutil
import tempfile
import time

# value, limit: the synthetic run lands at about a third of each limit or better
LIMITS = [
    ("line_mm_median", 0.40, "line camera: rays vs the true ones on the board, median (mm)"),
    ("line_mm_max", 0.80, "line camera: the same, worst of all viewpoints, angles and slit ends (mm)"),
    ("pose_mm", 0.50, "pose camera on the wrist: position (mm)"),
    ("pose_deg", 0.15, "pose camera on the wrist: angle (deg)"),
    ("f_px", 0.50, "pose camera focal length (px)"),
    ("c_px", 1.00, "pose camera centre (px)"),
    ("k_rel", 0.003, "scan line length (relative)"),
]


def run(keep=None, views=None, seed=7, errors=1.0, log=print, **truth_kw):
    from . import repo, solve, synth
    from .model import HeadGeometry
    t0 = time.time()
    out = keep or tempfile.mkdtemp(prefix="headcal_selftest_")
    try:
        urdf = repo.build_urdf()
        robot = repo.robot(urdf)
        truth = synth.make_truth(HeadGeometry.from_urdf(robot), seed=seed, errors=errors, **truth_kw)
        plan = synth.load_plan()
        if views:
            vps = plan["viewpoints"]
            plan = dict(plan, viewpoints=vps[::max(1, len(vps) // views)][:views])
        log("rendering a synthetic calibration scan: %d viewpoints, seed %d" % (len(plan["viewpoints"]), seed))
        synth.write_scan(out, truth, plan=plan, nominal_urdf=urdf, log=lambda *a: None)
        log("calibrating (%.0f s so far)" % (time.time() - t0))
        res = solve.run(os.path.join(out, "scan"), os.path.join(out, "pose"), os.path.join(out, "board.json"),
                        os.path.join(out, "cal"), log=lambda m: log("  " + m))
        got = synth.compare(truth, res, plan=plan, robot=robot)
        ok = True
        log("")
        log("%-74s %9s %9s" % ("against the known answer", "error", "limit"))
        for key, limit, text in LIMITS:
            passed = got[key] <= limit
            ok &= passed
            log("%-74s %9.4f %9.4f  %s" % (text, got[key], limit, "ok" if passed else "FAIL"))
        passed = got["slit_reversed_ok"]
        ok &= passed
        log("%-74s %19s  %s" % ("slit order", "right" if passed else "wrong", "ok" if passed else "FAIL"))
        for w in res.warnings:
            log("! " + w)
        log("")
        log("%s in %.0f s" % ("PASS" if ok else "FAIL", time.time() - t0))
        if keep:
            log("results in %s (report: %s)" % (keep, os.path.join(keep, "cal", "report.md")))
        return ok
    finally:
        if not keep:
            shutil.rmtree(out, ignore_errors=True)
