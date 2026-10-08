"""Command line: python -m hsisim <command> --help

  scan       render a scan session of a built-in scene, in the rig's on-disk format
  selftest   scan, calibrate with hsical, turn the scan into reflectance, compare with the truth
  evaluate   compare a calibrated session with its truth (after `python -m hsical calibrate`)
  preview    pictures of a session (needs matplotlib)
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import tempfile
import time
from pathlib import Path

from . import repo  # noqa: F401  (puts hsical and the ROS 2 packages' modules on the path)

# the quick self-test's scan: two views of the target, lines spread across it
QUICK = dict(plan="ring", views=["down", "tilt25_az90"], lines=12, steps_per_line=16)


def _settings(a):
    from .session import Settings
    kw = dict(scene=a.scene, lighting=a.lighting, binning=a.binning, seed=a.seed, errors=a.errors,
              exposure_us=a.exposure_ms * 1000.0, gain=a.gain, slices=a.slices, rays_per_point=a.rays,
              fps=a.fps, stamp_error_us=a.stamp_error_us, stamp_offset_us=a.stamp_offset_us,
              slit_reversed=a.slit_reversed)
    return Settings(**kw)


def _plan(a, quick=False):
    from .session import load_plan
    plan = a.plan or (QUICK["plan"] if quick else "ring")
    views = a.views.split(",") if a.views else (QUICK["views"] if quick and not a.plan else None)
    lines = a.lines if a.lines is not None else (QUICK["lines"] if quick and not a.plan else None)
    steps = a.steps_per_line if a.steps_per_line is not None else (
        QUICK["steps_per_line"] if quick and not a.plan else None)
    return load_plan(plan, lines=lines, steps_per_line=steps, start_angle_deg=a.start_angle_deg,
                     line_period_s=a.line_period_s, views=views)


def cmd_scan(a):
    from .session import simulate
    s = _settings(a)
    s.calibration = not a.no_calibration and not a.calibration_session
    simulate(a.out, _plan(a), s, calibration_session=a.calibration_session)


def _print_check(errs, log=print):
    from .evaluate import check
    rows = check(errs)
    for name, v, lim, ok in rows:
        log(f"  {name:34s} {v:9.4f}   limit {lim:g}   {'ok' if ok else 'FAIL'}")
    return all(ok for *_, ok in rows)


def _report(errs, log=print):
    log(f"  {errs['lines']} lines, {errs['cells_compared']} cells of a single material compared")
    for m, d in errs["per_material"].items():
        log(f"    {m:16s} {d['cells']:8d} cells   median error {d['median_error']:+.4f}")
    for b in errs.get("rare_earth_bands", []):
        log(f"    rare-earth band {b['nm']:5.0f} nm: measured {b['measured']:7.2f}, expected {b['expected']:7.2f}"
            f" ({b['error']:+.3f} nm)")


def cmd_evaluate(a):
    from .evaluate import evaluate
    errs = evaluate(a.session, a.calibration, spectra_dir=a.spectra)
    _report(errs)
    print("\nAgainst the truth:")
    good = _print_check(errs)
    if a.json:
        Path(a.json).write_text(json.dumps(errs, indent=1) + "\n")
    sys.exit(0 if good else 1)


def cmd_selftest(a):
    from hsical.pipeline import calibrate

    from .evaluate import evaluate
    from .session import simulate
    base = Path(a.keep) if a.keep else Path(tempfile.mkdtemp(prefix="hsisim_selftest_"))
    try:
        t0 = time.time()
        session, cal = base / "scan", base / "cal"
        print(f"Simulating a scan (binning {a.binning:g}) into {session} ...")
        simulate(session, _plan(a, quick=True), _settings(a), log=lambda m: print("  " + m))
        print("Calibrating with hsical ...")
        r = calibrate(session / "calibration", cal, log=lambda *m: None)
        print(f"  calibration checks {'passed' if r.ok else 'FAILED'}")
        print("Turning the scan into reflectance and comparing with the truth ...")
        errs = evaluate(session, cal, log=lambda m: print(m))
        _report(errs)
        print("\nAgainst the truth:")
        good = _print_check(errs) and r.ok
        (base / "selftest.json").write_text(json.dumps(errs, indent=1) + "\n")
        print(f"\nSelf-test {'passed' if good else 'FAILED'} ({time.time() - t0:.0f} s).")
        if a.keep:
            print(f"Session, calibration and spectra kept in {base}")
        sys.exit(0 if good else 1)
    finally:
        if not a.keep:
            shutil.rmtree(base, ignore_errors=True)


def cmd_preview(a):
    from .preview import preview
    preview(a.session, a.out)


def main(argv=None):
    p = argparse.ArgumentParser(prog="python -m hsisim", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command")

    def scan_args(q, quick=False):
        g = q.add_argument_group("the scan")
        g.add_argument("--plan", help="a plan file, or a plan in ros2/so101_scan_sweep/plans (ring, one_view)"
                                      + (": default, two views of ring with 12 lines each" if quick else
                                         " (default ring)"))
        g.add_argument("--views", help="only these viewpoints, by name, e.g. down,tilt25_az90")
        g.add_argument("--lines", type=int, help="lines per sweep (the plan's n_lines)")
        g.add_argument("--steps-per-line", type=int, help="mirror microsteps between lines (6400 a turn)")
        g.add_argument("--start-angle-deg", type=float, help="mirror angle of the first line")
        g.add_argument("--line-period-s", type=float, help="time per line")
        g = q.add_argument_group("the scene and the instrument")
        g.add_argument("--scene", default="relief", choices=["relief", "board", "white"])
        g.add_argument("--lighting", default="uniform", choices=["uniform", "lamp"])
        g.add_argument("--binning", type=float, default=4.0, help="1 = the full 3280 x 2464 sensor (slow), 2, 4")
        g.add_argument("--seed", type=int, default=7, help="the instrument's flaws, the pose errors and the noise")
        g.add_argument("--errors", type=float, default=1.0, help="scale of every pose error; 0 = perfect poses")
        g.add_argument("--exposure-ms", type=float, default=20.0)
        g.add_argument("--gain", type=float, default=1.0)
        g.add_argument("--slices", type=int, default=3, help="cuts across the slit's width")
        g.add_argument("--rays", type=int, default=12, help="rays through the objective per slit point")
        g.add_argument("--slit-reversed", action="store_true",
                       help="the camera mounted the other way along the slit (camera.json says so)")
        g = q.add_argument_group("the camera's clock")
        g.add_argument("--fps", type=float, help="frame rate (default 30, or 21 at --binning 1)")
        g.add_argument("--stamp-error-us", type=float, default=0.0,
                       help="how late the camera's frame stamps are against when row 0 was really read out")
        g.add_argument("--stamp-offset-us", type=float, default=0.0,
                       help="line_camera's stamp_offset_us, added to every stamp (-error cancels it)")

    q = sub.add_parser("scan", help="render a scan session")
    q.add_argument("out")
    scan_args(q)
    q.add_argument("--calibration-session", help="an hsical session of the same instrument (same --binning and "
                                                 "--seed) to point scan.json at, instead of rendering one")
    q.add_argument("--no-calibration", action="store_true", help="don't render the calibration session")
    q.set_defaults(func=cmd_scan)

    q = sub.add_parser("selftest", help="scan, calibrate, apply and compare with the truth")
    scan_args(q, quick=True)
    q.add_argument("--keep", help="keep the session, calibration and spectra in this folder")
    q.set_defaults(func=cmd_selftest)

    q = sub.add_parser("evaluate", help="compare a calibrated session with its truth")
    q.add_argument("session")
    q.add_argument("calibration", help="output folder of `python -m hsical calibrate <session>/calibration`")
    q.add_argument("--spectra", help="where to write the reflectance cubes (default <session>/spectra)")
    q.add_argument("--json", help="also write the errors to this file")
    q.set_defaults(func=cmd_evaluate)

    q = sub.add_parser("preview", help="pictures of a session")
    q.add_argument("session")
    q.add_argument("-o", "--out", help="default <session>/preview")
    q.set_defaults(func=cmd_preview)

    a = p.parse_args(argv)
    if not getattr(a, "func", None):
        p.print_help()
        return 1
    try:
        a.func(a)
    except (RuntimeError, ValueError, FileNotFoundError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
