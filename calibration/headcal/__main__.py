"""Command line: python -m headcal <command> --help   (run it from calibration/)

On the Jetson (Python 3.6 is fine):   record
On the PC (Python 3.10+):             board, plan, solve, synth, selftest

  board     print the tag board (SVG at true size) and write its board.json
  plan      the calibration scan's viewpoints, as a scan_sweep plan
  record    keep pose-camera stills while the calibration scan runs
  solve     calibrate from the scan folder and the stills
  synth     a synthetic calibration scan with a known answer
  selftest  synth, solve, and compare with the answer

Each command imports only what it needs, so `record` works on a Jetson with nothing but numpy.
"""

import argparse
import os
import sys


def cmd_board(a):
    from .board import Board
    if a.from_json:
        b = Board.load(a.from_json)
    else:
        b = Board(a.squares[0], a.squares[1], a.square_mm, a.marker_mm, a.dictionary)
    if a.span_mm:
        b = b.scaled(a.span_mm)
        print("board as printed: %.4f mm squares, %.4f mm tags" % (b.square_mm, b.marker_mm))
    if not os.path.isdir(a.out):
        os.makedirs(a.out)
    b.save(os.path.join(a.out, "board.json"))
    if not a.span_mm:
        with open(os.path.join(a.out, "board.svg"), "w") as f:
            f.write(b.svg(a.paper))
    names = ["board.json"] + ([] if a.span_mm else ["board.svg"])
    print("wrote %s" % ", ".join(os.path.join(a.out, n) for n in names))


def cmd_plan(a):
    from . import plan, repo
    robot = repo.robot(repo.build_urdf())
    p = plan.make_plan(robot, centre=tuple(a.centre), log=lambda s: print(s, file=sys.stderr))
    text = plan.plan_yaml(p, tuple(a.centre))
    if a.out:
        with open(a.out, "w") as f:
            f.write(text)
        print("wrote %d viewpoints to %s" % (len(p["viewpoints"]), a.out), file=sys.stderr)
    else:
        print(text)


def cmd_record(a):
    from hsical.capture import V4L2_DEFAULTS, open_camera
    from .record import focus, record
    kw = dict(device=a.device)
    for k in ("width", "height"):
        if getattr(a, k):
            kw[k] = getattr(a, k)
    for item in a.ctrl or []:
        k, v = item.split("=", 1)
        default = V4L2_DEFAULTS.get(k)
        if isinstance(default, tuple):
            kw[k] = tuple(x for x in v.split(";") if x)
        elif isinstance(default, (int, float)):
            kw[k] = float(v)
        else:
            kw[k] = v
    cam = open_camera("v4l2", dry_run=a.dry_run, **kw)
    if a.focus:
        focus(cam, a.out, exposure_us=a.exposure_ms * 1000.0, gain=a.gain, count=a.count)
        return
    record(cam, a.out, exposure_us=a.exposure_ms * 1000.0, gain=a.gain, auto=a.auto, period=a.period,
           count=a.count, keep_all=a.all, max_grabs=a.max_grabs, every=a.every)


def cmd_solve(a):
    from .solve import run
    nm = tuple(a.nm) if a.nm else None
    offsets = [j.strip() for j in a.joint_offsets.split(",") if j.strip()] if a.joint_offsets else ()
    res = run(a.scan, a.pose, a.board, a.out, nm=nm, joint_offsets=offsets)
    for w in res.warnings:
        print("! " + w)
    print("see %s" % os.path.join(a.out, "report.md"))


def cmd_synth(a):
    from . import repo, synth
    from .model import HeadGeometry
    urdf = repo.build_urdf()
    truth = synth.make_truth(HeadGeometry.from_urdf(repo.robot(urdf)), seed=a.seed, errors=a.errors,
                             slit_reversed=a.reverse_slit)
    plan = synth.load_plan()
    if a.views:
        plan = dict(plan, viewpoints=plan["viewpoints"][:a.views])
    print("Rendering a synthetic calibration scan into %s" % a.out)
    synth.write_scan(a.out, truth, plan=plan, nominal_urdf=urdf)


def cmd_selftest(a):
    from .selftest import run
    sys.exit(0 if run(keep=a.keep, views=a.views, seed=a.seed) else 1)


def main(argv=None):
    p = argparse.ArgumentParser(prog="python -m headcal", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command")

    q = sub.add_parser("board", help="print the tag board and write its board.json")
    q.add_argument("out", help="folder for board.svg and board.json")
    q.add_argument("--paper", default="letter", choices=["letter", "a4"])
    q.add_argument("--squares", type=int, nargs=2, default=[24, 17], metavar=("X", "Y"))
    q.add_argument("--square-mm", type=float, default=10.0)
    q.add_argument("--marker-mm", type=float, default=7.0)
    q.add_argument("--dictionary", default="DICT_4X4_250")
    q.add_argument("--from-json", help="start from this board.json instead")
    q.add_argument("--span-mm", type=float,
                   help="what 20 squares measure on your print: rewrites board.json to the printed size")
    q.set_defaults(func=cmd_board)

    q = sub.add_parser("plan", help="write the calibration scan plan (needs xacro)")
    q.add_argument("--centre", type=float, nargs=2, default=[0.26, 0.0], metavar=("X", "Y"),
                   help="where the board's centre lies, base_link metres")
    q.add_argument("--out", help="plan file (default: print it)")
    q.set_defaults(func=cmd_plan)

    q = sub.add_parser("record", help="keep pose-camera stills during the calibration scan (Jetson)")
    q.add_argument("out")
    q.add_argument("--device", default="/dev/video1", help="the pose camera (default /dev/video1)")
    q.add_argument("--width", type=int, help="sensor mode width (default 3264)")
    q.add_argument("--height", type=int, help="sensor mode height (default 2464)")
    q.add_argument("--exposure-ms", type=float, default=20.0)
    q.add_argument("--gain", type=float, default=1.0)
    q.add_argument("--auto", type=float, help="first adjust the exposure to this peak level (0-1), e.g. 0.7")
    q.add_argument("--period", type=float, default=1.0, help="seconds between grabs")
    q.add_argument("--every", type=float, default=3.0, help="while a view holds, another still this often (s)")
    q.add_argument("--count", type=int, default=0, help="stop after this many stills, or readings with --focus "
                                                        "(0: Ctrl+C)")
    q.add_argument("--max-grabs", type=int, default=0, help="stop after this many grabs (0: Ctrl+C)")
    q.add_argument("--all", action="store_true", help="keep every grab, not only still views")
    q.add_argument("--focus", action="store_true",
                   help="only print the view's sharpness over and over, to focus the lens by (keeps nothing)")
    q.add_argument("--ctrl", action="append", metavar="KEY=VALUE", help="override a V4L2 setting (as hsical)")
    q.add_argument("--dry-run", action="store_true", help="print the capture command, take nothing")
    q.set_defaults(func=cmd_record)

    q = sub.add_parser("solve", help="calibrate the head from a calibration scan")
    q.add_argument("scan", help="the scan folder scan_sweep and line_camera wrote")
    q.add_argument("pose", help="the folder `record` wrote")
    q.add_argument("-o", "--out", default="headcal_out")
    q.add_argument("--board", help="board.json (default: the default board, unscaled)")
    q.add_argument("--nm", type=float, nargs=2, metavar=("LO", "HI"),
                   help="wavelengths to add up into the sweeps' brightness (default: all)")
    q.add_argument("--joint-offsets", help="also fit constant offsets of these arm joints, as a check "
                                           "(e.g. shoulder_lift,elbow_flex,wrist_flex)")
    q.set_defaults(func=cmd_solve)

    q = sub.add_parser("synth", help="render a synthetic calibration scan (needs xacro)")
    q.add_argument("out")
    q.add_argument("--views", type=int, help="only the plan's first N viewpoints")
    q.add_argument("--seed", type=int, default=7)
    q.add_argument("--errors", type=float, default=1.0, help="how far off the CAD the head is, x typical")
    q.add_argument("--reverse-slit", action="store_true")
    q.set_defaults(func=cmd_synth)

    q = sub.add_parser("selftest", help="render, calibrate and compare with the known answer")
    q.add_argument("--keep", help="keep the scan and the results in this folder")
    q.add_argument("--views", type=int, help="only the plan's first N viewpoints")
    q.add_argument("--seed", type=int, default=7)
    q.set_defaults(func=cmd_selftest)

    a = p.parse_args(argv)
    if not getattr(a, "func", None):
        p.print_help()
        return 1
    try:
        a.func(a)
    except (RuntimeError, ValueError, FileNotFoundError) as e:
        print("error: %s" % e, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
