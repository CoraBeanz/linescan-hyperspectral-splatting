"""Command line: python -m hsical <command> --help

On the Jetson (Python 3.6 is fine):   capture, plan, focus, probe
On the PC (Python 3.10+, scipy):      inspect, calibrate, apply, synth, selftest

Each command imports only what it needs, so the capture commands work on a
Jetson that has nothing but numpy.
"""

import argparse
import os
import sys


def _camera(a):
    from .capture import V4L2_DEFAULTS, open_camera
    kw = {}
    for k in ("device", "width", "height"):
        if getattr(a, k, None) is not None:
            kw[k] = getattr(a, k)
    for item in getattr(a, "ctrl", None) or []:
        k, v = item.split("=", 1)
        default = V4L2_DEFAULTS.get(k)
        if isinstance(default, tuple):
            kw[k] = tuple(x for x in v.split(";") if x)
        elif isinstance(default, (int, float)):
            kw[k] = float(v)
        else:
            kw[k] = v
    return open_camera(a.backend, dry_run=a.dry_run, **kw)


def cmd_capture(a):
    from .capture import capture_set
    cam = _camera(a)
    capture_set(cam, a.session, a.name, a.kind, a.source, exposure_us=a.exposure_ms * 1000.0, gain=a.gain,
                frames=a.frames, auto=a.auto, note=a.note or "")


def cmd_plan(a):
    from .capture import run_plan
    run_plan(_camera(a), a.session, frames=a.frames, steps=a.steps.split(",") if a.steps else None)


def cmd_focus(a):
    from .capture import focus_loop
    folder = a.scratch
    if not os.path.isdir(folder):
        os.makedirs(folder)
    focus_loop(_camera(a), a.exposure_ms * 1000.0, a.gain, folder, a.count)


def cmd_probe(a):
    _camera(a).probe()


def cmd_inspect(a):
    from .session_check import inspect_session
    sys.exit(0 if inspect_session(a.session) else 1)


def cmd_calibrate(a):
    from .pipeline import calibrate
    r = calibrate(a.session, a.out, temp_k=a.temp_k, nm_step=a.nm_step, scale=a.binning,
                  flip_y=True if a.flip_y else None, transpose=a.transpose,
                  lamp_names=a.lamps.split(",") if a.lamps else None, report=not a.no_report)
    sys.exit(0 if r.ok else 2)


def cmd_apply(a):
    from .apply import apply_frames
    apply_frames(a.calibration, a.frames, a.dark, a.out, white=a.white, white_dark=a.white_dark,
                 white_reflectance=a.white_reflectance)


def cmd_synth(a):
    from .synth import Truth, write_session
    t = Truth(scale=a.binning, seed=a.seed, frames=a.frames, flip_x=a.flip_x, transpose=a.transpose)
    print(f"Rendering a synthetic session at binning {a.binning:g} into {a.out}")
    write_session(a.out, t)


def cmd_selftest(a):
    from .selftest import run
    sys.exit(0 if run(scale=a.binning, keep=a.keep, flip_x=a.flip_x, transpose=a.transpose) else 1)


def main(argv=None):
    p = argparse.ArgumentParser(prog="python -m hsical", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command")

    def camera_args(q):
        q.add_argument("--backend", default="auto", choices=["auto", "v4l2", "picamera2"])
        q.add_argument("--device", help="V4L2 device (default /dev/video0)")
        q.add_argument("--width", type=int, help="frame width (Jetson default 3264, Pi 3280)")
        q.add_argument("--height", type=int, help="frame height (default 2464)")
        q.add_argument("--ctrl", action="append", metavar="KEY=VALUE",
                       help="override a V4L2 setting, e.g. gain_scale=16 or exposure_ctrl=exposure_time_absolute")
        q.add_argument("--dry-run", action="store_true", help="print the capture commands, take nothing")

    q = sub.add_parser("capture", help="capture one frame set (Jetson or Pi)")
    q.add_argument("session")
    q.add_argument("name", help="folder name for the set, e.g. cfl, neon_long, dark_60000us")
    q.add_argument("--kind", required=True, choices=["dark", "flat", "wires", "lamp", "laser", "scene", "white"])
    q.add_argument("--source", help="cfl, neon, halogen+ptfe, laser, capped ...")
    q.add_argument("--exposure-ms", type=float, default=10.0)
    q.add_argument("--gain", type=float, default=1.0)
    q.add_argument("--frames", type=int, default=8)
    q.add_argument("--auto", type=float, help="first adjust the exposure to this peak level (0-1), e.g. 0.75")
    q.add_argument("--note")
    camera_args(q)
    q.set_defaults(func=cmd_capture)

    q = sub.add_parser("plan", help="walk through the whole first-light capture sequence")
    q.add_argument("session")
    q.add_argument("--frames", type=int, help="frames per set (default 16, 8 for the laser)")
    q.add_argument("--steps", help="only these steps, e.g. cfl,neon")
    camera_args(q)
    q.set_defaults(func=cmd_plan)

    q = sub.add_parser("focus", help="live width of the brightest lamp line, for focusing")
    q.add_argument("--exposure-ms", type=float, default=20.0)
    q.add_argument("--gain", type=float, default=1.0)
    q.add_argument("--count", type=int, default=0, help="stop after this many frames (0 = until Ctrl+C)")
    q.add_argument("--scratch", default="focus_tmp")
    camera_args(q)
    q.set_defaults(func=cmd_focus)

    q = sub.add_parser("probe", help="list the camera's formats and controls")
    camera_args(q)
    q.set_defaults(func=cmd_probe)

    q = sub.add_parser("inspect", help="check a session's frames before calibrating")
    q.add_argument("session")
    q.set_defaults(func=cmd_inspect)

    q = sub.add_parser("calibrate", help="calibrate from a session folder")
    q.add_argument("session")
    q.add_argument("-o", "--out", default="cal")
    q.add_argument("--temp-k", type=float, help="halogen filament temperature (default 2850 K)")
    q.add_argument("--nm-step", type=float, default=2.0, help="wavelength step of the rectified output")
    q.add_argument("--binning", type=float, help="sensor binning (worked out from the frame width)")
    q.add_argument("--flip-y", action="store_true", help="mirror the slit direction")
    q.add_argument("--transpose", action="store_true", default=None,
                   help="the spectrum runs along the camera's y axis (detected when not given)")
    q.add_argument("--lamps", help="only these lamp sets, e.g. cfl,neon")
    q.add_argument("--no-report", action="store_true")
    q.set_defaults(func=cmd_calibrate)

    q = sub.add_parser("apply", help="turn raw frames into rectified spectra")
    q.add_argument("calibration", help="folder with calibration.json and maps.npz")
    q.add_argument("frames", help="a frame set folder or a frame file")
    q.add_argument("--dark", help="dark set folder, frame file, or a number (the black level, 64)")
    q.add_argument("--white", help="white reference set (PTFE): gives reflectance instead of radiance")
    q.add_argument("--white-dark", help="dark for the white reference (default: --dark)")
    q.add_argument("--white-reflectance", type=float, default=0.98)
    q.add_argument("-o", "--out", default="spectra")
    q.set_defaults(func=cmd_apply)

    q = sub.add_parser("synth", help="render a synthetic session from the optical design")
    q.add_argument("out")
    q.add_argument("--binning", type=float, default=2.0, help="1 = full 3280 x 2464 (slow), 2, 4")
    q.add_argument("--frames", type=int, default=4)
    q.add_argument("--seed", type=int, default=7)
    q.add_argument("--flip-x", action="store_true")
    q.add_argument("--transpose", action="store_true")
    q.set_defaults(func=cmd_synth)

    q = sub.add_parser("selftest", help="render, calibrate and check against the known answer")
    q.add_argument("--binning", type=float, default=4.0)
    q.add_argument("--keep", help="keep the session and calibration in this folder")
    q.add_argument("--flip-x", action="store_true")
    q.add_argument("--transpose", action="store_true")
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
