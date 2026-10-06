"""Capture raw calibration frames on the Jetson Nano (V4L2) or a Raspberry Pi (picamera2).

This module only needs numpy and runs on Python 3.6 (JetPack 4), so the
frames can be taken on the Jetson and copied to a PC for the calibration.

    python -m hsical capture session/ cfl --kind lamp --source cfl --exposure-ms 60
    python -m hsical plan session/          # the whole first-light sequence, step by step
    python -m hsical focus                   # live line width while you turn the lens

Raw frames matter: the camera's normal pipeline (nvarguscamerasrc, libcamera's
processed output) demosaics, white-balances and applies a tone curve, all of
which bend the numbers. V4L2 on the Jetson and picamera2's "raw" stream give
the sensor's own 10-bit values.
"""

import json
import os
import shutil
import subprocess
import sys
import time

import numpy as np

FULL = 1023
BLACK = 64.0

# Jetson IMX219 driver (JetPack 4.x): exposure in microseconds, gain in 1/16 steps,
# frame_rate in micro-frames-per-second. Override with --ctrl if your driver differs.
V4L2_DEFAULTS = dict(device="/dev/video0", width=3264, height=2464, pixfmt="RG10",
                     exposure_ctrl="exposure", gain_ctrl="gain", gain_scale=16.0,
                     rate_ctrl="frame_rate", rate_scale=1e6, max_fps=21.0, min_fps=2.0,
                     extra=("bypass_mode=0",), skip=4)


# ---------------------------------------------------------------------------
# Decoding (a copy of frames.decode_raw16, kept here so this file stays 3.6-safe)
# ---------------------------------------------------------------------------


def decode_raw16(data, width, height, bits=10, stride_bytes=None, shift=None, n_frames=None):
    """(N, H, W) uint16 frames from a dump of 16-bit pixels; returns (frames, stride, shift)."""
    words = np.frombuffer(data, dtype="<u2")
    if stride_bytes is None:
        per_row = None
        if n_frames and words.size % (n_frames * height) == 0 and words.size // (n_frames * height) >= width:
            per_row = words.size // (n_frames * height)
        for per in range(width, width + 513):
            if per_row is None and (per == width or (2 * per) % 64 == 0) and words.size % (per * height) == 0:
                per_row = per
        if per_row is None:
            raise ValueError("%d bytes is not a whole number of %dx%d frames" % (words.size * 2, width, height))
    else:
        per_row = stride_bytes // 2
    n = words.size // (per_row * height)
    frames = words[: n * per_row * height].reshape(n, height, per_row)[:, :, :width]
    if shift is None:
        low = max(float(np.percentile(frames[0], 0.5)), 1.0)
        shift = min((abs(np.log2(max(low / 2 ** s, 0.5) / BLACK)), s) for s in (0, 2, 4, 6))[1]
    mask = (1 << bits) - 1
    return (frames >> shift) & mask, per_row * 2, shift


def levels(frame, bits=10):
    """Exposure summary of one raw frame: peak above black (fraction of range), saturation."""
    f = np.asarray(frame, float)
    full = (1 << bits) - 1
    top = float(np.percentile(f, 99.95))
    return dict(black=float(np.percentile(f, 5)), peak=top, peak_frac=(top - BLACK) / (full - BLACK),
                saturated=float(np.mean(f >= full - 2)))


# ---------------------------------------------------------------------------
# Back ends
# ---------------------------------------------------------------------------


class V4L2Camera:
    """Raw frames through v4l2-ctl (the Jetson's IMX219 driver)."""

    def __init__(self, dry_run=False, **kw):
        self.cfg = dict(V4L2_DEFAULTS)
        self.cfg.update({k: v for k, v in kw.items() if v is not None})
        self.dry_run = dry_run
        if not dry_run and shutil.which("v4l2-ctl") is None:
            raise RuntimeError("v4l2-ctl not found; install it with: sudo apt install v4l-utils")

    @property
    def size(self):
        return int(self.cfg["width"]), int(self.cfg["height"])

    def command(self, path, exposure_us, gain, n):
        c = self.cfg
        fps = max(min(c["max_fps"], 0.9e6 / float(exposure_us)), c["min_fps"])
        ctrls = ["%s=%d" % (c["exposure_ctrl"], int(round(exposure_us))),
                 "%s=%d" % (c["gain_ctrl"], int(round(gain * c["gain_scale"]))),
                 "%s=%d" % (c["rate_ctrl"], int(round(fps * c["rate_scale"])))] + list(c["extra"])
        return ["v4l2-ctl", "-d", c["device"],
                "--set-fmt-video=width=%d,height=%d,pixelformat=%s" % (c["width"], c["height"], c["pixfmt"]),
                "--set-ctrl", ",".join(ctrls), "--stream-mmap", "--stream-skip=%d" % c["skip"],
                "--stream-count=%d" % n, "--stream-to=%s" % path]

    def grab(self, folder, exposure_us, gain, n):
        """Capture n frames into folder/frames.raw; return (path, meta for the folder)."""
        path = os.path.join(folder, "frames.raw")
        cmd = self.command(path, exposure_us, gain, n)
        print("  $ " + " ".join(cmd))
        w, h = self.size
        meta = dict(width=w, height=h, bits=10, format="raw16", backend="v4l2")
        if self.dry_run:
            return None, meta
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        out, err = p.communicate(timeout=60 + n * (exposure_us * 1e-6 + 0.5))
        if p.returncode != 0 or not os.path.exists(path):
            raise RuntimeError("v4l2-ctl failed:\n" + (err or out))
        with open(path, "rb") as fh:
            data = fh.read()
        frames, stride, shift = decode_raw16(data, w, h, n_frames=n)
        meta.update(stride_bytes=stride, raw_shift=shift, frames=int(frames.shape[0]))
        if frames.shape[0] < n:
            print("  ! only %d of %d frames arrived" % (frames.shape[0], n))
        return frames, meta

    def probe(self):
        for args in (["--list-formats-ext"], ["--list-ctrls"]):
            cmd = ["v4l2-ctl", "-d", self.cfg["device"]] + args
            print("$ " + " ".join(cmd))
            if not self.dry_run:
                subprocess.call(cmd)


class PiCamera:
    """Raw frames through picamera2 (Raspberry Pi OS Bullseye or later)."""

    def __init__(self, dry_run=False, width=3280, height=2464, **kw):
        self.dry_run = dry_run
        self.w, self.h = int(width or 3280), int(height or 2464)
        self.cam = None
        if not dry_run:
            from picamera2 import Picamera2
            self.cam = Picamera2()
            cfg = self.cam.create_still_configuration(raw={"format": "SRGGB10", "size": (self.w, self.h)},
                                                      buffer_count=2)
            self.cam.configure(cfg)
            self.cam.start()

    @property
    def size(self):
        return self.w, self.h

    def grab(self, folder, exposure_us, gain, n):
        meta = dict(width=self.w, height=self.h, bits=10, format="npy", backend="picamera2")
        print("  picamera2: ExposureTime=%d us, AnalogueGain=%.2f, %d frames" % (exposure_us, gain, n))
        if self.dry_run:
            return None, meta
        self.cam.set_controls({"ExposureTime": int(exposure_us), "AnalogueGain": float(gain),
                               "AeEnable": False, "AwbEnable": False})
        time.sleep(max(0.5, 3 * exposure_us * 1e-6))     # let the new settings take hold
        out = []
        for _ in range(n + 2):
            a = self.cam.capture_array("raw").view(np.uint16)[:, :self.w]
            out.append(a.copy())
        frames = np.stack(out[2:])
        meta["frames"] = int(frames.shape[0])
        return frames, meta

    def probe(self):
        if self.cam is not None:
            print(self.cam.sensor_modes)


def open_camera(backend="auto", dry_run=False, **kw):
    if backend == "auto":
        if os.path.exists("/dev/video0") and shutil.which("v4l2-ctl"):
            backend = "v4l2"
        else:
            try:
                import picamera2  # noqa: F401
                backend = "picamera2"
            except ImportError:
                if not dry_run:
                    raise RuntimeError("no camera back end found: needs v4l2-ctl and /dev/video0 "
                                       "(Jetson) or picamera2 (Raspberry Pi)")
                backend = "v4l2"
    if backend == "v4l2":
        return V4L2Camera(dry_run=dry_run, **kw)
    return PiCamera(dry_run=dry_run, width=kw.get("width"), height=kw.get("height"))


# ---------------------------------------------------------------------------
# Capturing one frame set, with an exposure helper
# ---------------------------------------------------------------------------


def capture_set(cam, session, name, kind, source=None, exposure_us=10000, gain=1.0, frames=8,
                auto=None, target=0.75, max_exposure_us=650000, note=""):
    """Capture a frame set into session/name/ and write its meta.json.

    auto: None, or a target peak level (fraction of full scale) to reach first,
    adjusting the exposure in up to four test shots."""
    folder = os.path.join(session, name)
    if not os.path.isdir(folder):
        os.makedirs(folder)
    if auto:
        for _ in range(4):
            test, _ = cam.grab(folder, exposure_us, gain, 1)
            if test is None:
                break
            lv = levels(test[0])
            print("  test shot at %.1f ms: peak %.0f%% of full scale" % (exposure_us / 1000.0, 100 * lv["peak_frac"]))
            if lv["saturated"] > 1e-5:
                exposure_us = exposure_us * 0.4
            elif lv["peak_frac"] < 0.6 * auto or lv["peak_frac"] > 0.95:
                exposure_us = exposure_us * auto / max(lv["peak_frac"], 0.01)
            else:
                break
            if exposure_us > max_exposure_us:
                print("  ! needs more than %.0f ms; raising the gain instead" % (max_exposure_us / 1000.0))
                gain = min(gain * exposure_us / max_exposure_us, 10.0)
                exposure_us = max_exposure_us
        _clear(folder)
    data, meta = cam.grab(folder, exposure_us, gain, frames)
    meta.update(kind=kind, source=source, exposure_us=float(exposure_us), gain=float(gain),
                captured=time.strftime("%Y-%m-%d %H:%M:%S"), note=note)
    if data is not None:
        if meta.get("format") == "npy":
            for i, f in enumerate(data):
                np.save(os.path.join(folder, "frame_%03d.npy" % i), f)
        lv = levels(data[0])
        meta["levels"] = lv
        print("  %s: %d frames at %.1f ms, gain %.2f; peak %.0f%% of full scale, %.3f%% saturated"
              % (name, data.shape[0], exposure_us / 1000.0, gain, 100 * lv["peak_frac"], 100 * lv["saturated"]))
    with open(os.path.join(folder, "meta.json"), "w") as fh:
        json.dump(meta, fh, indent=2)
    return meta


def _clear(folder):
    for f in os.listdir(folder):
        if f.endswith((".raw", ".npy")):
            os.remove(os.path.join(folder, f))


# ---------------------------------------------------------------------------
# The first-light plan
# ---------------------------------------------------------------------------

PLAN = [
    ("flat", "flat", "halogen+ptfe", dict(auto=0.75, exposure_us=20000, frames=16),
     "Halogen lamp on the PTFE sheet, the sheet filling the slit's view (on the bench without "
     "the objective: PTFE held against the slit, the halogen shining through it). Nothing else "
     "lit. Keep this setup for the next step."),
    ("wires", "wires", "halogen+ptfe", dict(same_as="flat", frames=16),
     "Same light. Lay 3 to 5 thin wires (0.1-0.3 mm: guitar string, magnet wire, a hair) across "
     "the PTFE or the slit, spread along its length."),
    ("cfl", "lamp", "cfl", dict(auto=0.75, exposure_us=30000, frames=16),
     "Compact fluorescent bulb lighting the slit (through the PTFE is fine). Room lights off."),
    ("cfl_long", "lamp", "cfl", dict(times="cfl", factor=10.0, frames=16),
     "Same CFL, ten times longer: the bright lines saturate on purpose, the faint argon lines "
     "in the near infrared come up."),
    ("neon", "lamp", "neon", dict(auto=0.75, exposure_us=200000, frames=16),
     "Neon lamp (the outlet tester's glow lamp) right at the slit. Mains voltage: keep fingers "
     "off its contacts and plug it in only while capturing."),
    ("neon_long", "lamp", "neon", dict(times="neon", factor=10.0, frames=16),
     "Same neon lamp, ten times the light: brings up the near infrared neon lines."),
    ("laser", "laser", "laser", dict(auto=0.6, exposure_us=1000, frames=8),
     "Red laser on the PTFE, a spot or line where the slit sees it. Never into the optics "
     "directly, never at eyes."),
]


def run_plan(cam, session, frames=None, steps=None, ask=input):
    """Walk through the first-light captures, then take darks at every exposure used."""
    if not os.path.isdir(session):
        os.makedirs(session)
    done = {}
    for name, kind, source, opts, text in PLAN:
        if steps and name not in steps:
            continue
        print("\n== %s ==\n%s" % (name, text))
        reply = ask("Enter to capture, s to skip, q to stop: ").strip().lower()
        if reply == "q":
            break
        if reply == "s":
            continue
        o = dict(opts)
        n = frames or o.pop("frames", 8)
        o.pop("frames", None)
        if "same_as" in o:
            ref = done.get(o.pop("same_as"))
            if ref:
                o.update(exposure_us=ref["exposure_us"], gain=ref["gain"])
        if "times" in o:
            ref = done.get(o.pop("times"))
            f = o.pop("factor")
            if ref:
                exp = ref["exposure_us"] * f
                gain = ref["gain"]
                if exp > 650000:
                    gain, exp = min(gain * exp / 650000, 10.0), 650000
                o.update(exposure_us=exp, gain=gain)
        while True:
            meta = capture_set(cam, session, name, kind, source, frames=n, **o)
            done[name] = meta
            again = ask("Keep it? Enter = yes, r = retake: ").strip().lower()
            if again != "r":
                break
            o.update(exposure_us=meta["exposure_us"], gain=meta["gain"], auto=None)
            e = ask("Exposure in ms for the retake [%.1f]: " % (meta["exposure_us"] / 1000.0)).strip()
            if e:
                o["exposure_us"] = float(e) * 1000.0
    settings = sorted({(m["exposure_us"], m["gain"]) for m in _session_metas(session)
                       if m.get("kind") != "dark"})
    if settings:
        print("\n== darks ==\nCap the lens (or cover the slit) and turn the lamps off.")
        if ask("Enter to capture darks at %d settings, s to skip: " % len(settings)).strip().lower() != "s":
            for exp, gain in settings:
                tag = "dark_%dus" % int(exp) + ("" if gain == 1.0 else "_gain%g" % gain)
                capture_set(cam, session, tag, "dark", "capped", exposure_us=exp, gain=gain,
                            frames=frames or 16)
    print("\nDone. Copy %s to the PC and run: python -m hsical calibrate %s -o cal"
          % (session, os.path.basename(os.path.normpath(session))))


def _session_metas(session):
    out = []
    for d in sorted(os.listdir(session)):
        p = os.path.join(session, d, "meta.json")
        if os.path.exists(p):
            with open(p) as fh:
                out.append(json.load(fh))
    return out


# ---------------------------------------------------------------------------
# Focus aid
# ---------------------------------------------------------------------------


def line_width(frame, rows=None):
    """FWHM (px) of the brightest line in a raw lamp frame, along whichever axis it runs."""
    f = np.asarray(frame, float) - BLACK
    k = np.array([0.25, 0.5, 0.25])
    for ax in (0, 1):  # erase the colour mosaic
        f = np.apply_along_axis(lambda v: np.convolve(v, k, mode="same"), ax, f)
    a, b = f.mean(0), f.mean(1)
    spec = a if np.std(np.diff(a)) > np.std(np.diff(b)) else b
    if rows is not None:
        spec = spec[rows[0]:rows[1]]
    spec = spec - np.percentile(spec, 20)
    i = int(np.argmax(spec))
    half = 0.5 * spec[i]
    lo, hi = i, i
    while lo > 0 and spec[lo] > half:
        lo -= 1
    while hi < spec.size - 1 and spec[hi] > half:
        hi += 1
    if spec[lo] > half or spec[hi] > half:
        return float("nan"), i
    left = lo + (half - spec[lo]) / (spec[lo + 1] - spec[lo])
    right = hi - (half - spec[hi]) / (spec[hi - 1] - spec[hi])
    return float(right - left), i


def focus_loop(cam, exposure_us, gain, folder, count=0):
    """Print the brightest line's width over and over; turn the focus for the smallest number."""
    best = None
    i = 0
    print("Brightest line width (px); smaller is sharper. Ctrl+C to stop.")
    try:
        while count == 0 or i < count:
            frames, _ = cam.grab(folder, exposure_us, gain, 1)
            if frames is None:
                return
            w, pos = line_width(frames[0])
            best = w if best is None or (w == w and w < best) else best
            lv = levels(frames[0])
            print("  width %6.2f px at %5d   (best %6.2f)   peak %3.0f%%" % (w, pos, best, 100 * lv["peak_frac"]))
            i += 1
    except KeyboardInterrupt:
        print()
