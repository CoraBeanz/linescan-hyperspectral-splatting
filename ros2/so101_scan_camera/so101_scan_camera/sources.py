"""Where frames come from: the IMX219 through V4L2, or a simulated camera for tests.

Both give RawFrame: the sensor's 10-bit values as an (H, W) uint16 array, the frame's start
(row 0 read out) in ROS time, and the exposure and gain it was taken with.

The IMX219 reads its rows out one after another (a rolling shutter), so row r stops exposing
at start + r x line_time and started one exposure earlier. SensorTiming turns that into the
window in which the rows that see the slit were exposing, which is what the mirror has to hold
still across.
"""

import threading
import time
from dataclasses import dataclass

import numpy as np

BLACK_LEVEL = 64.0
FULL_SCALE = 1023
BITS = 10

# The Jetson Nano's IMX219 modes (L4T R32 device tree: line_length 3448 pixel clocks at
# 182.4 MHz in every mode, so 18.9 us a row). The binned 1640 x 1232 mode keeps the whole
# slit and spectrum at 30 fps; 3264 x 2464 is full resolution at 21 fps, with almost no
# time between the last row of one frame and the first of the next.
IMX219_MODES = {
    (3264, 2464): dict(max_fps=21.0, line_time_us=18.904),
    (3264, 1848): dict(max_fps=28.0, line_time_us=18.904),
    (1920, 1080): dict(max_fps=30.0, line_time_us=18.904),
    (1640, 1232): dict(max_fps=30.0, line_time_us=18.904),
    (1280, 720): dict(max_fps=60.0, line_time_us=18.904),
}


@dataclass
class RawFrame:
    sequence: int
    sof_ns: int            # ROS time at which row 0 was read out
    exposure_us: float
    gain: float
    image: np.ndarray      # (H, W) uint16, 10-bit values in the low bits


@dataclass
class SensorTiming:
    """When the rows r0..r1 (the ones that see the slit) of a frame were exposing."""
    line_time_us: float
    r0: int
    r1: int

    def window(self, sof_ns, exposure_us):
        start = sof_ns + int(round((self.r0 * self.line_time_us - exposure_us) * 1000))
        end = sof_ns + int(round(self.r1 * self.line_time_us * 1000))
        return start, end


def guess_shift(frame, black=BLACK_LEVEL):
    """Bit shift that puts a 10-bit sensor's values back in the low bits of each 16-bit word.

    Capture paths differ in where they put the 10 bits (the Tegra VI writes them high on some
    chips). Every frame here has dark pixels outside the slit's image at the black level, about
    64 counts, so take the shift that brings the darkest pixels there (as hsical does)."""
    low = max(float(np.percentile(np.asarray(frame)[::4, ::4], 0.5)), 1.0)
    return min((abs(np.log2(max(low / 2 ** s, 0.5) / black)), s) for s in (0, 2, 4, 6))[1]


def decode_rg10(data, width, height, bytesperline=0, shift=0):
    """(H, W) uint16 10-bit values from one RG10 buffer (16-bit words, rows maybe padded).
    shift=None gives the words as stored, for guess_shift."""
    words = np.frombuffer(data, dtype="<u2")
    per_row = bytesperline // 2 if bytesperline else 0
    if per_row < width or per_row * height > words.size:
        per_row = words.size // height       # the driver's bytesperline doesn't fit its buffer
    if per_row < width:
        raise ValueError("a %d-byte buffer can't hold %dx%d 16-bit pixels" % (len(data), width, height))
    img = words[: per_row * height].reshape(height, per_row)[:, :width]
    if shift is None:
        return img
    return (img >> shift) & FULL_SCALE


def realtime_offset_ns():
    """CLOCK_REALTIME - CLOCK_MONOTONIC, read as tightly as Python allows."""
    best = None
    for _ in range(3):
        m0 = time.monotonic_ns()
        r = time.time_ns()
        m1 = time.monotonic_ns()
        if best is None or m1 - m0 < best[0]:
            best = (m1 - m0, r - (m0 + m1) // 2)
    return best[1]


class V4l2Source:
    """The IMX219 on a Jetson through /dev/video0 (the Tegra VI driver), raw RG10.

    Control names and units follow the calibration kit's (calibration/hsical/capture.py):
    exposure in us, gain in 1/16 steps, frame_rate in micro-frames per second, bypass_mode=0
    so the driver takes the mode from the format. Override them if your driver differs
    (`v4l2-ctl -d /dev/video0 --list-ctrls` lists what it has)."""

    def __init__(self, device="/dev/video0", width=1640, height=1232, fps=30.0, exposure_us=5000.0, gain=1.0,
                 exposure_ctrl="exposure", gain_ctrl="gain", gain_scale=16.0, rate_ctrl="frame_rate",
                 rate_scale=1e6, extra_ctrls=(("bypass_mode", 0),), raw_shift=-1, stamp_offset_us=0.0,
                 n_buffers=4, log=None):
        from so101_scan_camera.v4l2 import V4l2Device
        self.width, self.height, self.fps = int(width), int(height), float(fps)
        self.exposure_us, self.gain = float(exposure_us), float(gain)
        self.ctrl_names = dict(exposure=exposure_ctrl, gain=gain_ctrl, rate=rate_ctrl)
        self.gain_scale, self.rate_scale = float(gain_scale), float(rate_scale)
        self.extra = dict(extra_ctrls)
        self.shift = None if raw_shift < 0 else int(raw_shift)
        self._shift_votes = []
        self.stamp_offset_ns = int(round(stamp_offset_us * 1000))
        self.n_buffers = n_buffers
        self.log = log
        self.dev = V4l2Device(device)
        self.device = device
        self.driver_flags = {}
        self._lock = threading.Lock()

    def _controls(self):
        c = dict(self.extra)
        c[self.ctrl_names["exposure"]] = self.exposure_us
        c[self.ctrl_names["gain"]] = self.gain * self.gain_scale
        c[self.ctrl_names["rate"]] = self.fps * self.rate_scale
        return {k: v for k, v in c.items() if k}

    def start(self):
        fmt = self.dev.set_format(self.width, self.height, "RG10")
        self.bytesperline = fmt["bytesperline"]
        # bypass_mode first: on the Tegra driver it decides whether the format picks the mode
        extra = {k: v for k, v in self.extra.items() if k in self.dev.controls}
        if extra:
            self.dev.set_controls(extra)
        self.dev.set_controls({k: v for k, v in self._controls().items() if k not in extra})
        self.dev.start(self.n_buffers)

    def set_exposure(self, exposure_us=None, gain=None):
        with self._lock:
            if exposure_us is not None:
                self.exposure_us = float(exposure_us)
            if gain is not None:
                self.gain = float(gain)
            self.dev.set_controls({self.ctrl_names["exposure"]: self.exposure_us,
                                   self.ctrl_names["gain"]: self.gain * self.gain_scale})

    def read(self, timeout=1.0):
        f = self.dev.read(timeout)
        if f is None:
            return None
        self.driver_flags = dict(monotonic=f.monotonic, start_of_exposure=f.start_of_exposure)
        if f.error:
            return None
        if self.shift is None:
            self._shift_votes.append(guess_shift(decode_rg10(f.data, self.width, self.height, self.bytesperline,
                                                             None)))
            if len(self._shift_votes) < 3:
                return None        # still working out where the 10 bits are
            self.shift = max(set(self._shift_votes), key=self._shift_votes.count)
            if self.log:
                self.log("RG10 values sit %d bits up in each 16-bit word" % self.shift)
        img = decode_rg10(f.data, self.width, self.height, self.bytesperline, self.shift)
        ts = f.timestamp_ns + (realtime_offset_ns() if f.monotonic else 0)
        with self._lock:
            exposure, gain = self.exposure_us, self.gain
        return RawFrame(f.sequence, ts + self.stamp_offset_ns, exposure, gain, np.ascontiguousarray(img))

    def stop(self):
        self.dev.close()


class FakeSource:
    """A free-running camera on the ROS clock, for tests and for running without hardware.

    Frame k starts at t_start + k x period. render(sof_ns, exposure_us) makes its image;
    without one, frames are the black level plus noise. Frames are delivered when their last
    row has been read out, like a real camera's, plus `latency_s`."""

    def __init__(self, width=164, height=124, fps=30.0, exposure_us=3000.0, gain=1.0, render=None,
                 timing=None, latency_s=0.002, period_error_ppm=0.0, jitter_us=20.0, seed=0, clock=None):
        self.width, self.height, self.fps = int(width), int(height), float(fps)
        self.exposure_us, self.gain = float(exposure_us), float(gain)
        self.render = render
        self.timing = timing or SensorTiming(18.904, 0, self.height - 1)
        self.latency_ns = int(latency_s * 1e9)
        self.period_ns = 1e9 / self.fps * (1.0 + period_error_ppm * 1e-6)
        self.jitter_ns = jitter_us * 1000.0
        self.rng = np.random.default_rng(seed)
        self.clock = clock or time.time_ns
        self.sequence = 0
        self.t_start = None
        self._stopped = threading.Event()

    def start(self):
        self.t_start = self.clock() + 50_000_000
        self.sequence = 0

    def set_exposure(self, exposure_us=None, gain=None):
        if exposure_us is not None:
            self.exposure_us = float(exposure_us)
        if gain is not None:
            self.gain = float(gain)

    def next_sof_ns(self):
        return int(self.t_start + self.sequence * self.period_ns)

    def read(self, timeout=1.0):
        sof = self.next_sof_ns()
        _, end = self.timing.window(sof, self.exposure_us)
        due = max(end, sof + int(self.timing.line_time_us * 1000 * (self.height - 1))) + self.latency_ns
        wait = (due - self.clock()) * 1e-9
        if wait > timeout:
            self._stopped.wait(timeout)
            return None
        if wait > 0 and self._stopped.wait(wait):
            return None
        seq = self.sequence
        self.sequence += 1
        stamp = sof + int(self.rng.normal(0.0, self.jitter_ns)) if self.jitter_ns else sof
        if self.render is not None:
            img = self.render(sof, self.exposure_us, self.gain)
        else:
            img = (BLACK_LEVEL + self.rng.normal(0.0, 1.5, (self.height, self.width))).round()
        img = np.clip(np.asarray(img), 0, FULL_SCALE).astype(np.uint16)
        return RawFrame(seq, stamp, self.exposure_us, self.gain, img)

    def stop(self):
        self._stopped.set()
