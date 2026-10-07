"""Locking the scan mirror's line clock to the spectrograph camera's frames.

The IMX219 free-runs (it has no trigger input), so the camera is the master clock and the
mirror follows it. line_camera publishes when every frame was exposed (FrameStamp); with a
rolling shutter the rows that see the slit are exposed over a window that starts before the
frame's first row is read out (by the exposure time) and ends when the last of them is. The
mirror has to hold still across that whole window, so its moves have to fall in the gaps
between windows.

    frame k        |<--- window k --->|          |<--- window k+1 --->|
    mirror      ...still.....still....| move+settle |....still....still...
                                      ^ tick

For a sweep the bridge works out (plan_lock):
  * how many frame periods a line takes: enough for the line period asked for, and for one
    window plus a move, its settling, and a safety margin either side;
  * where in the frame the tick goes, so the window(s) of the line sit in the middle of the
    time the mirror holds still.
Then it starts the ESP32's line clock on that phase (SCAN t0= period=) and, as each EV LINE
reports when its tick really was, measures the phase error against the frame clock and corrects
it with NUDGE, and the period with PERIOD when the camera's or the ESP32's clock rate estimate
changes. Times here are ROS ns, kept as integers (a float holds today's time in ns only to
256 ns); ClockSync turns them into the ESP32's us.
"""

import math
import threading
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class FrameFit:
    """The camera's frame clock: frame k starts (row 0 read out) at t_ref_ns + k x period_ns."""
    t_ref_ns: int
    period_ns: float
    rms_ns: float
    frames: int
    window_start_ns: float   # the exposure window of the rows that matter, from a frame's start
    window_end_ns: float

    def nearest_offset_ns(self, t_ns, shift_ns=0.0):
        """t - shift minus the start of the frame nearest to it, in -period/2 .. period/2."""
        r = ((t_ns - self.t_ref_ns) - shift_ns) / self.period_ns
        return (r - round(r)) * self.period_ns


class FrameClock:
    """The last few seconds of camera frames, and a straight-line fit of their start times."""

    def __init__(self, keep=150, min_frames=15, stale_s=0.5):
        self.frames = deque(maxlen=keep)   # (sof_ns, window start ns, window end ns)
        self.min_frames = min_frames
        self.stale_ns = int(stale_s * 1e9)
        self._lock = threading.Lock()
        self._fit = None

    def add(self, sof_ns, window_start_ns, window_end_ns):
        with self._lock:
            if self.frames and sof_ns <= self.frames[-1][0]:
                if sof_ns < self.frames[-1][0] - 10**9:
                    self.frames.clear()     # the camera restarted, or the clock jumped
                else:
                    return
            self.frames.append((int(sof_ns), int(window_start_ns), int(window_end_ns)))
            self._fit = None

    def clear(self):
        with self._lock:
            self.frames.clear()
            self._fit = None

    def latest_ns(self):
        with self._lock:
            return self.frames[-1][0] if self.frames else None

    def fit(self, now_ns=None):
        """FrameFit, or None with too few frames or none in the last stale_s."""
        with self._lock:
            if self._fit is not None and (now_ns is None or now_ns - self.frames[-1][0] <= self.stale_ns):
                return self._fit
            frames = list(self.frames)
        if len(frames) < self.min_frames or (now_ns is not None and now_ns - frames[-1][0] > self.stale_ns):
            return None
        sof = np.array([f[0] for f in frames], dtype=np.int64)
        gaps = np.diff(sof)
        p0 = float(np.median(gaps))       # most gaps are one frame; a dropped frame makes a double
        if p0 <= 0:
            return None
        rel = (sof - sof[-1]).astype(np.float64)
        k = np.round(rel / p0)
        a = np.vstack([np.ones_like(k), k]).T
        (b, period), *_ = np.linalg.lstsq(a, rel, rcond=None)
        resid = rel - (b + period * k)
        last = frames[-1]
        fit = FrameFit(t_ref_ns=int(sof[-1]) + int(round(b)), period_ns=float(period),
                       rms_ns=float(np.sqrt(np.mean(resid ** 2))), frames=len(frames),
                       window_start_ns=float(last[1] - last[0]), window_end_ns=float(last[2] - last[0]))
        with self._lock:
            if self.frames and self.frames[-1] == last:
                self._fit = fit
        return fit


@dataclass
class LockPlan:
    frames_per_line: int
    good_frames: int        # frames per line exposed entirely while the mirror holds still
    frame_period_ns: float
    phase_ns: float         # a tick goes this long after the start of a frame (negative: before it)
    slack_ns: float         # spare time per line, split evenly either side of the windows
    busy_ns: float          # a line's move and settling, as planned
    margin_ns: float
    window_ns: float

    @property
    def line_period_ns(self):
        return self.frames_per_line * self.frame_period_ns


def plan_lock(fit, min_line_period_ns, busy_ns, margin_ns, tolerance=0.02):
    """The frame count and tick phase for lines at least min_line_period_ns apart that leave
    one whole exposure window still between moves of busy_ns, with margin_ns either side."""
    p = fit.period_ns
    window = fit.window_end_ns - fit.window_start_ns
    need = window + busy_ns + 2.0 * margin_ns
    m = max(1, math.ceil(min_line_period_ns / p - tolerance), math.ceil(need / p - 1e-9))
    good = int(math.floor((m * p - need) / p)) + 1
    slack = m * p - need - (good - 1) * p
    # the first good frame's window starts busy + margin + slack/2 after the tick
    phase = fit.window_start_ns - (busy_ns + margin_ns + slack / 2.0)
    return LockPlan(frames_per_line=m, good_frames=good, frame_period_ns=p, phase_ns=phase, slack_ns=slack,
                    busy_ns=busy_ns, margin_ns=margin_ns, window_ns=window)


def first_tick_ns(fit, plan, not_before_ns):
    """The first tick on the plan's phase at or after not_before_ns."""
    k = math.ceil(((not_before_ns - fit.t_ref_ns) - plan.phase_ns) / fit.period_ns)
    return fit.t_ref_ns + int(math.ceil(k * fit.period_ns + plan.phase_ns))


def phase_error_ns(fit, plan, tick_ns):
    """How late a tick is against the plan's phase on the frame clock (negative: early)."""
    return fit.nearest_offset_ns(tick_ns, plan.phase_ns)


def busy_ns(steps_per_line, vstart, settle_us, tick_us):
    """A line's move plus settling, from the ESP32's settings: the move starts at vstart (a step
    or two never gets faster), and every time lands on the step timer's grid."""
    return (abs(steps_per_line) / float(vstart) * 1e6 + settle_us + 2 * tick_us) * 1000.0
