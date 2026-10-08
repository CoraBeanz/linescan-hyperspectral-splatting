"""When the camera exposed each line, and where the mirror was meanwhile: the rig's two clocks.

The IMX219 free-runs (it has no trigger input) and reads its rows out one after another, a
rolling shutter: row r of a frame stops exposing at the frame's start (sof, row 0 read out)
plus r x line_time, and started one exposure earlier. On the rig:

  * line_camera stamps each frame with its sof, plus stamp_offset_us, and works out the window
    in which the rows that see the slit were exposing (so101_scan_camera.sources.SensorTiming);
  * the scan mirror's bridge locks each sweep to those frames (so101_scan_sweep.frame_lock):
    the line period becomes a whole number of frames, and its ticks go where the mirror's
    moves fall between the windows, with a margin either side;
  * the ESP32 moves the mirror from each tick at its start speed, reports the line once the
    mirror has settled (the line's stamp in lines.csv) and holds it until the next tick, less
    one step-timer tick (hold_until);
  * line_camera keeps the frames whose whole window falls inside a line's hold, with its own
    margin (so101_scan_camera.matcher.LineMatcher), and saves the first of them.

The simulator runs the same arithmetic with the same code. The camera's timestamps can be
off by stamp_error_us, as a driver's are when its timestamp isn't really row 0's read-out;
line_camera's stamp_offset_us is there to cancel that. Whatever is left moves the windows
the camera really exposed against the ones the bridge locked to, and a row that was still
exposing when the mirror moved sees a blend of the lines it moved between (mix()).

Times are integer ns on one clock: the ESP32's clock sync and the camera's clock rate are
taken as perfect, so only the stamps' offset is in error.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import repo  # noqa: F401
from so101_scan_camera.matcher import FrameInfo, Line, LineMatcher  # noqa: E402
from so101_scan_camera.sources import IMX219_MODES, SensorTiming  # noqa: E402
from so101_scan_sweep import frame_lock as fl  # noqa: E402

FULL_ROWS = 2464                 # the IMX219's rows, which hsical's binning divides


def camera_mode(binning):
    """The IMX219 mode line_camera would run for frames at this binning: (width, height), fps.

    The full sensor runs at up to 21 fps; the binned 1640 x 1232 mode, the camera node's
    default, at 30. The simulator's binning 4 is that mode binned once more in software."""
    mode = (3264, 2464) if binning < 2 else (1640, 1232)
    return mode, min(IMX219_MODES[mode]["max_fps"], 30.0)


@dataclass
class Mirror:
    """The scan mirror's timing, as the ESP32 and the bridge run it by default."""
    vstart: float = 1600.0          # steps/s: a line's move of a few microsteps never goes faster
    settle_us: float = 3000.0       # waited after a move before the line counts as settled
    tick_us: float = 50.0           # the step timer: hold_until is this much before the next tick
    lock_margin_s: float = 0.001    # the bridge's margin either side of an exposure window
    lock_lead_s: float = 0.15       # its first tick is at least this far ahead
    match_margin_s: float = 0.0005  # line_camera's margin when it matches frames to lines

    def move_ns(self, steps):
        return abs(steps) / self.vstart * 1e9


@dataclass
class Camera:
    """The camera's frame clock, true and as stamped."""
    t0_ns: int                      # when frame 0's row 0 was really read out
    fps: float
    line_time_us: float             # per row of the frames as written
    r0: int                         # the rows that see the slit
    r1: int
    exposure_us: float
    gain: float = 1.0
    stamp_error_us: float = 0.0     # how late the driver's stamps are against the true read-out
    stamp_offset_us: float = 0.0    # line_camera's correction, added to every stamp

    @property
    def period_ns(self):
        return 1e9 / self.fps

    @property
    def timing(self):
        return SensorTiming(self.line_time_us, self.r0, self.r1)

    @property
    def stamp_shift_ns(self):
        """Logged minus true: what's left of the stamps' error after line_camera's offset."""
        return int(round((self.stamp_error_us + self.stamp_offset_us) * 1000))

    def sof(self, j):
        """True read-out of frame j's row 0."""
        return self.t0_ns + int(round(j * self.period_ns))

    def logged(self, j):
        """FrameInfo for frame j as line_camera logs it: its stamp and slit rows' window."""
        sof = self.sof(j) + self.stamp_shift_ns
        start, end = self.timing.window(sof, self.exposure_us)
        return FrameInfo(sequence=j, sof_ns=sof, start_ns=start, end_ns=end, exposure_us=self.exposure_us,
                         gain=self.gain)

    def true_window(self, j):
        return self.timing.window(self.sof(j), self.exposure_us)

    def row_exposures(self, j, rows):
        """[rows, 2] when each row of frame j really exposed (ns, as floats). Rows outside the
        slit's are timed as its nearest end row: they see no light."""
        r = np.clip(np.asarray(rows, float), self.r0, self.r1)
        end = self.sof(j) + r * self.line_time_us * 1000.0
        return np.stack([end - self.exposure_us * 1000.0, end], axis=-1)


@dataclass
class LineTiming:
    index: int
    tick_ns: int                    # the mirror starts its move onto this line
    stamp_ns: int                   # it has settled: the line's stamp
    hold_until_ns: int              # it moves on (less one timer tick)
    move: tuple | None              # (start, end) ns of the move onto this line; None for line 0
    frames: list = field(default_factory=list)  # FrameInfo of the frames line_camera keeps, in order


@dataclass
class SweepTiming:
    lock: fl.LockPlan
    lines: list                     # LineTiming
    end_ns: int                     # the last line's hold ends

    @property
    def line_period_ns(self):
        return self.lock.line_period_ns


def plan_sweep(camera: Camera, mirror: Mirror, sweep_id, n_lines, steps_per_line, line_period_s, not_before_ns):
    """The bridge's lock for one sweep, the ESP32's line times, and which frames line_camera keeps."""
    first = camera.logged(0)
    fit = fl.FrameFit(t_ref_ns=first.sof_ns, period_ns=camera.period_ns, rms_ns=0.0, frames=1000,
                      window_start_ns=float(first.start_ns - first.sof_ns),
                      window_end_ns=float(first.end_ns - first.sof_ns))
    busy = fl.busy_ns(steps_per_line, mirror.vstart, mirror.settle_us, mirror.tick_us)
    lock = fl.plan_lock(fit, line_period_s * 1e9, busy, mirror.lock_margin_s * 1e9)
    tick0 = fl.first_tick_ns(fit, lock, not_before_ns + int(mirror.lock_lead_s * 1e9))
    move_ns = mirror.move_ns(steps_per_line)
    matcher = LineMatcher(margin_ns=mirror.match_margin_s * 1e9)
    lines = []
    for k in range(n_lines):
        tick = tick0 + int(round(k * lock.line_period_ns))
        nxt = tick0 + int(round((k + 1) * lock.line_period_ns))
        if k == 0:
            move, stamp = None, tick          # already there: it went to the start before the first tick
        else:
            move = (tick, tick + int(round(move_ns)))
            stamp = move[1] + int(round(mirror.settle_us * 1000))
        lt = LineTiming(k, tick, stamp, nxt - int(round(mirror.tick_us * 1000)), move)
        line = Line(sweep_id, k, lt.stamp_ns, lt.hold_until_ns, settled=True, last=k == n_lines - 1)
        # the frames whose logged window could fall inside the hold
        j = max(0, math.floor((lt.stamp_ns - fit.t_ref_ns - fit.window_end_ns) / camera.period_ns))
        while True:
            f = camera.logged(j)
            if f.start_ns > lt.hold_until_ns:
                break
            if matcher.fits(line, f):
                lt.frames.append(f)
            j += 1
        lines.append(lt)
    return SweepTiming(lock, lines, lines[-1].hold_until_ns)


def mix(camera: Camera, sweep: SweepTiming, k, j, angles, rows, sub_steps=6):
    """How frame j, kept for line k, really saw the scene: [(mirror angle, [rows] weight)].

    angles: the true mirror angle of every line of the sweep. Each row's weights sum to 1:
    the share of its exposure the mirror spent at each angle. A frame the mirror held still
    for gives [(angles[k], 1)]; a row that was still exposing during a move gets the angles
    along it, sub_steps of them per move."""
    lines = sweep.lines
    segments = []                                     # (start, end, angle) of the mirror's path
    if k > 0:
        a, b = lines[k].move
        segments += [(-math.inf, a, angles[k - 1])] + _ramp(a, b, angles[k - 1], angles[k], sub_steps)
        still_from = b
    else:
        still_from = -math.inf
    if k + 1 < len(lines):
        a, b = lines[k + 1].move
        segments += [(still_from, a, angles[k])] + _ramp(a, b, angles[k], angles[k + 1], sub_steps)
        segments.append((b, math.inf, angles[k + 1]))
    else:
        segments.append((still_from, math.inf, angles[k]))
    exp = camera.row_exposures(j, rows)
    span = exp[:, 1] - exp[:, 0]
    out = []
    for s, e, angle in segments:
        w = np.clip(np.minimum(exp[:, 1], e) - np.maximum(exp[:, 0], s), 0.0, None) / span
        if w.max() > 1e-9:
            out.append((float(angle), w))
    return out


def _ramp(a, b, angle_a, angle_b, n):
    """A move at constant speed from angle_a at a to angle_b at b, as n pieces at their middles."""
    t = np.linspace(a, b, n + 1)
    return [(float(t[i]), float(t[i + 1]), angle_a + (angle_b - angle_a) * (i + 0.5) / n) for i in range(n)]
