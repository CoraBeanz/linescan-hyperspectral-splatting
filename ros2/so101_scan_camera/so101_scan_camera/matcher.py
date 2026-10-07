"""Which camera frames belong to which scan line.

The mirror holds still for a line from the ScanLine's stamp (it settled) until its hold_until
(it moves on). A frame belongs to the line when the whole window in which the slit's rows were
exposing falls inside that, with a margin either side for the clocks' error; frames that
straddle a move belong to no line. With the bridge locked to the camera that is every frame,
or one frame in every frames_per_line; without the lock, only the frames that happen to fit.

Frames arrive in order, and the ScanLine of a line normally comes before its frames (it is sent
as the mirror settles, and a frame is only delivered once read out). So a frame that fits no
line yet waits briefly in case its line is late, and a line is finished as soon as a frame
exposed after its hold_until arrives, or after a timeout if frames stop.
"""

from collections import deque
from dataclasses import dataclass, field


@dataclass
class FrameInfo:
    sequence: int
    sof_ns: int
    start_ns: int          # the slit rows' exposure window
    end_ns: int
    exposure_us: float
    gain: float
    payload: object = None  # whatever the caller keeps with the frame (its image)


@dataclass
class Line:
    sweep_id: int
    index: int
    stamp_ns: int          # the mirror settled
    hold_until_ns: int     # the mirror moves on
    settled: bool
    last: bool
    angle: float = 0.0
    frames: list = field(default_factory=list)   # FrameInfo, in order
    arrived_ns: int = 0

    @property
    def status(self):
        if not self.settled:
            return "unsettled"
        return "ok" if self.frames else "no_frame"


class LineMatcher:
    def __init__(self, margin_ns=500_000, wait_frames=4, line_timeout_ns=1_500_000_000):
        self.margin_ns = int(margin_ns)
        self.line_timeout_ns = int(line_timeout_ns)
        self.lines = []                        # waiting for frames, in time order
        self.early = deque(maxlen=wait_frames)  # recent frames that fitted no line yet
        self.latest_start_ns = None            # the newest frame's window start

    def fits(self, line, frame):
        return (line.settled and line.stamp_ns + self.margin_ns <= frame.start_ns
                and frame.end_ns <= line.hold_until_ns - self.margin_ns)

    def add_line(self, line, now_ns=0):
        """A ScanLine. Returns (frames that were waiting for it, as (line, frame) pairs, and lines
        now finished)."""
        line.arrived_ns = now_ns
        matched = []
        if line.settled:
            for f in list(self.early):
                if self.fits(line, f):
                    self.early.remove(f)
                    line.frames.append(f)
                    matched.append((line, f))
        if not line.settled or (self.latest_start_ns is not None and self.latest_start_ns > line.hold_until_ns):
            return matched, [line]            # no frame to come can be in it
        self.lines.append(line)
        self.lines.sort(key=lambda ln: ln.stamp_ns)
        return matched, []

    def add_frame(self, frame):
        """A frame. Returns (the line it belongs to or None, lines now finished)."""
        self.latest_start_ns = frame.start_ns
        done = [ln for ln in self.lines if ln.hold_until_ns < frame.start_ns]
        if done:
            self.lines = [ln for ln in self.lines if ln.hold_until_ns >= frame.start_ns]
        for ln in self.lines:
            if self.fits(ln, frame):
                ln.frames.append(frame)
                return ln, done
        if not self.lines or frame.start_ns > self.lines[-1].stamp_ns:
            self.early.append(frame)          # maybe its line hasn't been heard of yet
        return None, done

    def expire(self, now_ns):
        """Lines that frames stopped coming for."""
        done = [ln for ln in self.lines if now_ns - max(ln.hold_until_ns, ln.arrived_ns) > self.line_timeout_ns]
        if done:
            self.lines = [ln for ln in self.lines if ln not in done]
        return done

    def flush(self):
        """Every line still waiting, finished now."""
        done, self.lines = self.lines, []
        self.early.clear()
        return done
