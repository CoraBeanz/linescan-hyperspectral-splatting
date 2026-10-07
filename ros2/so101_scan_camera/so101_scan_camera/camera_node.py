"""line_camera: the spectrograph camera, one scan line per frame.

    ros2 launch so101_scan_bringup scan_arm.launch.py camera:=v4l2 camera_calibration:=/data/hsical/cal
    ros2 run so101_scan_camera line_camera --ros-args -p source:=fake     # no camera needed

It reads the IMX219 raw through V4L2 (the calibration kit's path; sources.py), and for every
frame publishes when the rows that see the slit were exposed, which the mirror bridge locks its
line clock to. It pairs frames with the mirror's scan lines (matcher.py: a frame belongs to a
line when its whole exposure falls while the mirror held still), bins each line with the
calibration (binning.py: slit positions x wavelengths, mean raw counts), and while a recording
is open writes them into the scan folder (session.py says what goes where).

  ~/frame              FrameStamp        every frame: when its slit rows were exposed
  ~/preview            sensor_msgs/Image the latest frame, shrunk, a few times a second (mono8)
  ~/scan_preview       sensor_msgs/Image the sweep so far: a row per line, the slit across, the
                                         mean over the spectrum (mono8; needs a calibration)
  ~/start_recording    StartRecording    write lines into a scan folder (scan_sweep calls it)
  ~/stop_recording     std_srvs/Trigger  finish writing; the message says how many lines got frames
  ~/capture_reference  CaptureReference  average a dark or a white at the current exposure
  /scan_mirror/line    ScanLine          (subscribed) the mirror's scan lines

Exposure and gain are parameters (exposure_us, gain) and can be changed while it runs, e.g.
`ros2 param set /line_camera exposure_us 8000`; the next few frames are skipped while the sensor
takes the new setting. Without a calibration the lines can't be binned, so every line's raw
frame is kept instead (4 MB each): calibrate first (calibration/README.md) for real scans.

source:=fake runs a simulated camera on a made-up spectrograph (synthetic.py), whose scene
changes with the mirror's line, so everything downstream can be tried without hardware.
"""

import os
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from so101_scan_camera import session as sess
from so101_scan_camera.binning import SATURATED, LineBinner, load_calibration
from so101_scan_camera.matcher import FrameInfo, Line, LineMatcher
from so101_scan_camera.sources import BLACK_LEVEL, FULL_SCALE, IMX219_MODES, FakeSource, SensorTiming, V4l2Source
from so101_scan_interfaces.msg import FrameStamp, ScanLine
from so101_scan_interfaces.srv import CaptureReference, StartRecording

DATA_DIR = os.environ.get("SO101_SCAN_DATA", os.path.expanduser("~/so101_scan"))
FAKE_CALIBRATION = "/tmp/line_camera_fake"


class LineData:
    """A line's frames as they come in: the sum of their binned values, and its raw frame."""

    def __init__(self):
        self.sum = None
        self.n = 0
        self.saturated = 0
        self.raw = None

    def add(self, binned, saturated, raw=None):
        if binned is not None:
            self.sum = binned.astype(np.float64) if self.sum is None else self.sum + binned
        self.n += 1
        self.saturated = max(self.saturated, saturated)
        if raw is not None and self.raw is None:
            self.raw = raw

    def mean(self):
        return None if self.sum is None else (self.sum / self.n).astype(np.float32)


class SweepData:
    def __init__(self):
        self.lines = {}          # index -> binned mean, or None without frames
        self.touched = time.monotonic()
        self.counts = {}


class ReferenceJob:
    def __init__(self, n):
        self.n = n
        self.frames = []
        self.done = threading.Event()

    def add(self, image):
        if len(self.frames) < self.n:
            self.frames.append(image)
            if len(self.frames) == self.n:
                self.done.set()


def stamp_ns(t):
    return Time.from_msg(t).nanoseconds


class LineCamera(Node):
    def __init__(self, source_factory=None, **node_kwargs):
        super().__init__("line_camera", **node_kwargs)
        p = lambda name, value: self.declare_parameter(name, value).value   # noqa: E731
        self.source_kind = p("source", "v4l2")
        self.device = p("device", "/dev/video0")
        self.width, self.height = p("width", 1640), p("height", 1232)
        self.fps = p("fps", 30.0)
        self.exposure_us, self.gain = p("exposure_us", 5000.0), p("gain", 1.0)
        self.ctrls = dict(exposure_ctrl=p("exposure_ctrl", "exposure"), gain_ctrl=p("gain_ctrl", "gain"),
                          gain_scale=p("gain_scale", 16.0), rate_ctrl=p("rate_ctrl", "frame_rate"),
                          rate_scale=p("rate_scale", 1e6))
        self.bypass_mode = p("bypass_mode", 0)          # -1: leave it alone
        self.raw_shift = p("raw_shift", -1)             # -1: work it out from the first frames
        self.stamp_offset_us = p("stamp_offset_us", 0.0)
        line_time_us = p("line_time_us", 0.0)           # 0: the IMX219's, 18.9 us
        slit_rows = (p("slit_row_first", -1), p("slit_row_last", -1))
        calibration = p("calibration", "")
        self.slit_bins = p("slit_bins", 256)
        nm_step, nm_min, nm_max = p("nm_step", 0.0), p("nm_min", 0.0), p("nm_max", 0.0)
        self.slit_reversed = p("slit_reversed", False)
        self.match_margin = p("match_margin", 0.0005)
        raw_every = p("raw_every", -1)                  # -1: every line without a calibration, none with
        self.settle_frames = p("settle_frames", 3)
        self.frame_id = p("frame_id", "spectrograph_optical_frame")
        self.preview_period = 1.0 / max(p("preview_rate", 2.0), 0.01)
        self.preview_scale = max(1, p("preview_scale", 4))
        self.reference_dir = os.path.expanduser(p("reference_dir", os.path.join(DATA_DIR, "references")))
        self.max_queue_bytes = int(p("max_queue_mb", 256)) << 20

        self.calibration, self.binner = None, None
        if not calibration and self.source_kind == "fake":
            calibration = self.fake_calibration()
        if calibration:
            self.calibration = load_calibration(os.path.expanduser(calibration))
            self.binner = LineBinner(self.calibration, self.slit_bins, nm_step or None,
                                     (nm_min, nm_max) if nm_max > nm_min else None)
            if self.binner.raw_shape != (self.height, self.width):
                raise RuntimeError("the calibration in %s is for %dx%d frames, and the camera is set to %dx%d"
                                   % (calibration, self.binner.raw_shape[1], self.binner.raw_shape[0], self.width,
                                      self.height))
            self.get_logger().info("calibration %s: %d slit bins x %d bands, %.0f..%.0f nm, sensor rows %d..%d"
                                   % (self.calibration.path, self.binner.shape[0], self.binner.shape[1],
                                      self.binner.nm_edges[0], self.binner.nm_edges[-1], *self.binner.rows))
        else:
            self.get_logger().warning("no calibration: lines can't be binned, so every line's raw frame is kept "
                                      "(set calibration:= to an hsical calibration folder)")
        self.raw_every = (0 if self.binner else 1) if raw_every < 0 else raw_every
        if not line_time_us:
            line_time_us = IMX219_MODES.get((self.width, self.height), {}).get("line_time_us", 18.904)
        if slit_rows[0] < 0 or slit_rows[1] < slit_rows[0]:
            slit_rows = self.binner.rows if self.binner else (0, self.height - 1)
        self.timing = SensorTiming(float(line_time_us), int(slit_rows[0]), int(slit_rows[1]))

        self.matcher = LineMatcher(margin_ns=self.match_margin * 1e9)
        self._lock = threading.Lock()
        self._acc = {}              # (sweep_id, index) -> LineData
        self._sweeps = {}           # sweep_id -> SweepData
        self._finished = deque(maxlen=50)      # sweeps already written
        self._recent_lines = deque(maxlen=8)   # (stamp, hold_until, step), for the fake camera
        self.writer = None
        self.last_references = ""
        self._ref_job = None
        self._skip = 0
        self._last_frame_mono = None
        self._last_preview = 0.0
        self._last_scan_preview = 0.0
        self._warned_no_frames = False
        self.frames_seen = 0
        self.lines_finished = 0

        self.frame_pub = self.create_publisher(FrameStamp, "~/frame", 50)
        self.preview_pub = self.create_publisher(Image, "~/preview", 2)
        self.scan_preview_pub = self.create_publisher(Image, "~/scan_preview", 2)
        services = ReentrantCallbackGroup()
        self.create_subscription(ScanLine, "/scan_mirror/line", self.on_line, 200,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(StartRecording, "~/start_recording", self.handle_start, callback_group=services)
        self.create_service(Trigger, "~/stop_recording", self.handle_stop, callback_group=services)
        self.create_service(CaptureReference, "~/capture_reference", self.handle_reference, callback_group=services)
        self.create_timer(0.5, self.housekeeping, callback_group=MutuallyExclusiveCallbackGroup())
        self.add_on_set_parameters_callback(self.on_parameters)

        self.source = None
        self._source_factory = source_factory or self.make_source
        self._running = True
        self._thread = threading.Thread(target=self._capture, daemon=True)
        self._thread.start()

    # --- the camera ---------------------------------------------------------------------------

    def fake_calibration(self):
        """A made-up spectrograph's calibration for source:=fake, written once."""
        from so101_scan_camera.synthetic import Instrument
        path = os.path.join(FAKE_CALIBRATION, "%dx%d" % (self.width, self.height))
        if not os.path.exists(os.path.join(path, "maps.npz")):
            Instrument(self.width, self.height).save_calibration(path)
        return path

    def make_source(self):
        if self.source_kind == "v4l2":
            extra = (("bypass_mode", self.bypass_mode),) if self.bypass_mode >= 0 else ()
            return V4l2Source(self.device, self.width, self.height, self.fps, self.exposure_us, self.gain,
                              extra_ctrls=extra, raw_shift=self.raw_shift, stamp_offset_us=self.stamp_offset_us,
                              log=self.get_logger().info, **self.ctrls)
        if self.source_kind == "fake":
            from so101_scan_camera.synthetic import Instrument, scene_radiance
            inst = Instrument(self.width, self.height)

            def render(sof_ns, exposure_us, gain):
                # the scene moves on with the mirror's line; a frame exposed while it moved is a blur
                lines = self.line_steps_during(*self.timing.window(sof_ns, exposure_us))
                return inst.render(lambda t, nm: np.mean([scene_radiance(t, nm, s / 2.0) for s in lines], axis=0),
                                   exposure_us, gain)

            return FakeSource(self.width, self.height, self.fps, self.exposure_us, self.gain, render=render,
                              timing=self.timing)
        raise RuntimeError("source:=%s; use v4l2 or fake" % self.source_kind)

    def line_steps_during(self, start_ns, end_ns):
        """For the fake camera: the mirror steps of the lines a window overlaps (one if it held
        still the whole time), from the ScanLines heard so far."""
        steps = []
        for stamp, hold, step in list(self._recent_lines):
            if stamp <= end_ns and hold >= start_ns:
                if stamp <= start_ns and end_ns <= hold:
                    return [step]
                steps.append(step)
        if not steps and self._recent_lines:
            last = self._recent_lines[-1]
            steps = [last[2]] if start_ns >= last[0] else [last[2] - 1, last[2]]
        return steps or [0]

    def _capture(self):
        while self._running:
            if self.source is None:
                try:
                    source = self._source_factory()
                    source.start()
                    self.source = source
                    self.get_logger().info("camera started: %s %dx%d at %.1f fps, exposure %.0f us, gain %.2f"
                                           % (self.source_kind, self.width, self.height, self.fps,
                                              self.exposure_us, self.gain))
                except Exception as e:
                    self.get_logger().error("can't start the camera (%s); trying again" % e,
                                            throttle_duration_sec=30.0)
                    time.sleep(2.0)
                    continue
            try:
                frame = self.source.read(0.5)
            except Exception as e:
                self.get_logger().error("reading the camera failed: %s; restarting it" % e)
                self._drop_source()
                continue
            if frame is None:
                if self._last_frame_mono and time.monotonic() - self._last_frame_mono > 3.0 \
                        and not self._warned_no_frames:
                    self.get_logger().warning("no frames from the camera for 3 s")
                    self._warned_no_frames = True
                continue
            self._last_frame_mono = time.monotonic()
            self._warned_no_frames = False
            try:
                self.on_frame(frame)
            except Exception as e:
                self.get_logger().error("handling a frame failed: %s" % e, throttle_duration_sec=5.0)

    def _drop_source(self):
        source, self.source = self.source, None
        if source is not None:
            try:
                source.stop()
            except Exception:
                pass

    def on_frame(self, f):
        self.frames_seen += 1
        start, end = self.timing.window(f.sof_ns, f.exposure_us)
        msg = FrameStamp()
        msg.header.stamp = Time(nanoseconds=f.sof_ns).to_msg()
        msg.header.frame_id = self.frame_id
        msg.sequence = int(f.sequence)
        msg.exposure_start = Time(nanoseconds=start).to_msg()
        msg.exposure_end = Time(nanoseconds=end).to_msg()
        msg.exposure = f.exposure_us * 1e-6
        msg.gain = float(f.gain)
        self.frame_pub.publish(msg)
        if self._skip > 0:             # the sensor is still taking a new exposure or gain
            self._skip -= 1
            return
        job = self._ref_job
        if job is not None:
            job.add(f.image)
        now = time.monotonic()
        if now - self._last_preview >= self.preview_period:
            self._last_preview = now
            self.publish_preview(f.image)
        info = FrameInfo(f.sequence, f.sof_ns, start, end, f.exposure_us, f.gain, payload=f.image)
        with self._lock:
            line, done = self.matcher.add_frame(info)
        if line is not None:
            self.take(line, info)
        for ln in done:
            self.finish(ln)

    # --- lines --------------------------------------------------------------------------------

    def on_line(self, msg):
        line = Line(msg.sweep_id, msg.index, stamp_ns(msg.header.stamp), stamp_ns(msg.hold_until), msg.settled,
                    msg.last, msg.angle)
        self._recent_lines.append((line.stamp_ns, line.hold_until_ns, msg.step))
        with self._lock:
            sweep = self._sweeps.setdefault(line.sweep_id, SweepData())
            sweep.touched = time.monotonic()
            matched, done = self.matcher.add_line(line, self.get_clock().now().nanoseconds)
        for ln, f in matched:
            self.take(ln, f)
        for ln in done:
            self.finish(ln)

    def take(self, line, frame):
        """A frame exposed while the mirror held still on a line: bin it and keep it with the line."""
        image, frame.payload = frame.payload, None
        if self.binner is not None:
            binned, saturated = self.binner.bin(image)
        else:
            r0, r1 = self.timing.r0, self.timing.r1
            binned, saturated = None, int(np.count_nonzero(image[r0:r1 + 1] >= SATURATED))
        with self._lock:
            writer = self.writer
            keep = writer is not None and writer.keep_raw(line.index)
            self._acc.setdefault((line.sweep_id, line.index), LineData()).add(binned, saturated,
                                                                               image if keep else None)

    def finish(self, line):
        """A line no more frames can come for: into its sweep, and into the recording."""
        with self._lock:
            acc = self._acc.pop((line.sweep_id, line.index), None)
            if line.sweep_id in self._finished:
                self.get_logger().warning("sweep %d line %d came after its sweep was written; left out"
                                          % (line.sweep_id, line.index))
                return
            sweep = self._sweeps.setdefault(line.sweep_id, SweepData())
            sweep.lines[line.index] = acc.mean() if acc else None
            sweep.touched = time.monotonic()
            writer = self.writer
        status = line.status
        if writer is not None:
            status = writer.write_line(line, acc.saturated if acc else 0, acc.raw if acc else None,
                                       binned=self.binner is not None)
        sweep.counts[status] = sweep.counts.get(status, 0) + 1
        self.lines_finished += 1
        self.publish_scan_preview(sweep)
        if line.last:
            self.finish_sweep(line.sweep_id)

    def finish_sweep(self, sweep_id):
        with self._lock:
            sweep = self._sweeps.pop(sweep_id, None)
            pending = [k for k in self._acc if k[0] == sweep_id]
            writer = self.writer
            if sweep is not None:
                self._finished.append(sweep_id)
        if sweep is None:
            return
        n = max(sweep.lines) + 1 if sweep.lines else 0
        counts = ", ".join("%d %s" % (v, k) for k, v in sorted(sweep.counts.items()))
        self.get_logger().info("sweep %d: %d lines (%s)%s" % (sweep_id, n, counts or "none",
                                                              "" if writer else "; not recording"))
        if pending:
            self.get_logger().warning("sweep %d ended with %d lines still waiting for frames" % (sweep_id, len(pending)))
        if writer is not None and self.binner is not None and n:
            arr = np.full((n,) + self.binner.shape, np.nan, np.float32)
            for i, v in sweep.lines.items():
                if v is not None:
                    arr[i] = v
            writer.write_sweep(sweep_id, arr)

    def housekeeping(self):
        now = self.get_clock().now().nanoseconds
        with self._lock:
            done = self.matcher.expire(now)
        for ln in done:
            self.finish(ln)
        with self._lock:
            waiting = {ln.sweep_id for ln in self.matcher.lines}
            idle = [sid for sid, s in self._sweeps.items()
                    if sid not in waiting and time.monotonic() - s.touched > 3.0]
        for sid in idle:          # a sweep stopped before its last line
            self.finish_sweep(sid)

    # --- previews -----------------------------------------------------------------------------

    def image_msg(self, a):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = a.shape
        msg.encoding, msg.is_bigendian, msg.step = "mono8", 0, a.shape[1]
        msg.data = np.ascontiguousarray(a, np.uint8).tobytes()
        return msg

    def publish_preview(self, image):
        if self.preview_pub.get_subscription_count() == 0:
            return
        s = self.preview_scale
        self.preview_pub.publish(self.image_msg((image[::s, ::s] >> 2).astype(np.uint8)))

    def publish_scan_preview(self, sweep):
        now = time.monotonic()
        if self.binner is None or now - self._last_scan_preview < 0.2 \
                or self.scan_preview_pub.get_subscription_count() == 0:
            return
        self._last_scan_preview = now
        with self._lock:
            lines = dict(sweep.lines)
        if not lines:
            return
        n = max(lines) + 1
        img = np.zeros((n, self.binner.shape[0]))
        for i, v in lines.items():
            if v is not None:
                ok = np.isfinite(v)
                img[i] = np.where(ok, v - BLACK_LEVEL, 0.0).sum(axis=1) / np.maximum(ok.sum(axis=1), 1)
        top = float(np.percentile(img, 99.5)) or 1.0
        self.scan_preview_pub.publish(self.image_msg(np.clip(img / top * 255.0, 0, 255)))

    # --- parameters ---------------------------------------------------------------------------

    def on_parameters(self, params):
        changes = {}
        for prm in params:
            if prm.name in ("exposure_us", "gain"):
                if prm.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER) or prm.value <= 0:
                    return SetParametersResult(successful=False, reason="%s must be a positive number" % prm.name)
                changes[prm.name] = float(prm.value)
        if changes:
            self.exposure_us = changes.get("exposure_us", self.exposure_us)
            self.gain = changes.get("gain", self.gain)
            source = self.source
            if source is not None:
                try:
                    source.set_exposure(self.exposure_us, self.gain)
                except Exception as e:
                    return SetParametersResult(successful=False, reason=str(e))
            self._skip = self.settle_frames
            self.get_logger().info("exposure %.0f us, gain %.2f" % (self.exposure_us, self.gain))
        return SetParametersResult(successful=True)

    # --- recordings ---------------------------------------------------------------------------

    def camera_info(self):
        info = dict(
            source=self.source_kind, device=self.device if self.source_kind == "v4l2" else None,
            sensor=dict(width=self.width, height=self.height, bits=10, black_level=BLACK_LEVEL, bayer="RGGB",
                        fps=self.fps, line_time_us=self.timing.line_time_us,
                        slit_rows=[self.timing.r0, self.timing.r1]),
            exposure_us=self.exposure_us, gain=self.gain, slit_reversed=bool(self.slit_reversed),
            calibration=None, binning=None,
            timing=dict(stamps="ROS time; sof_ns is when a frame's row 0 was read out, and its exposure window "
                               "covers the slit rows from the first one's start to the last one's read-out",
                        match_margin_us=self.match_margin * 1e6, stamp_offset_us=self.stamp_offset_us))
        if self.calibration is not None:
            b = self.binner
            info["calibration"] = dict(path=self.calibration.path, maps_sha256=self.calibration.sha256,
                                       temp_k=self.calibration.temp_k)
            info["binning"] = dict(file="binned/binning.npz", slit_bins=b.shape[0], bands=b.shape[1],
                                   nm_range=[float(b.nm_edges[0]), float(b.nm_edges[-1])],
                                   nm_step=float(b.nm_edges[1] - b.nm_edges[0]),
                                   values="mean raw counts per cell (black level included)")
        return info

    def handle_start(self, request, response):
        directory = os.path.expanduser(request.directory)
        if not directory:
            response.success, response.message = False, "no directory given"
            return response
        if self._last_frame_mono is None or time.monotonic() - self._last_frame_mono > 2.0:
            response.success, response.message = False, "no frames from the camera"
            return response
        self.close_recording()
        try:
            writer = sess.SessionWriter(directory, self.camera_info(), self.binner, self.raw_every,
                                        self.max_queue_bytes, log=self.get_logger().error).start()
        except OSError as e:
            response.success, response.message = False, "can't write to %s: %s" % (directory, e)
            return response
        with self._lock:
            self.writer = writer
        response.success, response.calibrated, response.raw = True, self.binner is not None, self.raw_every > 0
        response.references = self.last_references
        response.message = "recording into %s (%s)" % (writer.dir, "binned lines" if self.binner else "raw frames only")
        self.get_logger().info(response.message)
        return response

    def close_recording(self):
        """Finish every line and sweep still open, then the recording. Returns its summary."""
        with self._lock:
            done = self.matcher.flush()
        for ln in done:
            self.finish(ln)
        with self._lock:
            sweeps = list(self._sweeps)
        for sid in sweeps:
            self.finish_sweep(sid)
        with self._lock:
            writer, self.writer = self.writer, None
        return (writer, writer.close()) if writer is not None else (None, None)

    def handle_stop(self, _request, response):
        writer, summary = self.close_recording()
        if writer is None:
            response.success, response.message = False, "not recording"
            return response
        response.success = True
        lines = ", ".join("%d %s" % (summary[k], k) for k in ("ok", "no_frame", "unsettled", "dropped")
                          if summary.get(k))
        response.message = "wrote %s: lines %s%s%s" % (
            writer.dir, lines or "none", "; %d raw frames skipped (the disk fell behind)" % summary["raw_skipped"]
            if summary["raw_skipped"] else "", "; %d write errors" % summary["errors"] if summary["errors"] else "")
        self.get_logger().info(response.message)
        return response

    def handle_reference(self, request, response):
        kind = request.kind.strip().lower()
        if kind not in ("dark", "white"):
            response.success, response.message = False, "kind must be dark or white"
            return response
        name = request.name.strip() or kind
        if os.sep in name or name.startswith("."):
            response.success, response.message = False, "name must be a plain folder name"
            return response
        n = int(request.frames) or 16
        with self._lock:
            writer = self.writer
        directory = os.path.expanduser(request.directory) if request.directory else (
            writer.dir if writer is not None else self.reference_dir)
        if self.source is None:
            response.success, response.message = False, "the camera isn't running"
            return response
        job = ReferenceJob(n)
        self._ref_job = job
        try:
            if not job.done.wait(n / max(self.fps, 1.0) * 2.0 + 5.0):
                response.success, response.message = False, "only %d of %d frames came" % (len(job.frames), n)
                return response
        finally:
            self._ref_job = None
        frames = job.frames
        mean = np.mean(np.stack(frames).astype(np.float32), axis=0)
        r0, r1 = self.timing.r0, self.timing.r1
        rows = mean[r0:r1 + 1]
        peak = (float(np.percentile(rows, 99.9)) - BLACK_LEVEL) / (FULL_SCALE - BLACK_LEVEL)
        saturated = float(np.mean(np.max(np.stack(frames)[:, r0:r1 + 1], axis=0) >= SATURATED))
        meta = dict(kind=kind, exposure_us=self.exposure_us, gain=self.gain, width=self.width, height=self.height,
                    captured=time.strftime("%Y-%m-%d %H:%M:%S"),
                    levels=dict(peak_frac=round(peak, 4), saturated=round(saturated, 6)))
        binned = None
        if self.binner is not None:
            binned = np.mean([self.binner.bin(f)[0] for f in frames], axis=0)
        try:
            if writer is not None and os.path.abspath(directory) == writer.dir:
                path = writer.write_reference(name, frames, meta, binned)
            else:
                path = sess.write_reference(os.path.join(directory, "reference", name), frames, meta)
                if binned is not None:
                    os.makedirs(os.path.join(directory, "binned"), exist_ok=True)
                    np.save(os.path.join(directory, "binned", "reference_%s.npy" % name), binned.astype(np.float32))
                    if not os.path.exists(os.path.join(directory, "binned", "binning.npz")):
                        self.binner.save(os.path.join(directory, "binned", "binning.npz"))
                self.last_references = os.path.abspath(directory)
        except OSError as e:
            response.success, response.message = False, "can't write the reference: %s" % e
            return response
        response.success, response.path = True, path
        response.peak_fraction, response.saturated = peak, saturated
        advice = ""
        if kind == "white":
            if saturated > 1e-4:
                advice = "; it saturates: lower exposure_us"
            elif peak < 0.4:
                advice = "; dim: raise exposure_us to bring the peak to 0.5-0.9"
        response.message = "%s: %d frames, peak %.0f%% of full scale, %.3f%% saturated%s" % (
            path, n, 100 * peak, 100 * saturated, advice)
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=3.0)
        try:
            self.close_recording()
        finally:
            self._drop_source()
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineCamera()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
