"""line_camera with the mirror bridge and the simulated ESP32, in one process.

The camera is simulated too, but it renders what the mirror really showed during each frame's
exposure (the fake ESP32's position at every instant, not the bridge's idea of it), so a frame
exposed while the mirror moved comes out as a blur of two lines. With the bridge locked to the
camera's frames, every line has to get a frame, and every binned line has to be its own line's
scene and nothing of its neighbours'.
"""

import csv
import json
import os
import threading
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from so101_scan_camera import spectral
from so101_scan_camera.binning import LineBinner, load_calibration
from so101_scan_camera.camera_node import LineCamera
from so101_scan_camera.sources import FakeSource, SensorTiming
from so101_scan_camera.synthetic import Instrument, halogen, scene_radiance
from so101_scan_interfaces.msg import FrameStamp, ScanLine
from so101_scan_interfaces.srv import CaptureReference, StartRecording, StartSweep
from so101_scan_sweep.bridge import ScanMirrorBridge
from so101_scan_sweep.fake_mirror import FakeMirror

WIDTH, HEIGHT, SLIT_BINS = 164, 124, 64
EXPOSURE_US = 4000.0
N_LINES = 20


class Client:
    def __init__(self, context):
        self.node = rclpy.create_node("camera_test", context=context)
        self.lines, self.frames = [], []
        self.node.create_subscription(ScanLine, "/scan_mirror/line", self.lines.append, 100)
        self.node.create_subscription(FrameStamp, "/line_camera/frame", self.frames.append, 100)
        self.clients = {
            "home": self.node.create_client(Trigger, "/scan_mirror/home"),
            "start_sweep": self.node.create_client(StartSweep, "/scan_mirror/start_sweep"),
            "start_recording": self.node.create_client(StartRecording, "/line_camera/start_recording"),
            "stop_recording": self.node.create_client(Trigger, "/line_camera/stop_recording"),
            "capture_reference": self.node.create_client(CaptureReference, "/line_camera/capture_reference"),
        }

    def call(self, name, request, timeout=30.0):
        client = self.clients[name]
        assert client.wait_for_service(timeout_sec=5.0), name
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        assert done.wait(timeout), name
        return future.result()

    @staticmethod
    def wait_for(predicate, timeout=10.0, what="it"):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, "timed out waiting for %s" % what
            time.sleep(0.02)


@pytest.fixture
def rig(tmp_path):
    inst = Instrument(WIDTH, HEIGHT, electrons_per_count=20.0)
    cal = inst.save_calibration(str(tmp_path / "cal"))
    binner = LineBinner(load_calibration(cal), SLIT_BINS)
    fake = FakeMirror(link=str(tmp_path / "mirror"), drift_ppm=50.0, seed=2).start()
    timing = SensorTiming(18.904, *binner.rows)          # what line_camera works out from the calibration
    exposures = []
    scene = [scene_radiance]          # what the camera looks at: radiance(t, nm, line)

    def render(sof_ns, exposure_us, gain):
        # the scene at every instant of the exposure: a blur if the mirror moved during it
        start, end = timing.window(sof_ns, exposure_us)
        steps = [fake.pos_at_wall(int(t)) for t in np.linspace(start, end, 9)]
        exposures.append((start, end, steps))
        return inst.render(lambda t, nm: np.mean([scene[0](t, nm, s / 2.0) for s in steps], axis=0),
                           exposure_us, gain)

    context = rclpy.Context()
    rclpy.init(context=context, domain_id=int(os.environ.get("TEST_DOMAIN_ID", 78)))
    bridge = ScanMirrorBridge(context=context, parameter_overrides=[
        Parameter("port", value=fake.link), Parameter("ping_period", value=0.2)])
    camera = LineCamera(
        source_factory=lambda: FakeSource(WIDTH, HEIGHT, 30.0, EXPOSURE_US, 1.0, render=render, timing=timing,
                                          period_error_ppm=-80.0),
        context=context, parameter_overrides=[
            Parameter("source", value="fake"), Parameter("width", value=WIDTH), Parameter("height", value=HEIGHT),
            Parameter("calibration", value=cal), Parameter("slit_bins", value=SLIT_BINS),
            Parameter("exposure_us", value=EXPOSURE_US), Parameter("reference_dir", value=str(tmp_path / "refs"))])
    client = Client(context)
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    for node in (bridge, camera, client.node):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client.wait_for(lambda: bridge.clock_sync.ready and bridge.frames.fit() is not None, 15.0,
                    "the bridge's clock sync and frame fit")
    yield dict(client=client, bridge=bridge, camera=camera, fake=fake, inst=inst, binner=binner,
               exposures=exposures, tmp=tmp_path, scene=scene)
    executor.shutdown()
    camera.destroy_node()
    bridge.destroy_node()
    client.node.destroy_node()
    rclpy.shutdown(context=context)
    fake.stop()


def expected_line(rig, step):
    clean = Instrument(WIDTH, HEIGHT)
    raw = clean.render(lambda t, nm: scene_radiance(t, nm, step / 2.0), EXPOSURE_US, 1.0, noise=False)
    return rig["binner"].bin(raw)[0]


def sweep(client, start, n, period):
    deadline = time.monotonic() + 3.0
    while True:   # the last sweep ends a line period after its last line
        res = client.call("start_sweep", StartSweep.Request(start_angle=start, steps_per_line=2, n_lines=n,
                                                            line_period=period))
        if res.accepted or "still running" not in res.message or time.monotonic() > deadline:
            break
        time.sleep(0.05)
    assert res.accepted, res.message
    client.wait_for(lambda: any(m.sweep_id == res.sweep_id and m.last for m in client.lines), 10.0, "the sweep")
    return res, [m for m in client.lines if m.sweep_id == res.sweep_id]


def check_lines(rig, scan, rows, res, lines):
    """frames.csv's rows and the binned lines of one sweep: each line that got a frame is its
    own step's scene, none of it blurred into the next line's."""
    binned = np.load(scan / "binned" / ("sweep_%03d.npy" % res.sweep_id))
    assert binned.shape == (len(lines),) + rig["binner"].shape
    for r, m in zip(rows, lines):
        assert (int(r["sweep_id"]), int(r["index"])) == (m.sweep_id, m.index)
        if r["status"] != "ok":
            assert r["status"] == "no_frame" and np.isnan(binned[m.index]).all()
            continue
        stamp = rclpy.time.Time.from_msg(m.header.stamp).nanoseconds
        hold = rclpy.time.Time.from_msg(m.hold_until).nanoseconds
        assert stamp < int(r["exposure_start_ns"]) < int(r["exposure_end_ns"]) < hold
        assert np.isfinite(binned[m.index]).all()
        own = np.median(np.abs(binned[m.index] / expected_line(rig, m.step) - 1))
        near = min(np.median(np.abs(binned[m.index] / expected_line(rig, m.step + d) - 1)) for d in (-2, 2))
        assert own < 0.03 and near > 3 * own, (m.index, own, near)


def test_sweeps_record_every_line_cleanly(rig):
    client, scan = rig["client"], rig["tmp"] / "scan"
    assert client.call("home", Trigger.Request()).success
    res = client.call("start_recording", StartRecording.Request(directory=str(scan)))
    assert res.success and res.calibrated and not res.raw, res.message

    # locked to the camera: a frame a line, every one of them still while exposed
    locked, locked_lines = sweep(client, -0.02, N_LINES, 0.0333)
    assert locked.frame_locked and locked.frames_per_line == 1, locked.message
    # not locked: lines on their own clock, so some frames catch the mirror moving
    rig["bridge"].frame_lock = False
    free, free_lines = sweep(client, 0.0, N_LINES, 0.04)
    assert not free.frame_locked and free.line_period == pytest.approx(0.04)
    time.sleep(0.3)   # the last line's frame
    ref = client.call("capture_reference", CaptureReference.Request(kind="dark", frames=4))
    assert ref.success and ref.path == str(scan / "reference" / "dark"), ref.message
    stop = client.call("stop_recording", Trigger.Request())
    assert stop.success and stop.message.startswith("wrote %s: lines " % scan), stop.message

    with open(scan / "frames" / "frames.csv") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 2 * N_LINES
    assert [r["status"] for r in rows[:N_LINES]] == ["ok"] * N_LINES
    assert all(int(r["n_frames"]) == 1 and r["file"] == "" for r in rows[:N_LINES])
    check_lines(rig, scan, rows[:N_LINES], locked, locked_lines)
    check_lines(rig, scan, rows[N_LINES:], free, free_lines)
    # the frames exposed while the mirror moved were the ones left out
    t0 = rclpy.time.Time.from_msg(free_lines[0].header.stamp).nanoseconds
    t1 = rclpy.time.Time.from_msg(free_lines[-1].hold_until).nanoseconds
    moved = [e for e in rig["exposures"] if len(set(e[2])) > 1 and t0 <= e[0] <= t1]
    used = {int(r["exposure_start_ns"]) for r in rows if r["status"] == "ok"}
    assert moved and not used & {start for start, _, _ in moved}

    info = json.loads((scan / "frames" / "camera.json").read_text())
    assert info["lines"]["ok"] >= N_LINES and info["binning"]["slit_bins"] == SLIT_BINS
    assert (scan / "binned" / "binning.npz").exists() and (scan / "binned" / "reference_dark.npy").exists()


def test_the_sweep_shows_in_colour_as_it_comes(rig):
    """With a white taken, the waterfalls are the scene's reflectance in true colour and CIR."""
    client = rig["client"]
    got = {}
    for mode in ("true_color", "cir"):
        client.node.create_subscription(Image, "/line_camera/scan_preview_" + mode,
                                        lambda m, k=mode: got.__setitem__(k, m), 5)
    assert client.call("home", Trigger.Request()).success
    rig["scene"][0] = lambda t, nm, line: 0.98 * halogen(nm)        # the PTFE sheet under the lamp
    time.sleep(0.3)
    ref = client.call("capture_reference", CaptureReference.Request(kind="white", frames=4))
    assert ref.success, ref.message
    rig["scene"][0] = scene_radiance
    time.sleep(0.3)
    res, lines = sweep(client, -0.02, N_LINES, 0.0333)
    assert res.frame_locked

    def full(m):
        return m is not None and m.height == N_LINES and np.frombuffer(bytes(m.data), np.uint8)[-3 * SLIT_BINS:].any()
    client.wait_for(lambda: full(got.get("true_color")) and full(got.get("cir")), 5.0, "the colour waterfalls")
    t = (np.arange(SLIT_BINS) + 0.5) / SLIT_BINS
    nm = 500.0 + 10.0 * np.arange(46)
    for mode, msg in got.items():
        assert (msg.encoding, msg.width, msg.step) == ("rgb8", SLIT_BINS, 3 * SLIT_BINS)
        img = np.frombuffer(bytes(msg.data), np.uint8).reshape(N_LINES, SLIT_BINS, 3).astype(int)
        w = spectral.weights(mode, nm)
        want = np.stack([spectral.srgb8(scene_radiance(t[:, None], nm[None, :], m.step / 2.0) / halogen(nm) @ w)
                         for m in sorted(lines, key=lambda m: m.index)]).astype(int)
        err = np.abs(img - want)
        # a small, noisy sensor: a few slit bins stray (most in blue, held from the 500 nm band)
        assert np.median(err) <= 3 and np.percentile(err, 90) <= 20, (mode, np.median(err), np.percentile(err, 90))
        np.testing.assert_allclose(img.mean(axis=1), want.mean(axis=1), atol=6)     # each line's mean colour


def test_without_frames_recording_is_refused(rig, tmp_path):
    client, camera = rig["client"], rig["camera"]
    camera._drop_source()           # the camera stops delivering
    camera._running = False
    time.sleep(2.2)
    res = client.call("start_recording", StartRecording.Request(directory=str(tmp_path / "x")))
    assert not res.success and "no frames" in res.message
    assert not client.call("stop_recording", Trigger.Request()).success
