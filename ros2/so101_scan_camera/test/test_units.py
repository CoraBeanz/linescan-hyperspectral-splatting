"""The parts that need no ROS graph: V4L2 numbers, raw decoding, binning, matching, writing."""

import csv
import ctypes
import json

import numpy as np
import pytest

from so101_scan_camera import v4l2
from so101_scan_camera.binning import SATURATED, LineBinner, Orientation, load_calibration
from so101_scan_camera.matcher import FrameInfo, Line, LineMatcher
from so101_scan_camera.session import SessionWriter
from so101_scan_camera.sources import BLACK_LEVEL, FakeSource, SensorTiming, decode_rg10, guess_shift
from so101_scan_camera.synthetic import Instrument

MS = 1_000_000


# --- V4L2 --------------------------------------------------------------------------------------

@pytest.mark.skipif(ctypes.sizeof(ctypes.c_void_p) != 8, reason="the numbers below are for 64-bit Linux")
def test_ioctl_numbers_match_the_kernels():
    # linux/videodev2.h on x86_64 and aarch64 (the Jetson)
    assert v4l2.VIDIOC_QUERYCAP == 0x80685600
    assert v4l2.VIDIOC_G_FMT == 0xC0D05604 and v4l2.VIDIOC_S_FMT == 0xC0D05605
    assert v4l2.VIDIOC_REQBUFS == 0xC0145608 and v4l2.VIDIOC_QUERYBUF == 0xC0585609
    assert v4l2.VIDIOC_QBUF == 0xC058560F and v4l2.VIDIOC_DQBUF == 0xC0585611
    assert v4l2.VIDIOC_STREAMON == 0x40045612 and v4l2.VIDIOC_STREAMOFF == 0x40045613
    assert v4l2.VIDIOC_G_EXT_CTRLS == 0xC0205647 and v4l2.VIDIOC_S_EXT_CTRLS == 0xC0205648
    assert v4l2.VIDIOC_QUERY_EXT_CTRL == 0xC0E85667
    assert v4l2.fourcc("RG10") == 0x30314752


def test_rg10_decoding_finds_where_the_bits_are():
    rng = np.random.default_rng(0)
    img = (BLACK_LEVEL + rng.normal(0, 2, (40, 60))).round().astype(np.uint16)
    img[10:30, 20:50] = rng.integers(100, 1000, (20, 30))
    for shift in (0, 6):
        words = np.zeros((40, 64), np.uint16)            # rows padded to 128 bytes
        words[:, :60] = img << shift
        data = words.tobytes()
        assert guess_shift(decode_rg10(data, 60, 40, 128, None)) == shift
        np.testing.assert_array_equal(decode_rg10(data, 60, 40, 128, shift), img)
        # a driver whose bytesperline doesn't match its buffer
        np.testing.assert_array_equal(decode_rg10(data, 60, 40, 100, shift), img)


def test_fake_source_frames_follow_the_clock():
    now = [10 * 10**9]
    src = FakeSource(16, 12, fps=50.0, jitter_us=0.0, clock=lambda: now[0], latency_s=0.0)
    src.start()                              # the first frame starts 50 ms from now
    assert src.read(timeout=0.0) is None
    now[0] += 51 * MS
    stamps = []
    for _ in range(4):
        f = src.read(timeout=0.0)
        stamps.append(f.sof_ns)
        now[0] += 20 * MS
    assert np.diff(stamps).tolist() == [20 * MS] * 3 and stamps[0] == 10 * 10**9 + 50 * MS
    t = SensorTiming(20.0, 10, 30)
    assert t.window(1_000_000, 5000.0) == (1_000_000 + 200_000 - 5_000_000, 1_000_000 + 600_000)


# --- binning -------------------------------------------------------------------------------------

@pytest.mark.parametrize("orientation", [Orientation(), Orientation(transpose=True, flip_x=True),
                                         Orientation(flip_x=True, flip_y=True)])
def test_binning_is_the_mean_of_each_cells_pixels(tmp_path, orientation):
    inst = Instrument(60, 48, orientation=orientation, nm_range=(480, 960), seed=4)
    maps = load_calibration(inst.save_calibration(str(tmp_path)))
    assert maps.raw_shape == (48, 60)
    b = LineBinner(maps, slit_bins=12, nm_step=20.0, nm_range=(500.0, 940.0))
    assert b.shape == (12, 22)
    raw = inst.render(lambda t, nm: 0.3 + 0.5 * t * (nm / 1000.0), 5000.0)
    raw[raw.shape[0] // 2, 3] = 1023                       # might be in a cell, might not
    got, saturated = b.bin(raw)

    # the same thing the slow way, in the calibration's turned frame
    turned = orientation.apply(raw).astype(float)
    i_s = np.searchsorted(b.slit_edges, maps.slit, side="right") - 1
    i_nm = np.searchsorted(b.nm_edges, maps.wavelength, side="right") - 1
    ok = (i_s >= 0) & (i_s < 12) & (i_nm >= 0) & (i_nm < 22) & ~maps.bad
    want = np.full(b.shape, np.nan)
    hot = 0
    for s in range(12):
        for k in range(22):
            cell = ok & (i_s == s) & (i_nm == k)
            if cell.any():
                v = turned[cell]
                hot += int((v >= SATURATED).sum())
                want[s, k] = np.nan if (v >= SATURATED).any() else v.mean()
    np.testing.assert_allclose(got, want, rtol=1e-6, equal_nan=True)
    assert saturated == hot
    assert np.array_equal(b.pixels, [[(ok & (i_s == s) & (i_nm == k)).sum() for k in range(22)] for s in range(12)])
    # bad pixels (some of them stuck at full scale) never count
    assert np.isfinite(got).sum() >= got.size - 2 and (raw >= SATURATED).sum() > hot


def test_binning_refuses_a_frame_of_another_size(tmp_path):
    inst = Instrument(60, 48)
    b = LineBinner(load_calibration(inst.save_calibration(str(tmp_path))), slit_bins=8)
    with pytest.raises(ValueError, match="60x48"):
        b.bin(np.zeros((60, 48), np.uint16))


# --- matching frames to lines --------------------------------------------------------------------

def frame(seq, start_ms, end_ms):
    return FrameInfo(seq, int((start_ms - 1) * MS), int(start_ms * MS), int(end_ms * MS), 5000.0, 1.0)


def line(index, stamp_ms, hold_ms, settled=True, last=False):
    return Line(0, index, int(stamp_ms * MS), int(hold_ms * MS), settled, last)


def test_frames_go_to_the_line_they_were_exposed_in():
    m = LineMatcher(margin_ns=0.5 * MS)
    a = line(0, 0, 30)
    assert m.add_line(a) == ([], [])
    assert m.add_frame(frame(1, 5, 12)) == (a, [])
    assert m.add_frame(frame(2, 27, 34))[0] is None          # straddles the move to line 1
    early = frame(3, 40, 47)                                  # line 1's ScanLine is late
    assert m.add_frame(early) == (None, [a])                  # line 0 can't get more frames
    b = line(1, 33, 63)
    matched, done = m.add_line(b)
    assert matched == [(b, early)] and done == []
    assert m.add_frame(frame(4, 62.8, 70))[0] is None         # inside the margin of the next move
    c = line(2, 66, 96, last=True)
    assert m.add_line(c) == ([], [])
    assert m.add_frame(frame(5, 72, 79)) == (c, [b])
    assert [f.sequence for f in a.frames] == [1] and [f.sequence for f in b.frames] == [3]
    assert (a.status, b.status) == ("ok", "ok")
    # frames stop: the line is finished after the timeout, from its hold_until
    assert m.expire(int(96 * MS) + m.line_timeout_ns - 1) == []
    assert m.expire(int(96 * MS) + m.line_timeout_ns + 1) == [c]
    assert c.status == "ok" and m.lines == []


def test_unsettled_and_frameless_lines():
    m = LineMatcher()
    u = line(0, 0, 30, settled=False)
    assert m.add_line(u) == ([], [u]) and u.status == "unsettled"
    m.add_frame(frame(1, 80, 87))
    late = line(1, 33, 63)                                    # its frames are long gone
    assert m.add_line(late) == ([], [late]) and late.status == "no_frame"
    waiting = line(2, 100, 130)
    m.add_line(waiting)
    assert m.flush() == [waiting] and m.lines == []


# --- the session writer --------------------------------------------------------------------------

def test_session_writer(tmp_path):
    w = SessionWriter(str(tmp_path / "scan"), dict(exposure_us=5000.0, gain=1.0), raw_every=2,
                      max_queue_bytes=10 << 20).start()
    img = np.full((12, 16), 70, np.uint16)
    for i in range(4):
        ln = line(i, 33 * i, 33 * i + 30)
        ln.sweep_id = 3
        if i != 1:
            ln.frames.append(frame(i, 33 * i + 5, 33 * i + 12))
        w.write_line(ln, saturated_px=i, raw=img + i if w.keep_raw(i) and i != 1 else None, binned=False)
    w.write_reference("dark", [img, img], dict(kind="dark", exposure_us=5000.0, gain=1.0))
    summary = w.close()
    assert summary == {"ok": 3, "no_frame": 1, "raw_skipped": 0, "errors": 0}
    with open(tmp_path / "scan" / "frames" / "frames.csv") as f:
        rows = list(csv.DictReader(f))
    assert [(r["index"], r["status"], r["file"]) for r in rows] == [
        ("0", "ok", "frames/sweep_003/frame_0000.npy"), ("1", "no_frame", ""), ("2", "ok",
         "frames/sweep_003/frame_0002.npy"), ("3", "ok", "")]
    assert rows[2]["exposure_start_ns"] == str(71 * MS) and rows[2]["n_frames"] == "1"
    np.testing.assert_array_equal(np.load(tmp_path / "scan" / rows[2]["file"]), img + 2)
    meta = json.loads((tmp_path / "scan" / "frames" / "sweep_003" / "meta.json").read_text())
    assert meta["kind"] == "scene" and (meta["width"], meta["height"]) == (16, 12)
    info = json.loads((tmp_path / "scan" / "frames" / "camera.json").read_text())
    assert info["format"] == "so101_scan frames v1" and info["lines"] == {"ok": 3, "no_frame": 1}
    ref = json.loads((tmp_path / "scan" / "reference" / "dark" / "meta.json").read_text())
    assert ref["kind"] == "dark" and ref["frames"] == 2


def test_session_writer_skips_raw_frames_when_the_disk_falls_behind(tmp_path):
    w = SessionWriter(str(tmp_path / "scan"), {}, raw_every=1, max_queue_bytes=1000).start()
    big = np.zeros((40, 40), np.uint16)                       # 3200 bytes: never fits the queue
    ln = line(0, 0, 30)
    ln.frames.append(frame(1, 5, 12))
    assert w.write_line(ln, raw=big, binned=False) == "dropped"
    assert w.close()["raw_skipped"] == 1
