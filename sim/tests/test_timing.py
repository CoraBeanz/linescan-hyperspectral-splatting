"""The camera's and the mirror's clocks: rolling shutter, the frame lock, stamp errors, and which
way the slit runs."""

import json

import numpy as np
import pytest

from hsisim import timing as T
from hsisim.session import Settings, load_plan, simulate
from hsisim.truth import ScanTruth, read_lines_csv

QUIET = dict(log=lambda *a: None)
SHORT = dict(plan="ring", lines=4, steps_per_line=16, views=["down"])


def _camera(**kw):
    return T.Camera(**dict(dict(t0_ns=0, fps=30.0, line_time_us=37.808, r0=20, r1=580, exposure_us=20000.0), **kw))


def test_lock_keeps_every_window_inside_a_hold():
    cam, mirror = _camera(), T.Mirror()
    st = T.plan_sweep(cam, mirror, 1, 10, 16, 1 / 30, not_before_ns=10 ** 9)
    # the slit rows' window is 21 + 20 ms, so a line needs a second frame for the move
    assert st.lock.frames_per_line == 2 and st.line_period_ns == pytest.approx(2e9 / 30)
    for lt in st.lines:
        assert len(lt.frames) == st.lock.good_frames == 1
        f = lt.frames[0]
        assert lt.stamp_ns + 500_000 <= f.start_ns and f.end_ns <= lt.hold_until_ns - 500_000
        assert f.end_ns - f.start_ns == pytest.approx((560 * 37.808 + 20000.0) * 1000, abs=2)
    # held still all the exposure, every row of the frame sees the line's own angle
    angles = np.arange(10) * 0.01
    for k, lt in enumerate(st.lines):
        views = T.mix(cam, st, k, lt.frames[0].sequence, angles, np.arange(616))
        assert len(views) == 1 and views[0][0] == angles[k] and (views[0][1] == 1.0).all()


def test_late_stamps_put_the_first_rows_on_the_move():
    """Stamps 15 ms late: the bridge locks to the stamps, so the frames really exposed 15 ms
    earlier, and the slit's first rows were still exposing while the mirror moved onto the line."""
    cam = _camera(stamp_error_us=15000.0)
    st = T.plan_sweep(cam, T.Mirror(), 1, 4, 16, 1 / 30, not_before_ns=10 ** 9)
    angles = np.arange(4) * 0.01
    views = T.mix(cam, st, 2, st.lines[2].frames[0].sequence, angles, np.arange(616))
    total = sum(w for _, w in views)
    assert np.allclose(total, 1.0)
    still = next(w for a, w in views if a == angles[2])
    assert still[20] < 0.8 and still[580] == 1.0           # row 20 is read out first, 580 last
    assert all(angles[1] <= a <= angles[2] for a, _ in views)
    # line_camera's offset cancels it
    cam = _camera(stamp_error_us=15000.0, stamp_offset_us=-15000.0)
    st = T.plan_sweep(cam, T.Mirror(), 1, 4, 16, 1 / 30, not_before_ns=10 ** 9)
    assert len(T.mix(cam, st, 2, st.lines[2].frames[0].sequence, angles, np.arange(616))) == 1


@pytest.fixture(scope="module")
def late(tmp_path_factory):
    """The same short scan with exact stamps, 15 ms late ones, and late ones corrected."""
    base = tmp_path_factory.mktemp("late")
    plan = load_plan(SHORT["plan"], lines=SHORT["lines"], steps_per_line=SHORT["steps_per_line"], views=SHORT["views"])
    for name, kw in (("exact", {}), ("late", dict(stamp_error_us=15000.0)),
                     ("corrected", dict(stamp_error_us=15000.0, stamp_offset_us=-15000.0))):
        simulate(base / name, plan, Settings(binning=4, calibration=False, **kw), **QUIET)
    return base


def test_late_stamps_blur_the_first_rows(late):
    t = read_lines_csv(late / "late" / "truth" / "frames_true.csv")
    assert t["moving_share"][0] == 0.0                     # line 0 doesn't move onto its line
    assert (t["moving_share"][1:] > 0.2).all()
    cam = json.loads((late / "late" / "frames" / "camera.json").read_text())
    r0, r1 = cam["sensor"]["slit_rows"]
    first, last = slice(r0 + 5, r0 + 90), slice(r1 - 120, r1 - 30)
    for k in range(1, SHORT["lines"]):
        a, b = (np.load(late / x / "frames" / "sweep_001" / f"frame_{k:04d}.npy").astype(float) - 64.0
                for x in ("exact", "late"))
        # each row summed along the spectrum: the light from one place along the slit. Its
        # difference over its shot noise (4 electrons a count) is about 0.8 where the two
        # frames saw the same thing
        pa, pb = a.sum(1), b.sum(1)
        z = np.abs(pa - pb) / np.sqrt(np.maximum(pa + pb, 1.0) / 4.0)
        # the rows read out first saw part of the previous line; the last ones didn't
        assert z[first].mean() > 5.0 and z[last].mean() < 2.0, (k, z[first].mean(), z[last].mean())


def test_corrected_stamps_give_the_exact_session(late):
    for f in ("frames/frames.csv", "lines.csv", "frames/sweep_001/frame_0002.npy"):
        assert (late / "exact" / f).read_bytes() == (late / "corrected" / f).read_bytes(), f
    cam = json.loads((late / "corrected" / "frames" / "camera.json").read_text())
    assert cam["timing"]["stamp_offset_us"] == -15000.0 and cam["simulated"]["stamp_error_us"] == 15000.0
    # the frames.csv stamps are what line_camera logs; the truth has when the frames really exposed
    logged = read_lines_csv(late / "late" / "frames" / "frames.csv")
    true = read_lines_csv(late / "late" / "truth" / "frames_true.csv")
    assert (logged["sof_ns"] - true["sof_ns"] == 15_000_000).all()


@pytest.fixture(scope="module")
def reversed_scan(tmp_path_factory):
    """A short scan with the camera mounted the other way along the slit, and its calibration."""
    from hsical.pipeline import calibrate
    base = tmp_path_factory.mktemp("reversed")
    plan = load_plan("ring", lines=8, steps_per_line=16, views=["down"])
    simulate(base, plan, Settings(binning=4, slit_reversed=True), **QUIET)
    result = calibrate(base / "calibration", base / "cal", **QUIET)
    assert result.ok
    return base, base / "cal"


def _dataset_profile_match(session, cal, out):
    """(correlation, mirrored correlation) of the dataset's near-infrared profile along the slit
    with the truth's, pixel 0 taken as h = -1 (the line camera's -x end)."""
    from so101_scan_camera.scan_to_dataset import convert
    doc = convert(session, out, calibration=str(cal))
    lines = np.load(out / "lines.npy")
    nm = np.array(doc["wavelengths_nm"])
    nir = (nm > 780) & (nm < 920)
    truth = ScanTruth(session)
    h = np.linspace(-1, 1, lines.shape[1])
    fwd, mirrored = [], []
    for k in range(lines.shape[0]):
        want = np.interp(h, truth.h, truth.reflectance(k, nm[nir]).mean(1))
        got = lines[k][:, nir].mean(1)
        ok = np.isfinite(got)
        if want[ok].std() > 0.02:
            fwd.append(np.corrcoef(got[ok], want[ok])[0, 1])
            mirrored.append(np.corrcoef(got[ok], want[::-1][ok])[0, 1])
    return float(np.median(fwd)), float(np.median(mirrored))


def test_slit_reversed_converts_the_right_way(calibrated, reversed_scan, tmp_path):
    """scan_to_dataset puts pixel 0 at the line camera's -x end either way the camera is mounted,
    because camera.json says which way the slit runs."""
    base, cal, _ = calibrated
    rev, rev_cal = reversed_scan
    assert json.loads((base / "frames" / "camera.json").read_text())["slit_reversed"] is False
    assert json.loads((rev / "frames" / "camera.json").read_text())["slit_reversed"] is True
    for session, c, name in ((base, cal, "plain"), (rev, rev_cal, "reversed")):
        fwd, mirrored = _dataset_profile_match(session, c, tmp_path / name)
        assert fwd > 0.95 and fwd > mirrored + 0.3, (name, fwd, mirrored)


def test_reversed_scan_passes_the_end_to_end_check(reversed_scan):
    from hsisim.evaluate import check, evaluate
    session, cal = reversed_scan
    errs = evaluate(session, cal, **QUIET)
    bad = [(n, v, lim) for n, v, lim, ok in check(errs) if not ok]
    assert not bad, bad
