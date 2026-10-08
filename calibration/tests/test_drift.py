"""The drift check: a fresh lamp frame against a saved calibration."""

import json
import subprocess
import sys
from pathlib import Path

import pytest

from hsical.drift import check
from hsical.synth import Truth, write_session

QUIET = dict(log=lambda *a: None)
HERE = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def lamps(synthetic, tmp_path_factory):
    """New CFL frames of the calibrated instrument: as it was, and with the spectrograph knocked.

    The knock moves the spectrum 6 full-sensor pixels along x and 8 along the slit, which is
    1.5 px and 2 rows at the fixture's binning of 4.
    """
    base, _ = synthetic
    out = tmp_path_factory.mktemp("drift")
    write_session(out / "same", Truth(scale=4, frames=4), kinds=["cfl"], **QUIET)
    write_session(out / "moved", Truth(scale=4, frames=4, shift_px=(61.0, -10.0)), kinds=["cfl"], **QUIET)
    return base / "cal", out


def test_unmoved_instrument_passes(lamps):
    cal, out = lamps
    r = check(cal, out / "same" / "cfl", **QUIET)
    assert r.ok, r.to_dict()
    assert abs(r.offset_px) < 0.15, r.offset_px
    assert r.worst_nm < 0.15 and r.rms_nm < 0.08, (r.worst_nm, r.rms_nm)
    assert sum(ln.used for ln in r.lines) >= 20
    assert abs(r.slit_shift) < 0.002, r.slit_shift
    assert r.dark == "dark dark_60000us", r.dark  # found beside the frames, at their exposure


def test_moved_instrument_is_caught(lamps):
    cal, out = lamps
    r = check(cal, out / "moved" / "cfl", **QUIET)
    assert not r.ok and not r.spectral_ok and not r.slit_ok
    assert abs(r.offset_px - 1.5) < 0.15, r.offset_px
    assert abs(r.slit_top_px - 2.0) < 0.4 and abs(r.slit_bottom_px - 2.0) < 0.4, (r.slit_top_px, r.slit_bottom_px)
    # every band of the slit sees the same move, so it's an offset, not a tilt
    assert abs(r.along_slit_nm) < 0.15 and abs(r.across_nm) < 0.15, (r.along_slit_nm, r.across_nm)


def test_command_line(lamps, tmp_path):
    cal, out = lamps

    def run(frames):
        report = tmp_path / f"{frames.parent.name}.json"
        p = subprocess.run([sys.executable, "-m", "hsical", "check", str(cal), str(frames), "--json", str(report)],
                           cwd=HERE, capture_output=True, text=True)
        return p, json.loads(report.read_text())

    p, d = run(out / "same" / "cfl")
    assert p.returncode == 0, p.stdout + p.stderr
    assert "Still calibrated" in p.stdout and d["ok"] is True
    assert {"offset_nm", "worst_nm", "slit_shift", "lines"} <= set(d)

    p, d = run(out / "moved" / "cfl")
    assert p.returncode == 2, p.stdout + p.stderr
    assert "has moved" in p.stdout and d["ok"] is False
