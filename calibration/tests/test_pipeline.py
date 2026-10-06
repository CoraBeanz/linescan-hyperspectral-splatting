"""End to end: render a session from the design, calibrate it, compare with the truth."""

import json

import numpy as np

from hsical.model import Calibration
from hsical.selftest import evaluate
from hsical.synth import compare


def _check(base, result):
    rows = evaluate(compare(result.calibration, base / "session"))
    bad = [(n, v, lim) for n, v, lim, ok in rows if not ok]
    assert not bad, bad
    assert result.ok, [c.to_dict() for c in result.checks if not c.ok]


def test_matches_truth(synthetic):
    _check(*synthetic)


def test_soft_lens(synthetic_soft):
    base, result = synthetic_soft
    _check(base, result)
    # the line shape search should find the width the laser line shows
    lsf = result.calibration.info["lsf"]
    laser = result.calibration.info["laser"]
    assert abs(lsf["fwhm_px"] / laser["fwhm_px"] - 1) < 0.08, (lsf["fwhm_px"], laser["fwhm_px"])


def test_camera_turned_and_mirrored(synthetic_turned):
    base, result = synthetic_turned
    o = result.calibration.orientation
    assert o.transpose and o.flip_x
    _check(base, result)


def test_saved_calibration_round_trip(synthetic):
    base, result = synthetic
    cal = Calibration.load(base / "cal")
    ref = result.calibration
    x = np.linspace(0, ref.width - 1, 7)
    y = np.linspace(0, ref.height - 1, 5)[:, None]
    assert np.allclose(cal.wavelength_at(x, y), ref.wavelength_at(x, y), atol=1e-4)
    assert np.allclose(cal.slit_at(x, y), ref.slit_at(x, y), atol=1e-6)
    assert np.array_equal(cal.nm_grid, ref.nm_grid.astype(np.float32))
    mx, my = cal.maps()
    assert mx.shape == (ref.s_grid.size, ref.nm_grid.size)
    d = json.loads((base / "cal" / "calibration.json").read_text())
    assert d["format"] == "hsical-1" and len(d["wavelength"]["table_nm"]) == ref.width
    assert (base / "cal" / "report.md").exists()


def test_laser_found(synthetic):
    base, result = synthetic
    truth = json.loads((base / "session" / "truth.json").read_text())
    laser = result.calibration.info["laser"]
    assert abs(laser["nm"] - truth["laser_nm"]) < 0.2
    assert laser["spread_nm"] < 0.1


def test_lines_cover_band(synthetic):
    _, result = synthetic
    lo, hi = result.calibration.info["dispersion"]["nm_covered"]
    assert lo < 550 and hi > 900


def test_long_exposures_add_lines(synthetic):
    # The long neon set is taken at gain 4, where hot pixels sit at full scale.
    # They are masked and filled, so they must not knock out the lines around them.
    _, result = synthetic
    used = [f for f in result.calibration.info["lines"] if f["used"]]
    nir = [f for f in used if f["lamp"] == "neon_long" and f["nm"] > 800]
    assert len(nir) >= 15, len(nir)
