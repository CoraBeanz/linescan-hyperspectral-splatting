"""End to end: calibrate the simulated instrument with hsical, turn the scan into
reflectance, and compare what comes out with what went in."""

import pytest

from hsisim.evaluate import LIMITS, check, evaluate


@pytest.fixture(scope="module")
def errors(calibrated):
    base, cal, result = calibrated
    assert result.ok, [c.to_dict() for c in result.checks if not c.ok]
    return evaluate(base, cal, log=lambda *a: None)


def test_every_limit_holds(errors):
    rows = check(errors)
    assert {r[0] for r in rows} >= set(LIMITS)       # every check ran
    bad = [(n, v, lim) for n, v, lim, ok in rows if not ok]
    assert not bad, bad


def test_reflectance_of_every_material(errors):
    for name, d in errors["per_material"].items():
        assert abs(d["median_error"]) < 0.01, (name, d)


def test_rare_earth_bands_found(errors):
    bands = errors["rare_earth_bands"]
    assert len(bands) >= 5                            # 528 to 874 nm, inside the calibrated range
    assert all(abs(b["error"]) < LIMITS["band_centre_max_nm"] for b in bands)


def test_slit_runs_the_right_way(errors):
    """Rectified slit row 0 is the line camera's -x end: the profile along the slit matches the
    truth's as it is, and not mirrored."""
    assert errors["slit_profile_correlation"] > 0.95
    assert errors["slit_profile_correlation_flipped"] < 0.5
