"""Shared fixtures: one synthetic session, rendered and calibrated once per test run."""

import pytest

from hsical.pipeline import calibrate
from hsical.synth import Truth, write_session


def _session(tmp_path_factory, name, **truth):
    base = tmp_path_factory.mktemp(name)
    write_session(base / "session", Truth(scale=4, frames=4, **truth), log=lambda *a: None)
    result = calibrate(base / "session", base / "cal", log=lambda *a: None)
    return base, result


@pytest.fixture(scope="session")
def synthetic(tmp_path_factory):
    """(folder, Result) for a scale-4 session with the camera mounted as designed."""
    return _session(tmp_path_factory, "plain")


@pytest.fixture(scope="session")
def synthetic_turned(tmp_path_factory):
    """The same, with the camera turned 90 degrees and mirrored."""
    return _session(tmp_path_factory, "turned", transpose=True, flip_x=True, seed=11)


@pytest.fixture(scope="session")
def synthetic_soft(tmp_path_factory):
    """A soft camera lens, like a real M12 one: twice the blur, lines about 6 nm wide."""
    return _session(tmp_path_factory, "soft", blur_px=12.0, blur_y_px=5.0, seed=5)
