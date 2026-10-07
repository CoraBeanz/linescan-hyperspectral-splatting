"""Shared fixtures: one small simulated scan session, rendered and calibrated once per run."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hsisim import repo  # noqa: E402,F401  (hsical and the ROS 2 modules)
from hsisim.session import Settings, load_plan, simulate  # noqa: E402

# two views of the relief target, 12 lines each, spread across it
PLAN = dict(plan="ring", views=["down", "tilt25_az90"], lines=12, steps_per_line=16)


@pytest.fixture(scope="session")
def scan(tmp_path_factory):
    """(folder, scan.json) of a binning-4 session with its calibration session."""
    base = tmp_path_factory.mktemp("scan")
    plan = load_plan(PLAN["plan"], lines=PLAN["lines"], steps_per_line=PLAN["steps_per_line"], views=PLAN["views"])
    info = simulate(base, plan, Settings(binning=4), log=lambda *a: None)
    return base, info


@pytest.fixture(scope="session")
def calibrated(scan):
    """(folder, calibration folder, hsical Result) for the scan's calibration session."""
    from hsical.pipeline import calibrate
    base, _ = scan
    result = calibrate(base / "calibration", base / "cal", log=lambda *a: None)
    return base, base / "cal", result
