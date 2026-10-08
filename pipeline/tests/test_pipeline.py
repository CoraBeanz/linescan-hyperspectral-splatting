"""Unit tests for the pipeline's own code: python -m pytest pipeline/tests

The whole chain is tested by running it (pipeline/configs/sim-quick.yaml in CI); these pin down
the pieces that decide pass or fail: the truth's pixel and band averages, which pixels count as
one material, the profile comparison, and reading the tools' output and the config.
"""

import json
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import first_light as fl  # noqa: E402
import sim_truth as st  # noqa: E402


def test_pixel_matrix_averages_each_pixels_stretch_of_the_slit():
    h = np.linspace(-1.06, 1.06, 1177)
    m = st.pixel_matrix(h, 8)
    assert m.shape == (8, 1177)
    assert np.allclose(m.sum(axis=1), 1.0)
    # pixel 0 is the -x end of the slit (h = -1), pixel 7 the +x end
    assert h[m[0] > 0].min() >= -1.0 and h[m[0] > 0].max() < -0.75
    assert h[m[7] > 0].min() >= 0.75 and h[m[7] > 0].max() <= 1.0
    # samples past the ends of the slit belong to no pixel
    assert m[:, h < -1.0].sum() == 0 and m[:, h > 1.0].sum() == 0
    ramp = st.pixel_matrix(h, 4) @ h
    assert np.allclose(ramp, [-0.75, -0.25, 0.25, 0.75], atol=0.01)


def test_pixel_matrix_refuses_a_slit_sampled_too_coarsely():
    with pytest.raises(ValueError):
        st.pixel_matrix(np.linspace(-1, 1, 5), 16)


def test_band_matrix_averages_the_wavelengths_in_each_band():
    nm = np.arange(440.0, 1061.0, 2.0)
    m = st.band_matrix(nm, np.array([500.0, 510.0]), 10.0)
    assert np.allclose(m.sum(axis=0), 1.0)
    assert np.allclose(nm @ m, [500.0, 510.0])
    with pytest.raises(ValueError):
        st.band_matrix(nm, np.array([1200.0]), 10.0)


def edge_weights(h, at=0.1):
    """Two materials (1 and 2) meeting at h = at, nothing (material 0) past |h| = 1."""
    w = np.zeros((len(h), 3))
    w[(h < at) & (np.abs(h) <= 1), 1] = 1.0
    w[(h >= at) & (np.abs(h) <= 1), 2] = 1.0
    w[np.abs(h) > 1, 0] = 1.0
    return w


def test_pure_pixels_keep_away_from_edges_and_the_slit_ends():
    h = np.linspace(-1.06, 1.06, 1177)
    width = 64
    pure = st.pure_pixels(edge_weights(h), h, width, margin_px=3)
    centre = -1 + (np.arange(width) + 0.5) * 2 / width
    px_from_edge = np.abs(centre - 0.1) * width / 2
    # pure only if the pixel's half width plus the margin stays clear of the edge
    assert not pure[px_from_edge < 3.4].any()
    assert pure[(px_from_edge > 3.6) & (np.abs(centre) < 0.85)].all()
    # the margin reaches past the slit's ends, onto the void
    assert not pure[0] and not pure[-1]
    # a pixel mixing two materials is never pure, even with no margin
    mixed = np.argmin(np.abs(centre - 0.1))
    assert not st.pure_pixels(edge_weights(h, at=centre[mixed]), h, width, margin_px=0)[mixed]


def test_profile_shift_and_correlation():
    x = np.arange(128, dtype=float)
    a = np.exp(-0.5 * ((x - 60) / 4) ** 2) + 0.5 * (x > 90)
    b = np.interp(x - 1.3, x, a)   # a moved 1.3 px towards higher pixels
    assert st.profile_shift(a, b) == pytest.approx(1.3, abs=0.15)
    assert st.profile_shift(a, a) == pytest.approx(0.0, abs=1e-9)
    assert st.correlation(a, a) == pytest.approx(1.0)
    assert st.correlation(a, a[::-1]) < 0.5


TRAIN_OUTPUT = """dataset   4 sweeps, 428 lines of 256 px x 46 bands
device    the CPU, Adam on the CPU
start     3366 Gaussians on the board plane, 1.00 mm apart
pose err  3.60 px rms (along the slit 2.58, across 2.51), max 6.28 at the recorded poses
iter  3000  rmse 0.0160   20985 Gaussians (+0 -0)    92 ms/iter  pose 1.690 px
done      20985 Gaussians in 268.4 s; RMSE vs measured lines 0.0162, vs noise-free lines 0.0201
pose err  1.69 px rms (along the slit 1.20, across 1.19), max 3.10 at the refined poses
wrote     out (scene/, sweep_head_pose.npy, log.csv, preview/)
"""


def test_parse_train_reads_splat_trains_summary():
    r = fl.parse_train(TRAIN_OUTPUT)
    assert r["device"] == "the CPU, Adam on the CPU"
    assert r["gaussians"] == 20985 and r["train_seconds"] == 268.4
    assert r["rmse_measured"] == 0.0162 and r["rmse_truth"] == 0.0201
    assert r["pose_error_recorded_px"] == 3.60 and r["pose_error_trained_px"] == 1.69
    assert r["pose_error_trained_along_px"] == 1.20 and r["pose_error_trained_across_px"] == 1.19
    # a real scan has no truth: those numbers are missing, not zero
    real = fl.parse_train("done      100 Gaussians in 1.0 s; RMSE vs measured lines 0.0300\n")
    assert real["rmse_measured"] == 0.03
    assert real["rmse_truth"] is None and real["pose_error_trained_px"] is None


def test_config_overrides(tmp_path):
    path = tmp_path / "c.yaml"
    path.write_text("name: x\nsim: {scene: board}\nlimits: {rmse_measured_max: 0.1}\n")
    cfg = fl.load_config(str(path), ["train.iterations=50", "sim.views=[\"down\"]", "scan.path=~/scans/a b",
                                     "limits.rmse_truth_max=null"])
    assert cfg["sim"]["scene"] == "board" and cfg["sim"]["plan"] == "ring"   # the rest from the defaults
    assert cfg["train"]["iterations"] == 50 and cfg["train"]["batch"] == 128
    assert cfg["sim"]["views"] == ["down"]
    assert cfg["scan"]["path"] == "~/scans/a b"
    assert cfg["limits"] == {"rmse_measured_max": 0.1, "rmse_truth_max": None}
    with pytest.raises(SystemExit):
        fl.load_config(str(path), ["source=camera"])
    with pytest.raises(SystemExit):
        fl.load_config(str(path), ["no_equals_sign"])


def test_checks_against_limits_and_the_report(tmp_path):
    cfg = fl.load_config(None, ["limits.rmse_measured_max=0.02", "limits.profile_correlation_min=0.99"])
    p = fl.Pipeline(cfg, str(tmp_path))
    p.check("train", "rmse", 0.019, "rmse_measured_max")
    p.check("truth", "corr", 0.98, "profile_correlation_min")
    p.check("truth", "nan never passes", float("nan"), "profile_correlation_min")
    p.check("train", "no limit", 1.5, "pose_error_px_max")
    p.check_true("viewer", "reads", True)
    oks = [c.get("ok") for c in p.report["checks"]]
    assert oks == [True, False, False, None, True]
    p.save()
    report = json.loads((tmp_path / "report.json").read_text())
    assert report["ok"] is False
    md = (tmp_path / "report.md").read_text()
    assert "**FAILED**" in md and "| train | rmse | 0.019 | ≤ 0.02 | ok |" in md
    assert "| truth | corr | 0.98 | ≥ 0.99 | **FAIL** |" in md


def test_the_shipped_configs_load():
    for name in os.listdir(os.path.join(fl.REPO, "pipeline", "configs")):
        cfg = fl.load_config(os.path.join(fl.REPO, "pipeline", "configs", name))
        assert set(cfg) == set(fl.DEFAULTS), name
        for key in cfg["limits"]:
            assert key.endswith("_max") or key.endswith("_min"), (name, key)
