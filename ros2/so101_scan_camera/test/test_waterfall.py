"""The colour waterfalls: spectra to colours as the viewer does it, and lines to colours as the
dataset will hold them."""

import json
import os

import numpy as np
import pytest

from so101_scan_camera import session as sess
from so101_scan_camera import spectral
from so101_scan_camera.binning import LineBinner, load_calibration
from so101_scan_camera.synthetic import Instrument, halogen
from so101_scan_camera.waterfall import Waterfall, dataset_bands, grey

WL = 500.0 + 10.0 * np.arange(46)       # the dataset's bands
EXPOSURE_US = 4000.0


def _sig(x):
    return 1.0 / (1.0 + np.exp(-x))


def _bump(x, mu, s):
    return np.exp(-0.5 * (x - mu) ** 2 / s ** 2)


# three of splat/src/spectra.cpp's materials
MATERIALS = {
    "leaf": lambda nm: (0.045 + 0.085 * _bump(nm, 553, 32) - 0.015 * _bump(nm, 676, 16)
                        + 0.46 * _sig((nm - 714) / 13) - 0.05 * _bump(nm, 970, 22)),
    "red": lambda nm: 0.05 + 0.72 * _sig((nm - 603) / 11),
    "blue": lambda nm: 0.06 + 0.28 * (1 - _sig((nm - 535) / 18)) + 0.48 * _sig((nm - 745) / 22),
    "paper": lambda nm: 0.86 - 0.05 * (nm - 500) / 450,
}

# viewer/js/spectral.js's trueColorWeights and colorInfraredWeights on WL, dotted with the
# materials (node, 2026-10-08)
VIEWER = {
    "leaf": ([0.06526453085814046, 0.11271017161656771, 0.061482991828334006],
             [0.5048886336072869, 0.048607163116947086, 0.11804733456190786]),
    "red": ([0.5813971877406006, 0.04287715260521299, 0.03677866186535711],
            [0.7699999985439088, 0.7426972505837858, 0.06657696957867597]),
    "blue": ([0.01709832953019827, 0.17989759129285643, 0.327274150617189],
             [0.5315361011872274, 0.0692860763626839, 0.15431977836339952]),
}


@pytest.mark.parametrize("name", sorted(VIEWER))
def test_colours_match_the_viewer(name):
    s = MATERIALS[name](WL)
    np.testing.assert_allclose(s @ spectral.true_color_weights(WL), VIEWER[name][0], atol=1e-12)
    np.testing.assert_allclose(s @ spectral.color_infrared_weights(WL), VIEWER[name][1], atol=1e-12)


def test_white_is_white_and_grey_is_grey():
    for mode in spectral.MODES:
        w = spectral.weights(mode, WL)
        np.testing.assert_allclose(np.ones(46) @ w, [1, 1, 1], atol=1e-12)
        np.testing.assert_allclose(np.full(46, 0.18) @ w, [0.18] * 3, atol=1e-12)
    assert list(spectral.srgb8([0.0, 0.18, 1.0, 2.0, np.nan])) == [0, 118, 255, 255, 0]


@pytest.fixture(scope="module")
def instrument(tmp_path_factory):
    inst = Instrument(164, 124)
    cal = inst.save_calibration(str(tmp_path_factory.mktemp("cal")))
    return inst, LineBinner(load_calibration(cal), 64)


def stripes(t, nm):
    """Red, leaf and white paper along the slit, under a halogen lamp."""
    r = np.where(t < 1 / 3, MATERIALS["red"](nm), np.where(t < 2 / 3, MATERIALS["leaf"](nm), MATERIALS["paper"](nm)))
    return r * halogen(nm)


def take(inst, binner, radiance, n=1):
    return np.mean([binner.bin(inst.render(radiance, EXPOSURE_US))[0] for _ in range(n)], axis=0)


def stripe_colours(img):
    """The median colour of the middle of each stripe."""
    w = img.shape[1]
    return [np.median(img[:, int(w * f) - 2:int(w * f) + 3].reshape(-1, 3), axis=0) for f in (1 / 6, 1 / 2, 5 / 6)]


def test_the_waterfall_shows_the_colours_the_dataset_will_have(instrument):
    inst, binner = instrument
    wf = Waterfall(binner)
    np.testing.assert_allclose(wf.nm, WL)
    lines = {i: (take(inst, binner, stripes), EXPOSURE_US, 1.0) for i in range(6)}
    lines[6] = (None, EXPOSURE_US, 1.0)                     # a line without a frame: black

    # no white yet: relative to the calibration's response and lamp, so near the right colours
    assert wf.basis == "response"
    near = wf.image(lines, "true_color")
    assert near.shape == (7, 64, 3) and near.dtype == np.uint8 and not near[6].any()

    wf.set_white(take(inst, binner, lambda t, nm: 0.98 * halogen(nm), 4), EXPOSURE_US, 1.0)
    assert wf.basis == "white"
    for mode in spectral.MODES:
        img = wf.image(lines, mode)
        want = [spectral.srgb8(MATERIALS[m](WL) @ spectral.weights(mode, WL)) for m in ("red", "leaf", "paper")]
        for got, exp in zip(stripe_colours(img[:6]), want):
            assert np.abs(got - exp).max() <= 12, (mode, got, exp)
    for got, exp in zip(stripe_colours(near[:6]), stripe_colours(wf.image(lines, "true_color")[:6])):
        assert np.abs(got - exp).max() <= 30


def test_the_white_reference_can_be_at_another_exposure(instrument):
    inst, binner = instrument
    wf = Waterfall(binner)
    white = np.mean([binner.bin(inst.render(lambda t, nm: 0.98 * halogen(nm), 2000.0, 2.0))[0] for _ in range(4)],
                    axis=0)
    wf.set_white(white, 2000.0, 2.0)
    img = wf.image({0: (take(inst, binner, stripes, 2), EXPOSURE_US, 1.0)}, "cir")
    want = spectral.srgb8(MATERIALS["leaf"](WL) @ spectral.color_infrared_weights(WL))
    assert np.abs(stripe_colours(img)[1] - want).max() <= 12


def test_saturated_slit_bins_show_magenta(instrument):
    inst, binner = instrument
    wf = Waterfall(binner)
    bright = take(inst, binner, lambda t, nm: np.where(t > 0.5, 60.0, 0.3) * halogen(nm))
    img = wf.image({0: (bright, EXPOSURE_US, 1.0)})
    assert (img[0, 50] == [255, 0, 255]).all() and not (img[0, 10] == [255, 0, 255]).all()
    assert not (wf.image({0: (bright, EXPOSURE_US, 1.0)}, mark_saturated=False)[0, 50] == [255, 0, 255]).all()


def test_without_a_response_it_waits_for_a_white(instrument):
    inst, binner = instrument
    wf = Waterfall(binner)
    wf.grid.response = None
    wf._set_denominator()
    lines = {0: (take(inst, binner, stripes), EXPOSURE_US, 1.0)}
    assert wf.basis is None and not wf.usable and wf.image(lines) is None
    wf.set_white(take(inst, binner, lambda t, nm: 0.98 * halogen(nm), 4), EXPOSURE_US, 1.0)
    assert wf.usable and wf.image(lines).shape == (1, 64, 3)


def test_lines_are_worked_out_once(instrument):
    inst, binner = instrument
    wf = Waterfall(binner)
    cache = {}
    line = take(inst, binner, stripes)
    first = wf.image({0: (line, EXPOSURE_US, 1.0)}, cache=cache)
    assert list(cache) == [0]
    cache[0] = (cache[0][0], np.zeros_like(cache[0][1]), cache[0][2])   # what's kept is what's drawn
    assert not wf.image({0: (line, EXPOSURE_US, 1.0)}, cache=cache).any()
    wf.add_dark(np.full(binner.shape, 64.0), 1000.0, 1.0)              # new references: worked out again
    assert (wf.image({0: (line, EXPOSURE_US, 1.0)}, cache=cache) == first).all()


def test_references_load_from_where_capture_reference_left_them(instrument, tmp_path):
    inst, binner = instrument
    folder = tmp_path / "refs"
    for kind, radiance in (("dark", 0.0), ("white", lambda t, nm: 0.98 * halogen(nm))):
        frames = [inst.render(radiance, EXPOSURE_US) for _ in range(3)]
        sess.write_reference(str(folder / "reference" / kind), frames,
                             dict(kind=kind, exposure_us=EXPOSURE_US, gain=1.0))
        os.makedirs(folder / "binned", exist_ok=True)
        np.save(folder / "binned" / ("reference_%s.npy" % kind),
                np.mean([binner.bin(f)[0] for f in frames], axis=0).astype(np.float32))
    binner.save(str(folder / "binned" / "binning.npz"))
    wf = Waterfall(binner)
    said = wf.load_references(str(folder))
    assert "dark dark" in said and "white white" in said and wf.basis == "white"
    assert wf.dark_for(EXPOSURE_US, 1.0) is not wf.dark_for(2 * EXPOSURE_US, 1.0)

    other = LineBinner(binner.maps, 32)          # binned on another grid: left alone
    wf2 = Waterfall(other)
    assert "another grid" in wf2.load_references(str(folder)) and wf2.basis == "response"
    meta = json.loads((folder / "reference" / "white" / "meta.json").read_text())
    assert meta["kind"] == "white"


def test_bands_and_grey(instrument):
    np.testing.assert_allclose(dataset_bands(np.array([480.0, 990.0])), WL)
    np.testing.assert_allclose(dataset_bands(np.array([520.0, 700.0])), np.arange(530.0, 700.0, 10.0))
    inst, binner = instrument
    lines = {0: (take(inst, binner, stripes), EXPOSURE_US, 1.0), 2: (None, EXPOSURE_US, 1.0)}
    g = grey(lines, 64)
    assert g.shape == (3, 64) and g.dtype == np.uint8 and not g[1:].any()
    assert (grey(lines, 64, slit_reversed=True)[0] == g[0, ::-1]).all()
    wf = Waterfall(binner, slit_reversed=True)
    assert (wf.image(lines)[0] == Waterfall(binner).image(lines)[0, ::-1]).all()
