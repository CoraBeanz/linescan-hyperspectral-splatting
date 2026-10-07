"""The spectrograph and IMX219: light lands at the right wavelength and slit row, and frames
look like the camera's."""

import numpy as np
import pytest

from hsisim.instrument import FULL_DN, Spectrograph
from hsical.synth import Truth


@pytest.fixture(scope="module")
def sp():
    return Spectrograph(Truth(scale=4, seed=3))


def _ratio(sp, refl, slices=None):
    """Signal over a white sheet's: the light's shape without the lamp and the sensor."""
    r = np.asarray(refl, float)
    if r.ndim == 2:
        r = r[None]
    centres = [0.0] if slices is None else slices
    with np.errstate(invalid="ignore", divide="ignore"):  # no light off the slit's ends
        return sp.signal(r, centres) / sp.uniform_signal(1.0)


def _mid(sp):
    return sp.H // 2


def test_a_narrow_band_lands_at_its_wavelength(sp):
    for centre in (550.0, 700.0, 900.0):
        bump = np.exp(-0.5 * ((sp.nm - centre) / 2.0) ** 2)
        ratio = _ratio(sp, np.broadcast_to(bump, (sp.h.size, sp.nm.size)))
        for row in (sp.H // 4, _mid(sp), 3 * sp.H // 4):
            col = int(np.nanargmax(ratio[row]))
            assert sp.inst.nm_map[row, col] == pytest.approx(centre, abs=1.0), (centre, row)


def test_light_lands_on_its_slit_rows(sp):
    """Light from h > 0 only: lit rows are those whose h is above 0, so the slit row grows with h."""
    refl = np.where(sp.h[:, None] > 0, 1.0, 0.0) * np.ones(sp.nm.size)
    ratio = _ratio(sp, refl)
    col = sp.W // 2
    h = sp.inst.h_map[:, col]
    assert np.all(np.diff(h[np.abs(h) < 1]) > 0)
    inside = np.abs(h) < 0.9
    assert ratio[inside & (h < -0.1), col] == pytest.approx(0.0, abs=0.02)
    assert ratio[inside & (h > 0.1), col] == pytest.approx(1.0, abs=0.02)


def test_light_across_the_slit_lands_across_its_image(sp):
    """A band seen by one edge of the slit only lands at that edge's place in the slit's image."""
    bump = np.exp(-0.5 * ((sp.nm - 700.0) / 2.0) ** 2)
    one = np.broadcast_to(bump, (sp.h.size, sp.nm.size))
    zero = np.zeros_like(one)
    centres = [-1 / 3, 0.0, 1 / 3]
    row = _mid(sp)
    peaks = []
    for k in (0, 2):
        refl = np.stack([one if j == k else zero for j in range(3)])
        peaks.append(float(np.nanargmax(_ratio(sp, refl, centres)[row])))
    width = sp.widths[int(round(np.mean(peaks)))]
    assert abs(peaks[1] - peaks[0]) == pytest.approx(2 / 3 * width, abs=1.5)


def test_frames_look_like_the_camera(sp):
    dark = sp.dark(20000.0, 2)
    assert all(f.dtype == np.uint16 and f.shape == (sp.H, sp.W) for f in dark)
    assert np.median(dark[0]) == pytest.approx(sp.truth.black_dn, abs=1.0)
    white = sp.uniform_signal()
    level = sp.level_for(white, 20000.0, fill=0.8)
    frame = sp.expose(white, level, 20000.0)[0].astype(float)
    assert frame.max() <= FULL_DN
    peak = np.percentile(frame, 99.9)
    want = sp.truth.black_dn + 0.8 * (FULL_DN - sp.truth.black_dn)
    assert peak == pytest.approx(want, rel=0.1)
    # the NoIR sensor's mosaic: neighbouring pixels under the same light differ
    r, c = sp.H // 2 & ~1, sp.W // 2 & ~1
    quad = frame[r:r + 2, c:c + 2] - sp.truth.black_dn
    assert quad.max() / max(quad.min(), 1.0) > 1.1


def test_pixel_maps_follow_the_frames(sp):
    nm, h = sp.pixel_maps()
    assert nm.shape == h.shape == sp.expose(np.zeros((sp.H, sp.W)), 0.0, 1000.0)[0].shape
    assert np.nanmin(nm) < 480 and np.nanmax(nm) > 1000
