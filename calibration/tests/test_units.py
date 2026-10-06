"""Small pieces, checked on their own."""

import numpy as np
import pytest

from hsical import capture, frames, lines, profiles
from hsical.design import DesignMap, interp_extrap
from hsical.poly import Poly1D, Poly2D
from hsical.response import channel_curves, planck_rel


def test_centroid_is_unbiased():
    x = np.arange(80, dtype=float)
    rng = np.random.default_rng(0)
    centres = 40 + rng.uniform(-0.5, 0.5, 50)
    rows = np.array([profiles.slit_profile(x, c, 1.0, 23.4, 3.0) + 0.02 for c in centres])
    got, flux, ok = profiles.centroid(rows, np.round(centres), 18.0)
    assert ok.all()
    assert np.max(np.abs(got - centres)) < 2e-3


def test_slit_profile_fit_recovers_shape():
    x = np.arange(60, dtype=float)
    y = profiles.slit_profile(x, 29.3, 5.0, 12.0, 2.0) + 0.1
    r = profiles.fit_slit_profile(x, y, 30.0, 10.0, 3.0)
    assert r["ok"]
    assert abs(r["center"] - 29.3) < 1e-3 and abs(r["width"] - 12.0) < 1e-2 and abs(r["sigma"] - 2.0) < 1e-2


def test_binomial_smooth_removes_mosaic():
    img = np.tile(np.array([[1.0, 3.0], [2.0, 5.0]]), (10, 10))
    s = profiles.binomial_smooth(img)[2:-2, 2:-2]
    assert np.allclose(s, s.mean())


def test_interp_extrap_scalar_and_array():
    xp, fp = np.array([0.0, 1.0, 2.0]), np.array([0.0, 2.0, 3.0])
    assert interp_extrap(-1.0, xp, fp) == pytest.approx(-2.0)
    assert np.allclose(interp_extrap(np.array([0.5, 3.0]), xp, fp), [1.0, 4.0])


def test_design_map_is_smooth_past_the_grid():
    d = DesignMap(scale=4)
    h = np.array([0.99, 1.0, 1.01, 1.2])
    rows = d.row(h, 750.0)
    assert np.all(np.diff(rows) > 0)
    assert d.dispersion(750.0) > 1.3     # px per nm at binning 4


def test_poly_fits_and_inverts():
    rng = np.random.default_rng(1)
    x, y = rng.uniform(0, 100, 300), rng.uniform(0, 50, 300)
    p = Poly2D([(1, 0), (0, 1), (2, 1)], 50, 50, 25, 25, [0.3, -1.2, 0.5])
    z = p(x, y)
    q = Poly2D(p.terms, 50, 50, 25, 25)
    q.fit(x, y, z)
    assert np.allclose(q.coef, p.coef)
    p1 = Poly1D(3, 50, 50, [500.0, 200.0, 5.0, 1.0])
    assert p1.inverse(p1(37.5), 0, 100) == pytest.approx(37.5, abs=1e-6)


@pytest.mark.parametrize("shift,repeat", [(0, False), (4, False), (4, True), (6, False)])
def test_raw16_decoding(shift, repeat):
    rng = np.random.default_rng(2)
    img = (64 + rng.integers(0, 900, (40, 50))).astype(np.uint32)
    img[:, :5] = 64
    words = img << shift
    if repeat:  # some drivers copy the top data bits into the spare high bits
        words |= ((words >> 12) & 0x3) << 14
    padded = np.zeros((3, 40, 64), "<u2")
    padded[:, :, :50] = words
    a = frames.decode_raw16(padded.tobytes(), 50, 40)
    b, stride, sh = capture.decode_raw16(padded.tobytes(), 50, 40)
    assert a.shape == (3, 40, 50) and np.array_equal(a[1], img)
    assert np.array_equal(b[2], img) and stride == 128 and sh == shift


def test_orientation_undoes_the_mounting():
    from hsical.synth import Instrument, Truth
    a = np.arange(12.0).reshape(3, 4)
    for t, fx, fy in [(True, True, False), (False, True, True), (True, False, True)]:
        inst = Instrument.__new__(Instrument)
        inst.t = Truth(transpose=t, flip_x=fx, flip_y=fy)
        cam = inst.orient(a)
        assert np.array_equal(frames.Orientation(t, fx, fy).apply(cam), a)


def test_catalog_and_groups():
    cfl, neon = lines.catalog("cfl"), lines.catalog("neon")
    assert any(abs(ln.nm - 546.0735) < 1e-3 for ln in cfl)
    assert all(ln.quality in "ABC" for ln in cfl + neon)
    gs = lines.groups(neon, 4.5)
    assert len(gs) > 20
    assert all(0 <= g.purity <= 1 and g.blend_nm >= 0 and g.ref in g.members for g in gs)


def test_planck_and_channels():
    assert planck_rel(700.0, 2850.0) == pytest.approx(1.0)
    assert planck_rel(900.0, 2850.0) > planck_rel(500.0, 2850.0)
    nm = np.linspace(500, 1000, 40)
    R = np.ones((4, 40))
    R[1::2, 1::2] = np.where((nm > 600) & (nm < 800), 10.0, 1.0)[1::2]   # red pixels at odd/odd
    curves, red = channel_curves(R, np.arange(4), nm)
    assert red == (1, 1)
    assert curves["R"][20] > curves["G"][20]


def test_focus_metric_on_a_line():
    x = np.arange(400, dtype=float)
    line = profiles.slit_profile(x, 211.3, 500.0, 20.0, 3.0) + 64
    frame = np.tile(line, (60, 1))
    w, pos = capture.line_width(frame)
    assert abs(pos - 211) <= 2 and 19 < w < 24
