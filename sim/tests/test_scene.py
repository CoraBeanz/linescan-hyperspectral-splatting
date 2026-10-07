"""Ray casting, textures and lighting."""

import numpy as np
import pytest

from hsisim import spectra
from hsisim.scene import Bitmap, Box, Checker, Lamp, Plane, Scene, Speckle, Sphere, Texture, yaw
from hsisim.scenes import NIR, RELIEF_HEIGHTS, make

ID = spectra.material_id
DOWN = np.array([[0.0, 0.0, -1.0]])


def _ray(x, y, z=1.0, d=DOWN):
    return np.array([[x, y, z]], float), np.asarray(d, float)


def test_plane_hit_from_above():
    plane = Plane(0.1, Texture.solid("wood"))
    t, n = plane.intersect(*_ray(0.3, -0.2))
    assert t[0] == pytest.approx(0.9)
    assert n[0].tolist() == [0.0, 0.0, 1.0]
    t, _ = plane.intersect(*_ray(0.0, 0.0, d=[[0.0, 0.0, 1.0]]))  # looking away
    assert np.isinf(t[0])


def test_box_faces_and_misses():
    box = Box((0.0, 0.0, 0.05), (0.02, 0.04, 0.1), Texture.solid("red_paint"),
              sides=Texture.solid("blue_paint"), rotation=yaw(30))
    t, n = box.intersect(*_ray(0.001, 0.002))
    assert t[0] == pytest.approx(0.9)
    assert n[0] @ [0, 0, 1] == pytest.approx(1.0)
    t, _ = box.intersect(*_ray(0.1, 0.0))
    assert np.isinf(t[0])
    # from the side, along the box's own -x axis: hits the +x face, 1 cm out from the centre
    d = -yaw(30)[:, 0]
    o = np.array([[0.0, 0.0, 0.05]]) - 0.5 * d
    t, n = box.intersect(o, d[None])
    assert t[0] == pytest.approx(0.5 - 0.01)
    assert n[0] == pytest.approx(yaw(30)[:, 0])
    s = Scene([box])
    hits = s.cast(o, d[None])
    assert hits.material[0] == ID("blue_paint")


def test_box_extent_matches_its_size():
    box = Box((0, 0, 0), (0.02, 0.04, 0.1), Texture.solid("gray18"), rotation=yaw(90))
    assert box.extent([1, 0, 0]) == pytest.approx(0.02)   # turned 90 deg: the 4 cm side runs along x
    assert box.extent([0, 1, 0]) == pytest.approx(0.01)
    assert box.extent([0, 0, 1]) == pytest.approx(0.05)


def test_sphere_hit():
    ball = Sphere((0.0, 0.0, 0.0), 0.01, Texture.solid("leaf"))
    t, n = ball.intersect(*_ray(0.0, 0.0, 0.5))
    assert t[0] == pytest.approx(0.49)
    assert n[0] == pytest.approx([0.0, 0.0, 1.0])


def test_nearest_solid_wins_and_void_when_nothing():
    scene = Scene([Plane(0.0, Texture.solid("wood")),
                   Box((0.0, 0.0, 0.01), (0.02, 0.02, 0.02), Texture.solid("ptfe"))])
    o = np.array([[0.0, 0.0, 1.0], [0.1, 0.0, 1.0], [0.0, 0.0, 1.0]])
    d = np.array([[0.0, 0.0, -1.0], [0.0, 0.0, -1.0], [0.0, 0.0, 1.0]])
    hits = scene.cast(o, d)
    assert hits.material.tolist() == [ID("ptfe"), ID("wood"), ID("void")]
    assert hits.t[:2] == pytest.approx([0.98, 1.0])
    assert hits.shading.tolist() == [1.0, 1.0, 0.0]


def test_checker_and_layers():
    t = Texture().add(Checker(0.01, ("white_paper", "carbon_black")))
    t.add(Checker(1.0, ("leaf", "leaf")), (0.1, 0.1, 0.2, 0.2))
    u = np.array([0.005, 0.015, 0.025, 0.15])
    v = np.array([0.005, 0.005, 0.005, 0.15])
    assert t.ids(u, v).tolist() == [ID("white_paper"), ID("carbon_black"), ID("white_paper"), ID("leaf")]
    assert t.materials() == {"white_paper", "carbon_black", "leaf"}


def test_bitmap_reads_like_text():
    b = Bitmap(NIR, 1.0, "carbon_black", "ir_black_dye")
    nr, nc = len(NIR), len(NIR[0])
    rows, cols = np.mgrid[0:nr, 0:nc]
    u = cols.ravel() - nc / 2 + 0.5        # cell centres, the first row at +v
    v = nr / 2 - rows.ravel() - 0.5
    got = (b.ids(u, v) == ID("carbon_black")).reshape(nr, nc)
    want = np.array([[ch == "#" for ch in r] for r in NIR])
    assert (got == want).all()


def test_speckle_is_fixed_and_dotted():
    s = Speckle(0.001, 0.0004, "white_paper", "carbon_black", density=0.6, seed=3)
    u, v = np.meshgrid(np.linspace(0, 0.02, 200), np.linspace(0, 0.02, 200))
    a = s.ids(u.ravel(), v.ravel())
    assert (a == s.ids(u.ravel(), v.ravel())).all()
    share = (a == ID("carbon_black")).mean()
    assert 0.15 < share < 0.45                  # 60% of cells with a dot of radius 0.4 cell
    other = Speckle(0.001, 0.0004, "white_paper", "carbon_black", density=0.6, seed=4).ids(u.ravel(), v.ravel())
    assert (other != a).mean() > 0.1


def test_lamp_shading_and_shadow():
    target = (0.0, 0.0, 0.0)
    lamp = Lamp(position=(0.0, 0.0, 0.3), target=target, ambient=0.05)
    blocker = Box((0.05, 0.0, 0.15), (0.01, 0.01, 0.01), Texture.solid("gray18"))
    scene = Scene([Plane(0.0, Texture.solid("ptfe")), blocker], lighting=lamp)
    # straight under the lamp, level: 1. Under the blocker's shadow: ambient only
    p = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]])
    n = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]])
    shade = lamp.shading(scene, p, n)
    assert shade[0] == pytest.approx(1.0)
    assert shade[1] == pytest.approx(0.05)
    # further out and unblocked: cosine and distance fall-off
    far = lamp.shading(scene, np.array([[-0.3, 0.0, 0.0]]), n[:1])[0]
    cos, d2 = 0.3 / np.hypot(0.3, 0.3), 0.3 ** 2 + 0.3 ** 2
    assert far == pytest.approx(0.05 + 0.95 * cos / d2 * 0.3 ** 2)


def test_relief_scene_heights():
    scene = make("relief")
    tx, ty, tz = scene.target
    # straight down onto each pillar's centre: the hit is at its height above the 2 mm plate
    for r in range(5):
        for c in range(5):
            o = np.array([[tx + (c - 2) * 0.01, ty + (r - 2) * 0.01, tz + 0.5]])
            hits = scene.cast(o, DOWN)
            assert hits.point[0, 2] == pytest.approx(tz + 0.002 + RELIEF_HEIGHTS[r][c] * 1e-3, abs=1e-9)
    # every material it uses is in the library
    assert set(scene.materials()) <= set(spectra.NAMES)
    assert "rare_earth" in scene.materials() and "ptfe" in scene.materials()


def test_unknown_scene():
    with pytest.raises(ValueError):
        make("nothing")
