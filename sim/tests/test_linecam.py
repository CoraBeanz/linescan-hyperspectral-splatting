"""The objective, the rays of a scan line, and how they meet the scene and the URDF."""

import numpy as np
import pytest

from hsisim import repo, spectra
from hsisim.linecam import Objective, SlitRays
from hsisim.scene import Box, Checker, Plane, Scene, Texture
from hsical.design import DesignMap
from so101_scan_sweep.line_log import LinePoser


@pytest.fixture(scope="module")
def objective():
    return Objective.from_design(DesignMap().config)


def test_objective_matches_the_design_and_the_urdf(objective):
    assert objective.slit_distance_mm == pytest.approx(17.4107, abs=1e-3)   # 1 / (1/15.6 - 1/150)
    assert objective.half_line_m == pytest.approx(0.0215385, rel=1e-5)      # 5 mm slit x 8.615
    assert objective.line_width_m == pytest.approx(0.00043077, rel=1e-4)
    poser = LinePoser(repo.robot())
    assert poser.scene_distance == pytest.approx(objective.focus_mm * 1e-3)
    assert poser.half_line == pytest.approx(objective.half_line_m, rel=1e-4)


def _focal_plane(rays, z):
    """Where every ray crosses the plane at camera z, [n_h, K, N, 2]."""
    t = (z - rays.origin[:, 2]) / rays.direction[:, 2]
    p = rays.origin + t[:, None] * rays.direction
    return p[:, :2].reshape(*rays.shape, 2)


def test_rays_meet_in_focus(objective):
    sharp = Objective.from_design(DesignMap().config, blur_um=0.0)
    h = np.linspace(-1, 1, 9)
    rays = SlitRays(sharp, h, slices=3, per_point=16, seed=1)
    xy = _focal_plane(rays, sharp.focus_mm * 1e-3)
    # every bundle meets on the scan line: x from h, y within the line's width, its own slice
    assert xy[..., 0] == pytest.approx(np.broadcast_to(h[:, None, None] * sharp.half_line_m, rays.shape), abs=1e-12)
    w = sharp.line_width_m
    assert np.abs(xy[..., 1]).max() <= w / 2 + 1e-12
    for k in range(3):
        y = xy[:, k, :, 1] / w
        assert (y >= rays.slice_centre[k] - 1 / 6 - 1e-9).all() and (y <= rays.slice_centre[k] + 1 / 6 + 1e-9).all()
    # out of focus the bundle spreads by the aperture's width times the defocus
    near = _focal_plane(rays, 0.75 * sharp.focus_mm * 1e-3)
    spread = near[..., 0].max(-1) - near[..., 0].min(-1)
    assert spread.max() <= 2 * sharp.aperture_radius_m * 0.25 + 1e-9
    assert spread.mean() > 0.5 * sharp.aperture_radius_m * 0.25


def test_line_ends_match_line_poser(objective):
    """h = -1 and +1 land on the URDF's scan line ends, in that order, for a real arm pose."""
    robot = repo.robot()
    poser = LinePoser(robot)
    from hsisim.session import load_plan
    vp = load_plan("ring").viewpoints[1]
    _, cam = poser.poses(vp.joints, 0.02)
    sharp = Objective.from_design(DesignMap().config, blur_um=0.0)
    rays = SlitRays(sharp, [-1.0, 1.0], slices=1, per_point=8, seed=2)
    o, d = rays.world(cam)
    centre = cam[:3, :3] @ [0.0, 0.0, 1.0]  # the camera's view direction
    z = sharp.focus_mm * 1e-3
    # where the rays cross the scan line's plane (the focal plane, square to the view)
    t = (z - (o - cam[:3, 3]) @ centre) / (d @ centre)
    p = (o + t[:, None] * d).reshape(2, 8, 3).mean(1)
    a, b = poser.line_ends(cam)
    # the bundles cross the slit's width at random, so they average to within its half-width
    w = sharp.line_width_m / 2
    assert np.linalg.norm(p[0] - a) < w and np.linalg.norm(p[1] - b) < w
    assert np.linalg.norm(b - a) == pytest.approx(2 * poser.half_line)  # the URDF rounds it to 1 um


def test_weights_on_a_white_sheet(objective):
    h = np.linspace(-1.0, 1.0, 21)
    rays = SlitRays(objective, h, slices=3, per_point=12, seed=3)
    cam = np.eye(4)
    cam[:3, :3] = np.diag([1.0, -1.0, -1.0])  # looking straight down, x along +x
    cam[2, 3] = 0.15
    scene = Scene([Plane(0.0, Texture.solid("ptfe"))])
    w, depth = rays.weights(scene, cam, len(spectra.NAMES))
    assert w.shape == (3, 21, len(spectra.NAMES))
    assert w.sum(-1) == pytest.approx(np.ones((3, 21)))
    assert (w.argmax(-1) == spectra.material_id("ptfe")).all()
    assert depth == pytest.approx(np.full(21, 0.15), abs=1e-6)


def test_weights_split_at_an_edge(objective):
    """A sheet half white, half black under the line: the shares follow h, with a soft edge
    from the objective's blur and the defocus."""
    h = np.linspace(-1.0, 1.0, 81)
    rays = SlitRays(objective, h, slices=1, per_point=32, seed=4)
    cam = np.eye(4)
    cam[:3, :3] = np.diag([1.0, -1.0, -1.0])
    cam[2, 3] = 0.15
    # white for x >= 0, black below: the camera's x is the world's, so white for h > 0
    scene = Scene([Plane(0.0, Texture().add(Checker(1.0, ("white_paper", "carbon_black"), origin=(0.0, -0.5))))])
    w, _ = rays.weights(scene, cam, len(spectra.NAMES))
    white = w[0, :, spectra.material_id("white_paper")]
    assert white[h < -0.1] == pytest.approx(0.0, abs=1e-12)
    assert white[h > 0.1] == pytest.approx(1.0, abs=1e-12)
    assert 0.2 < white[np.argmin(np.abs(h))] < 0.8


def test_reachable_culls_far_solids(objective):
    rays = SlitRays(objective, np.linspace(-1, 1, 5), slices=1, per_point=4, seed=5)
    cam = np.eye(4)
    cam[:3, :3] = np.diag([1.0, -1.0, -1.0])
    cam[2, 3] = 0.15
    near = Box((0.0, 0.0, 0.005), (0.01, 0.01, 0.01), Texture.solid("gray18"))
    far = Box((0.0, 0.1, 0.005), (0.01, 0.01, 0.01), Texture.solid("gray18"))   # 10 cm across the line
    end = Box((0.04, 0.0, 0.005), (0.01, 0.01, 0.01), Texture.solid("gray18"))  # past the line's end
    edge = Box((0.024, 0.0, 0.005), (0.01, 0.01, 0.01), Texture.solid("gray18"))  # over the line's end
    scene = Scene([Plane(0.0, Texture.solid("wood")), near, far, end, edge])
    assert rays.reachable(scene, cam) == [0, 1, 4]
