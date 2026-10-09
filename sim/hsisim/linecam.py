"""What one scan line sees: rays from the slit, through the objective, into the scene.

The objective (16 mm, f/4, focused at 150 mm in the design) images the scene onto the
50 um x 5 mm slit. Seen from the scene it is a camera at line_camera_optical_frame, the
objective reflected in the scan mirror: z looks at the scene, x runs along the slit. The
slit's image in the scene is the in-focus scan line, 41.9 mm long and 0.42 mm wide at
150 mm.

Slit position h runs from -1 to +1 between the slit's ends, as in hsical (h is the
Optiland model's field coordinate): h = +1 is the camera's +x end, and on the sensor the
slit row grows with h. The splat dataset's pixel 0 is therefore the h = -1 end.

Light reaching one point of the slit comes from every point of the objective's 4 mm
aperture, so each point is traced as a bundle: rays from points spread over the aperture
through the point's conjugate in the focal plane, which the objective's own blur (a
Gaussian, a few microns at the slit) moves a little. In focus the bundle meets at one
scene point; nearer or further away it spreads, and that is the depth of field, with the
right occlusion at depth edges. Each line's slit is cut across its width into a few
slices, because light from different places across the slit lands at different places
along the spectrum (instrument.py).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np


@dataclass
class Objective:
    focal_mm: float = 16.0
    f_number: float = 4.0
    focus_mm: float = 150.0        # scene distance the objective is focused at
    slit_length_mm: float = 5.0
    slit_width_mm: float = 0.050
    blur_um: float = 5.0           # the objective's own blur at the slit, 1 sigma (a guess for an M12 lens)
    # the scan line's true length over the design's (a head off its CAD numbers), and the distortion
    # along the slit as headcal models it: h = u + k1 u^3, with u where the ray points along the line
    # (-1 and +1 at its ends)
    line_scale: float = 1.0
    slit_k1: float = 0.0

    @classmethod
    def from_design(cls, config, **kw):
        """From the Optiland map's config (hsical's DesignMap.config)."""
        return cls(focal_mm=config["f_obj"], f_number=config["fno_obj"], focus_mm=config["scene_dist"],
                   slit_length_mm=config["slit_len"], slit_width_mm=config["slit_width"], **kw)

    @property
    def slit_distance_mm(self):
        return 1.0 / (1.0 / self.focal_mm - 1.0 / self.focus_mm)

    @property
    def magnification(self):
        """Scene size over slit size at the focus distance."""
        return self.focus_mm / self.slit_distance_mm

    @property
    def half_line_m(self):
        return 0.5e-3 * self.slit_length_mm * self.magnification

    @property
    def line_width_m(self):
        return 1e-3 * self.slit_width_mm * self.magnification

    def line_x(self, h):
        """How far along the in-focus scan line (m) slit position h looks."""
        h = np.asarray(h, float)
        u = h.copy()
        for _ in range(4):
            u = u - (u + self.slit_k1 * u ** 3 - h) / (1.0 + 3.0 * self.slit_k1 * u ** 2)
        return u * (self.half_line_m * self.line_scale)

    @property
    def aperture_radius_m(self):
        return 0.5e-3 * self.focal_mm / self.f_number

    def to_dict(self):
        return dict(asdict(self), slit_distance_mm=self.slit_distance_mm, half_line_m=self.half_line_m,
                    line_width_m=self.line_width_m)


class SlitRays:
    """A fixed bundle of rays for every (slice, slit position): the same for every line.

    h: slit positions (the radiance grid along the slit). slices: cuts across the slit's
    width. per_point: rays per (slice, h), spread over the aperture. The sample patterns
    are drawn once from `seed`, so lines differ only by what they look at."""

    def __init__(self, objective: Objective, h, slices=3, per_point=12, seed=0):
        self.obj = objective
        self.h = np.asarray(h, float)
        self.slices, self.per_point = int(slices), int(per_point)
        rng = np.random.default_rng(seed)
        n_h, K, N = self.h.size, self.slices, self.per_point
        # aperture: stratified in rings (sqrt for equal areas) and turned at random per point
        j = (np.arange(N) + rng.random((n_h, K, N))) / N
        rr = objective.aperture_radius_m * np.sqrt(j)
        phi = 2.399963 * np.arange(N) + 2 * np.pi * rng.random((n_h, K, 1))
        ax, ay = rr * np.cos(phi), rr * np.sin(phi)
        # across the slit: stratified within each slice, in an order unrelated to the aperture's
        order = np.argsort(rng.random((n_h, K, N)), axis=-1)
        frac = (np.arange(K)[None, :, None] + (order + rng.random((n_h, K, N))) / N) / K
        self.slice_centre = (np.arange(K) + 0.5) / K - 0.5   # in slit widths, -0.5 .. 0.5
        m = objective.magnification
        blur = objective.blur_um * 1e-6 * m
        ex, ey = blur * rng.standard_normal((2, n_h, K, N))
        target = np.stack([
            np.broadcast_to(objective.line_x(self.h)[:, None, None], (n_h, K, N)) + ex,
            (frac - 0.5) * objective.line_width_m + ey,
            np.full((n_h, K, N), objective.focus_mm * 1e-3)], -1)
        origin = np.stack([ax, ay, np.zeros_like(ax)], -1)
        self.reach = np.abs(target[..., :2]).reshape(-1, 2).max(0)  # furthest x, y in the focal plane
        d = target - origin
        self.origin = origin.reshape(-1, 3)                  # camera frame, m
        self.direction = (d / np.linalg.norm(d, axis=-1, keepdims=True)).reshape(-1, 3)
        self.shape = (n_h, K, N)

    def world(self, camera_pose):
        """Origins and directions in the frame camera_pose (4 x 4, camera -> world) maps to."""
        R, t = camera_pose[:3, :3], camera_pose[:3, 3]
        return self.origin @ R.T + t, self.direction @ R.T

    def reachable(self, scene, camera_pose):
        """Indices of the solids these rays can reach from camera_pose: those that come
        near the thin fan of rays (a cheap, conservative cull)."""
        R, t = camera_pose[:3, :3], camera_pose[:3, 3]
        a, zf = self.obj.aperture_radius_m, self.obj.focus_mm * 1e-3
        keep = []
        for k, s in enumerate(scene.solids):
            b = s.bound()
            if b is None:
                keep.append(k)
                continue
            cc = R.T @ (np.asarray(b[0], float) - t)
            ex, ey, ez = (s.extent(R[:, i]) for i in range(3))
            z = cc[2] + ez  # the solid's far side
            if z <= 0:
                continue
            spread = a * (1 + z / zf)  # how far a ray can be from the focal-plane point it aims at
            rx, ry = self.reach * z / zf
            if abs(cc[0]) <= ex + spread + rx and abs(cc[1]) <= ey + spread + ry:
                keep.append(k)
        return keep

    def weights(self, scene, camera_pose, n_materials):
        """[slices, h, materials]: each material's share of the light at every slit point,
        times its shading (so a row sums to the mean shading: 1 under uniform light).
        Also returns the mean camera z (m) of what each slit point sees, NaN where nothing."""
        o, d = self.world(camera_pose)
        hits = scene.cast(o, d, self.reachable(scene, camera_pose))
        n_h, K, N = self.shape
        idx = (np.arange(n_h * K).repeat(N)) * n_materials + hits.material
        w = np.bincount(idx, weights=hits.shading, minlength=n_h * K * n_materials) / N
        w = w.reshape(n_h, K, n_materials).transpose(1, 0, 2)
        hit = np.isfinite(hits.t).reshape(n_h, K * N)
        z = np.where(hit, (hits.t * self.direction[:, 2]).reshape(n_h, K * N), 0.0)
        with np.errstate(invalid="ignore", divide="ignore"):
            depth = z.sum(1) / hit.sum(1)
        return w, depth
