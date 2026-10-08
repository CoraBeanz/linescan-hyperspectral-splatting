"""Coverage planning: viewpoints that sweep the scan line across every side of an object.

    ros2 run so101_scan_sweep make_plan --object 0.26 0 --size 0.06 0.06 0.012 --out ~/so101_scan/plans/relief.yaml

The object is a box in base_link, standing on the table unless told otherwise. Its top and
four sides are covered with points a few millimetres apart (the bottom sits on the table),
and each candidate view, a direction (tilt off straight down, and azimuth) aimed where that
direction meets the box, is solved for the arm's joints like make_plan's views. A view sees a
point when, at some line of its sweep, the point is

  * inside the scan line's fan: within the slit's length across and the sweep's lines along;
  * in focus: within focus_depth of the scene distance. The objective is f/4 with a 16 mm focal
    length, so 4 mm across: 30 mm out of focus at 0.15 m blurs a point over 0.8 mm, two of the
    50 um slit's 0.42 mm footprints, which is about where the splat stops gaining detail;
  * facing the line camera within max_incidence: a surface seen more obliquely gives a
    stretched, dim line.

The box is convex, so a point that faces the camera isn't hidden by the rest of the box; the
arm and the head are not checked for hiding it. Views whose arm or head would come within
the clearance of the table or the object, or that the arm can't reach, are left out.

Then views are picked greedily, each the one that sees the most points still seen by fewer
than views_per_point views (a splat needs a point from more than one direction to place it in
depth), until every point that some view can see has enough, or max_views. The plan visits
them in an order that keeps each move short, and says how much of the surface it covers.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from so101_scan_sweep.line_log import LinePoser
from so101_scan_sweep.plan import ARM_JOINTS

RAD_PER_STEP = 2.0 * math.pi / 6400     # a mirror microstep; the ESP32's INFO says, and it has been 6400 a turn
FOCUS_DEPTH = 0.03                      # m either side of the scene distance (see above)
MAX_INCIDENCE = math.radians(60.0)
FACES = ("top", "+x", "-x", "+y", "-y")


@dataclass
class Box:
    centre: np.ndarray       # m, base_link
    size: np.ndarray         # m, along base_link x, y, z

    @classmethod
    def standing(cls, x, y, size, z=None, table_z=None):
        """A box whose bottom's middle is at (x, y, z), z the table top by default."""
        size = np.asarray(size, float)
        z = table_z if z is None else z
        return cls(np.array([x, y, z + size[2] / 2.0]), size)

    @property
    def lo(self):
        return self.centre - self.size / 2.0

    @property
    def hi(self):
        return self.centre + self.size / 2.0

    def corners(self):
        return np.array([[x, y, z] for x in (self.lo[0], self.hi[0]) for y in (self.lo[1], self.hi[1])
                         for z in (self.lo[2], self.hi[2])])

    def contains(self, points, margin=0.0):
        p = np.atleast_2d(points)
        return np.all((p >= self.lo - margin) & (p <= self.hi + margin), axis=1)

    def ray_hit(self, origin, direction):
        """Where a ray first meets the box, or None (the slab method)."""
        near, far = -np.inf, np.inf
        for k in range(3):
            if abs(direction[k]) < 1e-12:
                if not self.lo[k] <= origin[k] <= self.hi[k]:
                    return None
                continue
            t1, t2 = (self.lo[k] - origin[k]) / direction[k], (self.hi[k] - origin[k]) / direction[k]
            near, far = max(near, min(t1, t2)), min(far, max(t1, t2))
        if near > far or far < 0:
            return None
        return origin + max(near, 0.0) * np.asarray(direction)

    def surface(self, spacing=0.005):
        """Points on the top and the four sides, about `spacing` apart, their outward normals and
        which face each is on."""
        pts, normals, faces = [], [], []

        def grid(a_len, b_len):
            na, nb = max(1, int(round(a_len / spacing))), max(1, int(round(b_len / spacing)))
            a = (np.arange(na) + 0.5) / na - 0.5
            b = (np.arange(nb) + 0.5) / nb - 0.5
            return np.array(np.meshgrid(a * a_len, b * b_len)).reshape(2, -1).T

        c, s = self.centre, self.size
        for name, axis, sign in (("top", 2, 1), ("+x", 0, 1), ("-x", 0, -1), ("+y", 1, 1), ("-y", 1, -1)):
            others = [k for k in range(3) if k != axis]
            uv = grid(s[others[0]], s[others[1]])
            p = np.tile(c, (len(uv), 1))
            p[:, others[0]] += uv[:, 0]
            p[:, others[1]] += uv[:, 1]
            p[:, axis] += sign * s[axis] / 2.0
            n = np.zeros(3)
            n[axis] = sign
            pts.append(p)
            normals.append(np.tile(n, (len(p), 1)))
            faces += [name] * len(p)
        return np.vstack(pts), np.vstack(normals), np.array(faces)


def sweep_angles(sweep, rad_per_step=RAD_PER_STEP):
    """The mirror angle of every line of a sweep (plan.Sweep)."""
    return sweep.start_angle + np.arange(sweep.n_lines) * sweep.steps_per_line * rad_per_step


def sweep_sees(poser, joints, sweep, points, normals, rad_per_step=RAD_PER_STEP, focus_depth=FOCUS_DEPTH,
               max_incidence=MAX_INCIDENCE, samples=24):
    """Which points one viewpoint's sweep sees. Lines are sampled; each sampled line stands for
    the lines halfway to its neighbours (the view turns twice the mirror)."""
    angles = sweep_angles(sweep, rad_per_step)
    pick = np.unique(np.round(np.linspace(0, len(angles) - 1, min(samples, len(angles)))).astype(int))
    gap = 2.0 * abs(sweep.steps_per_line) * rad_per_step * (np.diff(pick).max() if len(pick) > 1 else 1)
    tan_slit = poser.half_line / poser.scene_distance
    cos_inc = math.cos(max_incidence)
    seen = np.zeros(len(points), bool)
    for i in pick:
        _, cam = poser.poses(joints, float(angles[i]))
        r, t = cam[:3, :3], cam[:3, 3]
        x, y, z = ((points - t) @ r).T
        to_cam = t - points
        to_cam /= np.linalg.norm(to_cam, axis=1, keepdims=True)
        seen |= ((z > 0) & (np.abs(z - poser.scene_distance) <= focus_depth) & (np.abs(x) <= tan_slit * z)
                 & (np.abs(np.arctan2(y, z)) <= gap / 2.0 + 1e-12) & ((normals * to_cam).sum(axis=1) >= cos_inc))
    return seen


def view_direction(tilt, azimuth):
    return np.array([math.sin(tilt) * math.cos(azimuth), math.sin(tilt) * math.sin(azimuth), -math.cos(tilt)])


def aim_points(box, direction, tile):
    """Where views along `direction` aim: the box's silhouette across the view, cut into tiles
    about `tile` wide, each aimed where its middle ray first meets the box."""
    d = direction / np.linalg.norm(direction)
    u = np.cross(d, [0.0, 0.0, 1.0])
    u = u / np.linalg.norm(u) if np.linalg.norm(u) > 1e-6 else np.array([1.0, 0.0, 0.0])
    v = np.cross(d, u)
    rel = box.corners() - box.centre
    out = []
    pu, pv = rel @ u, rel @ v
    nu = max(1, int(math.ceil((pu.max() - pu.min()) / tile - 1e-9)))
    nv = max(1, int(math.ceil((pv.max() - pv.min()) / tile - 1e-9)))
    for a in (np.arange(nu) + 0.5) / nu - 0.5:
        for b in (np.arange(nv) + 0.5) / nv - 0.5:
            o = box.centre + a * (pu.max() - pu.min()) * u + b * (pv.max() - pv.min()) * v
            hit = box.ray_hit(o - d * 10.0, d)
            if hit is not None:
                out.append(hit)
    return out


@dataclass
class Candidate:
    name: str
    tilt: float              # rad
    azimuth: float
    aim: np.ndarray
    joints: dict
    seen: np.ndarray = field(repr=False)


def plan_coverage(robot, box, sweep, tilts_deg, azimuths_deg, solve_view, body_points, views_per_point=2,
                  max_views=12, clearance=0.01, spacing=0.005, focus_depth=FOCUS_DEPTH, max_incidence=MAX_INCIDENCE,
                  rad_per_step=RAD_PER_STEP, log=print):
    """Pick viewpoints covering the box. solve_view(target, direction) -> (joints, reason) and
    body_points(joints) -> points on the arm and head come from make_plan. Returns (chosen
    candidates in visiting order, stats)."""
    poser = LinePoser(robot)
    points, normals, faces = box.surface(spacing)
    swing = abs(sweep_angles(sweep, rad_per_step)[[0, -1]]).max() * 2.0
    tile = min(2.0 * poser.half_line, 2.0 * math.tan(swing) * poser.scene_distance) * 0.9
    cands = []
    for tilt in tilts_deg:
        for az in (azimuths_deg if tilt else [0.0]):
            d = view_direction(math.radians(tilt), math.radians(az))
            label = "down" if tilt == 0 else "tilt%g_az%g" % (tilt, az)
            aims = aim_points(box, d, tile)
            for k, aim in enumerate(aims):
                name = label if len(aims) == 1 else "%s_%d" % (label, k + 1)
                q, reason = solve_view(aim, d)
                if q is None:
                    log("skipped %s: %s" % (name, reason))
                    continue
                if box.contains(body_points(q), clearance).any():
                    log("skipped %s: the arm or head would come within %.0f mm of the object" % (name, clearance * 1e3))
                    continue
                seen = sweep_sees(poser, q, sweep, points, normals, rad_per_step, focus_depth, max_incidence)
                if not seen.any():
                    log("skipped %s: its sweep sees none of the object" % name)
                    continue
                cands.append(Candidate(name, math.radians(tilt), math.radians(az), aim, q, seen))

    count = np.zeros(len(points), int)
    chosen = []
    seeable = np.any([c.seen for c in cands], axis=0) if cands else np.zeros(len(points), bool)
    while len(chosen) < max_views:
        left = [c for c in cands if c not in chosen]
        if not left:
            break
        need = count < views_per_point
        gains = [int((c.seen & need).sum()) for c in left]
        best = max(range(len(left)), key=lambda i: (gains[i], -left[i].tilt))
        if gains[best] == 0:
            break
        chosen.append(left[best])
        count += left[best].seen

    order = visiting_order(chosen)
    stats = dict(surface_points=int(len(points)), seen=round(float((count > 0).mean()), 3),
                 seen_enough=round(float((count >= views_per_point).mean()), 3), views_per_point=views_per_point,
                 candidates=len(cands), faces={})
    for f in FACES:
        on = faces == f
        stats["faces"][f] = dict(seen=round(float((count[on] > 0).mean()), 3),
                                 seen_enough=round(float((count[on] >= views_per_point).mean()), 3),
                                 seeable=round(float(seeable[on].mean()), 3))
    return order, stats, (points, normals, faces, count)


def visiting_order(views):
    """Start with the view closest to straight down, then always the nearest in joint space (the
    joint that moves furthest sets a move's time)."""
    if not views:
        return []
    left = sorted(views, key=lambda c: c.tilt)
    order = [left.pop(0)]
    while left:
        prev = order[-1].joints
        nxt = min(left, key=lambda c: max(abs(c.joints[j] - prev[j]) for j in ARM_JOINTS))
        left.remove(nxt)
        order.append(nxt)
    return order
