"""A calibration scan plan: viewpoints that look at the tag board from many directions.

The plan is an ordinary scan_sweep plan (ros2/so101_scan_sweep/so101_scan_sweep/plan.py), so
the calibration scan is taken like any other scan. What makes it a calibration plan is the
spread of its viewpoints, which is what lets the solver tell the unknowns apart:

  - tilts from straight down to 35 degrees, from every side: turns about every axis, for the
    hand-eye solve and the pose camera's lens model
  - the scan line's centre on different parts of the board, so its corners land at every
    point along the slit
  - the board up to 12 mm nearer and farther than the scan line's focus: the line camera's
    field (k) and where its centre of projection sits only separate at several distances
  - one microstep a line (half the usual step), so the board's squares span about 30 lines

The arm has five joints, so a view's position and direction fix the slit's turn about it; the
solver gets that turn from the tilts and azimuths instead.
"""

from __future__ import annotations

import math

import numpy as np

from . import repo

TABLE_Z = -0.0024          # the table top in base_link (the SO-101 base's underside)
BOARD_CENTRE = (0.26, 0.0)  # the ring plan's target: well inside the arm's reach

SWEEP = {"start_angle_deg": -6.0, "steps_per_line": 1, "n_lines": 213, "line_period_s": 0.0333}
MOVE = {"max_joint_speed_deg": 20.0, "min_move_s": 1.5, "settle_s": 1.0}

# (tilt deg, azimuth deg, offset along x mm, offset along y mm, height above the board mm): the
# scan line's centre goes to the board centre plus the offset; a positive height puts the board
# that much farther than the line's focus
VIEWS = [
    (0, 0, 0, 0, 0), (0, 0, -30, -25, 12), (0, 0, 30, 25, -12), (0, 0, 30, -25, 0), (0, 0, -30, 25, 0),
    (0, 0, 0, 40, 6), (0, 0, 0, -40, -6), (0, 0, 40, 0, 6), (0, 0, -40, 0, -6),
    (20, 60, -13, 7, 10), (20, 90, 0, 0, 0), (20, 120, -13, -8, 0), (20, 180, 0, -15, -10),
    (20, 240, 13, -8, 10), (20, 270, 0, 0, -6), (20, 300, 13, 8, 0),
    (35, 90, 0, 0, 12), (35, 120, 0, 0, 0), (35, 150, 0, 0, -12), (35, 210, 0, 0, 0), (35, 240, 0, 0, 6),
    (35, 270, 0, 0, 12),
]

# starting guesses for IK (make_plan's scan pose family)
SEEDS = [(0, -0.2, 1.0, -0.6, -1.52), (0, 0.5, 0.0, -0.4, -1.52), (0, 0.2, 0.6, -1.0, -1.52),
         (0, -0.2, 1.0, -0.6, 1.62), (0, 0.5, 0.3, -1.2, -1.52), (0, -0.6, 1.4, -0.4, -1.52)]


def view_direction(tilt, azimuth):
    return np.array([math.sin(tilt) * math.cos(azimuth), math.sin(tilt) * math.sin(azimuth), -math.cos(tilt)])


def lowest_point(robot, q):
    """Lowest z of the head's collision box and the arm's link origins, in base_link."""
    zs = [robot.fk(n, q)[2, 3] for n in ("upper_arm_link", "lower_arm_link", "wrist_link", "wrist_roll_link")]
    head = robot.fk("scan_head_link", q)
    col = robot.root.find("link[@name='scan_head_link']/collision")
    size = np.array([float(v) for v in col.find("geometry/box").get("size").split()])
    centre = np.array([float(v) for v in col.find("origin").get("xyz").split()])
    for corner in np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3):
        zs.append((head @ np.append(centre + corner * size / 2, 1.0))[2])
    return min(zs)


def solve_view(robot, target, direction, previous=None, clearance=0.012):
    """Arm joints that put the scan line's centre on target, looking along direction (or None)."""
    kin = repo.kinematics()
    limits = robot.limits()
    seeds = [dict(zip(repo.ARM_JOINTS, s)) for s in SEEDS]
    if previous:
        seeds.insert(0, dict(previous))
    best = None
    for q0 in seeds:
        q, err = kin.solve_ik(robot, "scan_line_frame", repo.ARM_JOINTS, q0, position=target, z_axis=direction,
                              limits=limits)
        if err > 1e-5 or np.dot(robot.fk("scan_line_frame", q)[:3, 2], direction) < 0.99:
            continue
        if lowest_point(robot, q) < TABLE_Z + clearance:
            continue
        cost = sum((q[j] - (previous or q0)[j]) ** 2 for j in repo.ARM_JOINTS)
        if best is None or cost < best[1]:
            best = (q, cost)
    return best[0] if best else None


def make_plan(robot, centre=BOARD_CENTRE, board_z=TABLE_Z, views=VIEWS, name="headcal", log=print):
    """The plan as a dict (scan_sweep's YAML layout)."""
    out, previous = [], None
    for tilt, az, dx, dy, dz in views:
        target = np.array([centre[0] + dx * 1e-3, centre[1] + dy * 1e-3, board_z + dz * 1e-3])
        q = solve_view(robot, target, view_direction(math.radians(tilt), math.radians(az)), previous)
        label = "t%d_a%d_%+d_%+d_%+d" % (tilt, az, round(dx), round(dy), round(dz))
        if q is None:
            log("skipped %s: out of reach or too close to the table" % label)
            continue
        previous = q
        out.append({"name": label, "joints_deg": {j: round(math.degrees(q[j]), 3) for j in repo.ARM_JOINTS}})
    return {"name": name, "home_mirror": True, "target_m": [float(centre[0]), float(centre[1]), float(board_z)],
            "move": dict(MOVE), "sweep": dict(SWEEP), "camera": {"record": "required", "references": ""},
            "viewpoints": out}


def plan_yaml(plan, centre=BOARD_CENTRE):
    import yaml
    head = ("# Hand-eye calibration scan, from `python -m headcal plan`. Put the tag board flat on the\n"
            "# table, its centre at x = %.3f, y = %.3f m in base_link (the centre ticks on its edges),\n"
            "# light it evenly, start the pose-camera recorder, then run this plan with scan_sweep.\n"
            "# Every viewpoint looks at the board; see calibration/headcal/README.md.\n" % tuple(centre))
    return head + yaml.safe_dump(plan, sort_keys=False, default_flow_style=None)


def mirror_angles(sweep):
    """Reported mirror angle of each line (rad): 6400 microsteps a turn."""
    step = 2.0 * math.pi / 6400.0
    return math.radians(sweep["start_angle_deg"]) + np.arange(sweep["n_lines"]) * sweep["steps_per_line"] * step
