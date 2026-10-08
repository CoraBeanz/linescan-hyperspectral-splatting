"""make_plan: write a scan plan whose viewpoints all look at one spot from different directions,
or that cover an object.

    ros2 run so101_scan_sweep make_plan --target 0.26 0 0.0 --tilts 0 25 --azimuths 0 90 180 270 \\
        --out ~/so101_scan/plans/ring.yaml
    ros2 run so101_scan_sweep make_plan --object 0.26 0 --size 0.06 0.06 0.012 --out ~/so101_scan/plans/relief.yaml

A Gaussian splat needs the scene seen from several directions. For each (tilt, azimuth) this
finds the arm joints that put the middle of the in-focus scan line on --target (base_link, m;
the table top is at z = -0.0024) with the line camera looking along that direction: tilt 0
looks straight down, tilt 25 leans the view 25 deg off vertical, and the azimuth says which
way it leans (0: away from the base, along +X; 90: toward +Y, the arm's left). The SO-101 has
five joints, enough for the view's position and direction; the slit's turn about the view
follows from them, and the plan lists it.

Views the arm can't reach, or that bring the arm or head within --clearance of the table, are
left out with a note.

With --object X Y [Z] and --size W D H (a box in base_link, m, standing on the table unless Z
says where its bottom is) it plans for coverage instead (coverage.py): it tries views from
every --tilts and --azimuths aimed at the box, works out which of the box's top and sides each
one's sweep sees in focus, and keeps the fewest views that see every point --views-per-point
times, visiting them in an order that keeps the moves short. The plan lists what each view sees
and how much of each face is covered.

Nothing moves: check the plan with `scan_sweep --plan ... --dry-run`, which plays it on the
running stack without moving the arm or the mirror.
"""

import argparse
import math
import os
import sys
from functools import partial

import numpy as np
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from so101_scan_description.kinematics import Robot, solve_ik
from so101_scan_sweep import coverage as cov
from so101_scan_sweep import plan as plan_mod
from so101_scan_sweep.plan import ARM_JOINTS, DEFAULTS

TABLE_Z = -0.0024  # the bottom of the SO-101 base in base_link

# starting guesses for the solver: the scan pose family found by sampling the joint space
SEEDS = [(0, -0.2, 1.0, -0.6, -1.52), (0, 0.5, 0.0, -0.4, -1.52), (0, 0.2, 0.6, -1.0, -1.52),
         (0, -0.2, 1.0, -0.6, 1.62), (0, 0.5, 0.3, -1.2, -1.52), (0, -0.6, 1.4, -0.4, -1.52)]


def load_robot():
    share = get_package_share_directory("so101_scan_description")
    doc = xacro.process_file(os.path.join(share, "urdf", "so101_scan.urdf.xacro"),
                             mappings={"use_mock_hardware": "true"})
    return Robot(doc.toxml())


def view_direction(tilt, azimuth):
    return np.array([math.sin(tilt) * math.cos(azimuth), math.sin(tilt) * math.sin(azimuth), -math.cos(tilt)])


def body_points(robot, q, dense=False):
    """Points on the arm and head in base_link: the arm's link origins and the corners of the
    head's collision box (dense: and a grid over the box's faces)."""
    links = ("upper_arm_link", "lower_arm_link", "wrist_link", "wrist_roll_link")
    pts = [robot.fk(name, q)[:3, 3] for name in links]
    head = robot.fk("scan_head_link", q)
    col = robot.root.find("link[@name='scan_head_link']/collision")
    size = np.array([float(v) for v in col.find("geometry/box").get("size").split()])
    centre = np.array([float(v) for v in col.find("origin").get("xyz").split()])
    steps = [-1, -0.5, 0, 0.5, 1] if dense else [-1, 1]
    for corner in np.array(np.meshgrid(steps, steps, steps)).T.reshape(-1, 3):
        pts.append((head @ np.append(centre + corner * size / 2, 1.0))[:3])
    return np.array(pts)


def lowest_point(robot, q):
    """Lowest z of the head's collision box and the arm's link origins, in base_link."""
    return float(body_points(robot, q)[:, 2].min())


def solve_view(robot, target, direction, previous=None, clearance=0.01, table_z=TABLE_Z, iterations=300,
               first=False):
    """Joints for one view, or (None, reason). first: take the first seed that works rather than
    the one closest to `previous` (quicker, for trying many views)."""
    limits = robot.limits()
    seeds = [dict(zip(ARM_JOINTS, s)) for s in SEEDS]
    if previous:
        seeds.insert(0, dict(previous))
    best, reason = None, "the arm can't reach it"
    for q0 in seeds:
        q, err = solve_ik(robot, "scan_line_frame", ARM_JOINTS, q0, position=target, z_axis=direction,
                          limits=limits, iterations=iterations)
        if err > 1e-5 or np.dot(robot.fk("scan_line_frame", q)[:3, 2], direction) < 0.99:
            continue
        if lowest_point(robot, q) < table_z + clearance:
            reason = "the arm or head would come within %.0f mm of the table" % (clearance * 1e3)
            continue
        cost = sum((q[j] - (previous or q0)[j]) ** 2 for j in ARM_JOINTS)
        if best is None or cost < best[1]:
            best = (q, cost)
        if first:
            break
    return (best[0], None) if best else (None, reason)


def slit_direction(robot, q):
    """Which way the slit runs across the scene, as a compass angle in base_link (deg)."""
    x = robot.fk("line_camera_optical_frame", q)[:3, 0]
    return math.degrees(math.atan2(x[1], x[0]))


def make_plan(robot, target, tilts, azimuths, name, clearance=0.01, sweep=None, log=print):
    views, previous = [], None
    for tilt in tilts:
        for az in (azimuths if tilt else [0.0]):  # straight down has no azimuth
            d = view_direction(math.radians(tilt), math.radians(az))
            q, reason = solve_view(robot, target, d, previous, clearance)
            label = "down" if tilt == 0 else "tilt%g_az%g" % (tilt, az)
            if q is None:
                log("skipped %s: %s" % (label, reason))
                continue
            previous = q
            views.append({
                "name": label,
                "joints_deg": {j: round(math.degrees(q[j]), 2) for j in ARM_JOINTS},
                "slit_heading_deg": round(slit_direction(robot, q), 1)})
    plan = {"name": name, "target_m": [float(v) for v in target],
            "move": dict(DEFAULTS["move"]), "sweep": dict(sweep or DEFAULTS["sweep"]), "viewpoints": views}
    return plan


def make_coverage_plan(robot, box, tilts, azimuths, name, clearance=0.01, sweep=None, views_per_point=2,
                       max_views=12, spacing=0.005, log=print):
    """A plan whose viewpoints cover a coverage.Box; see coverage.py."""
    sweep = dict(sweep or DEFAULTS["sweep"])
    swp = plan_mod.from_dict({"sweep": sweep, "viewpoints": [{"name": "x"}]}).viewpoints[0].sweep
    views, stats, _ = cov.plan_coverage(
        robot, box, swp, tilts, azimuths, partial(solve_view, robot, clearance=clearance, iterations=60, first=True),
        partial(body_points, robot, dense=True), views_per_point, max_views, clearance, spacing, log=log)
    plan = {"name": name,
            "object": {"centre_m": [round(float(v), 4) for v in box.centre],
                       "size_m": [round(float(v), 4) for v in box.size]},
            "coverage": stats, "move": dict(DEFAULTS["move"]), "sweep": sweep, "viewpoints": []}
    for v in views:
        plan["viewpoints"].append({
            "name": v.name,
            "joints_deg": {j: round(math.degrees(v.joints[j]), 2) for j in ARM_JOINTS},
            "slit_heading_deg": round(slit_direction(robot, v.joints), 1),
            "aim_m": [round(float(a), 4) for a in v.aim],
            "sees": round(float(v.seen.mean()), 3)})
    return plan


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--target", type=float, nargs=3, default=[0.26, 0.0, TABLE_Z], metavar=("X", "Y", "Z"),
                    help="where the middle of the scan line goes, base_link metres")
    ap.add_argument("--object", type=float, nargs="+", metavar="X Y [Z]",
                    help="plan to cover a box: the middle of its bottom, base_link metres (Z: the table top)")
    ap.add_argument("--size", type=float, nargs=3, metavar=("W", "D", "H"),
                    help="the box's size along base_link x, y and z, metres")
    ap.add_argument("--tilts", type=float, nargs="+", help="deg off straight down (default 0 25; 0 20 35 50 "
                                                           "with --object)")
    ap.add_argument("--azimuths", type=float, nargs="+", help="deg (default 0 90 180 270; every 45 with --object)")
    ap.add_argument("--clearance", type=float, default=0.01, help="m kept from the table (and the object)")
    ap.add_argument("--views-per-point", type=int, default=2, help="with --object: views each point should get")
    ap.add_argument("--max-views", type=int, default=12, help="with --object: at most this many viewpoints")
    ap.add_argument("--spacing", type=float, default=0.005, help="with --object: m between surface points")
    ap.add_argument("--name", help="the plan's name (default: ring, or object)")
    ap.add_argument("--out", help="plan file to write (default: print it)")
    args, _ = ap.parse_known_args(argv)
    log = lambda s: print(s, file=sys.stderr)    # noqa: E731
    if args.object is not None:
        if len(args.object) not in (2, 3) or args.size is None:
            ap.error("--object takes X Y or X Y Z, and needs --size W D H")
        box = cov.Box.standing(args.object[0], args.object[1], args.size,
                               args.object[2] if len(args.object) == 3 else None, table_z=TABLE_Z)
        plan = make_coverage_plan(load_robot(), box, args.tilts or [0.0, 20.0, 35.0, 50.0],
                                  args.azimuths or list(range(0, 360, 45)), args.name or "object", args.clearance,
                                  views_per_point=args.views_per_point, max_views=args.max_views,
                                  spacing=args.spacing, log=log)
        c = plan["coverage"]
        log("%d viewpoints from %d candidates: %.0f%% of the surface seen, %.0f%% by %d or more views"
            % (len(plan["viewpoints"]), c["candidates"], 100 * c["seen"], 100 * c["seen_enough"],
               c["views_per_point"]))
        for face, f in c["faces"].items():
            if f["seen"] < 0.9:
                log("  the %s face: %.0f%% seen (%.0f%% could be from these tilts and azimuths)"
                    % (face, 100 * f["seen"], 100 * f["seeable"]))
    else:
        plan = make_plan(load_robot(), np.array(args.target), args.tilts or [0.0, 25.0],
                         args.azimuths or [0.0, 90.0, 180.0, 270.0], args.name or "ring", args.clearance, log=log)
    if not plan["viewpoints"]:
        print("no reachable views; try a target closer to the arm or smaller tilts", file=sys.stderr)
        return 1
    text = yaml.safe_dump(plan, sort_keys=False, default_flow_style=None)
    if args.out:
        path = os.path.expanduser(args.out)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            f.write(text)
        print("wrote %d viewpoints to %s" % (len(plan["viewpoints"]), args.out), file=sys.stderr)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
