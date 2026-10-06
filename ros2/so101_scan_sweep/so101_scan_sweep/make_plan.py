"""make_plan: write a scan plan whose viewpoints all look at one spot from different directions.

    ros2 run so101_scan_sweep make_plan --target 0.26 0 0.0 --tilts 0 25 --azimuths 0 90 180 270 \\
        --out ~/so101_scan/plans/ring.yaml

A Gaussian splat needs the scene seen from several directions. For each (tilt, azimuth) this
finds the arm joints that put the middle of the in-focus scan line on --target (base_link, m;
the table top is at z = -0.0024) with the line camera looking along that direction: tilt 0
looks straight down, tilt 25 leans the view 25 deg off vertical, and the azimuth says which
way it leans (0: away from the base, along +X; 90: toward +Y, the arm's left). The SO-101 has
five joints, enough for the view's position and direction; the slit's turn about the view
follows from them, and the plan lists it.

Views the arm can't reach, or that bring the arm or head within --clearance of the table, are
left out with a note. Nothing moves: check the plan in RViz first (`scan_sweep` with mock
hardware draws every line).
"""

import argparse
import math
import os
import sys

import numpy as np
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from so101_scan_description.kinematics import Robot, solve_ik
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


def lowest_point(robot, q):
    """Lowest z of the head's collision box and the arm's link origins, in base_link."""
    links = ("upper_arm_link", "lower_arm_link", "wrist_link", "wrist_roll_link")
    zs = [robot.fk(name, q)[2, 3] for name in links]
    head = robot.fk("scan_head_link", q)
    col = robot.root.find("link[@name='scan_head_link']/collision")
    size = np.array([float(v) for v in col.find("geometry/box").get("size").split()])
    centre = np.array([float(v) for v in col.find("origin").get("xyz").split()])
    for corner in np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3):
        zs.append((head @ np.append(centre + corner * size / 2, 1.0))[2])
    return min(zs)


def solve_view(robot, target, direction, previous=None, clearance=0.01, table_z=TABLE_Z):
    """Joints for one view, or (None, reason)."""
    limits = robot.limits()
    seeds = [dict(zip(ARM_JOINTS, s)) for s in SEEDS]
    if previous:
        seeds.insert(0, dict(previous))
    best, reason = None, "the arm can't reach it"
    for q0 in seeds:
        q, err = solve_ik(robot, "scan_line_frame", ARM_JOINTS, q0, position=target, z_axis=direction,
                          limits=limits)
        if err > 1e-5 or np.dot(robot.fk("scan_line_frame", q)[:3, 2], direction) < 0.99:
            continue
        if lowest_point(robot, q) < table_z + clearance:
            reason = "the arm or head would come within %.0f mm of the table" % (clearance * 1e3)
            continue
        cost = sum((q[j] - (previous or q0)[j]) ** 2 for j in ARM_JOINTS)
        if best is None or cost < best[1]:
            best = (q, cost)
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


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--target", type=float, nargs=3, default=[0.26, 0.0, TABLE_Z], metavar=("X", "Y", "Z"),
                    help="where the middle of the scan line goes, base_link metres")
    ap.add_argument("--tilts", type=float, nargs="+", default=[0.0, 25.0], help="deg off straight down")
    ap.add_argument("--azimuths", type=float, nargs="+", default=[0.0, 90.0, 180.0, 270.0], help="deg")
    ap.add_argument("--clearance", type=float, default=0.01, help="m kept from the table")
    ap.add_argument("--name", default="ring")
    ap.add_argument("--out", help="plan file to write (default: print it)")
    args, _ = ap.parse_known_args(argv)
    plan = make_plan(load_robot(), np.array(args.target), args.tilts, args.azimuths, args.name, args.clearance,
                     log=lambda s: print(s, file=sys.stderr))
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
