"""check_plan: check a scan plan against the arm's soft limits before running it.

    ros2 run so101_scan_safety check_plan --plan ~/so101_scan/plans/ring.yaml

For every viewpoint it prints how far the arm stays inside its workspace and how much of the
servos' stall torque each joint holds against gravity, then checks the moves between
viewpoints, which the trajectory controller makes as straight lines in joint space, every 1
deg along the way. A plan fails where the servo driver would stop the arm short: a viewpoint
or a stretch of a move that takes the head or arm out of the workspace, or a joint past its
gravity budget (so101_scan_safety/model.py has the details, and the URDF the numbers).
scan_sweep and move_arm run the same check, from wherever the arm is, before they move it.

--start-deg checks the first move too, from that pose (joint order: shoulder_pan
shoulder_lift elbow_flex wrist_flex wrist_roll). The head is where scan_arm.launch.py puts it:
where calibration/headcal measured it if $SO101_SCAN_DATA/head_calibration.yaml exists, else
the CAD numbers; --head-calibration names another file, or none.
"""

import argparse
import math
import os
import sys
from dataclasses import dataclass, field

import yaml

from so101_scan_safety.model import ARM_JOINTS, ArmModel

PATH_STEP = math.radians(1.0)


@dataclass
class Report:
    rows: list = field(default_factory=list)       # (viewpoint name, Evaluation) per viewpoint with joints
    problems: list = field(default_factory=list)   # the plan fails
    warnings: list = field(default_factory=list)

    @property
    def ok(self):
        return not self.problems


def path(q0, q1, step=PATH_STEP):
    """Joint positions along the straight line from q0 to q1, q1 last."""
    n = max(1, int(math.ceil(max(abs(q1[j] - q0[j]) for j in ARM_JOINTS) / step)))
    return [{j: q0[j] + (q1[j] - q0[j]) * k / n for j in ARM_JOINTS} for k in range(1, n + 1)]


def check_move(model, q0, q1, e0=None):
    """The first place along the move from q0 to q1 where the driver would stop the arm, as
    (joint positions, evaluation there), or None."""
    prev = e0 or model.evaluate(q0)
    for q in path(q0, q1):
        e = model.evaluate(q)
        if model.blocks(e, prev):
            return q, e
        prev = e
    return None


def check_plan(model, viewpoints, start=None):
    """Check viewpoints [(name, {joint: rad})] in order, and the moves between them (from start,
    the arm's pose now, for the first one). Viewpoints without joints sweep wherever the arm is
    and are skipped."""
    report = Report()
    prev, prev_name = start, "the start"
    for name, joints in viewpoints:
        if not joints:
            continue
        e = model.evaluate(joints)
        report.rows.append((name, e))
        report.problems += ["viewpoint %s: %s" % (name, p) for p in model.problems(e)]
        report.warnings += ["viewpoint %s: %s" % (name, w) for w in model.warnings(e)]
        if prev is not None:
            stop = check_move(model, prev, joints)
            if stop is not None:
                q, e_stop = stop
                why = model.problems(e_stop) or ["a soft limit"]
                report.problems.append("moving from %s to %s: %s at %s deg" % (
                    prev_name, name, "; ".join(why), " ".join("%.0f" % math.degrees(q[j]) for j in ARM_JOINTS)))
        prev, prev_name = joints, name
    return report


def viewpoints_from_yaml(path_):
    """[(name, {joint: rad})] from a plan file (the format in so101_scan_sweep's plan.py)."""
    with open(os.path.expanduser(path_)) as f:
        data = yaml.safe_load(f) or {}
    out = []
    for i, v in enumerate(data.get("viewpoints") or []):
        joints = v.get("joints_deg") or {}
        out.append((str(v.get("name", "view%d" % i)), {j: math.radians(float(joints[j])) for j in ARM_JOINTS}
                    if joints else {}))
    return out


HEAD_CALIBRATION = os.path.join(os.environ.get("SO101_SCAN_DATA", os.path.expanduser("~/so101_scan")),
                                "head_calibration.yaml")


def load_robot(head_calibration=HEAD_CALIBRATION):
    """The URDF as scan_arm.launch.py builds it, and the head calibration it used ("" for the
    CAD numbers): the default file only if it exists, a named one always, none for none."""
    import xacro
    from ament_index_python.packages import get_package_share_directory

    from so101_scan_description.kinematics import Robot
    head = os.path.expanduser(head_calibration or "")
    if head.lower() in ("", "none", "cad") or (head_calibration == HEAD_CALIBRATION and not os.path.exists(head)):
        head = ""
    elif not os.path.exists(head):
        raise SystemExit("no head calibration at %s (--head-calibration none for the CAD numbers)" % head)
    mappings = {"use_mock_hardware": "true"}
    if head:
        mappings["head_calibration"] = head
    share = get_package_share_directory("so101_scan_description")
    return Robot(xacro.process_file(os.path.join(share, "urdf", "so101_scan.urdf.xacro"),
                                    mappings=mappings).toxml()), head


def format_report(model, report):
    lines = ["%-16s %10s   %s" % ("viewpoint", "workspace", "  ".join("%13s" % j[:13] for j in ARM_JOINTS))]
    for name, e in report.rows:
        lines.append("%-16s %7.0f mm   %s" % (name[:16], e.workspace_margin * 1e3, "  ".join(
            "%12.0f%%" % (100 * e.gravity_load[j]) for j in ARM_JOINTS)))
    lines.append("(workspace: mm to spare inside it; joints: gravity torque as a share of stall, limit %.0f%%)"
                 % (100 * model.limits.max_gravity_load))
    lines += ["warning: " + w for w in report.warnings]
    lines += ["PROBLEM: " + p for p in report.problems]
    lines.append("ok: the driver will let this plan through" if report.ok else
                 "the driver would stop the arm short of this plan")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--plan", required=True, help="plan YAML (see so101_scan_sweep/plans)")
    ap.add_argument("--start-deg", type=float, nargs=5, metavar="DEG", help="check the first move from here")
    ap.add_argument("--head-calibration", default=HEAD_CALIBRATION,
                    help="head_calibration.yaml from headcal, or none for the CAD numbers")
    args, _ = ap.parse_known_args(argv)
    robot, head = load_robot(args.head_calibration)
    print("scanner head: " + ("calibrated, from " + head if head else "the CAD numbers"))
    model = ArmModel(robot)
    start = {j: math.radians(v) for j, v in zip(ARM_JOINTS, args.start_deg)} if args.start_deg else None
    report = check_plan(model, viewpoints_from_yaml(args.plan), start)
    print(format_report(model, report))
    return 0 if report.ok else 1


if __name__ == "__main__":
    sys.exit(main())
