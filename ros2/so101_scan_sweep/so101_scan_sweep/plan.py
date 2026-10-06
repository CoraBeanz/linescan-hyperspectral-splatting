"""Scan plans: where the arm goes, and the mirror sweep it runs at each viewpoint.

A plan is a YAML file written by hand or by `make_plan`; plans/ has examples. Angles are in
degrees, since that is what people write; everything is converted to radians on loading.

    name: one_view
    output_dir: ~/so101_scan/scans     # a new folder <name>_<date>-<time> per run
    move:
      max_joint_speed_deg: 30          # the joint that moves furthest sets each move's duration
      min_move_s: 1.0                  # no move quicker than this
      settle_s: 0.5                    # wait after a move before sweeping, while the arm stops wobbling
    sweep:                             # defaults for every viewpoint
      start_angle_deg: -6.0            # mirror angle of the first line; 0 is the 45 deg rest
      steps_per_line: 2                # microsteps between lines; negative sweeps the other way
      n_lines: 107
      line_period_s: 0.0333            # one camera frame per line
    viewpoints:
      - name: above
        joints_deg: {shoulder_pan: 0, shoulder_lift: 0, elbow_flex: 50, wrist_flex: -50, wrist_roll: -87}
        sweep: {n_lines: 50}           # optional, overrides the defaults for this viewpoint
"""

import copy
import math
import os
from dataclasses import dataclass, field

import yaml

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

DEFAULTS = {
    "name": "scan",
    "output_dir": os.path.join(os.environ.get("SO101_SCAN_DATA", "~/so101_scan"), "scans"),
    "home_mirror": True,
    "move": {"max_joint_speed_deg": 30.0, "min_move_s": 1.0, "settle_s": 0.5},
    "sweep": {"start_angle_deg": -6.0, "steps_per_line": 2, "n_lines": 107, "line_period_s": 0.0333},
}


class PlanError(ValueError):
    pass


@dataclass
class Sweep:
    start_angle: float     # rad
    steps_per_line: int
    n_lines: int
    line_period: float     # s


@dataclass
class Viewpoint:
    name: str
    joints: dict           # rad, all five arm joints; empty to sweep wherever the arm is
    sweep: Sweep


@dataclass
class Plan:
    name: str
    output_dir: str
    home_mirror: bool
    max_joint_speed: float  # rad/s
    min_move_time: float    # s
    settle_time: float      # s
    viewpoints: list = field(default_factory=list)
    source: dict = field(default_factory=dict)  # the YAML as loaded, defaults filled in


def _merge(base, over):
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        out[k] = _merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def _sweep(d, where):
    unknown = set(d) - set(DEFAULTS["sweep"])
    if unknown:
        raise PlanError("%s: unknown sweep keys %s" % (where, sorted(unknown)))
    s = Sweep(math.radians(float(d["start_angle_deg"])), int(d["steps_per_line"]), int(d["n_lines"]),
              float(d["line_period_s"]))
    if s.n_lines < 1 or s.line_period <= 0 or s.steps_per_line == 0:
        raise PlanError("%s: need n_lines >= 1, line_period_s > 0 and steps_per_line != 0" % where)
    return s


def from_dict(data):
    d = _merge(DEFAULTS, data)
    vps = d.get("viewpoints") or []
    if not vps:
        raise PlanError("the plan has no viewpoints")
    plan = Plan(name=str(d["name"]), output_dir=os.path.expanduser(str(d["output_dir"])),
                home_mirror=bool(d["home_mirror"]),
                max_joint_speed=math.radians(float(d["move"]["max_joint_speed_deg"])),
                min_move_time=float(d["move"]["min_move_s"]), settle_time=float(d["move"]["settle_s"]),
                source=d)
    if plan.max_joint_speed <= 0:
        raise PlanError("max_joint_speed_deg must be positive")
    for i, v in enumerate(vps):
        name = str(v.get("name", "view%d" % i))
        joints = v.get("joints_deg") or {}
        missing = set(ARM_JOINTS) - set(joints)
        extra = set(joints) - set(ARM_JOINTS)
        if joints and (missing or extra):
            raise PlanError("viewpoint %s: joints_deg needs exactly %s (missing %s, unknown %s)"
                            % (name, ARM_JOINTS, sorted(missing), sorted(extra)))
        plan.viewpoints.append(Viewpoint(
            name, {j: math.radians(float(joints[j])) for j in ARM_JOINTS} if joints else {},
            _sweep(_merge(d["sweep"], v.get("sweep")), "viewpoint %s" % name)))
    return plan


def load(path):
    with open(os.path.expanduser(path)) as f:
        return from_dict(yaml.safe_load(f) or {})


def check_limits(plan, limits):
    """Problems with the plan against the URDF's joint limits, as a list of strings."""
    out = []
    for v in plan.viewpoints:
        for j, q in v.joints.items():
            lo, hi = limits[j]
            if not lo <= q <= hi:
                out.append("%s: %s = %.1f deg is outside %.1f..%.1f" % (
                    v.name, j, math.degrees(q), math.degrees(lo), math.degrees(hi)))
    return out
