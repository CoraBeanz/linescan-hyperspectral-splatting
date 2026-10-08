"""The arm's soft limits in Python, for checking scan plans before they run.

The servo driver keeps every goal it sends inside two soft limits (so101_scan_hardware's
arm_model.hpp), and this is the same model with the same numbers, which both read from the
<ros2_control> block of the URDF:

  * the workspace: the origins of the arm's links and the corners of the head's collision box
    stay table_clearance above the table, and the head's corners stay out of a cylinder around
    the shoulder_pan axis that holds the base, the shoulder servo and the electronics;
  * gravity: no joint holds more than max_gravity_load of the servo's stall torque against the
    weight of the links and the head beyond it, from the masses in the URDF (the head's comes
    from the CAD model's mass budget).

A plan that passes here is one the driver lets through, rather than one that stops halfway.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from so101_scan_description.kinematics import rpy_matrix

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRAVITY = 9.81

# The driver's defaults (arm_model.hpp, feetech_sts_system.cpp); the URDF sets them all.
DEFAULTS = {
    "table_z": -0.0024, "table_clearance": 0.01, "base_keepout_radius": 0.08, "base_keepout_top": 0.14,
    "stall_torque": 1.62, "warn_gravity_load": 0.5, "max_gravity_load": 0.6,
    "workspace_links": "upper_arm_link lower_arm_link wrist_link wrist_roll_link",
    "workspace_box_link": "scan_head_link",
}


@dataclass
class Limits:
    table_z: float
    table_clearance: float
    base_keepout_radius: float
    base_keepout_top: float
    stall_torque: float
    warn_gravity_load: float
    max_gravity_load: float
    workspace_links: list
    workspace_box_link: str

    @classmethod
    def from_urdf(cls, robot):
        """The numbers in the URDF's <ros2_control><hardware>, or the driver's defaults."""
        values = dict(DEFAULTS)
        hw = robot.root.find("ros2_control/hardware")
        if hw is not None:
            values.update({p.get("name"): (p.text or "").strip() for p in hw.findall("param")
                           if p.get("name") in DEFAULTS})
        out = {k: (str(v) if k == "workspace_box_link" else float(v)) for k, v in values.items()
               if k != "workspace_links"}
        return cls(workspace_links=str(values["workspace_links"]).split(), **out)


@dataclass
class Evaluation:
    workspace_margin: float       # m inside the workspace; negative is outside
    workspace_limit: str          # the limit closest to being crossed: "table" or "base"
    workspace_part: str           # which part: a link name, or "head"
    torque: dict = field(default_factory=dict)        # N m of gravity on each joint
    gravity_load: dict = field(default_factory=dict)  # that as a fraction of stall_torque

    def worst_gravity(self):
        """(joint, fraction of stall) for the joint that holds the most."""
        return max(self.gravity_load.items(), key=lambda kv: kv[1])


class ArmModel:
    def __init__(self, robot, limits=None, joints=ARM_JOINTS):
        self.robot = robot
        self.limits = limits or Limits.from_urdf(robot)
        self.joints = list(joints)
        self.masses = []   # (link, kg, centre of mass in the link frame)
        for name, link in robot.links.items():
            inertial = link.find("inertial")
            if inertial is None:
                continue
            origin = inertial.find("origin")
            com = [float(v) for v in (origin.get("xyz") if origin is not None else "0 0 0").split()]
            self.masses.append((name, float(inertial.find("mass").get("value")), np.array(com)))
        self.below = {j: self._below(robot.joints[j]["child"]) for j in self.joints}
        box = robot.root.find("link[@name='%s']/collision" % self.limits.workspace_box_link)
        if box is None or box.find("geometry/box") is None:
            raise ValueError("link %s needs a box collision geometry" % self.limits.workspace_box_link)
        size = np.array([float(v) for v in box.find("geometry/box").get("size").split()])
        origin = box.find("origin")
        centre = np.array([float(v) for v in (origin.get("xyz") if origin is not None else "0 0 0").split()])
        rpy = [float(v) for v in (origin.get("rpy") if origin is not None and origin.get("rpy") else "0 0 0").split()]
        rot = rpy_matrix(*rpy)
        self.corners = [centre + rot @ (np.array(c) * size / 2)
                        for c in ((x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1))]

    def _below(self, link):
        out, grew = {link}, True
        while grew:
            grew = False
            for j in self.robot.joints.values():
                if j["parent"] in out and j["child"] not in out:
                    out.add(j["child"])
                    grew = True
        return out

    def evaluate(self, q):
        """Workspace margin and gravity torques at joint positions q ({joint: rad})."""
        lim = self.limits
        r = self.robot
        best = [math.inf, "", ""]

        def consider(margin, limit, part):
            if margin < best[0]:
                best[:] = [margin, limit, part]

        floor = lim.table_z + lim.table_clearance
        for name in lim.workspace_links:
            consider(r.fk(name, q)[2, 3] - floor, "table", name)
        pan = r.joints["shoulder_pan"]
        axis_at = (r.fk(pan["parent"], q) @ pan["origin"])[:3, 3]
        head = r.fk(lim.workspace_box_link, q)
        for corner in self.corners:
            w = (head @ np.append(corner, 1.0))[:3]
            consider(w[2] - floor, "table", "head")
            dr = math.hypot(w[0] - axis_at[0], w[1] - axis_at[1]) - lim.base_keepout_radius
            dz = w[2] - lim.base_keepout_top
            consider(math.hypot(dr, dz) if dr > 0 and dz > 0 else max(dr, dz), "base", "head")

        torque = {}
        g = np.array([0.0, 0.0, -GRAVITY])
        poses = {name: r.fk(name, q) for name, _, _ in self.masses}
        for jn in self.joints:
            j = r.joints[jn]
            frame = r.fk(j["parent"], q) @ j["origin"]
            axis = frame[:3, :3] @ (np.array(j["axis"]) / np.linalg.norm(j["axis"]))
            tau = 0.0
            for name, mass, com in self.masses:
                if name in self.below[jn]:
                    w = (poses[name] @ np.append(com, 1.0))[:3]
                    tau += float(np.dot(np.cross(w - frame[:3, 3], mass * g), axis))
            torque[jn] = tau
        return Evaluation(best[0], best[1], best[2], torque,
                          {jn: abs(t) / lim.stall_torque for jn, t in torque.items()})

    # --- what's wrong with a pose, in words -----------------------------------------------

    def problems(self, e):
        """The soft limits evaluation e breaks, as sentences."""
        lim = self.limits
        out = []
        if e.workspace_margin < 0:
            gap = lim.table_clearance + e.workspace_margin
            if e.workspace_limit == "table" and gap < 0:
                out.append("the %s goes %.1f mm below the table top" % (e.workspace_part, -gap * 1e3))
            elif e.workspace_limit == "table":
                out.append("the %s comes within %.1f mm of the table (keeps %.0f mm)" % (
                    e.workspace_part, gap * 1e3, lim.table_clearance * 1e3))
            else:
                out.append("the %s goes %.0f mm into the space kept clear around the base" % (
                    e.workspace_part, -e.workspace_margin * 1e3))
        for jn, load in e.gravity_load.items():
            if load > lim.max_gravity_load:
                out.append("%s holds %.0f%% of its stall torque against gravity (limit %.0f%%)" % (
                    jn, 100 * load, 100 * lim.max_gravity_load))
        return out

    def warnings(self, e):
        lim = self.limits
        return ["%s holds %.0f%% of its stall torque against gravity (warning at %.0f%%)" % (
                    jn, 100 * load, 100 * lim.warn_gravity_load)
                for jn, load in e.gravity_load.items() if lim.warn_gravity_load < load <= lim.max_gravity_load]

    def blocks(self, to, frm):
        """Whether the driver refuses a step from evaluation frm to evaluation to: one that goes,
        or goes further, out of the workspace or past the gravity budget."""
        lim = self.limits
        if to.workspace_margin < 0 and to.workspace_margin < frm.workspace_margin:
            return True
        return any(to.gravity_load[j] > lim.max_gravity_load and to.gravity_load[j] > frm.gravity_load[j]
                   for j in self.joints)
