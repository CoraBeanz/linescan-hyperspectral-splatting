"""Forward kinematics straight from URDF XML, with numpy only (no KDL needed).

scan_sweep uses it for the head-to-line-camera chain at each line's mirror angle, and the
description tests use it to compare the URDF with the vendored SO-101 and with the mirror
geometry. Mimic joints follow their source joint, as robot_state_publisher does.
"""

import math
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(roll, pitch, yaw):
    """URDF's fixed-axis roll, pitch, yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle(axis, angle):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def transform(rotation=np.eye(3), translation=(0, 0, 0)):
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def _floats(text, default):
    return [float(v) for v in (text or default).split()]


class Robot:
    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml) if isinstance(urdf_xml, str) else urdf_xml
        self.root = root
        self.links = {ln.get("name"): ln for ln in root.findall("link")}
        self.joints = {}
        self.parent_joint = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            axis = j.find("axis")
            mimic = j.find("mimic")
            info = {
                "name": j.get("name"),
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "origin": transform(
                    rpy_matrix(*_floats(o.get("rpy") if o is not None else None, "0 0 0")),
                    _floats(o.get("xyz") if o is not None else None, "0 0 0")),
                "axis": _floats(axis.get("xyz") if axis is not None else None, "1 0 0"),
                "mimic": None if mimic is None else (
                    mimic.get("joint"), float(mimic.get("multiplier", 1)), float(mimic.get("offset", 0))),
            }
            self.joints[info["name"]] = info
            self.parent_joint[info["child"]] = info

    def limits(self):
        """{joint: (lower, upper)} for the joints with limits."""
        out = {}
        for j in self.root.findall("joint"):
            lim = j.find("limit")
            if lim is not None and lim.get("lower") is not None and j.get("type") == "revolute":
                out[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
        return out

    def movable(self):
        return [n for n, j in self.joints.items() if j["type"] != "fixed" and j["mimic"] is None]

    def position(self, name, q):
        j = self.joints[name]
        if j["mimic"]:
            src, mult, off = j["mimic"]
            return mult * self.position(src, q) + off
        return q.get(name, 0.0)

    def fk(self, link, q, base=None):
        """4x4 pose of `link` in `base` (the root link by default) at joint positions `q`."""
        t = np.eye(4)
        while link in self.parent_joint and link != base:
            j = self.parent_joint[link]
            motion = np.eye(4)
            if j["type"] in ("revolute", "continuous"):
                motion = transform(axis_angle(j["axis"], self.position(j["name"], q)))
            elif j["type"] == "prismatic":
                motion = transform(translation=np.asarray(j["axis"]) * self.position(j["name"], q))
            t = j["origin"] @ motion @ t
            link = j["parent"]
        if base is not None and link != base:
            raise ValueError("%s is not above the link in the tree" % base)
        return t


def solve_ik(robot, link, joints, q0, position=None, z_axis=None, base=None, limits=None,
             iterations=300, tolerance=1e-7, damping=1e-4, max_step=0.2):
    """Joint positions that put `link`'s origin at `position` and point its z axis along
    `z_axis` (either may be None), by damped least squares from `q0`.

    joints: the joint names to move. limits: {joint: (lower, upper)}, e.g. Robot.limits().
    Returns (q, error_norm); check the error, since an unreachable target still returns.
    """
    q = dict(q0)
    names = list(joints)
    target_z = None if z_axis is None else np.asarray(z_axis, float) / np.linalg.norm(z_axis)

    def residual(qq):
        t = robot.fk(link, qq, base)
        parts = []
        if position is not None:
            parts.append(np.asarray(position, float) - t[:3, 3])
        if target_z is not None:
            parts.append(np.cross(t[:3, 2], target_z))
        return np.concatenate(parts)

    err = residual(q)
    for _ in range(iterations):
        if np.linalg.norm(err) < tolerance:
            break
        jac = np.zeros((len(err), len(names)))
        for k, n in enumerate(names):
            dq = dict(q)
            dq[n] += 1e-6
            jac[:, k] = (err - residual(dq)) / 1e-6
        step = np.linalg.solve(jac.T @ jac + damping * np.eye(len(names)), jac.T @ err)
        step *= min(1.0, max_step / np.abs(step).max())  # small steps: the arm is far from linear
        for k, n in enumerate(names):
            q[n] += step[k]
            if limits and n in limits:
                q[n] = min(max(q[n], limits[n][0]), limits[n][1])
        err = residual(q)
    return q, float(np.linalg.norm(err))
