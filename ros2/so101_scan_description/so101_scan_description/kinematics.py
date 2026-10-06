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
