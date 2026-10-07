"""Static gravity torque on the SO-101 joints, with and without the HSI head.

Uses the masses and centres of mass in TheRobotStudio's URDF. The stock
gripper (gripper_link and the moving jaw) is swapped for the head as a point
mass at its computed centre of mass. Forward kinematics follow the URDF joint
origins, so the poses match the FreeCAD model for the same joint angles.
"""

import xml.etree.ElementTree as ET

import FreeCAD as App

from .so101 import _origin

G = 9.81
STALL_NM = 16.5 * G / 100.0   # STS3215 7.4 V (C001): 16.5 kg.cm stall at 6 V
GRIPPER_LINKS = ("gripper_link", "moving_jaw_so101_v1_link")


def parse_inertials(path):
    root = ET.parse(path).getroot()
    out = {}
    for ln in root.findall("link"):
        i = ln.find("inertial")
        if i is None:
            continue
        m = float(i.find("mass").get("value"))
        out[ln.get("name")] = (m, _origin(i).Base)   # mm
    joints = []
    for j in root.findall("joint"):
        axis = j.find("axis")
        joints.append({
            "name": j.get("name"),
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "placement": _origin(j),
            "axis": (App.Vector(*[float(a) for a in axis.get("xyz").split()]) if axis is not None
                     else App.Vector(0, 0, 1)),
        })
    return out, joints


def forward(joints, q):
    """Global placement of every link frame, and of every joint (axis frame)."""
    links = {"base_link": App.Placement()}
    jframes = {}
    pending = list(joints)
    while pending:
        rest = []
        for j in pending:
            if j["parent"] not in links:
                rest.append(j)
                continue
            jp = links[j["parent"]].multiply(j["placement"])
            angle = q.get(j["name"], 0.0) if j["type"] == "revolute" else 0.0
            links[j["child"]] = jp.multiply(App.Placement(App.Vector(), App.Rotation(j["axis"], angle)))
            jframes[j["name"]] = (jp, j)
        if len(rest) == len(pending):
            break
        pending = rest
    return links, jframes


def subtree(joints, link):
    out = {link}
    grew = True
    while grew:
        grew = False
        for j in joints:
            if j["parent"] in out and j["child"] not in out:
                out.add(j["child"])
                grew = True
    return out


def torques(urdf, q, head=None):
    """Gravity torque (N.m) on each arm joint for pose q (degrees).

    head: (mass kg, CoM in the gripper_link frame, mm) replaces the stock
    gripper; None keeps the stock gripper."""
    inert, joints = parse_inertials(urdf)
    links, jframes = forward(joints, q)
    masses = []   # (kg, world point mm)
    for name, (m, c) in inert.items():
        if head is not None and name in GRIPPER_LINKS:
            continue
        if name in links:
            masses.append((name, m, links[name].multVec(c)))
    if head is not None:
        masses.append(("hsi_head", head[0], links["gripper_link"].multVec(head[1])))
    g = App.Vector(0, 0, -G)
    out = {}
    for jname in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
        jp, j = jframes[jname]
        down = subtree(joints, j["child"])
        if "gripper_link" in down:
            down.add("hsi_head")
        axis = jp.Rotation.multVec(j["axis"])
        tau = 0.0
        for name, m, p in masses:
            if name in down:
                r = (p - jp.Base) * 0.001
                tau += (r.cross(g * m)).dot(axis)
        out[jname] = tau
    return out


def worst_case(urdf, head=None, step=15):
    """Largest |torque| per joint over a grid of lift/elbow/wrist angles."""
    worst = {}
    rng = range(-90, 91, step)
    for a in rng:
        for b in rng:
            for c in rng:
                t = torques(urdf, {"shoulder_lift": a, "elbow_flex": b, "wrist_flex": c}, head)
                for k, v in t.items():
                    if abs(v) > abs(worst.get(k, (0, None))[0]):
                        worst[k] = (v, (a, b, c))
    return worst
