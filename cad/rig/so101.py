"""SO-101 follower arm, built from TheRobotStudio's URDF and meshes.

Each URDF joint becomes two nested App::Part containers: one holding the
joint's fixed origin, and one inside it that rotates about its Z axis by the
joint angle in the Params sheet (q_shoulder_pan ... q_wrist_roll, degrees).
The next link's meshes and joints live inside that rotating container, so
editing a joint angle and pressing Recompute poses the whole arm, head
included.

Meshes are imported once each, decimated, and reused through App::Link, which
keeps the .FCStd small (the STS3215 servo mesh appears six times).
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import FreeCAD as App
import Mesh

from .expr import SHEET

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
# URDF colours: printed parts and servos
COLORS = {"3d_printed": (1.0, 0.82, 0.12), "sts3215": (0.1, 0.1, 0.1)}
# Gripper parts the scanner head replaces; kept, hidden, under "Stock gripper"
GRIPPER_LINKS = {"moving_jaw_so101_v1_link"}
DECIMATE = 0.6  # fraction of triangles removed on import


def _rpy(r, p, y):
    # URDF rpy is extrinsic X-Y-Z, which is FreeCAD's (yaw, pitch, roll) order
    return App.Rotation(math.degrees(y), math.degrees(p), math.degrees(r))


def _origin(el):
    o = el.find("origin")
    xyz = [float(v) * 1000.0 for v in (o.get("xyz", "0 0 0").split())] if o is not None else [0, 0, 0]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0").split())] if o is not None else [0, 0, 0]
    return App.Placement(App.Vector(*xyz), _rpy(*rpy))


def parse_urdf(path):
    root = ET.parse(path).getroot()
    links = {}
    for ln in root.findall("link"):
        vis = []
        for v in ln.findall("visual"):
            mesh = v.find("geometry/mesh")
            if mesh is None:
                continue
            mat = v.find("material")
            vis.append({
                "file": Path(mesh.get("filename")).name,
                "placement": _origin(v),
                "material": mat.get("name") if mat is not None else "3d_printed",
            })
        links[ln.get("name")] = vis
    joints = []
    for j in root.findall("joint"):
        axis = j.find("axis")
        joints.append({
            "name": j.get("name"),
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "placement": _origin(j),
            "axis": [float(a) for a in axis.get("xyz").split()] if axis is not None else [0, 0, 1],
        })
    return links, joints


class MeshLibrary:
    """One decimated Mesh::Feature per STL, hidden, shared through App::Link."""

    def __init__(self, doc, asset_dir, gui):
        self.doc, self.dir, self.gui = doc, Path(asset_dir), gui
        self.group = doc.addObject("App::DocumentObjectGroup", "SO101_mesh_library")
        self.group.Label = "SO101 mesh library (hidden, used by links)"
        self.masters = {}

    def get(self, fname, material):
        key = (fname, material)
        if key not in self.masters:
            m = Mesh.Mesh(str(self.dir / fname))
            m.transform(App.Matrix(1000, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1))
            if DECIMATE:
                m.decimate(0.0, DECIMATE)
            name = "SO101_mesh_" + Path(fname).stem
            f = self.doc.addObject("Mesh::Feature", name)
            f.Mesh = m
            f.Label = Path(fname).stem
            self.group.addObject(f)
            if self.gui:
                f.ViewObject.ShapeColor = COLORS.get(material, (0.8, 0.8, 0.8))
                f.ViewObject.Visibility = False
            f.Visibility = False
            f.addProperty("App::PropertyString", "SourceFile", "SO101")
            f.SourceFile = fname
            self.masters[key] = f
        return self.masters[key]


def build_arm(doc, P, urdf_path, asset_dir, gui=False):
    """Create the posed arm. Returns (arm container, {link name: container})."""
    links, joints = parse_urdf(urdf_path)
    lib = MeshLibrary(doc, asset_dir, gui)
    arm = doc.addObject("App::Part", "SO101")
    arm.Label = "SO-101 follower arm (TheRobotStudio, Apache-2.0)"
    frames = {}

    def add_visuals(link, container):
        for i, v in enumerate(links.get(link, [])):
            master = lib.get(v["file"], v["material"])
            lk = doc.addObject("App::Link", "SO101_%s_v%d" % (link, i))
            lk.Label = "%s / %s" % (link, Path(v["file"]).stem)
            lk.LinkedObject = master
            lk.Placement = v["placement"]
            container.addObject(lk)

    base = doc.addObject("App::Part", "SO101_base_link")
    base.Label = "base_link"
    arm.addObject(base)
    frames["base_link"] = base
    add_visuals("base_link", base)

    by_parent = {}
    for j in joints:
        by_parent.setdefault(j["parent"], []).append(j)

    def grow(link):
        for j in by_parent.get(link, []):
            parent_c = frames[link]
            jo = doc.addObject("App::Part", "SO101_J_%s" % j["name"])
            jo.Label = "joint %s (origin)" % j["name"]
            jo.Placement = j["placement"]
            parent_c.addObject(jo)
            child = doc.addObject("App::Part", "SO101_%s" % j["child"])
            child.Label = j["child"]
            jo.addObject(child)
            if j["type"] == "revolute":
                ax = App.Vector(*j["axis"])
                child.Placement = App.Placement(App.Vector(), App.Rotation(ax, 0))
                alias = "q_" + j["name"]
                q = P.sheet.get(alias) if j["name"] in ARM_JOINTS or j["name"] == "gripper" else 0
                child.Placement = App.Placement(App.Vector(), App.Rotation(ax, float(q)))
                child.setExpression(".Placement.Rotation.Angle", "%s.%s" % (SHEET, alias))
            frames[j["child"]] = child
            add_visuals(j["child"], child)
            grow(j["child"])

    grow("base_link")

    # The scanner head replaces the gripper: hide the stock gripper parts but
    # keep them in the tree so they can be shown for comparison.
    stock = doc.addObject("App::Part", "SO101_stock_gripper")
    stock.Label = "Stock gripper (replaced by the HSI head, hidden)"
    g = frames["gripper_link"]
    g.addObject(stock)
    for o in list(g.Group):
        if o.TypeId == "App::Link":
            g.removeObject(o)
            stock.addObject(o)
    jaw_origin = doc.getObject("SO101_J_gripper")
    g.removeObject(jaw_origin)
    stock.addObject(jaw_origin)
    stock.Visibility = False
    if gui:
        stock.ViewObject.Visibility = False
    return arm, frames
