"""Export the scanner head from the FreeCAD model for the URDF.

Reads cad/so101_hsi_rig.FCStd (and the mass budget in cad/build_report.json)
and writes, in this package:

    urdf/scan_head_params.xacro   every number urdf/scan_head.xacro needs: the
                                  mount on the wrist-roll horn, the mirror shaft,
                                  the objective, the pose camera, mass and inertia
    meshes/scan_head/*.stl        the head's parts grouped by colour, in the head
                                  frame; the mirror and its clamp in the mirror
                                  joint's frame, so they turn with it in RViz

Run it with FreeCAD's own Python from the repo root, after the CAD model
changes (rebuilding cad/ rewrites both files this reads):

    freecadcmd -c "exec(open('ros2/so101_scan_description/scripts/export_head_from_cad.py').read())"

On Windows, freecadcmd is FreeCADCmd.exe in FreeCAD's bin folder.
test/test_description.py checks the result against cad/build_report.json, so
it fails when the CAD model has moved on and this needs running again.
"""

import json
import math
import os
from pathlib import Path

import FreeCAD as App
import MeshPart

if "__file__" in globals():
    REPO = Path(__file__).resolve().parents[3]
else:
    REPO = Path(os.getcwd())
CAD = REPO / "cad"
PKG = REPO / "ros2" / "so101_scan_description"
MESHES = PKG / "meshes" / "scan_head"
PARAMS = PKG / "urdf" / "scan_head_params.xacro"

# Parts that turn with the scan mirror; everything else in the head is rigid.
MIRROR_PARTS = {"HSI_rotor": "mirror_clamp", "HSI_magnet": "mirror_clamp", "HSI_mirror": "mirror_glass"}
# Render materials of the rigid parts, grouped into a few coloured meshes.
GROUPS = {
    "printed": "head_printed",
    "anodized_black": "head_dark", "pcb_black": "head_dark", "chip": "head_dark",
    "steel": "head_metal", "aluminum": "head_metal", "blade": "head_metal", "magnet": "head_metal",
    "glass": "head_glass", "grating": "head_glass",
    "filter_glass": "head_filter",
    "pcb_green": "head_pcb", "connector": "head_pcb",
}
LINEAR_DEFLECTION = 0.15   # mm; coarse enough to keep the meshes small, fine enough to look right
ANGULAR_DEFLECTION = 0.4   # rad


def leaves(container):
    return [o for o in container.Group
            if o.isDerivedFrom("Part::Feature") and o.Visibility and not o.Shape.isNull()]


def rel_placement(obj, frame):
    return frame.getGlobalPlacement().inverse().multiply(obj.getGlobalPlacement())


def mesh_of(shapes):
    mesh = None
    for s in shapes:
        m = MeshPart.meshFromShape(Shape=s, LinearDeflection=LINEAR_DEFLECTION,
                                   AngularDeflection=ANGULAR_DEFLECTION, Relative=False)
        if mesh is None:
            mesh = m
        else:
            mesh.addMesh(m)
    return mesh


def rpy(rot):
    """URDF roll-pitch-yaw (extrinsic X, Y, Z) of a FreeCAD rotation."""
    yaw, pitch, roll = rot.toEuler()
    return [math.radians(roll), math.radians(pitch), math.radians(yaw)]


def fmt(v):
    return ("%.6f" % v).rstrip("0").rstrip(".") if abs(v) > 1e-12 else "0"


def main():
    doc = App.openDocument(str(CAD / "so101_hsi_rig.FCStd"))
    sheet = doc.getObject("Params")
    head = doc.getObject("HSI_head")
    get = sheet.get

    # --- meshes -------------------------------------------------------------
    MESHES.mkdir(parents=True, exist_ok=True)
    rigid, moving = {}, {}
    lo = [1e9] * 3
    hi = [-1e9] * 3
    for part in head.Group:
        if part.TypeId != "App::Part" or not part.Visibility:
            continue
        for leaf in leaves(part):
            if part.Name in MIRROR_PARTS:
                # in the part's own frame, which is the mirror joint's frame
                s = leaf.Shape.copy()
                moving.setdefault(MIRROR_PARTS[part.Name], []).append(s)
            else:
                s = leaf.Shape.copy()
                s.Placement = rel_placement(part, head).multiply(s.Placement)
                group = GROUPS.get(getattr(leaf, "RenderMaterial", "printed"), "head_dark")
                rigid.setdefault(group, []).append(s)
            bb = leaf.Shape.copy()
            bb.Placement = rel_placement(part, head).multiply(bb.Placement)
            b = bb.BoundBox
            lo = [min(lo[0], b.XMin), min(lo[1], b.YMin), min(lo[2], b.ZMin)]
            hi = [max(hi[0], b.XMax), max(hi[1], b.YMax), max(hi[2], b.ZMax)]
    written = []
    for name, shapes in sorted(list(rigid.items()) + list(moving.items())):
        path = MESHES / ("%s.stl" % name)
        mesh_of(shapes).write(str(path))
        written.append(path.name)

    # --- frames ---------------------------------------------------------------
    rotor = doc.getObject("HSI_rotor")
    rotor_pl = rel_placement(rotor, head)
    rest = rpy(rotor_pl.Rotation)[0] - math.radians(get("mirror_scan"))
    pose = doc.getObject("HSI_pose_camera")
    pose_pl = rel_placement(pose, head)

    report = json.loads((CAD / "build_report.json").read_text())
    mass_kg = report["mass"]["total_g"] / 1000.0
    com = [c / 1000.0 for c in report["mass"]["com_head_mm"]]
    size = [(h - l) / 1000.0 for l, h in zip(lo, hi)]
    ixx = mass_kg / 12.0 * (size[1] ** 2 + size[2] ** 2)
    iyy = mass_kg / 12.0 * (size[0] ** 2 + size[2] ** 2)
    izz = mass_kg / 12.0 * (size[0] ** 2 + size[1] ** 2)

    mm = 0.001
    props = [
        ("head_mount_z", -get("horn_z") * mm, "head origin: wrist-roll horn face, %g mm along the roll axis "
         "from wrist_roll_link (horn_z)" % get("horn_z")),
        ("mirror_shaft_y", rotor_pl.Base.y * mm, "scan mirror shaft axis (along X), Y (y_shaft)"),
        ("mirror_shaft_z", rotor_pl.Base.z * mm, "scan mirror shaft axis, Z (z_shaft)"),
        ("mirror_rest", rest, "mirror angle at rest: its face turned %g deg about X, 45 deg to the optical axis"
         % math.degrees(rest)),
        ("mirror_face_offset", get("mirror_e") * mm, "mirror front surface to the shaft axis (mirror_e)"),
        ("mirror_home", math.radians(get("mirror_home")), "mirror angle where the hall sensor sees the magnet "
         "(mirror_home, %g deg)" % get("mirror_home")),
        ("mirror_scan_half", math.radians(get("scan_half") / 2.0), "mirror turn either side of rest for a full scan "
         "(the view turns twice as far: scan_half = %g deg)" % get("scan_half")),
        ("objective_y", get("y_axis") * mm, "optical axis Y (y_axis)"),
        ("objective_z", get("z_obj") * mm, "objective principal plane Z (z_obj)"),
        ("scene_distance", get("scene_dist") * mm, "objective to the in-focus scan line, along the folded axis "
         "(scene_dist)"),
        ("scan_line_half_length", get("half_line") * mm, "half length of the scan line on the scene (half_line)"),
        ("objective_focal_length", get("f_obj") * mm, "objective focal length (f_obj)"),
        ("slit_length", get("slit_len") * mm, "slit length (slit_len)"),
        ("pose_camera_x", pose_pl.Base.x * mm, "Pi NoIR v2 pose camera board, lens centre on the PCB back"),
        ("pose_camera_y", pose_pl.Base.y * mm, ""),
        ("pose_camera_z", pose_pl.Base.z * mm, ""),
        ("pose_camera_roll", rpy(pose_pl.Rotation)[0], "board frame: +Z out of the lens, +Y toward the wrist"),
        ("pose_camera_lens", 4.5 * mm, "PCB back to the front of the lens module (RPI-CAM-V2 drawing)"),
        ("head_mass", mass_kg, "mass budget in cad/build_report.json, wrist puck included"),
        ("head_com_x", com[0], "centre of mass in the head frame (build_report.json)"),
        ("head_com_y", com[1], ""),
        ("head_com_z", com[2], ""),
        ("head_ixx", ixx, "inertia of a solid box the head's size (%.0f x %.0f x %.0f mm) about the centre of mass"
         % tuple(s * 1000 for s in size)),
        ("head_iyy", iyy, ""),
        ("head_izz", izz, ""),
    ]
    lines = ['<?xml version="1.0"?>',
             "<!-- Generated by scripts/export_head_from_cad.py from cad/so101_hsi_rig.FCStd",
             "     (Params sheet and part placements) and cad/build_report.json. Do not edit by hand:",
             "     change the CAD model, rebuild it, and run the script again.",
             "     Head frame: origin on the wrist-roll horn face, +Z away from the wrist,",
             "     +Y toward the scene (scan window), X along the slit and the mirror shaft.",
             "     Lengths in metres, angles in radians. -->",
             '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">']
    for name, value, note in props:
        if note:
            lines.append("  <!-- %s -->" % note)
        lines.append('  <xacro:property name="%s" value="%s"/>' % (name, fmt(value)))
    lines += ["</robot>", ""]
    PARAMS.write_text("\n".join(lines))
    print("wrote", PARAMS)
    print("wrote", ", ".join(written), "in", MESHES)
    App.closeDocument(doc.Name)


main()
