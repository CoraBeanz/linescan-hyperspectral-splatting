"""Build the SO-101 + hyperspectral line-scan head assembly in FreeCAD.

Run from the repository root with FreeCAD 1.0 or newer:

    python3 cad/vendor/so101/fetch_assets.py          # SO-101 meshes, once
    freecadcmd cad/build_rig.py                        # headless
    freecad cad/build_rig.py                           # or with the GUI (keeps colours)

Writes:
    cad/so101_hsi_rig.FCStd     parametric assembly (edit the Params sheet, Recompute)
    cad/step/hsi_head.step      the head, every printed and bought part in place
    cad/stl/*.stl               printed parts, laid out for printing
    cad/build_report.json       mass budget, servo torques, clearances

Environment variables:
    RIG_MOTOR=17HM08            NEMA 17 pancake scan stepper instead of the NEMA 8 (see rig/params.py)
    RIG_NO_EXPORT=1             only build and save the .FCStd
    RIG_RENDER_DIR=<dir>        also write per-part OBJ meshes + manifest for cad/render
"""

import json
import os
import sys
from pathlib import Path

import FreeCAD as App
import MeshPart

CAD = Path(__file__).resolve().parent if "__file__" in globals() else Path.cwd() / "cad"
sys.path.insert(0, str(CAD))

from rig import params, printed, so101, statics, vendor  # noqa: E402
from rig.expr import Frame, Params  # noqa: E402

URDF = CAD / "vendor" / "so101" / "so101_new_calib.urdf"
ASSETS = CAD / "vendor" / "so101" / "assets"
GUI = App.GuiUp

PETG = 1.27e-3       # g / mm^3
FILL = 0.80          # 2.5 mm walls are mostly perimeters; 25% infill elsewhere
HARDWARE_G = 12.0    # screws, nuts, heat-set inserts, glue (estimate)


def build(doc):
    sheet = doc.addObject("Spreadsheet::Sheet", "Params")
    sheet.Label = "Params"
    params.fill_sheet(sheet)
    doc.recompute()
    P = Params(sheet)

    if not ASSETS.exists() or not any(ASSETS.glob("*.stl")):
        sys.path.insert(0, str(CAD / "vendor" / "so101"))
        import fetch_assets
        if not fetch_assets.fetch():
            raise RuntimeError("SO-101 meshes missing or changed upstream")
    arm, frames = so101.build_arm(doc, P, URDF, ASSETS, gui=GUI)

    # The head hangs off the wrist-roll output: gripper_link's -Z points away
    # from the wrist, so the head frame is that frame turned 180 deg about X.
    head = doc.addObject("App::Part", "HSI_head")
    head.Label = "Hyperspectral line-scan head"
    frames["gripper_link"].addObject(head)
    head.Placement = App.Placement(App.Vector(0, 0, -P.horn_z.v), App.Rotation(App.Vector(1, 0, 0), 180))
    head.setExpression(".Placement.Base.z", "-" + P.horn_z.s)

    parts = [fn(doc, head, P) for fn in printed.ALL if fn is not printed.bench_puck]
    acc = doc.addObject("App::Part", "Accessories")
    acc.Label = "Accessories (hidden): bench puck"
    parts.append(printed.bench_puck(doc, acc, P))
    acc.Visibility = False

    cf = Frame((0, P.y_cam, P.z_cam), P.theta)
    rf = printed.rotor_frame(P)
    vendor.stepper(doc, head, P)
    vendor.scan_mirror(doc, head, P, rf)
    vendor.magnet(doc, head, P, rf)
    vendor.hall_sensor(doc, head, P)
    zf = P.obj_back - P.obj_len
    vendor.filter_disc(doc, head, P, zf - P.filt_t)
    vendor.m12_lens(doc, head, "HSI_objective", "Objective: Commonlands CIL161 16 mm (at f/4)",
                    Frame((0, P.y_axis, P.obj_back)), P.obj_od, P.obj_len, P.obj_thread)
    vendor.m12_holder(doc, head, "HSI_obj_holder", "M12 holder U0756M10 (objective)",
                      Frame((0, P.y_axis, P.z_obj_plate)), P)
    rec = P.blade_t + 0.05
    vendor.slit_blades(doc, head, P, P.z_slit + rec)
    vendor.field_lens(doc, head, P, P.z_slit + rec)
    vendor.m12_holder(doc, head, "HSI_coll_holder", "M12 holder U0756M10 (collimator)",
                      Frame((0, P.y_axis, P.z_slit + P.slit_t)), P, flip=True)
    vendor.m12_lens(doc, head, "HSI_collimator", "Collimator: Arducam LN016 25 mm (reversed)",
                    Frame((0, P.y_axis, P.z_slit + P.coll_bfl)), P.coll_od, P.coll_len, P.coll_thread,
                    flip=True)
    vendor.grating_film(doc, head, P, P.z_film)
    vendor.m12_lens(doc, head, "HSI_cam_lens", "Camera lens: Commonlands CIL122 12 mm",
                    cf.sub(0, 0, P.f_cam - P.cam_bfl), P.cam_od, P.cam_len, P.cam_thread)
    vendor.camera_board(doc, head, P, cf)
    vendor.pose_camera(doc, head, P, P.y_axis + P.y_in_pos + P.wall + 3.5)
    doc.recompute()

    bad = [o.Name for o in doc.Objects if "Invalid" in o.State or "Error" in o.State]
    if bad:
        raise RuntimeError("objects failed to recompute: %s" % ", ".join(bad))
    return P, head, parts, frames


# ---------------------------------------------------------------- geometry helpers

def leaf_features(container):
    """Visible solid features directly inside a container."""
    return [o for o in container.Group
            if o.isDerivedFrom("Part::Feature") and o.Visibility and not o.Shape.isNull()]


def shape_in(container, frame_obj):
    """Compound of a container's visible solids, in the coordinates of frame_obj."""
    import Part
    rel = frame_obj.getGlobalPlacement().inverse().multiply(container.getGlobalPlacement())
    shapes = []
    for o in leaf_features(container):
        s = o.Shape.copy()
        s.Placement = rel.multiply(s.Placement)
        shapes.append(s)
    return Part.makeCompound(shapes)


def head_children(head):
    return [o for o in head.Group if o.TypeId == "App::Part"]


# ---------------------------------------------------------------- checks

def interference(head, tol=0.5):
    kids = head_children(head)
    shapes = {k.Name: shape_in(k, head) for k in kids}
    names = sorted(shapes)
    hits = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            sa, sb = shapes[a], shapes[b]
            if not sa.BoundBox.intersect(sb.BoundBox):
                continue
            v = sa.common(sb).Volume
            if v > tol:
                hits.append((a, b, round(v, 2)))
    return hits


def rotor_sweep(head, P, step=10):
    """Turn the rotor, mirror and magnet a full revolution; report the gaps."""
    import Part
    doc = head.Document
    moving = Part.makeCompound([shape_in(doc.getObject(n), head)
                                for n in ("HSI_rotor", "HSI_mirror", "HSI_magnet")])
    fixed = {n: shape_in(doc.getObject(n), head)
             for n in ("HSI_shell", "HSI_lid", "HSI_filter_cap", "HSI_hall", "HSI_objective")}
    centre = App.Vector(0, P.y_shaft.v, P.z_shaft.v)
    worst = {}
    for k in range(0, 360, step):
        m = moving.copy()
        m.rotate(centre, App.Vector(1, 0, 0), k)
        for n, s in fixed.items():
            d = m.distToShape(s)[0]
            if n not in worst or d < worst[n][0]:
                worst[n] = (round(d, 2), k)
    return worst


def _bands(points, step=2.5):
    import math
    out = {}
    for v in points:
        r = round(int(math.hypot(v.x, v.y) // step) * step, 1)
        lo, hi = out.get(r, (1e9, -1e9))
        out[r] = (min(lo, v.z), max(hi, v.z))
    return out


def _link_points(frames, link, to_obj):
    c = frames[link]
    inv = to_obj.getGlobalPlacement().inverse()
    pts = []
    for lk in c.Group:
        if lk.TypeId != "App::Link" or not lk.Visibility:
            continue
        pl = inv.multiply(c.getGlobalPlacement().multiply(lk.Placement))
        pts += [pl.multVec(App.Vector(p.x, p.y, p.z)) for p in lk.LinkedObject.Mesh.Points]
    return pts


def roll_clearance(frames, head):
    """The head turns with the wrist roll (about +/-160 deg), the wrist bracket
    does not. Per radius band, the gap between the bracket's highest point and
    the head's lowest point; negative means they hit at some roll angle."""
    wrist = _bands(_link_points(frames, "wrist_link", head))
    hpts = []
    for k in head_children(head):
        s = shape_in(k, head)
        if not s.isNull():
            hpts += s.tessellate(0.3)[0]
    mine = _bands(hpts)
    gaps = {r: round(mine[r][0] - wrist[r][1], 2) for r in sorted(mine) if r in wrist}
    worst = min(gaps.items(), key=lambda kv: kv[1])
    return {"min_gap_mm": worst[1], "at_radius_mm": worst[0], "by_radius": gaps}


def flex_clearance(frames, head, doc):
    """Wrist-flex angles at which the head stays clear of the forearm, for
    every wrist-roll angle (checked against the head's main bounding boxes)."""
    import math
    jf = doc.getObject("SO101_J_wrist_flex")
    wl = frames["wrist_link"]
    # head pose relative to the wrist_link frame, with the roll at zero
    gr = frames["gripper_link"]
    w_to_h0 = wl.getGlobalPlacement().inverse().multiply(head.getGlobalPlacement())
    rot = gr.Placement.Rotation
    roll_now = math.degrees(rot.Angle) * (1 if rot.Axis.z >= 0 else -1)
    boxes = []
    for n in ("HSI_shell", "HSI_lid", "HSI_stepper", "HSI_camera", "HSI_puck"):
        bb = shape_in(doc.getObject(n), head).BoundBox
        boxes.append(bb)
    arm_pts = []
    for link in ("lower_arm_link", "upper_arm_link"):
        if link in frames:
            arm_pts += _link_points(frames, link, jf)      # in the joint frame
    free = {}
    for roll in range(-150, 151, 30):
        ok = []
        for flex in range(-95, 96, 5):
            # joint frame -> wrist_link frame at this flex
            wl_pl = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 0, 1), flex))
            # rolling the joint turns the head about its own Z the opposite way
            r_pl = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 0, 1), roll_now - roll))
            inv = r_pl.inverse().multiply(w_to_h0.inverse()).multiply(wl_pl.inverse())
            hit = False
            for v in arm_pts:
                h = inv.multVec(v)
                if any(bb.isInside(h) for bb in boxes):
                    hit = True
                    break
            if not hit:
                ok.append(flex)
        free[roll] = [min(ok), max(ok)] if ok else None
    spans = [v for v in free.values() if v]
    every = [max(v[0] for v in spans), min(v[1] for v in spans)] if len(spans) == len(free) else None
    return {"all_rolls": every, "by_roll": free}


def com_of(s):
    vol, mom = 0.0, App.Vector()
    for so in s.Solids:
        vol += so.Volume
        mom += so.CenterOfMass * so.Volume
    return mom * (1.0 / vol) if vol else s.BoundBox.Center


def mass_budget(head, parts):
    rows = []
    total_m = 0.0
    mom = App.Vector()
    printed_names = {p["container"].Name: p for p in parts if p["container"].getParentGeoFeatureGroup() == head}
    for k in head_children(head):
        s = shape_in(k, head)
        if s.isNull() or s.Volume <= 0:
            continue
        com = com_of(s)
        if k.Name in printed_names:
            m = s.Volume * PETG * FILL
            src = "PETG, %.1f cm3 at %d%%" % (s.Volume / 1000.0, FILL * 100)
        elif k.Name in vendor.MASS_G:
            m, tag = vendor.MASS_G[k.Name]
            src = tag
        else:
            continue
        rows.append({"part": k.Label, "g": round(m, 1), "source": src})
        total_m += m
        mom += com * m
    rows.append({"part": "Screws, nuts, inserts, glue", "g": HARDWARE_G, "source": "est"})
    total_m += HARDWARE_G
    com = mom * (1.0 / (total_m - HARDWARE_G))
    return rows, total_m, com


# ---------------------------------------------------------------- exports

def export_stl(parts, outdir):
    outdir.mkdir(parents=True, exist_ok=True)
    out = []
    for p in parts:
        s = p["final"].Shape.copy()
        s.Placement = App.Placement(App.Vector(), p["print_rot"]).multiply(s.Placement)
        bb = s.BoundBox
        s.translate(App.Vector(-bb.Center.x, -bb.Center.y, -bb.ZMin))
        m = MeshPart.meshFromShape(Shape=s, LinearDeflection=0.02, AngularDeflection=0.2, Relative=False)
        path = outdir / ("%s.stl" % p["name"])
        m.write(str(path))
        out.append({"file": "stl/%s.stl" % p["name"], "part": p["label"], "qty": p["qty"],
                    "print": p["note"], "volume_cm3": round(s.Volume / 1000.0, 1),
                    "size_mm": [round(bb.XLength, 1), round(bb.YLength, 1), round(bb.ZLength, 1)]})
    return out


def export_step(head, path):
    """One named solid per part (printed and bought), in head coordinates."""
    import Import
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = App.newDocument("hsi_head_export")
    objs = []
    for k in head_children(head):
        s = shape_in(k, head)
        if s.isNull():
            continue
        f = tmp.addObject("Part::Feature", k.Name)
        f.Label = k.Label
        f.Shape = s
        if GUI:
            vols = {}
            for o in leaf_features(k):
                m = getattr(o, "RenderMaterial", "printed")
                vols[m] = vols.get(m, 0.0) + o.Shape.Volume
            f.ViewObject.ShapeColor = vendor.MATERIAL_RGB.get(max(vols, key=vols.get), (0.6, 0.6, 0.6))
        objs.append(f)
    tmp.recompute()
    Import.export(objs, str(path))
    App.closeDocument(tmp.Name)


def export_render_meshes(doc, head, P, outdir):
    """One OBJ per visible part, in world millimetres, plus a manifest."""
    outdir.mkdir(parents=True, exist_ok=True)
    manifest = []
    roots = [doc.getObject("SO101")]       # the head sits inside gripper_link
    seen = set()

    def walk(c):
        for o in c.Group:
            if o.Name in seen:
                continue
            seen.add(o.Name)
            if o.TypeId == "App::Part":
                if o.Visibility:
                    walk(o)
            elif o.TypeId == "App::Link" and o.Visibility:
                src = o.LinkedObject
                m = src.Mesh.copy()
                m.Placement = c.getGlobalPlacement().multiply(o.Placement)
                f = outdir / ("%s.obj" % o.Name)
                m.write(str(f))
                mat = "arm_printed" if "sts3215" not in src.SourceFile else "servo"
                manifest.append({"file": f.name, "material": mat, "group": "arm"})
            elif o.isDerivedFrom("Part::Feature") and o.Visibility and not o.Shape.isNull():
                s = o.Shape.copy()
                s.Placement = c.getGlobalPlacement().multiply(s.Placement)
                m = MeshPart.meshFromShape(Shape=s, LinearDeflection=0.03, AngularDeflection=0.15,
                                           Relative=False)
                f = outdir / ("%s.obj" % o.Name)
                m.write(str(f))
                mat = getattr(o, "RenderMaterial", "printed")
                manifest.append({"file": f.name, "material": mat, "group": c.Name})

    for r in roots:
        walk(r)
    hp = head.getGlobalPlacement()
    scan = {"mirror": list(hp.multVec(App.Vector(0, P.y_axis.v, P.z_mirror.v))),
            "dir": list(hp.Rotation.multVec(App.Vector(0, 1, 0))),
            "xdir": list(hp.Rotation.multVec(App.Vector(1, 0, 0))),
            "half_line": P.half_line.v}
    # label anchors for the annotated cut-away (head frame -> world)
    def w(x, y, z):
        return list(hp.multVec(App.Vector(float(x), float(y), float(z))))
    v = P.sheet.get
    labels = {
        "Scan mirror": w(-6, v("y_axis"), v("z_mirror")),
        "Mirror clamp": w(-10, v("y_shaft") - 6, v("z_shaft") - 4),
        "Objective + 4 mm stop": w(-4, v("y_axis") - 6, v("obj_back") - v("obj_len") / 2),
        "Slit + field lens": w(-12, v("y_axis") - 9, v("z_slit") + 2),
        "Collimator": w(-4, v("y_axis") - 7, v("z_slit") + v("coll_bfl") + v("coll_len") / 2),
        "Grating": w(-12, v("y_axis") - 9, v("z_film") - 1.5),
        "Camera lens": w(-4, v("y_cam") - 6, v("z_cam") - 4),
        "IMX219 board": w(-10, v("y_sensor") - 1, v("z_sensor") + 2.5),
        "Wrist puck": w(-14, -3, 2.5),
    }
    info = {"parts": manifest, "head_placement": [list(hp.Base), list(hp.Rotation.Q)], "scan": scan,
            "labels": labels}
    (outdir / "manifest.json").write_text(json.dumps(info, indent=1))
    return len(manifest)


def apply_colors(doc):
    for o in doc.Objects:
        if hasattr(o, "RenderMaterial") and o.ViewObject is not None:
            rgb = vendor.MATERIAL_RGB.get(o.RenderMaterial, (0.6, 0.6, 0.6))
            o.ViewObject.ShapeColor = rgb
            if o.RenderMaterial == "glass":
                o.ViewObject.Transparency = 60


def main():
    doc = App.newDocument("so101_hsi_rig")
    P, head, parts, frames = build(doc)
    out_fc = CAD / "so101_hsi_rig.FCStd"
    if GUI:
        apply_colors(doc)
    report = {}
    if not os.environ.get("RIG_NO_EXPORT"):
        report["interference_mm3"] = interference(head)
        report["rotor_full_turn_min_gap_mm"] = rotor_sweep(head, P)
        rows, m, com = mass_budget(head, parts)
        report["mass"] = {"rows": rows, "total_g": round(m, 1), "com_head_mm": [round(c, 1) for c in com]}
        report["roll_clearance"] = roll_clearance(frames, head)
        report["flex_free_deg_by_roll"] = flex_clearance(frames, head, doc)
        gl = head.Placement.multVec(com)    # head CoM in the gripper_link frame
        pose = {j: float(P.sheet.get("q_" + j)) for j in so101.ARM_JOINTS}
        report["torque_Nm"] = {
            "stall_Nm": round(statics.STALL_NM, 2),
            "pose": pose,
            "stock_gripper": {k: round(v, 3) for k, v in statics.torques(URDF, pose).items()},
            "with_head": {k: round(v, 3) for k, v in statics.torques(URDF, pose, (m / 1000.0, gl)).items()},
            "worst_stock": {k: [round(v[0], 3), v[1]] for k, v in statics.worst_case(URDF).items()},
            "worst_head": {k: [round(v[0], 3), v[1]] for k, v in statics.worst_case(URDF, (m / 1000.0, gl)).items()},
        }
        report["stations_mm"] = {a: round(float(P.sheet.get(a)), 2) for a in (
            "z_mirror", "z_obj", "z_slit", "z_coll", "z_grat", "z_film", "g2c", "z_cam", "y_cam",
            "z_sensor", "y_sensor", "theta", "hall_y", "hall_z", "y_shaft", "z_shaft")}
        report["stl"] = export_stl(parts, CAD / "stl")
        export_step(head, CAD / "step" / "hsi_head.step")
        (CAD / "build_report.json").write_text(json.dumps(report, indent=1))
    rd = os.environ.get("RIG_RENDER_DIR")
    if rd:
        n = export_render_meshes(doc, head, P, Path(rd))
        print("render meshes:", n)
    doc.saveAs(str(out_fc))
    print("saved", out_fc)
    return doc, report


_doc, _report = main()
print(json.dumps({k: v for k, v in _report.items() if k not in ("stl", "mass")}, indent=1)[:6000])
if GUI and os.environ.get("RIG_EXIT"):
    os._exit(0)
