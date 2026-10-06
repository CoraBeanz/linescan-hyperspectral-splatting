"""Render the rig with Blender Cycles from the meshes build_rig.py exports.

    RIG_RENDER_DIR=/tmp/rig_meshes freecadcmd cad/build_rig.py
    blender -b -P cad/render/render_blender.py -- /tmp/rig_meshes cad/renders [shot ...]

Shots: rig (arm scanning a table), head (outside), head_open (lid and motor
hidden, showing the optical train). Tested with Blender 4.0.
"""

import json
import math
import os
import sys
from pathlib import Path

import bpy
from mathutils import Quaternion, Vector

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
MESH_DIR = Path(argv[0])
OUT_DIR = Path(argv[1])
SHOTS = argv[2:] or ["rig", "head", "head_open"]
SAMPLES = int(os.environ.get("RIG_SAMPLES", 128))
SCALE = float(os.environ.get("RIG_SCALE", 1.0))      # 0.5 for quick previews

# name: (base colour, metallic, roughness, transmission, coat)
MATS = {
    "printed": ((0.018, 0.018, 0.02), 0.0, 0.5, 0.0, 0.0),
    "printed_accent": ((0.85, 0.25, 0.03), 0.0, 0.45, 0.0, 0.0),
    "arm_printed": ((0.92, 0.6, 0.08), 0.0, 0.62, 0.0, 0.0),
    "servo": ((0.02, 0.02, 0.022), 0.0, 0.35, 0.0, 0.3),
    "aluminum": ((0.75, 0.76, 0.78), 1.0, 0.3, 0.0, 0.0),
    "anodized_black": ((0.02, 0.02, 0.022), 0.6, 0.35, 0.0, 0.0),
    "steel": ((0.7, 0.7, 0.72), 1.0, 0.25, 0.0, 0.0),
    "glass": ((0.85, 0.95, 1.0), 0.0, 0.02, 1.0, 0.0),
    "filter_glass": ((0.9, 0.25, 0.05), 0.0, 0.05, 0.9, 0.0),
    "mirror": ((0.95, 0.95, 0.97), 1.0, 0.02, 0.0, 0.0),
    "grating": ((0.45, 0.55, 0.95), 0.8, 0.12, 0.0, 0.0),
    "pcb_green": ((0.02, 0.18, 0.06), 0.0, 0.4, 0.0, 0.5),
    "pcb_black": ((0.012, 0.012, 0.014), 0.0, 0.4, 0.0, 0.5),
    "chip": ((0.03, 0.03, 0.035), 0.2, 0.3, 0.0, 0.0),
    "connector": ((0.85, 0.82, 0.72), 0.0, 0.5, 0.0, 0.0),
    "magnet": ((0.7, 0.7, 0.72), 1.0, 0.3, 0.0, 0.0),
    "blade": ((0.85, 0.85, 0.87), 1.0, 0.18, 0.0, 0.0),
}


_CACHE = {}


def material(name, cache=_CACHE):
    if name in cache:
        return cache[name]
    col, metal, rough, trans, coat = MATS.get(name, MATS["printed"])
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*col, 1.0)
    b.inputs["Metallic"].default_value = metal
    b.inputs["Roughness"].default_value = rough
    b.inputs["Transmission Weight"].default_value = trans
    b.inputs["Coat Weight"].default_value = coat
    if trans > 0:
        b.inputs["IOR"].default_value = 1.52
    cache[name] = m
    return m


def emissive(name, rgb, strength, alpha=1.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    em = nt.nodes.new("ShaderNodeEmission")
    em.inputs["Color"].default_value = (*rgb, 1)
    em.inputs["Strength"].default_value = strength
    if alpha < 1.0:
        tr = nt.nodes.new("ShaderNodeBsdfTransparent")
        mix = nt.nodes.new("ShaderNodeMixShader")
        mix.inputs[0].default_value = alpha
        nt.links.new(tr.outputs[0], mix.inputs[1])
        nt.links.new(em.outputs[0], mix.inputs[2])
        nt.links.new(mix.outputs[0], out.inputs[0])
    else:
        nt.links.new(em.outputs[0], out.inputs[0])
    return m


def load(manifest):
    objs = []
    for part in manifest["parts"]:
        before = set(bpy.data.objects)
        bpy.ops.wm.obj_import(filepath=str(MESH_DIR / part["file"]), forward_axis="Y", up_axis="Z")
        for o in set(bpy.data.objects) - before:
            o.scale = (0.001, 0.001, 0.001)
            o.data.materials.clear()
            o.data.materials.append(material(part["material"]))
            o["group"] = part["group"]
            # FreeCAD writes one normal per facet; drop them and let auto smooth
            # keep edges sharp and cylinders round
            bpy.context.view_layer.objects.active = o
            o.select_set(True)
            if o.data.has_custom_normals:
                bpy.ops.mesh.customdata_custom_splitnormals_clear()
            for poly in o.data.polygons:
                poly.use_smooth = True
            if hasattr(o.data, "use_auto_smooth"):
                o.data.use_auto_smooth = True
                o.data.auto_smooth_angle = math.radians(35)
            o.select_set(False)
            objs.append(o)
    bpy.context.view_layer.update()
    return objs


def bbox(objs):
    lo = Vector((1e9, 1e9, 1e9))
    hi = -lo
    for o in objs:
        for c in o.bound_box:
            w = o.matrix_world @ Vector(c)
            lo = Vector(map(min, lo, w))
            hi = Vector(map(max, hi, w))
    return lo, hi


def add_plane(z, size, rgb):
    bpy.ops.mesh.primitive_plane_add(size=size, location=(0, 0, z))
    p = bpy.context.active_object
    m = bpy.data.materials.new("floor")
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*rgb, 1)
    b.inputs["Roughness"].default_value = 0.8
    p.data.materials.append(m)
    return p


def area_light(loc, target, power, size, name):
    bpy.ops.object.light_add(type="AREA", location=loc)
    lt = bpy.context.active_object
    lt.name = name
    lt.data.energy = power
    lt.data.size = size
    d = Vector(target) - Vector(loc)
    lt.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    return lt


def camera(target, direction, dist, lens=50):
    bpy.ops.object.camera_add()
    cam = bpy.context.active_object
    cam.data.lens = lens
    d = Vector(direction).normalized()
    cam.location = Vector(target) + d * dist
    cam.rotation_euler = (-d).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam
    return cam


def scan_overlay(info):
    """Scan line on the table and a translucent fan from the mirror."""
    s = info.get("scan")
    if not s:
        return []
    m = Vector(s["mirror"]) * 0.001
    d = Vector(s["dir"])
    x = Vector(s["xdir"])
    t = (m.z - 0.0005) / -d.z
    c = m + d * t
    h = s["half_line"] * 0.001
    a, b = c + x * h, c - x * h
    me = bpy.data.meshes.new("fan")
    me.from_pydata([m, a, b], [], [(0, 1, 2)])
    fan = bpy.data.objects.new("fan", me)
    bpy.context.collection.objects.link(fan)
    fan.data.materials.append(emissive("fan", (1.0, 0.15, 0.05), 3.0, alpha=0.3))
    bpy.ops.mesh.primitive_cube_add(size=1, location=c + Vector((0, 0, 0.0003)))
    ln = bpy.context.active_object
    ln.scale = (0.0012 if abs(x.x) < 0.5 else 2 * h, 2 * h if abs(x.x) < 0.5 else 0.0012, 0.0004)
    ln.data.materials.append(emissive("line", (1.0, 0.1, 0.03), 60.0))
    return [fan, ln]


def setup_render(path, w, h):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.samples = SAMPLES
    # the denoiser is optional (Ubuntu's Blender is built without OpenImageDenoise)
    sc.cycles.use_denoising = bool(getattr(bpy.app.build_options, "openimagedenoise", False))
    sc.cycles.sample_clamp_indirect = 3.0
    sc.render.resolution_x, sc.render.resolution_y = int(w * SCALE), int(h * SCALE)
    sc.render.film_transparent = False
    sc.view_settings.view_transform = "AgX" if "AgX" in [i.identifier for i in
                                                           sc.view_settings.bl_rna.properties["view_transform"].enum_items] else "Filmic"
    sc.view_settings.look = "None"
    sc.render.filepath = str(path)
    world = bpy.data.worlds.new("w")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.82, 0.84, 0.87, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.35
    sc.world = world


def shot(name, manifest):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    _CACHE.clear()
    objs = load(manifest)
    groups = {o["group"] for o in objs}
    if name in ("head", "head_open"):
        for o in objs:
            if o["group"] == "arm" or (name == "head_open" and o["group"] in ("HSI_lid", "HSI_stepper")):
                bpy.data.objects.remove(o)
        objs = [o for o in bpy.data.objects if o.type == "MESH"]
        bpy.context.view_layer.update()
    lo, hi = bbox(objs)
    ctr = (lo + hi) / 2
    size = (hi - lo).length
    hp = manifest["head_placement"]
    q = Quaternion((hp[1][3], hp[1][0], hp[1][1], hp[1][2]))
    hx, hy, hz = (q @ Vector(v) for v in ((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    if name == "rig":
        add_plane(0.0, 4.0, (0.6, 0.58, 0.55))
        scan_overlay(manifest)
        direction = Vector((0.95, 0.8, 0.5))
        cam = camera(ctr + Vector((0.03, 0, 0.03)), direction, size * 1.55, lens=45)
    elif name == "head":
        # from below and in front: scan window, hood and pose camera on the
        # scene side, motor on the lid
        direction = -hx * 0.75 + hy * 0.75 - hz * 0.55
        cam = camera(ctr, direction, size * 2.0, lens=60)
    else:
        direction = -hx * 1.0 + hz * 0.15 - hy * 0.45
        cam = camera(ctr, direction, size * 1.75, lens=60)
    w, h = 1800, 1200
    # light power scales with distance squared so every shot gets the same exposure
    key_at = ctr + (cam.location - ctr).normalized() * size * 1.6 + Vector((0, 0, 1.6)) * size
    area_light(key_at, ctr, 260 * size ** 2, size * 1.2, "key")
    area_light(ctr + Vector((-1.2, -1.0, 0.8)) * size, ctr, 90 * size ** 2, size * 2.0, "fill")
    area_light(ctr + Vector((0.3, -0.4, 2.2)) * size, ctr, 140 * size ** 2, size * 0.8, "top")
    setup_render(OUT_DIR / ("%s.png" % name), w, h)
    labels = manifest.get("labels") if name == "head_open" else None
    if labels:
        from bpy_extras.object_utils import world_to_camera_view
        sc = bpy.context.scene
        pts = {}
        for k, v in labels.items():
            c = world_to_camera_view(sc, cam, Vector(v) * 0.001)
            pts[k] = [c.x, 1.0 - c.y]
        (OUT_DIR / ("%s_labels.json" % name)).write_text(json.dumps(pts, indent=1))
    bpy.ops.render.render(write_still=True)
    print("wrote", OUT_DIR / ("%s.png" % name), "groups:", len(groups))


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((MESH_DIR / "manifest.json").read_text())
    for s in SHOTS:
        shot(s, manifest)


main()
