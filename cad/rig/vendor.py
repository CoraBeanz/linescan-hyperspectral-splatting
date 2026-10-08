"""Bought parts, modelled from primitives at datasheet dimensions.

Each part is an App::Part container whose placement is bound to the Params
sheet, holding simple Part primitives in the part's own local frame. They are
envelopes for fit and mass checks, not detailed vendor models: lens barrels
are stepped cylinders, boards are plates with their main components.

Every primitive carries a `RenderMaterial` property that the exporter and
renderer use.
"""

import FreeCAD as App

from .expr import E, Builder, Frame
from .params import MOTOR, MOTORS

MATERIAL_RGB = {
    "printed": (0.16, 0.16, 0.17),      # black PETG
    "printed_accent": (0.95, 0.45, 0.10),
    "arm_printed": (1.0, 0.82, 0.12),
    "servo": (0.10, 0.10, 0.10),
    "aluminum": (0.78, 0.79, 0.80),
    "anodized_black": (0.08, 0.08, 0.09),
    "steel": (0.62, 0.63, 0.65),
    "glass": (0.70, 0.85, 0.90),
    "filter_glass": (0.85, 0.35, 0.10),
    "mirror": (0.92, 0.93, 0.95),
    "grating": (0.55, 0.60, 0.85),
    "pcb_green": (0.10, 0.40, 0.20),
    "pcb_black": (0.05, 0.05, 0.05),
    "chip": (0.12, 0.12, 0.14),
    "connector": (0.90, 0.88, 0.80),
    "magnet": (0.70, 0.72, 0.75),
    "blade": (0.80, 0.81, 0.83),
}

# Listed or estimated masses (g) of the bought parts, for the mass budget
MASS_G = {
    "HSI_stepper": (MOTORS[MOTOR]["mass_g"], "listing"),
    "HSI_objective": (5.0, "listing"),
    "HSI_collimator": (8.0, "listing"),
    "HSI_cam_lens": (7.0, "listing"),
    "HSI_obj_holder": (1.5, "est"),
    "HSI_coll_holder": (1.5, "est"),
    "HSI_camera": (14.0, "est"),
    "HSI_pose_camera": (3.0, "ds"),
    "HSI_mirror": (3.0, "est"),
    "HSI_filter": (1.0, "est"),
    "HSI_field_lens": (1.2, "est"),
    "HSI_slit_blades": (1.0, "est"),
    "HSI_grating": (0.1, "est"),
    "HSI_magnet": (0.24, "est"),
    "HSI_hall": (0.3, "est"),
    "HSI_hall_board": (0.5, "est"),
}


class Part(Builder):
    """Builder that tags each primitive with a render material."""

    mat = "printed"

    def _new(self, kind, name):
        o = super()._new(kind, name)
        o.addProperty("App::PropertyString", "RenderMaterial", "Render")
        o.RenderMaterial = self.mat
        return o

    def m(self, mat):
        self.mat = mat
        return self


def container(doc, parent, name, label, frame=None, fixed_rot=None):
    """App::Part placed at frame.o, rotated about X by frame.a (both bound).

    fixed_rot replaces the X rotation with a constant one (only for untilted
    frames, so the bound angle never fights a compound rotation)."""
    c = doc.addObject("App::Part", name)
    c.Label = label
    parent.addObject(c)
    if frame is not None:
        o = frame.o
        if fixed_rot is not None and frame.tilted:
            raise ValueError("fixed_rot needs an untilted frame")
        rot = fixed_rot or App.Rotation(App.Vector(1, 0, 0), frame.a.v)
        c.Placement = App.Placement(App.Vector(o[0].v, o[1].v, o[2].v), rot)
        for comp, e in zip("xyz", o):
            if not e.const:
                c.setExpression(".Placement.Base." + comp, e.s)
        if fixed_rot is None and not frame.a.const:
            c.setExpression(".Placement.Rotation.Angle", frame.a.s)
    return c


def m12_lens(doc, parent, name, label, frame, od, length, thread, flip=False):
    """M12 lens. Local frame: rear end (image side) at z=0, front at z=-length.

    flip=True turns it to face the other way (used for the collimator, which
    works backwards with its image side toward the slit)."""
    rot = App.Rotation(App.Vector(1, 0, 0), 180) if flip else None
    c = container(doc, parent, name, label, frame, fixed_rot=rot)
    b = Part(doc, c, name)
    b.m("anodized_black").cyl("thread", "z", (0, 0, -thread), 6.0, thread)
    b.cyl("barrel", "z", (0, 0, -length), E.of(od) / 2, E.of(length) - thread)
    b.m("glass").cyl("front_glass", "z", (0, 0, E.of(0.3) - length), E.of(od) / 2 - 2.0, 0.6)
    b.cyl("rear_glass", "z", (0, 0, -0.55), 4.0, 0.6)
    return c


def m12_holder(doc, parent, name, label, frame, P, h, flip=False):
    """Plastic M12 holder cut down to height h. Local frame: mounting face at z=0, body toward -z."""
    rot = App.Rotation(App.Vector(1, 0, 0), 180) if flip else None
    c = container(doc, parent, name, label, frame, fixed_rot=rot)
    b = Part(doc, c, name).m("anodized_black")
    body = b.cyl("body", "z", (0, 0, -h), P.h12_d / 2, h)
    base = b.cbox("base", 0, 0, -P.h12_ear_t, P.h12_ear, P.h12_w, P.h12_ear_t)
    bore = b.cyl("bore", "z", (0, 0, -h - 1), 6.0, h + 2)
    holes = [b.cyl("hole%d" % i, "z", (s * P.h12_pitch / 2, 0, -P.h12_ear_t - 1), 1.1, P.h12_ear_t + 2)
             for i, s in enumerate((-1, 1))]
    u = b.fuse("solid", [body, base])
    b.cut("holder", u, [bore] + holes)
    return c


def stepper(doc, parent, P):
    """Scan stepper (RIG_MOTOR) bolted to the outside of the lid, shaft +X."""
    x_face = -P.x_in - P.lid_motor_t
    f = Frame((x_face, P.y_shaft, P.z_shaft))
    c = container(doc, parent, "HSI_stepper", MOTORS[MOTOR]["label"], f)
    b = Part(doc, c, "stepper").m("anodized_black")
    w, L = P.mot_w, P.mot_len
    body = b.box("body", -L, -w / 2, -w / 2, L, w, w)
    corners = []
    for i, (sy, sz) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        # a diamond prism along X on each corner cuts the chamfer
        corners.append(b.prism("corner%d" % i, "x", (-L - 1, sy * w / 2, sz * w / 2), 4, P.mot_chamfer, L + 2))
    thread_r = 1.25 if MOTORS[MOTOR]["bolts_into_motor"] else 0.8      # M3 / M2 tap drill
    holes = [b.cyl("thread%d" % i, "-x", (0.01, sy * P.mot_hole_pitch / 2, sz * P.mot_hole_pitch / 2), thread_r,
                   P.mot_hole_depth)
             for i, (sy, sz) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1)))]
    b.cut("housing", body, corners + holes)
    b.m("steel").cyl("boss", "x", (0, 0, 0), P.mot_boss_d / 2, P.mot_boss_h)
    b.cyl("shaft", "x", (0, 0, 0), P.mot_shaft_d / 2, P.mot_shaft_len)
    # lead strain relief on the +Z flat, near the rear face
    x0, rl, rw, rh = MOTORS[MOTOR]["relief"]
    b.m("connector").box("strain_relief", -L + x0, -rw / 2, w / 2 - 0.5, rl, rw, rh)
    return c


def scan_mirror(doc, parent, P, rotor_frame):
    """Front-surface mirror; its frame turns with the shaft (angle in the sheet)."""
    c = container(doc, parent, "HSI_mirror", "Scan mirror: 20 x 20 mm cut from a RUEHALF 3 mm front-surface sheet",
                  rotor_frame)
    b = Part(doc, c, "mirror")
    b.m("mirror").box("glass", -P.mirror_l / 2, P.mirror_e - P.mirror_t, -P.mirror_w / 2,
                      P.mirror_l, P.mirror_t, P.mirror_w)
    return c


def magnet(doc, parent, P, rotor_frame):
    c = container(doc, parent, "HSI_magnet", "Home magnet 5 x 1.6 mm", rotor_frame)
    b = Part(doc, c, "magnet").m("magnet")
    b.cyl("disc", "x", (P.pad_x1 - P.mag_t, -P.mag_r, 0), P.mag_d / 2, P.mag_t)
    return c


def filter_disc(doc, parent, P, z_front):
    f = Frame((0, P.y_axis, z_front))
    c = container(doc, parent, "HSI_filter", "Long-pass filter: Edmund #54-652, GG-495, 12.5 mm", f)
    Part(doc, c, "filter").m("filter_glass").cyl("disc", "z", (0, 0, 0), P.filt_d / 2, P.filt_t)
    return c


def field_lens(doc, parent, P, z_flat):
    """Plano-convex, flat face toward the slit at z_flat; modelled as flat + cone cap."""
    f = Frame((0, P.y_axis, z_flat))
    c = container(doc, parent, "HSI_field_lens", "Field lens: Edmund #49-840, PCX 12.7 mm, f = 15 mm", f)
    b = Part(doc, c, "field_lens").m("glass")
    edge = b.cyl("edge", "z", (0, 0, 0), P.fl_d / 2, P.fl_et)
    dome = b.cone("dome", "z", (0, 0, P.fl_et), P.fl_d / 2, P.fl_d / 2 * 0.25, P.fl_ct - P.fl_et)
    b.fuse("lens", [edge, dome])
    return c


def slit_blades(doc, parent, P, z_back):
    """Two blade pieces, edges slit_width apart, backs resting at z_back."""
    f = Frame((0, P.y_axis, z_back))
    c = container(doc, parent, "HSI_slit_blades", "Slit: two snap-off blade pieces, 50 um gap", f)
    b = Part(doc, c, "blade").m("blade")
    g = P.slit_width / 2
    b.box("upper", -P.blade_l / 2, g, -P.blade_t, P.blade_l, P.blade_w, P.blade_t)
    b.box("lower", -P.blade_l / 2, -g - P.blade_w, -P.blade_t, P.blade_l, P.blade_w, P.blade_t)
    return c


def grating_film(doc, parent, P, z_film):
    f = Frame((0, P.y_axis, z_film))
    c = container(doc, parent, "HSI_grating", "Grating film 500 l/mm (grooves along X)", f)
    Part(doc, c, "grating").m("grating").cbox("film", 0, 0, -0.3, P.grat_w, P.grat_w, P.grat_t)
    return c


def camera_board(doc, parent, P, frame):
    """B0152 IMX219 NoIR board, outside the end wall, in the tilted camera frame.

    frame: origin at the camera lens principal plane, +Z toward the sensor.
    The sensor sits at z = f_cam with the PCB behind it."""
    c = container(doc, parent, "HSI_camera", "Arducam B0152 IMX219 NoIR (M12)", frame)
    b = Part(doc, c, "cam")
    z_pcb = P.f_cam + P.sensor_above_pcb
    pcb = b.m("pcb_black").cbox("pcb_blank", 0, 0, z_pcb, P.brd_w, P.brd_h, P.brd_t)
    holes = [b.cyl("hole%d" % i, "z", (sx * P.brd_hole / 2, sy * P.brd_hole_y / 2, z_pcb - 1), 1.1, P.brd_t + 2)
             for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1)))]
    b.cut("pcb", pcb, holes)
    b.m("chip").cbox("sensor", 0, 0, P.f_cam, 8.5, 8.5, P.sensor_above_pcb)
    # the board's own M12 holder, now carrying the CIL122
    hb = b.m("pcb_black").cbox("holder_body", 0, 0, z_pcb - P.brd_holder_h, P.brd_holder_w,
                                P.brd_holder_w, P.brd_holder_h)
    bore = b.cyl("holder_bore", "z", (0, 0, z_pcb - P.brd_holder_h - 1), 6.0, P.brd_holder_h + 0.5)
    b.cut("holder", hb, [bore])
    back = z_pcb + P.brd_t
    b.m("connector").cbox("fpc_connector", P.brd_w / 2 - 3.5, 0, back, 5.5, 21, 2.5)
    b.m("chip").cbox("bridge_ic", -6, 4, back, 6, 6, 1.0)
    return c


def pose_camera(doc, parent, P, y_board):
    """Pi NoIR v2 on standoffs on the +Y face, lens looking at the scene.

    Local frame: origin at the lens centre on the PCB underside, +Z out of the
    lens (world +Y), +Y toward the wrist (world -Z). Hole and connector
    positions from the RPI-CAM-V2_1 drawing."""
    f = Frame((0, y_board, P.z_pose))
    rot = App.Rotation(App.Vector(1, 0, 0), -90)
    c = container(doc, parent, "HSI_pose_camera", "Pose camera: Pi NoIR v2 (owned)", f, fixed_rot=rot)
    b = Part(doc, c, "posecam")
    x0, y0 = -12.49, -14.40   # board corner relative to the lens centre
    pcb = b.m("pcb_green").box("pcb_blank", x0, y0, 0, P.pi_w, P.pi_h, 1.0)
    holes = [b.cyl("hole%d" % i, "z", (x0 + hx, y0 + hy, -1), 1.1, 3)
             for i, (hx, hy) in enumerate(((2.0, 2.0), (22.98, 2.0), (2.0, 14.52), (22.98, 14.52)))]
    b.cut("pcb", pcb, holes)
    b.m("chip").cbox("lens_module", 0, 0, 1.0, 8.5, 8.5, 3.5)
    b.m("anodized_black").cyl("lens", "z", (0, 0, 4.5), 2.8, 1.2)
    b.m("connector").box("fpc_connector", x0 + 2.05, y0 + 18.33, -2.7, 20.88, 5.52, 2.7)
    return c


def hall_sensor(doc, parent, P):
    """A3144 in a pocket in the +X wall, face toward the magnet (-X)."""
    f = Frame((P.x_in, P.hall_y, P.hall_z))
    c = container(doc, parent, "HSI_hall", "A3144 hall sensor (home switch)", f)
    b = Part(doc, c, "hall").m("chip")
    b.box("body", 0.1, -2.05, -1.5, 1.5, 4.1, 3.0)
    b.m("steel")
    for i, y in enumerate((-1.27, 0, 1.27)):
        # bent at the body, out through the wall's slot and the breakout, trimmed 1 mm proud of it
        b.box("lead%d" % i, 1.6, y - 0.2, -1.2, P.wall + P.hb_t - 0.6, 0.4, 0.4)
    return c


# local x -> +Y, local y -> +Z, local z -> +X: a board lying flat on the +X face
_ON_PLUS_X = App.Rotation(App.Matrix(0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1))


def hall_board(doc, parent, P):
    """Hall-sensor breakout (pcb/hall_breakout) glued flat on the outside of the +X wall.

    The A3144's leads come out of the wall's slot and through the board from
    behind. Local frame: origin at the board's top corner on the -Y side, on
    its back face; KiCad's board coordinates (bx, by), y down, sit at
    (bx, -by, 0). Hole and part positions from pcb/tools/boards.py."""
    f = Frame((P.x_in + P.wall, P.hall_y - P.hb_lead_x, P.hall_z - 1.0 + P.hb_lead_y))
    c = container(doc, parent, "HSI_hall_board", "Hall-sensor breakout (pcb/hall_breakout)", f,
                  fixed_rot=_ON_PLUS_X)
    b = Part(doc, c, "hallpcb").m("pcb_green")
    pcb = b.box("pcb_blank", 0, -P.hb_h, 0, P.hb_w, P.hb_h, P.hb_t)
    holes = [b.cyl("lead_hole%d" % i, "z", (P.hb_lead_x + dx, -P.hb_lead_y, -1), 0.375, P.hb_t + 2)
             for i, dx in enumerate((-1.27, 0, 1.27))]
    pads = (4.46, 7.0, 9.54)                        # cable pads S, G, +: 1 mm holes
    holes += [b.cyl("pad_hole%d" % i, "z", (x, -7.6, -1), 0.5, P.hb_t + 2) for i, x in enumerate(pads)]
    # two 1.5 mm holes for a strain-relief tie, and the M2 hole
    for i, (x, y, d) in enumerate(((2.4, 10.6, 1.5), (11.6, 10.6, 1.5), (12.0, 2.3, 2.2))):
        holes.append(b.cyl("hole%d" % i, "z", (x, -y, -1), d / 2, P.hb_t + 2))
    b.cut("pcb", pcb, holes)
    b.m("connector").cbox("c1", 7.635, -1.9, P.hb_t, 2.0, 1.25, 0.85)
    # the start of the 3-wire cable: soldered into the pads, it leaves past the board's lower edge
    b.m("chip")
    for i, x in enumerate(pads):
        b.cyl("wire%d" % i, "-y", (x, -7.6, P.hb_t + 0.65), 0.65, P.hb_h - 6.6)
    return c


def controller_board(doc, parent, P, frame):
    """Scan-mirror controller (pcb/scan_controller) with the DevKit and the driver plugged in.

    Local frame: origin at the board's top-left corner (USB edge, Jetson
    header side) on its underside, z up; KiCad's board coordinates (bx, by),
    y down, sit at (bx, -by). Positions from pcb/tools/boards.py and the
    footprints in pcb/lib; heights from pcb/README.md."""
    c = container(doc, parent, "CTRL_board", "Scan-mirror controller (pcb/scan_controller)", frame)
    b = Part(doc, c, "ctl").m("pcb_green")
    w, h, t, hi = P.ctl_w, P.ctl_h, P.ctl_t, P.ctl_hole_in
    pcb = b.rbox("pcb_blank", w / 2, -h / 2, 0, w, h, t, P.ctl_r)
    holes = [b.cyl("hole%d" % i, "z", (x, -y, -1), 1.6, t + 2)
             for i, (x, y) in enumerate(((hi, hi), (w - hi, hi), (hi, h - hi), (w - hi, h - hi)))]
    b.cut("pcb", pcb, holes)
    sock = 8.5                                      # female header height
    seat = E.of(t) + sock + 2.5                     # a plugged-in module's underside (2.5 mm of male header)
    b.m("chip")
    for i, x in enumerate((13.97, 39.37)):          # ESP32-DevKitC, 2x 1x19
        b.box("devkit_header%d" % i, x - 1.27, -54.61, t, 2.54, 48.26, sock)
    for i, x in enumerate((46.99, 59.69)):          # TMC2209, 2x 1x8
        b.box("driver_header%d" % i, x - 1.27, -34.29, t, 2.54, 20.32, sock)
    b.m("pcb_black").box("devkit_pcb", 12.72, -57.68, seat, 27.9, 54.4, 1.6)
    b.box("module_pcb", 17.67, -57.68, seat + 1.6, 18.0, 25.5, 0.8)
    b.m("steel").box("module_shield", 18.67, -50.78, seat + 2.4, 16.0, 17.6, 2.3)
    b.box("usb", 22.92, -7.8, seat + 1.6, 7.5, 5.5, 2.5)
    b.m("pcb_black").box("driver_pcb", 45.72, -34.29, seat, 15.24, 20.32, 1.6)
    b.m("aluminum").cbox("heatsink", 53.34, -24.13, seat + 1.6, 12.0, 12.0, 9.4)
    b.m("chip").box("jack", 80.2, -18.5, t, 14.8, 9.0, 11.0)                 # J1, 12 V
    b.m("connector").box("terminal", 83.69, -31.04, t, 11.04, 11.17, 10.0)   # J2, 12 V
    b.box("motor_conn", 62.14, -30.77, t, 6.75, 13.4, 7.0)                    # J3, JST XH 4-pin
    b.box("hall_conn", 42.77, -64.9, t, 10.9, 6.75, 7.0)                      # J4, JST XH 3-pin
    b.m("anodized_black").cyl("c1", "z", (61.75, -8.5, t), 3.15, 11.0)       # 100 uF
    b.m("chip").box("jetson_header", 1.27, -39.37, t, 2.54, 7.62, 2.5)       # J5
    b.m("steel")
    for i, y in enumerate((33.02, 35.56, 38.1)):
        b.cbox("jetson_pin%d" % i, 2.54, -y, E.of(t) + 2.5, 0.64, 0.64, 6.0)
    return c
