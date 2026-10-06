"""3D-printed parts of the HSI head, all bound to the Params sheet.

Head frame: origin at the face of the wrist-roll servo horn, +Z along the roll
axis away from the wrist, +Y toward the scene, X along the slit and the mirror
shaft. The optical axis runs along +Z at (x = 0, y = y_axis).

Parts:
  shell       housing: floor with the dovetail slot, scan window and hood,
              slots for the optic carriers, camera end wall with the board
              standoffs, pose-camera standoffs, hall-sensor pocket
  lid         closes the open -X side and carries the stepper
  rotor       mirror clamp on the motor shaft, with the home magnet
  puck        bolts to the wrist-roll horn; dovetail rail the head slides on
  bench_puck  the same rail on a plate with a 1/4"-20 nut, for bench tests
  obj_plate   carries the objective's M12 holder
  slit_block  slit blades, field lens, collimator holder on its back
  grat_plate  grating film
  filter_cap  long-pass filter and the 4 mm aperture stop on the objective

Each function returns a dict with the container, the final solid feature and
how to lay it on the print bed; build_rig.py exports STLs from these.
"""

import FreeCAD as App

from .expr import E, Frame, tan
from .params import MOTOR, MOTORS
from .vendor import Part, container


def _ctx(P):
    """Common expressions."""
    c = {}
    c["xi"] = P.x_in
    c["xo"] = P.x_in + P.wall
    c["ylo_i"] = P.y_axis - P.y_in_neg
    c["yhi_i"] = P.y_axis + P.y_in_pos
    c["ylo"] = c["ylo_i"] - P.wall
    c["yhi"] = c["yhi_i"] + P.wall
    c["zb"] = P.z_shell
    # camera frame: origin at the camera lens principal plane, +Z to the sensor
    c["cf"] = Frame((0, P.y_cam, P.z_cam), P.theta)
    c["zw_in"] = P.f_cam + P.sensor_above_pcb - P.standoff_h - P.carrier_t   # end wall, local z
    c["zw_out"] = c["zw_in"] + P.carrier_t
    c["L"] = P.z_sensor + 40 - P.z_shell                # long enough to pass the tilted end
    return c


def _print(name, label, cont, final, rot, note, qty=1):
    final.addProperty("App::PropertyString", "PrintNote", "Print")
    final.PrintNote = note
    return {"name": name, "label": label, "container": cont, "final": final,
            "print_rot": rot, "note": note, "qty": qty}


def carrier_slots(P):
    """Z ranges [z0, z1] of the slide-in carriers (ribs go either side)."""
    return [
        ("obj_plate", P.z_obj_plate, P.z_obj_plate + P.carrier_t),
        ("slit_block", P.z_slit, P.z_slit + P.slit_t),
        ("grat_plate", P.z_film - P.carrier_t, P.z_film),
    ]


def shell(doc, parent, P):
    c = _ctx(P)
    xi, xo, ylo, yhi, ylo_i, yhi_i, zb = c["xi"], c["xo"], c["ylo"], c["yhi"], c["ylo_i"], c["yhi_i"], c["zb"]
    cf, zw_in, zw_out, L = c["cf"], c["zw_in"], c["zw_out"], c["L"]
    cont = container(doc, parent, "HSI_shell", "Housing (printed)")
    b = Part(doc, cont, "shell")

    # --- solids added before the interior is hollowed
    outer = b.box("outer", -xi, ylo, zb, xo + xi, yhi - ylo, L)
    # window hood; its underside is chamfered 45 deg so it prints without support
    h = P.win / 2 + 2
    hh = P.hood_h + 0.5
    hood = b.wedge("hood", (h, yhi - 0.5, 0), P.z_mirror - h, P.z_mirror + h, P.z_mirror - h + hh,
                   P.z_mirror + h, hh, 2 * h, axis="zy-x")
    boss_p = b.cyl("lidboss_p", "x", (-xi, yhi + 1, P.z_lid_screw), 3.5, 8)
    boss_n = b.cyl("lidboss_n", "x", (-xi, ylo - 1, P.z_lid_screw), 3.5, 8)
    # camera end wall: a flange in the camera frame, wide enough for the 36 mm board
    flange_raw = b.box("flange_raw", -xi - 4.5, -20, zw_in, xo + xi + 4.5, 50, P.carrier_t, cf)
    above = b.box("flange_trim", -xi - 10, yhi, zb, xo + xi + 20, 60, L + 20)
    flange = b.cut("flange", flange_raw, [above])
    body = b.fuse("body", [outer, hood, boss_p, boss_n, flange])

    # --- hollow it: interior up to the end wall, open on -X; mirror sweep in the floor
    inner_raw = b.box("inner_raw", -xi - 1, ylo_i, P.z_floor, 2 * xi + 1, yhi_i - ylo_i, L)
    past_in = b.box("past_wall_in", -60, -80, zw_in, 120, 160, 100, cf)
    inner = b.cut("inner", inner_raw, [past_in])
    past_out = b.box("past_wall_out", -60, -80, zw_out, 120, 160, 100, cf)
    sweep = b.cyl("sweep", "x", (-xi - 0.5, P.y_shaft, P.z_shaft), P.sweep_r, 2 * xi + 0.5)
    hollow = b.cut("hollow", body, [past_out, inner, sweep])

    # --- ribs for the carriers, camera standoffs, pose-camera standoffs
    adds = []
    for name, z0, z1 in carrier_slots(P):
        for side, zr in (("f", z0 - P.fit - P.rib_w), ("b", z1 + P.fit)):
            n = "%s_%s" % (name, side)
            adds.append(b.box("rib_py_" + n, -xi, yhi_i - P.rib_h, zr, 2 * xi, P.rib_h + 0.01, P.rib_w))
            adds.append(b.box("rib_ny_" + n, -xi, ylo_i - 0.01, zr, 2 * xi, P.rib_h + 0.01, P.rib_w))
            adds.append(b.box("rib_px_" + n, xi - P.rib_h, ylo_i, zr, P.rib_h + 0.01, yhi_i - ylo_i, P.rib_w))
    hb = P.brd_hole / 2
    cam_holes = ((-1, -1), (1, -1), (-1, 1), (1, 1))
    for i, (sx, sy) in enumerate(cam_holes):
        adds.append(b.cyl("cam_standoff%d" % i, "z", (sx * hb, sy * hb, zw_out - 0.5), 3.0, P.standoff_h + 0.5, cf))
    # Pi camera v2 holes, relative to its lens centre (local x, local y -> world -Z)
    pose_holes = ((-10.49, -12.40), (10.49, -12.40), (-10.49, 0.12), (10.49, 0.12))
    for i, (hx, hy) in enumerate(pose_holes):
        adds.append(b.cyl("pose_standoff%d" % i, "y", (hx, yhi - 0.5, P.z_pose - hy), 2.5, 4.0))
    solid = b.fuse("ribbed", [hollow] + adds)

    # --- holes and pockets
    cuts = []
    cuts.append(b.box("window", -P.win / 2, yhi_i - 1, P.z_mirror - P.win / 2, P.win, P.wall + P.hood_h + 2, P.win))
    # dovetail slot, open at -X, blind 2 mm before the +X face
    tf = tan(P.rail_flank)
    h0 = P.z_shell - P.puck_top                       # slot starts this far up the rail
    hs = P.rail_h + P.fit                             # slot top above the puck
    w0 = P.rail_w / 2 + h0 * tf + P.fit
    w1 = P.rail_w / 2 + hs * tf + P.fit
    cuts.append(b.wedge("dovetail_slot", (-xi - 1, 0, zb - 0.01), -w0, w0, -w1, w1,
                        hs - h0 + 0.01, xo + xi - 1, axis="yzx"))
    # two long M3x35 through-bolts: +X face -> floor -> lid -> the NEMA 17's
    # lower holes, or nuts on the lid for a smaller motor
    zl = P.z_shaft - P.bolt_pitch / 2
    for i, s in enumerate((-1, 1)):
        y = P.y_shaft + s * P.bolt_pitch / 2
        cuts.append(b.cyl("bolt%d" % i, "x", (-xi - 1, y, zl), P.m3_clear / 2, xo + xi + 2))
        cuts.append(b.cyl("bolt_head%d" % i, "x", (xo - 3, y, zl), P.m3_head / 2, 4))
    # hall sensor pocket and lead slot in the +X wall
    cuts.append(b.box("hall_pocket", xi - 0.01, P.hall_y - 2.25, P.hall_z - 1.65, 1.71, 4.5, 3.3))
    cuts.append(b.box("hall_leads", xi + 1.0, P.hall_y - 2.2, P.hall_z - 1.5, P.wall, 4.4, 1.0))
    # end wall: opening for the camera's M12 holder and lens; M2 inserts in the standoffs
    cuts.append(b.cyl("cam_opening", "z", (0, 0, zw_in - 1), 10.0, P.carrier_t + 2, cf))
    for i, (sx, sy) in enumerate(cam_holes):
        cuts.append(b.cyl("cam_insert%d" % i, "z", (sx * hb, sy * hb, zw_out + P.standoff_h - 4.0),
                          P.insert_m2_d / 2, 4.1, cf))
    for i, (hx, hy) in enumerate(pose_holes):
        cuts.append(b.cyl("pose_insert%d" % i, "y", (hx, yhi - 1.0, P.z_pose - hy), P.insert_m2_d / 2, 4.6))
    for i, y in enumerate((yhi + 1, ylo - 1)):
        cuts.append(b.cyl("lid_insert%d" % i, "x", (-xi - 0.1, y, P.z_lid_screw), P.insert_d / 2, 6.6))
    final = b.cut("Housing", solid, cuts)
    final.Label = "Housing"
    return _print("housing", "Housing", cont, final, App.Rotation(),
                  "floor (dovetail side) on the bed, camera end up; 0.2 mm layers, 3 perimeters, "
                  "20% gyroid; supports only under the camera flange")


def lid(doc, parent, P):
    c = _ctx(P)
    xi, ylo, yhi, zb, cf, zw_in, L = c["xi"], c["ylo"], c["yhi"], c["zb"], c["cf"], c["zw_in"], c["L"]
    cont = container(doc, parent, "HSI_lid", "Lid with motor mount (printed)")
    b = Part(doc, cont, "lid")
    plate = b.box("plate", -xi - P.lid_t, ylo, zb, P.lid_t, yhi - ylo, L)
    half = P.lid_pad_half
    pad = b.box("motor_pad", -xi - P.lid_motor_t, P.y_shaft - half, zb, P.lid_motor_t, 2 * half,
                P.z_shaft + half - zb)
    ears = [b.cyl("ear%d" % i, "x", (-xi - P.lid_t, y, P.z_lid_screw), 3.5, P.lid_t)
            for i, y in enumerate((yhi + 1, ylo - 1))]
    body = b.fuse("body", [plate, pad] + ears)
    cuts = [b.box("past_wall", -60, -80, zw_in - P.fit, 120, 160, 100, cf)]
    x_out = -xi - P.lid_motor_t - 0.1
    cuts.append(b.cyl("shaft_hole", "x", (x_out, P.y_shaft, P.z_shaft), P.mot_shaft_d / 2 + 1,
                      P.lid_motor_t + 0.2))
    cuts.append(b.cyl("boss_pocket", "x", (x_out, P.y_shaft, P.z_shaft), P.mot_boss_d / 2 + 0.25,
                      P.mot_boss_h + 0.3))
    into_motor = MOTORS[MOTOR]["bolts_into_motor"]
    for i, (sy, sz) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
        y = P.y_shaft + sy * P.mot_hole_pitch / 2
        z = P.z_shaft + sz * P.mot_hole_pitch / 2
        cuts.append(b.cyl("mot_hole%d" % i, "x", (x_out, y, z), P.mot_screw_clear / 2, P.lid_motor_t + 0.2))
        # screws from inside, heads flush with the inner face; on the NEMA 17
        # the lower pair are the long bolts instead
        if sz > 0 or not into_motor:
            cuts.append(b.cyl("mot_cbore%d" % i, "x", (-xi - P.mot_head_h, y, z), P.mot_head_d / 2,
                              P.mot_head_h + 0.1))
    if not into_motor:
        # the long bolts pass beside the motor and end in a washer and nut on the
        # pad (a nut trap would leave half a millimetre at the pad's bottom edge)
        for i, s in enumerate((-1, 1)):
            y = P.y_shaft + s * P.bolt_pitch / 2
            z = P.z_shaft - P.bolt_pitch / 2
            cuts.append(b.cyl("bolt_hole%d" % i, "x", (x_out, y, z), P.m3_clear / 2, P.lid_motor_t + 0.2))
    for i, y in enumerate((yhi + 1, ylo - 1)):
        cuts.append(b.cyl("ear_hole%d" % i, "x", (-xi - P.lid_t - 0.1, y, P.z_lid_screw), P.m3_clear / 2,
                          P.lid_t + 0.2))
    final = b.cut("Lid", body, cuts)
    final.Label = "Lid"
    return _print("lid", "Lid", cont, final, App.Rotation(App.Vector(0, 1, 0), 90),
                  "inner (flat) face on the bed; 0.2 mm layers, 4 perimeters around the motor holes")


def rotor_frame(P):
    return Frame((0, P.y_shaft, P.z_shaft), E(45) + P.mirror_scan)


def rotor(doc, parent, P):
    """Clamp hub on the shaft, a pad the mirror is glued to, and a magnet tab."""
    xi = P.x_in
    cont = container(doc, parent, "HSI_rotor", "Mirror clamp (printed)", rotor_frame(P))
    b = Part(doc, cont, "rotor").m("printed_accent")
    x0 = -xi + 0.5
    x1 = x0 + P.hub_len                               # at or past the shaft end
    back = P.mirror_e - P.mirror_t                    # mirror back = pad face (local y)
    hub = b.box("hub", x0, -8.5, -5, x1 - x0, back + 8.5, 10)
    pad = b.box("pad", x0, back - 2, -8.5, P.mirror_l / 2 - x0, 2, 17)
    tab = b.box("tab", P.mirror_l / 2 - 3, -P.mag_r - 3, -3.5, 3, back - 2 + P.mag_r + 3, 7)
    solid = b.fuse("body", [hub, pad, tab])
    xm = x0 + P.pinch_dx
    cuts = [
        b.cyl("bore", "x", (x0 - 1, 0, 0), P.mot_shaft_d / 2 + 0.05, x1 - x0 + 1.1),
        b.box("split", x0 - 1, -9, -0.6, x1 - x0 + 2, 9, 1.2),
        b.cyl("pinch", "z", (xm, -5.5, -6), P.m3_clear / 2, 12),
        b.cyl("pinch_head", "z", (xm, -5.5, 3), P.m3_head / 2, 3),
        b.prism("pinch_nut", "z", (xm, -5.5, -5.1), 6, 3.33, 2.6),
        b.cyl("magnet_pocket", "x", (P.mirror_l / 2 - P.mag_t - 0.2, -P.mag_r, 0), P.mag_d / 2 + 0.1,
              P.mag_t + 0.3),
    ]
    final = b.cut("Mirror_clamp", solid, cuts)
    final.Label = "Mirror clamp"
    return _print("mirror_clamp", "Mirror clamp", cont, final, App.Rotation(App.Vector(1, 0, 0), -90),
                  "mirror pad face on the bed; 0.15 mm layers, 100% infill; M3x8 pinch screw + nut")


def _rail(b, name, P, base, length):
    tf = tan(P.rail_flank)
    w0 = P.rail_w / 2
    w1 = w0 + P.rail_h * tf
    return b.wedge(name, base, -w0, w0, -w1, w1, P.rail_h + 0.01, length, axis="yzx")


def puck(doc, parent, P):
    c = _ctx(P)
    xi, xo = c["xi"], c["xo"]
    cont = container(doc, parent, "HSI_puck", "Wrist puck with dovetail rail (printed)")
    b = Part(doc, cont, "puck")
    disc = b.cyl("disc", "z", (0, 0, 0), P.puck_d / 2, P.puck_top)
    # a narrow rim around the horn disc centres the puck; kept inside r = 12 so
    # it clears the wrist bracket, which rises to z = -1.4 just outside
    rim = b.cyl("rim", "z", (0, 0, -P.puck_rim), P.horn_d / 2 + 2.0, P.puck_rim + 0.01)
    rail = _rail(b, "rail", P, (-xi + P.fit, 0, P.puck_top - 0.01), xo + xi - 2 - 2 * P.fit)
    solid = b.fuse("body", [disc, rim, rail])
    top = P.puck_top + P.rail_h + 1
    cuts = [
        b.cyl("horn_recess", "z", (0, 0, -P.puck_rim - 0.1), P.horn_d / 2 + 0.2, P.puck_rim + 0.1),
        b.cyl("horn_screw", "z", (0, 0, -0.1), P.horn_boss_d / 2 + 0.5, P.horn_boss_h + 0.4),
    ]
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
        x, y = sx * P.horn_sq / 2, sy * P.horn_sq / 2
        cuts.append(b.cyl("hole%d" % i, "z", (x, y, -P.puck_rim - 0.1), P.m3_clear / 2, top + P.puck_rim))
        # M3x6 into the horn: 3.5 mm of puck under the head, tip flush with the horn's back
        z_head = E(6) - P.horn_t
        cuts.append(b.cyl("cbore%d" % i, "z", (x, y, z_head), P.m3_head / 2, top - z_head))
    final = b.cut("Wrist_puck", solid, cuts)
    final.Label = "Wrist puck"
    return _print("wrist_puck", "Wrist puck", cont, final, App.Rotation(),
                  "horn side on the bed (bridges the 20 mm horn recess); 100% infill; 4x M3x6 into the horn")


def bench_puck(doc, parent, P):
    """Same rail on a 50 x 40 plate with a 1/4"-20 nut trap: tripod or optical table."""
    c = _ctx(P)
    xi, xo = c["xi"], c["xo"]
    cont = container(doc, parent, "HSI_bench_puck", "Bench puck: rail on a tripod plate (printed)")
    cont.Placement = App.Placement(App.Vector(0, 0, -9.0), App.Rotation())
    b = Part(doc, cont, "bench")
    plate = b.box("plate", -25, -20, 0, 50, 40, 9)
    rail = _rail(b, "rail", P, (-xi + P.fit, 0, 8.99), xo + xi - 2 - 2 * P.fit)
    solid = b.fuse("body", [plate, rail])
    cuts = [
        b.cyl("tripod_hole", "z", (0, 0, -0.1), 3.3, 20),
        b.prism("tripod_nut", "z", (0, 0, -0.1), 6, 6.5, 5.9),
        b.cyl("table_slot_a", "z", (-19, 0, -0.1), 3.3, 10),
        b.cyl("table_slot_b", "z", (19, 0, -0.1), 3.3, 10),
    ]
    final = b.cut("Bench_puck", solid, cuts)
    final.Label = "Bench puck"
    return _print("bench_puck", "Bench puck", cont, final, App.Rotation(),
                  "flat side on the bed; 1/4\"-20 hex nut pressed in from below")


def _carrier_blank(b, P, z0, t):
    c = _ctx(P)
    xi, ylo_i, yhi_i = c["xi"], c["ylo_i"], c["yhi_i"]
    return b.box("plate", -xi + P.fit, ylo_i + P.fit, z0, 2 * xi - 2 * P.fit, yhi_i - ylo_i - 2 * P.fit, t)


def obj_plate(doc, parent, P):
    cont = container(doc, parent, "HSI_obj_plate", "Objective carrier (printed)")
    b = Part(doc, cont, "objplate")
    z0 = P.z_obj_plate
    blank = _carrier_blank(b, P, z0, P.carrier_t)
    cuts = [b.cyl("aperture", "z", (0, P.y_axis, z0 - 0.1), 5.0, P.carrier_t + 0.2)]
    for i, s in enumerate((-1, 1)):
        cuts.append(b.cyl("holder_screw%d" % i, "z", (s * P.h12_pitch / 2, P.y_axis, z0 - 0.1),
                          P.m2_pilot / 2, P.carrier_t + 0.2))
    final = b.cut("Objective_carrier", blank, cuts)
    final.Label = "Objective carrier"
    return _print("objective_carrier", "Objective carrier", cont, final, App.Rotation(),
                  "flat; M12 holder on the face toward the mirror with 2x M2x5")


def slit_block(doc, parent, P):
    cont = container(doc, parent, "HSI_slit_block", "Slit block (printed)")
    b = Part(doc, cont, "slit").m("printed")
    z0 = P.z_slit
    blank = _carrier_blank(b, P, z0, P.slit_t)
    rec = P.blade_t + 0.05
    cuts = [
        # blades lie flush in a recess on the front face, edges meeting on the axis
        b.box("blade_recess", -P.blade_l / 2 - 0.3, P.y_axis - P.blade_w - 0.4, z0 - 0.1,
              P.blade_l + 0.6, 2 * P.blade_w + 0.8, rec + 0.1),
        # field lens drops in from the front, flat face up against the blades,
        # convex side seated on the edge of the 10 mm aperture
        b.cyl("lens_pocket", "z", (0, P.y_axis, z0 + rec - 0.01), P.fl_d / 2 + 0.15, 3.3),
        b.cyl("aperture", "z", (0, P.y_axis, z0 - 0.1), 5.0, P.slit_t + 0.2),
    ]
    for i, s in enumerate((-1, 1)):
        cuts.append(b.cyl("holder_screw%d" % i, "z", (s * P.h12_pitch / 2, P.y_axis, z0 + P.slit_t - 4),
                          P.m2_pilot / 2, 4.1))
    final = b.cut("Slit_block", blank, cuts)
    final.Label = "Slit block"
    return _print("slit_block", "Slit block", cont, final, App.Rotation(App.Vector(1, 0, 0), 180),
                  "back face on the bed so the blade recess prints flat; 0.12 mm layers for the recess")


def grat_plate(doc, parent, P):
    cont = container(doc, parent, "HSI_grat_plate", "Grating carrier (printed)")
    b = Part(doc, cont, "gratplate")
    z1 = P.z_film
    z0 = z1 - P.carrier_t
    blank = _carrier_blank(b, P, z0, P.carrier_t)
    cuts = [
        b.cbox("aperture", 0, P.y_axis, z0 - 0.1, 12, 12, P.carrier_t + 0.2),
        b.cbox("film_recess", 0, P.y_axis, z1 - 0.3, P.grat_w + 0.4, P.grat_w + 0.4, 0.4),
    ]
    final = b.cut("Grating_carrier", blank, cuts)
    final.Label = "Grating carrier"
    return _print("grating_carrier", "Grating carrier", cont, final, App.Rotation(),
                  "flat, film recess up; tape the film in with its grooves along X")


def filter_cap(doc, parent, P):
    """Push-on cap: 4 mm stop, then the 17 mm long-pass disc, then a sleeve on the lens."""
    cont = container(doc, parent, "HSI_filter_cap", "Filter cap and stop (printed)")
    b = Part(doc, cont, "fcap")
    zf = P.obj_back - P.obj_len                       # objective front face
    front = 1.2
    z0 = zf - P.filt_t - front
    sleeve = 5.0
    r_out = P.filt_d / 2 + 0.2 + 1.2
    body = b.cyl("body", "z", (0, P.y_axis, z0), r_out, front + P.filt_t + sleeve)
    cuts = [
        b.cyl("stop", "z", (0, P.y_axis, z0 - 0.1), P.pupil_d / 2, front + 0.2),
        b.cyl("filter_seat", "z", (0, P.y_axis, zf - P.filt_t - 0.05), P.filt_d / 2 + 0.2, P.filt_t + 0.06),
        b.cyl("sleeve", "z", (0, P.y_axis, zf), P.obj_od / 2 + 0.15, sleeve + 0.1),
    ]
    final = b.cut("Filter_cap", body, cuts)
    final.Label = "Filter cap"
    return _print("filter_cap", "Filter cap", cont, final, App.Rotation(),
                  "stop face on the bed; check the sleeve is a snug push fit on the lens")


ALL = [shell, lid, rotor, puck, bench_puck, obj_plate, slit_block, grat_plate, filter_cap]
