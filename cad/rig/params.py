"""Every number the rig is built from.

`rows()` returns the contents of the Params spreadsheet: one row per value,
with an alias, the value (or a formula starting with "="), a unit, what it is,
and where the number came from. Formulas reference other aliases, so derived
positions update when an input changes.

Source tags used in the last column:
  model     optics/spectrograph_model.py, layout C (read live via optics_link)
  ds        vendor datasheet or drawing
  listing   vendor product page
  measured  measured on the part in hand, or from TheRobotStudio's SO-101 meshes
  est       estimate: no published number; check the part when it arrives
  pcb       the KiCad boards in pcb/ (pcb/README.md, pcb/tools/boards.py)
  design    a choice made for this rig
"""

import os

from .optics_link import read_layout

# Scan stepper options; pick one with RIG_MOTOR (default 8HS11). The housing
# is the same for both: two long M3 bolts from the +X face clamp the lid on,
# threading into the 17HM08's lower holes, or into nuts on the lid for the
# NEMA 8. Rows: alias, value, what, source.
MOTORS = {
    "17HM08": {
        "label": "Stepper 17HM08-1204S (NEMA 17 pancake, 0.9 deg, 150 g)",
        "mass_g": 150.0,
        "bolts_into_motor": True,
        "relief": (1.5, 4.5, 7.2, 3.5),     # lead strain relief: from rear, length, width, height
        "rows": [
            ("mot_w", 42.3, "body square", "ds"),
            ("mot_len", 21.0, "body length (vendor STEP: 20)", "listing"),
            ("mot_chamfer", 4.0, "body corner chamfer", "ds"),
            ("mot_boss_d", 22.0, "pilot boss diameter", "ds"),
            ("mot_boss_h", 2.0, "pilot boss height", "ds"),
            ("mot_shaft_d", 5.0, "shaft diameter (round, no flat)", "ds"),
            ("mot_shaft_len", 17.0, "shaft length past the mounting face", "ds"),
            ("mot_hole_pitch", 31.0, "screw spacing, M3", "ds"),
            ("mot_hole_depth", 3.0, "M3 thread depth", "ds"),
            ("mot_screw_clear", "=m3_clear", "screw clearance in the lid", "design"),
            ("mot_head_d", "=m3_head", "screw head counterbore in the lid", "design"),
            ("mot_head_h", 3.0, "counterbore depth: M3 x 6 heads flush inside", "design"),
            ("lid_motor_t", 6.0, "lid thickness where the stepper bolts on", "design"),
            ("lid_pad_half", "=mot_w / 2 + 0.5", "half-size of the lid's motor pad", "design"),
        ],
    },
    "8HS11": {
        "label": "Stepper 8HS11-0204S (NEMA 8, 1.8 deg, 60 g)",
        "mass_g": 60.0,
        "bolts_into_motor": False,
        "relief": (1.0, 3.0, 5.0, 2.0),
        "rows": [
            ("mot_w", 20.3, "body square", "listing"),
            ("mot_len", 28.0, "body length", "listing"),
            ("mot_chamfer", 1.5, "body corner chamfer", "est"),
            ("mot_boss_d", 15.0, "pilot boss diameter", "est"),
            ("mot_boss_h", 1.5, "pilot boss height", "est"),
            ("mot_shaft_d", 4.0, "shaft diameter (7 mm D flat at the end)", "listing"),
            ("mot_shaft_len", 10.0, "shaft length past the mounting face", "listing"),
            ("mot_hole_pitch", 16.0, "screw spacing, M2", "listing"),
            ("mot_hole_depth", 2.5, "M2 thread depth", "est"),
            ("mot_screw_clear", 2.4, "screw clearance in the lid", "design"),
            ("mot_head_d", 4.2, "screw head counterbore in the lid", "design"),
            ("mot_head_h", 2.2, "counterbore depth: M2 x 4 heads flush inside", "design"),
            ("lid_motor_t", 4.0, "lid thickness where the stepper bolts on", "design"),
            ("lid_pad_half", "=bolt_pitch / 2 + 4",
             "half-size of the lid's motor pad (the long bolts' nuts sit on it)", "design"),
        ],
    },
}
MOTOR = os.environ.get("RIG_MOTOR", "8HS11")
if MOTOR not in MOTORS:
    raise ValueError("RIG_MOTOR must be one of %s, not %r" % (", ".join(MOTORS), MOTOR))


def rows():
    m = read_layout("C")
    R = []

    def sec(title):
        R.append((None, title))

    def p(alias, value, unit, what, src):
        R.append((alias, value, unit, what, src))

    sec("Optical model (layout C of optics/spectrograph_model.py)")
    p("f_obj", m["f_obj"], "mm", "objective focal length", "model")
    p("fno_obj", m["fno_obj"], "", "objective f-number (4 mm printed stop)", "model")
    p("scene_dist", m["scene_dist"], "mm", "objective to scene", "model")
    p("slit_len", m["slit_len"], "mm", "slit length (along X)", "model")
    p("slit_width", m["slit_width"], "mm", "slit width", "model")
    p("f_field", m["f_field"], "mm", "field lens focal length in the model (bought lens is 15 mm)", "model")
    p("f_coll", m["f_coll"], "mm", "collimator focal length", "model")
    p("coll_to_grating", m["coll_to_grating"], "mm", "collimator to grating", "model")
    p("grating_to_cam", m["grating_to_cam"], "mm", "grating to camera lens (along the tilted axis)", "model")
    p("f_cam", m["f_cam"], "mm", "camera lens focal length", "model")
    p("lines_per_mm", m["lines_per_mm"], "1/mm", "grating groove density", "model")
    p("wl_center_um", m["center_wl_um"], "um", "wavelength on the camera axis", "model")
    p("sensor_w", m["sensor_w"], "mm", "IMX219 active area, dispersion direction", "model")
    p("sensor_h", m["sensor_h"], "mm", "IMX219 active area, along the slit", "model")
    p("scan_half", 12, "deg", "half-angle of the scan fan (63 mm patch at 150 mm)", "design")

    sec("Derived optics")
    p("s_img", "=1 / (1 / f_obj - 1 / scene_dist)", "mm", "objective to slit (thin lens)", "model")
    p("theta", "=asin(wl_center_um * lines_per_mm / 1000) / 1 deg", "deg",
      "1st-order angle at wl_center = camera tilt", "model")
    p("pupil_d", "=f_obj / fno_obj", "mm", "entrance pupil = stop diameter", "model")
    p("half_line", "=0.5 * slit_len * scene_dist / s_img", "mm", "half length of the scan line on the scene", "model")
    p("field_half", "=atan(half_line / scene_dist) / 1 deg", "deg", "field half-angle along the slit", "model")

    sec("Head layout (head frame: origin at the wrist-roll horn face, +Z away from the wrist, +Y toward the scene)")
    p("y_shaft", -2.3, "mm",
      "mirror shaft Y; the long lid bolts go either side of it, clear of the dovetail slot and the -Y face", "design")
    p("y_axis", "=y_shaft + mirror_e / sqrt(2)", "mm", "optical axis offset in Y from the roll axis", "design")
    p("z_shell", "=puck_top + 0.2", "mm", "underside of the housing", "design")
    p("bolt_pitch", 31.0, "mm", "spacing of the two long lid bolts (the NEMA 17 hole pattern)", "design")
    p("z_shaft", "=(z_shell + z_floor) / 2 + bolt_pitch / 2", "mm",
      "mirror shaft height: puts the two long lid bolts mid-floor", "design")
    p("mirror_e", "=2.5 + 1.6 + mirror_t", "mm",
      "mirror face offset from the shaft axis (1.6 mm of hub under the glass on a 5 mm shaft)", "design")
    p("mirror_to_obj", 21, "mm", "mirror centre to objective principal plane", "design")
    p("mirror_scan", 0, "deg", "scan mirror angle away from its 45 deg rest (scan is +/- scan_half / 2)", "design")
    p("mirror_home", -40, "deg", "mirror angle where the magnet faces the hall sensor", "design")
    p("mag_r", 8.0, "mm", "magnet radius from the shaft axis", "design")
    p("z_mirror", "=z_shaft + mirror_e / sqrt(2)", "mm", "mirror centre on the optical axis", "design")
    p("hall_y", "=y_shaft - mag_r * cos(45 + mirror_home)", "mm", "hall sensor Y (magnet at home)", "design")
    p("hall_z", "=z_shaft - mag_r * sin(45 + mirror_home)", "mm", "hall sensor Z (magnet at home)", "design")
    p("hall_lead_dz", 2.2, "mm", "A3144 leads, bent 90 deg under the body: their run out through the wall, below "
      "hall_z (the breakout's lead holes follow it)", "design")
    p("z_obj", "=z_mirror + mirror_to_obj", "mm", "objective principal plane", "model")
    p("z_slit", "=z_obj + s_img", "mm", "slit plane", "model")
    p("z_coll", "=z_slit + f_coll", "mm", "collimator principal plane", "model")
    p("z_grat", "=z_coll + coll_to_grating", "mm", "grating plane in the model (not used: see z_film)", "model")
    p("obj_back", "=z_obj + f_obj - obj_bfl", "mm", "objective rear end (image side)", "design")
    p("z_obj_plate", "=obj_back + h12_lift", "mm", "objective carrier front face = M12 holder mounting face", "design")
    p("slit_t", "=coll_bfl - h12_lift", "mm", "slit block thickness (collimator holder bolts to its back)", "design")
    p("grat_gap", 1.0, "mm", "collimator front to grating carrier", "design")
    p("z_film", "=z_slit + coll_bfl + coll_len + grat_gap + carrier_t", "mm",
      "grating film as built: as close to the collimator as the parts allow", "design")
    p("cam_front", "=cam_len - f_cam + cam_bfl", "mm", "camera lens front ahead of its principal plane", "est")
    p("g2c", "=cam_front + cam_od / 2 * tan(theta) + 1", "mm",
      "grating to camera lens as built (the model's 3 mm is inside the lens barrel)", "design")
    p("z_cam", "=z_film + g2c * cos(theta)", "mm", "camera lens principal plane, Z", "design")
    p("y_cam", "=y_axis - g2c * sin(theta)", "mm", "camera lens principal plane, Y", "design")
    p("z_sensor", "=z_cam + f_cam * cos(theta)", "mm", "sensor centre, Z", "design")
    p("y_sensor", "=y_cam - f_cam * sin(theta)", "mm", "sensor centre, Y", "design")
    p("standoff_h", 3.0, "mm", "camera board standoffs above the end wall", "design")
    p("z_pose", "=z_mirror + 22", "mm", "pose camera lens centre, Z", "design")
    p("win", 16.0, "mm", "scan window, square", "design")
    p("hood_h", 4.0, "mm", "window hood height", "design")
    p("z_lid_screw", "=z_film - 8", "mm", "lid screws on the side walls, Z", "design")

    sec("Bought parts")
    # M12 lenses: OD of the front barrel, overall length, thread length, and the
    # mechanical back focal length (rear of the barrel to the image at infinity).
    p("obj_od", 14.0, "mm", "CIL161 objective: front barrel OD", "est")
    p("obj_len", 18.5, "mm", "CIL161: overall length (17-20)", "est")
    p("obj_thread", 9.0, "mm", "CIL161: M12 thread length", "est")
    p("obj_bfl", 5.0, "mm", "CIL161: rear of barrel to image at infinity (4-6)", "est")
    p("coll_od", 17.0, "mm", "LN016 collimator: front barrel OD", "listing")
    p("coll_len", 21.0, "mm", "LN016: overall length", "listing")
    p("coll_thread", 9.0, "mm", "LN016: M12 thread length", "est")
    p("coll_bfl", 6.0, "mm", "LN016: rear of barrel to image at infinity", "est")
    p("cam_od", 15.0, "mm", "CIL122 camera lens: front barrel OD (14-16)", "est")
    p("cam_len", 19.0, "mm", "CIL122: overall length (17-21)", "est")
    p("cam_thread", 9.0, "mm", "CIL122: M12 thread length", "est")
    p("cam_bfl", 5.0, "mm", "CIL122: rear of barrel to image at infinity (4-6)", "est")
    p("h12_h", 14.0, "mm", "M12 holder (uxcell B00R1J42T8, plastic) height as bought", "listing")
    p("h12_ear", 24.0, "mm", "M12 holder length across the screw ears", "listing")
    p("h12_w", 17.0, "mm", "M12 holder base width", "listing")
    p("h12_d", 15.0, "mm", "M12 holder round body above the base", "est")
    p("h12_pitch", 20.0, "mm", "M12 holder screw spacing (plain holes)", "listing")
    p("h12_ear_t", 2.0, "mm", "M12 holder base and ear thickness", "est")
    p("h12_lift", 1.0, "mm", "lens rear end above its holder's base with the lens screwed fully in", "design")
    p("h12_cut_obj", "=min(obj_thread + h12_lift; h12_h)", "mm",
      "objective holder height after cutting it down from the lens end", "design")
    p("h12_cut_coll", "=min(coll_thread + h12_lift; h12_h)", "mm",
      "collimator holder height after cutting it down from the lens end", "design")
    p("brd_w", 36.0, "mm", "B0152 camera board, along X (slit)", "listing")
    p("brd_h", 36.0, "mm", "B0152 board, along the dispersion", "listing")
    p("brd_t", 1.6, "mm", "B0152 PCB thickness", "est")
    p("brd_hole", 29.0, "mm", "B0152 corner hole spacing along X (measure before printing)", "est")
    p("brd_hole_y", "=brd_hole", "mm", "B0152 corner hole spacing along the dispersion, if not square", "est")
    p("brd_holder_h", 10.0, "mm", "B0152 M12 holder height above the PCB", "est")
    p("brd_holder_w", 14.0, "mm", "B0152 M12 holder body width", "est")
    p("sensor_above_pcb", 1.0, "mm", "sensor surface above the PCB", "est")
    part_no = MOTORS[MOTOR]["label"].split()[1]
    for alias, value, what, src in MOTORS[MOTOR]["rows"]:
        p(alias, value, "mm", "%s: %s" % (part_no, what), src)
    p("mirror_l", 20.0, "mm", "scan mirror, cut from a RUEHALF 100 x 100 mm front-surface sheet: along the shaft",
      "design")
    p("mirror_w", 20.0, "mm", "scan mirror, across the shaft (cut to this or a little under)", "design")
    p("mirror_t", 3.0, "mm", "scan mirror thickness", "listing")
    p("filt_d", 12.5, "mm", "long-pass disc (Edmund #54-652, SCHOTT GG-495) diameter", "listing")
    p("filt_tol", 0.38, "mm", "long-pass disc diameter tolerance (+/-)", "listing")
    p("filt_t", 3.0, "mm", "long-pass disc thickness (+/-0.2)", "listing")
    p("fl_d", 12.7, "mm", "field lens (Edmund #49-840, PCX f = 15) diameter", "ds")
    p("fl_ct", 5.25, "mm", "field lens centre thickness (+/-0.1)", "ds")
    p("fl_et", 1.94, "mm", "field lens edge thickness", "ds")
    p("fl_r", 7.75, "mm", "field lens convex radius", "ds")
    p("grat_w", 15.0, "mm", "grating film piece, square", "design")
    p("grat_t", 0.25, "mm", "grating film thickness", "est")
    p("blade_l", 20.0, "mm", "slit blade piece length (along the slit)", "design")
    p("blade_w", 9.0, "mm", "slit blade piece width (9 mm snap-off blade)", "listing")
    p("blade_t", 0.38, "mm", "slit blade thickness", "listing")
    p("pi_w", 25.0, "mm", "Pi camera v2 board width (pose camera)", "ds")
    p("pi_h", 23.862, "mm", "Pi camera v2 board height", "ds")
    p("horn_z", 3.0, "mm", "horn face above the gripper_link origin (from the SO-101 follower mesh)", "measured")
    p("horn_d", 20.0, "mm", "STS3215 horn disc diameter", "measured")
    p("horn_t", 2.5, "mm", "STS3215 horn disc thickness", "measured")
    p("horn_sq", 9.9, "mm", "horn screw square (4x M3, 14 mm bolt circle)", "measured")
    p("horn_boss_d", 5.4, "mm", "horn centre screw head diameter", "measured")
    p("horn_boss_h", 1.5, "mm", "horn centre screw head proud of the horn face", "measured")
    p("mag_d", 5.0, "mm", "home magnet diameter (NdFeB disc, magnetized through its thickness)", "measured")
    p("mag_t", 1.6, "mm", "home magnet thickness; the clamp pocket is cut to match", "measured")

    sec("Boards (KiCad, pcb/)")
    p("ctl_w", 95.0, "mm", "scan-mirror controller, along its USB edge", "pcb")
    p("ctl_h", 66.0, "mm", "scan-mirror controller, from the USB edge to the hall-connector edge", "pcb")
    p("ctl_t", 1.6, "mm", "controller board thickness", "pcb")
    p("ctl_r", 2.0, "mm", "controller board corner radius", "pcb")
    p("ctl_hole_in", 3.5, "mm", "controller's four M3 holes, in from each edge", "pcb")
    p("ctl_leads", 3.0, "mm", "through-hole leads under the controller, longest", "pcb")
    p("hb_w", 14.0, "mm", "hall breakout, along Y on the head", "pcb")
    p("hb_h", 12.4, "mm", "hall breakout, along Z on the head", "pcb")
    p("hb_t", 0.8, "mm", "hall breakout thickness (ordered 0.8 mm)", "pcb")
    p("hb_lead_x", 7.0, "mm", "hall breakout: the A3144's middle lead hole, from the board's -Y edge", "pcb")
    p("hb_lead_y", 4.6, "mm", "hall breakout: the A3144's lead holes, down from the board's +Z edge", "pcb")

    sec("Printed parts")
    p("wall", 2.5, "mm", "housing wall thickness", "design")
    p("fit", 0.25, "mm", "clearance for sliding fits, per side", "design")
    p("carrier_t", 3.0, "mm", "optic carrier plate thickness", "design")
    p("rib_h", 1.6, "mm", "slot rib height", "design")
    p("rib_w", 1.6, "mm", "slot rib width", "design")
    p("x_in", 13.5, "mm", "housing interior half-width in X", "design")
    p("y_in_pos", 13.0, "mm", "interior extent toward the scene (+Y) from the axis", "design")
    p("y_in_neg", 20.7, "mm", "interior extent away from the scene (-Y) from the axis", "design")
    p("lid_t", 2.5, "mm", "lid plate thickness", "design")
    p("hub_len", "=max(mot_shaft_len - lid_motor_t; 9)", "mm", "mirror clamp hub length along the shaft", "design")
    p("pinch_dx", "=max(3.6; (mot_shaft_len - lid_motor_t) / 2)", "mm",
      "mirror clamp pinch screw, from the hub's motor end (over the shaft, nut trap inside the hub)", "design")
    p("pad_x1", 12.5, "mm", "mirror clamp: +X end of the pad and the magnet tab (takes mirrors 20 to 25 mm long)",
      "design")
    p("fl_ap_r", 5.0, "mm", "slit block aperture behind the field lens, radius", "design")
    p("fl_seat", "=fl_ct + 0.1 - fl_r + sqrt(fl_r * fl_r - fl_ap_r * fl_ap_r)", "mm",
      "field lens pocket depth: convex face on the aperture edge, flat face level with the blades at +0.1 mm CT",
      "design")
    p("z_floor", 12.0, "mm", "top of the housing floor (interior starts here)", "design")
    p("sweep_r", 13.2, "mm", "clearance radius around the shaft for a full mirror turn", "design")
    p("puck_d", 29.0, "mm", "wrist puck diameter (clears the wrist bracket's lug at r = 15 mm)", "design")
    p("puck_top", 5.0, "mm", "puck top face height", "design")
    p("puck_rim", 1.0, "mm", "puck rim depth around the horn disc", "design")
    p("rail_h", 4.0, "mm", "dovetail rail height", "design")
    p("rail_w", 17.0, "mm", "dovetail rail width at its root", "design")
    p("rail_flank", 25.0, "deg", "dovetail flank angle from vertical", "design")
    p("m3_clear", 3.3, "mm", "M3 clearance hole", "design")
    p("m3_head", 5.8, "mm", "M3 socket head counterbore", "design")
    p("m2_pilot", 1.7, "mm", "M2 self-tapping pilot hole", "design")
    p("insert_d", 4.5, "mm", "hole for M3 heat-set inserts (Pofsnnx: knurl 5.0, lead-in 4.2)", "design")
    p("insert_l", 6.0, "mm", "M3 heat-set insert length", "listing")
    p("insert_m2_d", 2.85, "mm", "hole for M2 heat-set inserts (Pofsnnx: knurl 3.0, lead-in 2.7)", "design")
    p("insert_m2_l", 4.0, "mm", "M2 heat-set insert length", "listing")

    sec("Controller tray (printed; base_link frame: it sits on the table behind the SO-101's base)")
    p("table_z", -2.4, "mm", "table top: the underside of the SO-101's base", "measured")
    p("tray_x", -35.0, "mm", "tray's edge nearest the arm (the back of the Waveshare plate is at x = -31)", "design")
    p("tray_floor", 3.0, "mm", "tray floor thickness", "design")
    p("tray_wall", 2.0, "mm", "tray wall thickness", "design")
    p("tray_gap", 0.5, "mm", "board edge to the tray wall", "design")
    p("tray_post", "=ctl_leads + 3", "mm", "standoff height: the board's underside above the floor", "design")
    p("tray_post_d", 8.6, "mm", "standoff diameter around its M3 heat-set insert (merges with the walls)", "design")
    p("tray_strip", 18.0, "mm", "strip along the board's USB edge with the two table-screw slots", "design")
    p("tray_slot_w", 6.6, "mm", "table-screw slot width (1/4-20 or M6 screws)", "design")
    p("tray_slot_travel", 25.4, "mm",
      "table-screw slot travel: a full pitch of a 1 in or 25 mm hole grid, so both slots always find a hole",
      "design")
    p("tray_slot_y", 25.0, "mm", "table-screw slot centres, either side of the tray's middle", "design")

    sec("Arm pose (degrees from the URDF zero: upper arm up, forearm level). "
        "Shown: scanning a table 150 mm below the objective")
    p("q_shoulder_pan", 0, "deg", "base rotation", "design")
    p("q_shoulder_lift", -50, "deg", "shoulder", "design")
    p("q_elbow_flex", 90, "deg", "elbow", "design")
    p("q_wrist_flex", -40, "deg", "wrist pitch", "design")
    p("q_wrist_roll", -90, "deg", "wrist roll: -90 puts the scan window facing down", "design")
    p("q_gripper", 0, "deg", "stock gripper jaw (hidden)", "design")
    return R


def fill_sheet(sheet):
    """Write rows() into a Spreadsheet::Sheet. Column A label, B value, C unit, D note, E source."""
    def text(cell, t):
        sheet.set(cell, "'" + t)

    text("A1", "Name")
    text("B1", "Value")
    text("C1", "Unit")
    text("D1", "What")
    text("E1", "Source")
    sheet.setStyle("A1:E1", "bold")
    r = 2
    for row in rows():
        if row[0] is None:
            r += 1
            text("A%d" % r, row[1])
            sheet.setStyle("A%d" % r, "bold")
            r += 1
            continue
        alias, value, unit, what, src = row
        text("A%d" % r, alias)
        sheet.set("B%d" % r, value if isinstance(value, str) else repr(float(value)))
        sheet.setAlias("B%d" % r, alias)
        text("C%d" % r, unit)
        text("D%d" % r, what)
        text("E%d" % r, src)
        r += 1
    sheet.setColumnWidth("A", 130)
    sheet.setColumnWidth("D", 420)
    return sheet
