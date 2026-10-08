"""The project library pcb/lib: rig.kicad_sym and rig.pretty.

Both boards use only this library, so they open the same way on any machine
and in CI, whatever KiCad libraries are installed there. Most parts are
copied unchanged from KiCad's own libraries; the three sockets and leads
below are drawn here because KiCad has no part for them.
"""
import shutil

from sexpr import A, dump, top_symbol

# Copied from KiCad's symbol libraries: {library: [symbol, ...]}
SYMBOLS = {
    "Device": ["R", "C", "C_Polarized", "LED", "D_Schottky", "Polyfuse", "D_Zener"],
    "Connector": ["Barrel_Jack_Switch", "Screw_Terminal_01x02"],
    "Connector_Generic": ["Conn_01x03", "Conn_01x04"],
    "Jumper": ["SolderJumper_3_Bridged12"],
    "power": ["GND", "+3V3", "+5V", "PWR_FLAG"],
}
# KiCad draws a one-way TVS diode like a zener; give it its own name.
RENAMED = {"D_Zener": ("D_TVS_Unidirectional", "Unidirectional TVS diode")}

# Copied from KiCad's footprint libraries: {library: [footprint, ...]}
FOOTPRINTS = {
    "Resistor_SMD": ["R_0805_2012Metric"],
    "Capacitor_SMD": ["C_0805_2012Metric", "C_0805_2012Metric_Pad1.18x1.45mm_HandSolder"],
    "LED_SMD": ["LED_0805_2012Metric"],
    "Diode_SMD": ["D_SMA"],
    "Fuse": ["Fuse_1812_4532Metric"],
    "Capacitor_THT": ["CP_Radial_D6.3mm_P2.50mm"],
    "Connector_BarrelJack": ["BarrelJack_Horizontal"],
    "TerminalBlock_Phoenix": ["TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal"],
    "Connector_JST": ["JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", "JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical"],
    "Connector_PinHeader_2.54mm": ["PinHeader_1x03_P2.54mm_Vertical"],
    "Jumper": ["SolderJumper-3_P1.3mm_Bridged12_RoundedPad1.0x1.5mm_NumberLabels"],
    "MountingHole": ["MountingHole_3.2mm_M3", "MountingHole_2.2mm_M2"],
}

# ESP32-DevKitC V4 headers, pin 1 at the antenna end (Espressif's user guide)
DEVKITC_J2 = ["3V3", "EN", "SENSOR_VP", "SENSOR_VN", "IO34", "IO35", "IO32", "IO33", "IO25", "IO26",
              "IO27", "IO14", "IO12", "GND", "IO13", "SD2", "SD3", "CMD", "5V"]
DEVKITC_J3 = ["GND", "IO23", "IO22", "TXD0", "RXD0", "IO21", "GND", "IO19", "IO18", "IO5", "IO17",
              "IO16", "IO4", "IO0", "IO2", "IO15", "SD1", "SD0", "CLK"]

# BIGTREETECH TMC2209 V1.2/V1.3 stepstick, from BTT's manual. Pads 1-8 run
# down the left header (EN at the top), 9-16 up the right one (VM at the top).
TMC2209_LEFT = [("1", "EN", "input"), ("2", "MS1", "input"), ("3", "MS2", "input"),
                ("4", "PDN_UART", "bidirectional"), ("5", "PDN", "passive"), ("6", "CLK", "input"),
                ("7", "STEP", "input"), ("8", "DIR", "input")]
TMC2209_RIGHT = [("16", "VM", "power_in"), ("15", "GND", "power_in"), ("14", "A2", "passive"),
                 ("13", "A1", "passive"), ("12", "B1", "passive"), ("11", "B2", "passive"),
                 ("10", "VDD", "power_in"), ("9", "GND", "power_in")]


def _font(size=1.27):
    return [A("effects"), [A("font"), [A("size"), size, size]]]


def _sym_prop(name, value, x, y, angle=0, hide=False):
    p = [A("property"), name, value, [A("at"), x, y, angle], [A("show_name"), A("no")],
         [A("do_not_autoplace"), A("no")]]
    if hide:
        p.append([A("hide"), A("yes")])
    p.append(_font())
    return p


def _sym_pin(etype, x, y, angle, name, number, length=2.54):
    return [A("pin"), A(etype), A("line"), [A("at"), x, y, angle], [A("length"), length],
            [A("name"), name, _font()], [A("number"), number, _font()]]


def _symbol(name, ref, value, footprint, datasheet, description, keywords, rect, pins, y_ref, y_val):
    x0, y0, x1, y1 = rect
    return [A("symbol"), name,
            [A("pin_names"), [A("offset"), 1.016]],
            [A("exclude_from_sim"), A("no")], [A("in_bom"), A("yes")], [A("on_board"), A("yes")],
            [A("in_pos_files"), A("yes")], [A("duplicate_pin_numbers_are_jumpers"), A("no")],
            _sym_prop("Reference", ref, 0, y_ref),
            _sym_prop("Value", value, 0, y_val),
            _sym_prop("Footprint", footprint, 0, 0, hide=True),
            _sym_prop("Datasheet", datasheet, 0, 0, hide=True),
            _sym_prop("Description", description, 0, 0, hide=True),
            _sym_prop("ki_keywords", keywords, 0, 0, hide=True),
            [A("symbol"), name + "_0_1",
             [A("rectangle"), [A("start"), x0, y0], [A("end"), x1, y1],
              [A("stroke"), [A("width"), 0.254], [A("type"), A("default")]],
              [A("fill"), [A("type"), A("background")]]]],
            [A("symbol"), name + "_1_1"] + pins,
            [A("embedded_fonts"), A("no")]]


def custom_symbols():
    pins = []
    for k, n in enumerate(DEVKITC_J2, 1):
        t = {"3V3": "power_out", "5V": "power_out", "GND": "passive", "EN": "passive"}.get(
            n, "input" if n in ("IO34", "IO35", "SENSOR_VP", "SENSOR_VN") else "bidirectional")
        pins.append(_sym_pin(t, -15.24, 22.86 - (k - 1) * 2.54, 0, n, str(k)))
    for k, n in enumerate(DEVKITC_J3, 1):
        t = "passive" if n == "GND" else "bidirectional"
        pins.append(_sym_pin(t, 15.24, 22.86 - (k - 1) * 2.54, 180, n, str(19 + k)))
    esp = _symbol("ESP32-DevKitC", "U", "ESP32-DevKitC", "rig:ESP32-DevKitC_Socket",
                  "https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html",
                  "Espressif ESP32-DevKitC V4 (38 pins) in two 1x19 female headers 25.4 mm apart. "
                  "Pins 1-19 are header J2, pins 20-38 header J3.",
                  "ESP32 DevKitC socket", (-12.7, 25.4, 12.7, -25.4), pins, 27.94, -27.94)

    pins = [_sym_pin(t, -12.7, 8.89 - i * 2.54, 0, n, num) for i, (num, n, t) in enumerate(TMC2209_LEFT)]
    pins += [_sym_pin(t, 12.7, 8.89 - i * 2.54, 180, n, num) for i, (num, n, t) in enumerate(TMC2209_RIGHT)]
    tmc = _symbol("BTT_TMC2209", "U", "BTT TMC2209", "rig:StepStick_Socket_BTT_TMC2209",
                  "https://github.com/bigtreetech/BIGTREETECH-TMC2209-V1.2",
                  "BIGTREETECH TMC2209 V1.2/V1.3 stepper driver module in two 1x8 female headers (StepStick)",
                  "TMC2209 stepstick stepper driver socket", (-10.16, 11.43, 10.16, -11.43), pins, 13.97, -13.97)

    pins = [_sym_pin("power_in", 0, 10.16, 270, "VCC", "1"),
            _sym_pin("power_in", 0, -10.16, 90, "GND", "2"),
            _sym_pin("open_collector", 10.16, 0, 180, "OUT", "3")]
    hall = _symbol("A3144", "U", "A3144", "rig:A3144_UA_Leads",
                   "https://www.allegromicro.com/~/media/Files/Datasheets/A3141-2-3-4-Datasheet.ashx",
                   "Allegro A3144 unipolar hall switch, UA (SIP-3) package, open-collector output",
                   "hall switch magnet", (-7.62, 7.62, 7.62, -7.62), pins, 8.89, -8.89)
    return [esp, tmc, hall]


def build_symbol_lib(kicad_symbols, out_file):
    blocks = []
    for lib, names in SYMBOLS.items():
        text = (kicad_symbols / (lib + ".kicad_sym")).read_text(encoding="utf-8")
        for name in names:
            blk = top_symbol(text, name)
            if "(extends" in blk:
                raise ValueError("%s:%s is a derived symbol; copy its parent instead" % (lib, name))
            if name in RENAMED:
                new, descr = RENAMED[name]
                blk = blk.replace('(symbol "%s"' % name, '(symbol "%s"' % new, 1)
                blk = blk.replace('(symbol "%s_' % name, '(symbol "%s_' % new)
                blk = blk.replace('(property "Value" "%s"' % name, '(property "Value" "%s"' % new)
                blk = blk.replace('(property "Description" "Zener diode"', '(property "Description" "%s"' % descr)
            blocks.append("\t" + blk)
    blocks += [dump(s, 1) for s in custom_symbols()]
    out_file.parent.mkdir(parents=True, exist_ok=True)
    out_file.write_text('(kicad_symbol_lib\n\t(version 20251024)\n\t(generator "kicad_symbol_editor")\n'
                        '\t(generator_version "10.0")\n' + "\n".join(blocks) + "\n)\n", encoding="utf-8")


# --- footprints ---------------------------------------------------------------

def _fp_text(kind, text, x, y, layer, angle=0, size=1.0, thick=0.15, hide=False):
    head = [A("property"), kind, text] if kind in ("Reference", "Value") else [A("fp_text"), A("user"), text]
    node = head + [[A("at"), x, y, angle], [A("layer"), layer]]
    if hide:
        node.append([A("hide"), A("yes")])
    node.append([A("effects"), [A("font"), [A("size"), size, size], [A("thickness"), thick]]])
    return node


def _line(x0, y0, x1, y1, layer, width):
    return [A("fp_line"), [A("start"), x0, y0], [A("end"), x1, y1],
            [A("stroke"), [A("width"), width], [A("type"), A("solid")]], [A("layer"), layer]]


def _rect(x0, y0, x1, y1, layer, width):
    return [A("fp_rect"), [A("start"), x0, y0], [A("end"), x1, y1],
            [A("stroke"), [A("width"), width], [A("type"), A("solid")]], [A("fill"), A("no")], [A("layer"), layer]]


def _circle(x, y, r, layer, width):
    return [A("fp_circle"), [A("center"), x, y], [A("end"), x + r, y],
            [A("stroke"), [A("width"), width], [A("type"), A("solid")]], [A("fill"), A("no")], [A("layer"), layer]]


def _tht_pad(number, x, y, shape="circle", size=(1.7, 1.7), drill=1.0):
    return [A("pad"), number, A("thru_hole"), A(shape), [A("at"), x, y], [A("size"), size[0], size[1]],
            [A("drill"), drill], [A("layers"), "*.Cu", "*.Mask"], [A("remove_unused_layers"), A("no")]]


def _model(name, x=0.0):
    """One of KiCad's 3D models, shifted x mm along the footprint (for the renders)."""
    return [A("model"), "${KICAD10_3DMODEL_DIR}/" + name, [A("offset"), [A("xyz"), x, 0, 0]],
            [A("scale"), [A("xyz"), 1, 1, 1]], [A("rotate"), [A("xyz"), 0, 0, 0]]]


def _footprint(name, descr, tags, items, ref_at, val_at, models=()):
    return [A("footprint"), name, [A("version"), 20260206], [A("generator"), "build_boards.py"],
            [A("layer"), "F.Cu"], [A("descr"), descr], [A("tags"), tags],
            _fp_text("Reference", "REF**", ref_at[0], ref_at[1], "F.SilkS", ref_at[2] if len(ref_at) > 2 else 0),
            _fp_text("Value", name, val_at[0], val_at[1], "F.Fab", val_at[2] if len(val_at) > 2 else 0),
            [A("attr"), A("through_hole")]] + items + [[A("embedded_fonts"), A("no")]] + list(models)


SOCKET = "Connector_PinSocket_2.54mm.3dshapes/PinSocket_1x%02d_P2.54mm_Vertical.step"


def devkitc_socket():
    """Two 1x19 female headers 25.4 mm apart. Header J2 (pads 1-19) on the left
    and J3 (pads 20-38) on the right with the antenna end at the top, as in
    Espressif's pin layout. Parts may sit between the rows: the DevKit rides
    about 11 mm up on the sockets, so the courtyard covers the headers only."""
    L = 18 * 2.54
    it = []
    for k in range(19):
        it.append(_tht_pad(str(k + 1), 0, k * 2.54, "rect" if k == 0 else "circle"))
        it.append(_tht_pad(str(k + 20), 25.4, k * 2.54, "rect" if k == 0 else "circle"))
    for x in (0, 25.4):
        it.append(_rect(x - 1.33, -1.33, x + 1.33, L + 1.33, "F.SilkS", 0.12))
        it.append(_rect(x - 1.8, -1.8, x + 1.8, L + 1.8, "F.CrtYd", 0.05))
        it.append(_rect(x - 1.27, -1.27, x + 1.27, L + 1.27, "F.Fab", 0.1))
    # pin-1 marks, outside each header
    it.append(_line(-1.9, -1.33, -1.9, 1.0, "F.SilkS", 0.12))
    it.append(_line(27.3, -1.33, 27.3, 1.0, "F.SilkS", 0.12))
    # the DevKitC's own board (48.2 x 27.9 mm) and its micro-USB socket
    it.append(_rect(-1.25, L / 2 - 24.1, 26.65, L / 2 + 24.1, "F.Fab", 0.1))
    it.append(_rect(9.0, L + 0.5, 16.4, L + 3.4, "F.Fab", 0.1))
    it.append(_fp_text("user", "USB", 12.7, L - 0.5, "F.SilkS"))
    it.append(_fp_text("user", "ESP32-DevKitC 38-pin", 12.7, 13.0, "F.SilkS", angle=90))
    it.append(_fp_text("user", "antenna end", 12.7, 1.5, "F.Fab"))
    it.append(_fp_text("user", "${REFERENCE}", 12.7, L / 2, "F.Fab", angle=90))
    return _footprint("ESP32-DevKitC_Socket",
                      "Socket for an Espressif ESP32-DevKitC V4 (38 pins): two 1x19 2.54 mm female headers, "
                      "rows 25.4 mm apart. Pads 1-19 are header J2 (3V3 to 5V), pads 20-38 header J3 (GND to CLK).",
                      "ESP32 DevKitC socket female header", it, (12.7, -3.0), (12.7, L + 5.0),
                      [_model(SOCKET % 19), _model(SOCKET % 19, 25.4)])


def stepstick_socket():
    """BTT TMC2209 in two 1x8 female headers, 12.7 mm apart (StepStick).
    The module's DIAG pin hangs over the board with no socket under it."""
    it = []
    for k in range(8):
        it.append(_tht_pad(str(k + 1), 0, k * 2.54, "rect" if k == 0 else "circle"))
        it.append(_tht_pad(str(16 - k), 12.7, k * 2.54))
    it.append(_rect(-1.4, -1.4, 14.1, 19.18, "F.SilkS", 0.12))
    it.append(_line(-2.0, -0.8, -2.0, 0.8, "F.SilkS", 0.12))
    it.append(_fp_text("user", "EN", 2.3, 0, "F.SilkS"))
    it.append(_fp_text("user", "VM", 10.4, 0, "F.SilkS"))
    it.append(_fp_text("user", "DIR", 2.7, 17.78, "F.SilkS"))
    it.append(_fp_text("user", "GND", 9.9, 17.78, "F.SilkS"))
    it.append(_fp_text("user", "TMC2209", 6.35, 9.4, "F.SilkS", angle=90))
    it.append(_rect(-1.27, -1.27, 13.97, 19.05, "F.Fab", 0.1))
    it.append(_circle(5.08, 0, 0.5, "F.Fab", 0.1))
    it.append(_fp_text("user", "DIAG", 5.08, 1.5, "F.Fab", size=0.6, thick=0.1))
    it.append(_rect(-1.52, -1.52, 14.22, 19.3, "F.CrtYd", 0.05))
    it.append(_fp_text("user", "${REFERENCE}", 6.35, 12.0, "F.Fab"))
    return _footprint("StepStick_Socket_BTT_TMC2209",
                      "BIGTREETECH TMC2209 V1.2/V1.3 in two 1x8 2.54 mm female headers 12.7 mm apart. "
                      "Pads 1-8: EN MS1 MS2 PDN_UART PDN CLK STEP DIR; 9-16: GND VDD B2 B1 A1 A2 GND VM.",
                      "TMC2209 stepstick socket", it, (6.35, -2.6), (6.35, 20.5),
                      [_model(SOCKET % 8), _model(SOCKET % 8, 12.7)])


def a3144_leads():
    """Holes for an A3144's three leads at 1.27 mm. The sensor sits behind the
    board (in the head's wall) with its printed face toward the magnet, so
    seen from this side its pins run right to left: pad 1 (VCC) on the right."""
    it = [_tht_pad("1", 1.27, 0, "rect", (1.05, 1.7), 0.75),
          _tht_pad("2", 0, 0, "oval", (1.05, 1.7), 0.75),
          _tht_pad("3", -1.27, 0, "oval", (1.05, 1.7), 0.75),
          _rect(-2.05, -2.65, 2.05, 0.65, "F.Fab", 0.1),
          _fp_text("user", "body behind board", 0, -3.3, "F.Fab", size=0.5, thick=0.08),
          _rect(-2.0, -1.1, 2.0, 1.1, "F.CrtYd", 0.05),
          _fp_text("user", "${REFERENCE}", 0, -1.4, "F.Fab", size=0.5, thick=0.08)]
    return _footprint("A3144_UA_Leads",
                      "Allegro A3144 in the UA (SIP-3) package, mounted behind the board with its leads through it. "
                      "Pad 1 VCC, 2 GND, 3 OUT, seen from the solder side.",
                      "hall A3144 UA SIP-3", it, (0, 2.2), (0, 3.2))


def npth_hole():
    """A plain 1.5 mm hole, for tying the hall cable down with thread or a thin cable tie."""
    it = [[A("pad"), "", A("np_thru_hole"), A("circle"), [A("at"), 0, 0], [A("size"), 1.5, 1.5], [A("drill"), 1.5],
           [A("layers"), "*.Cu", "*.Mask"]],
          _circle(0, 0, 1.0, "F.CrtYd", 0.05),
          _circle(0, 0, 0.75, "F.Fab", 0.1)]
    fp = _footprint("Hole_NPTH_1.5mm", "Unplated 1.5 mm hole for cable strain relief", "hole strain relief", it,
                    (0, -2.0), (0, 2.0))
    fp[fp.index([A("attr"), A("through_hole")])] = [A("attr"), A("exclude_from_pos_files"), A("exclude_from_bom")]
    return fp


def wire_pads():
    """Three holes 2.54 mm apart for soldering a cable straight in (or a pin header)."""
    it = [_tht_pad("1", 0, 0, "rect"), _tht_pad("2", 2.54, 0), _tht_pad("3", 5.08, 0),
          _rect(-1.27, -1.27, 6.35, 1.27, "F.Fab", 0.1),
          _rect(-1.5, -1.5, 6.58, 1.5, "F.CrtYd", 0.05),
          _fp_text("user", "${REFERENCE}", 2.54, 0, "F.Fab", size=0.8, thick=0.12)]
    return _footprint("Wire_Pads_1x03_P2.54mm", "Three 1.0 mm holes on 2.54 mm for a soldered-in cable",
                      "cable wire solder pads", it, (2.54, -2.4), (2.54, 2.4))


def build_footprint_lib(kicad_footprints, out_dir):
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True)
    for lib, names in FOOTPRINTS.items():
        for name in names:
            # copyfile, not copy: KiCad's installed files are read-only
            shutil.copyfile(kicad_footprints / (lib + ".pretty") / (name + ".kicad_mod"),
                            out_dir / (name + ".kicad_mod"))
    for fp in (devkitc_socket(), stepstick_socket(), a3144_leads(), npth_hole(), wire_pads()):
        (out_dir / (fp[1] + ".kicad_mod")).write_text(dump(fp) + "\n", encoding="utf-8")
