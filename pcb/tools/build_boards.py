#!/usr/bin/env python3
"""Generate the rig's KiCad boards: the project library, both schematics and
both layouts, from the definitions in boards.py.

Run it with the Python that ships with KiCad 10, which has the pcbnew module:

    Windows:  "%LOCALAPPDATA%\\Programs\\KiCad\\10.0\\bin\\python.exe" pcb\\tools\\build_boards.py
    Linux:    python3 pcb/tools/build_boards.py      (with KiCad 10 installed)

It overwrites pcb/lib and each board's .kicad_sch, .kicad_pcb and .kicad_pro.
After editing a board by hand in KiCad, don't rerun it, or the edits are lost.
pcb/tools/fab.py then runs ERC and DRC and writes the JLCPCB files.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import boards  # noqa: E402
from library import build_footprint_lib, build_symbol_lib  # noqa: E402
from schematic import Schematic  # noqa: E402
from sexpr import find, find_all, parse  # noqa: E402

PCB = HERE.parent
LIB = PCB / "lib"
SYMLIB = LIB / "rig.kicad_sym"
FPLIB = LIB / "rig.pretty"
OX, OY = 100.0, 100.0  # where the board's top-left corner sits on KiCad's page


def kicad_paths():
    exe = Path(sys.executable).resolve()
    if os.name == "nt":
        share = exe.parent.parent / "share" / "kicad"
        cli = exe.parent / "kicad-cli.exe"
    else:
        share = Path(os.environ.get("KICAD_SHARE", "/usr/share/kicad"))
        cli = Path(shutil.which("kicad-cli") or "kicad-cli")
    sym = Path(os.environ.get("KICAD10_SYMBOL_DIR", share / "symbols"))
    fp = Path(os.environ.get("KICAD10_FOOTPRINT_DIR", share / "footprints"))
    return cli, sym, fp


CLI, KICAD_SYM, KICAD_FP = kicad_paths()


def run(*args):
    r = subprocess.run([str(CLI)] + [str(a) for a in args], capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout, r.stderr)
        raise SystemExit("kicad-cli %s failed" % " ".join(map(str, args[:2])))
    return r.stdout


# --- project files -------------------------------------------------------------

def write_lib_tables(d):
    (d / "sym-lib-table").write_text(
        '(sym_lib_table\n\t(version 7)\n\t(lib (name "rig")(type "KiCad")(uri "${KIPRJMOD}/../lib/rig.kicad_sym")'
        '(options "")(descr "Parts for the line-scan rig boards"))\n)\n', encoding="utf-8")
    (d / "fp-lib-table").write_text(
        '(fp_lib_table\n\t(version 7)\n\t(lib (name "rig")(type "KiCad")(uri "${KIPRJMOD}/../lib/rig.pretty")'
        '(options "")(descr "Footprints for the line-scan rig boards"))\n)\n', encoding="utf-8")


def write_project(d, name, root_uuid):
    """Design rules sit inside JLCPCB's standard 2-layer limits with room to spare."""
    rules = {"allow_blind_buried_vias": False, "allow_microvias": False, "max_error": 0.005,
             "min_clearance": 0.15, "min_connection": 0.0, "min_copper_edge_clearance": 0.5,
             "min_groove_width": 0.0, "min_hole_clearance": 0.25, "min_hole_to_hole": 0.25,
             "min_microvia_diameter": 0.2, "min_microvia_drill": 0.1, "min_resolved_spokes": 1,
             "min_silk_clearance": 0.0, "min_text_height": 0.8, "min_text_thickness": 0.12,
             "min_through_hole_diameter": 0.3, "min_track_width": 0.15, "min_via_annular_width": 0.13,
             "min_via_diameter": 0.5, "solder_mask_to_copper_clearance": 0.0, "use_height_for_length_calcs": True}
    default = {"name": "Default", "clearance": 0.2, "track_width": 0.25, "via_diameter": 0.6, "via_drill": 0.3,
               "microvia_diameter": 0.3, "microvia_drill": 0.1, "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25,
               "diff_pair_width": 0.2, "bus_width": 12, "line_style": 0, "wire_width": 6,
               "pcb_color": "rgba(0, 0, 0, 0.000)", "schematic_color": "rgba(0, 0, 0, 0.000)",
               "priority": 2147483647, "tuning_profile": ""}
    proj = {
        "board": {"3dviewports": [], "design_settings": {
            "meta": {"version": 2}, "rules": rules,
            "track_widths": [0.0, 0.25, 0.3, 0.4, 0.5, 0.8],
            "via_dimensions": [{"diameter": 0.0, "drill": 0.0}, {"diameter": 0.6, "drill": 0.3}],
            "drc_exclusions": []}, "layer_presets": [], "viewports": []},
        "boards": [],
        "libraries": {"pinned_footprint_libs": [], "pinned_symbol_libs": []},
        "meta": {"filename": name + ".kicad_pro", "version": 3},
        "net_settings": {"classes": [default], "meta": {"version": 4}, "net_colors": None,
                         "netclass_assignments": None, "netclass_patterns": []},
        "pcbnew": {"last_paths": {"gencad": "", "idf": "", "netlist": "", "plot": "", "pos_files": "",
                                  "specctra_dsn": "", "step": "", "svg": "", "vrml": ""},
                   "page_layout_descr_file": ""},
        "sheets": [[root_uuid, "Root"]],
        "text_variables": {},
    }
    (d / (name + ".kicad_pro")).write_text(json.dumps(proj, indent=2) + "\n", encoding="utf-8")


# --- schematic ---------------------------------------------------------------------

def build_schematic(path, name, title, comments, parts, flags, notes, paper):
    sch = Schematic(name, SYMLIB.read_text(encoding="utf-8"), title, boards.DATE, "A", comments, paper)
    for ref, sym, value, fp, nets, fields, (x, y), extra in parts:
        sch.part(ref, sym, value, x, y, nets, fp, fields, dnp=extra.get("dnp", False),
                 in_bom=extra.get("in_bom", True), ref_at=extra.get("ref_at"), val_at=extra.get("val_at"))
    for net, x, y in flags:
        sch.flag(net, x, y)
    for x, y, size, s in notes:
        sch.text(s, x, y, size)
    sch.write(path)
    run("sch", "upgrade", "--force", path)
    return sch.root


def read_netlist(sch_path):
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "net.net"
        run("sch", "export", "netlist", "--format", "kicadsexpr", "-o", out, sch_path)
        root = parse(out.read_text(encoding="utf-8"))
    comps = {}
    for c in find_all(find(root, "components"), "comp"):
        ref = find(c, "ref")[1]
        props = {find(p, "name")[1] for p in find_all(c, "property")}
        fields = {find(f, "name")[1]: (f[2] if len(f) > 2 else "") for f in find_all(find(c, "fields"), "field")}
        comps[ref] = {"value": find(c, "value")[1], "footprint": find(c, "footprint")[1],
                      "tstamp": find(c, "tstamps")[1], "dnp": "dnp" in props,
                      "no_bom": "exclude_from_bom" in props, "fields": fields, "pins": {}}
    nets = []
    for n in find_all(find(root, "nets"), "net"):
        name = find(n, "name")[1]
        nets.append(name)
        for node in find_all(n, "node"):
            comps[find(node, "ref")[1]]["pins"][find(node, "pin")[1]] = name
    return comps, nets


# --- layout --------------------------------------------------------------------------

def build_layout(path, comps, nets, size, place, routes, vias, texts, holes, thickness, refs,
                 back_text=(), relief=(), m2=None, zones=("F.Cu", "B.Cu")):
    import pcbnew

    def mm(v):
        return pcbnew.FromMM(v)

    def pt(x, y):
        return pcbnew.VECTOR2I(mm(x + OX), mm(y + OY))

    board = pcbnew.NewBoard(str(path))
    layer_id = {"F.Cu": pcbnew.F_Cu, "B.Cu": pcbnew.B_Cu, "F.SilkS": pcbnew.F_SilkS, "B.SilkS": pcbnew.B_SilkS}
    board.SetCopperLayerCount(2)
    board.GetDesignSettings().SetBoardThickness(mm(thickness))
    netinfo = {}
    for name in nets:
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        netinfo[name] = ni

    def footprint(name, ref, value, x, y, rot):
        fp = pcbnew.FootprintLoad(str(FPLIB), name)
        if fp is None:
            raise SystemExit("footprint %s missing from %s" % (name, FPLIB))
        fp.SetFPID(pcbnew.LIB_ID("rig", name))
        fp.SetReference(ref)
        fp.SetValue(value)
        board.Add(fp)
        fp.SetPosition(pt(x, y))
        fp.SetOrientationDegrees(rot)
        r = fp.Reference()
        r.SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8)))
        r.SetTextThickness(mm(0.12))
        return fp

    fps = {}
    for ref, c in sorted(comps.items()):
        x, y, rot = place[ref]
        fp = footprint(c["footprint"].split(":")[1], ref, c["value"], x, y, rot)
        fp.SetPath(pcbnew.KIID_PATH("/" + c["tstamp"]))
        if c["dnp"]:
            fp.SetDNP(True)
        if c["no_bom"]:
            fp.SetExcludedFromBOM(True)
        for k, v in c["fields"].items():
            if k not in ("Footprint", "Datasheet", "Description"):
                fp.SetField(k, v)
                fp.GetField(k).SetVisible(False)
        if ref in refs:
            r = fp.Reference()
            if refs[ref] is None:
                r.SetVisible(False)
            else:
                rx, ry, ra = refs[ref]
                r.SetPosition(pt(rx, ry))
                r.SetTextAngleDegrees(ra)
                r.SetKeepUpright(False)
        for pad in fp.Pads():
            net = c["pins"].get(pad.GetNumber())
            if net:
                pad.SetNet(netinfo[net])
        fps[ref] = fp

    def board_only(fp):
        fp.SetBoardOnly(True)
        fp.SetExcludedFromBOM(True)
        fp.SetExcludedFromPosFiles(True)
        fp.Reference().SetVisible(False)

    for i, (x, y) in enumerate(holes, 1):
        board_only(footprint("MountingHole_3.2mm_M3", "H%d" % i, "M3", x, y, 0))
    if m2:
        board_only(footprint("Hole_M2_2.2mm", "H%d" % (len(holes) + 1), "M2", m2[0], m2[1], 0))
    for i, (x, y) in enumerate(relief, 1):
        board_only(footprint("Hole_NPTH_1.5mm", "H%d" % (len(holes) + 1 + i), "tie", x, y, 0))

    # outline: rounded rectangle
    w, h = size
    r = 2.0 if w > 30 else 1.0
    k = r * (1 - 0.5 ** 0.5)

    def edge(shape, *p):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(shape)
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.1))
        if shape == pcbnew.SHAPE_T_SEGMENT:
            s.SetStart(pt(*p[0]))
            s.SetEnd(pt(*p[1]))
        else:
            s.SetArcGeometry(pt(*p[0]), pt(*p[1]), pt(*p[2]))
        board.Add(s)

    edge(pcbnew.SHAPE_T_SEGMENT, (r, 0), (w - r, 0))
    edge(pcbnew.SHAPE_T_SEGMENT, (w, r), (w, h - r))
    edge(pcbnew.SHAPE_T_SEGMENT, (w - r, h), (r, h))
    edge(pcbnew.SHAPE_T_SEGMENT, (0, h - r), (0, r))
    edge(pcbnew.SHAPE_T_ARC, (w - r, 0), (w - k, k), (w, r))
    edge(pcbnew.SHAPE_T_ARC, (w, h - r), (w - k, h - k), (w - r, h))
    edge(pcbnew.SHAPE_T_ARC, (r, h), (k, h - k), (0, h - r))
    edge(pcbnew.SHAPE_T_ARC, (0, r), (k, k), (r, 0))

    def full(net):
        # labels on the root sheet make nets named /STEP; power symbols make GND
        return net if net in netinfo else "/" + net

    # tracks
    def point(p, net):
        if isinstance(p[0], str):
            pad = fps[p[0]].FindPadByNumber(p[1])
            if pad.GetNetname() != net:
                raise SystemExit("route for %s ends on %s pad %s, which is %s" % (net, p[0], p[1], pad.GetNetname()))
            return pad.GetPosition()
        return pt(*p)

    for net, width, pts in routes:
        net = full(net)
        ps = [point(p, net) for p in pts]
        for a, b in zip(ps, ps[1:]):
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(a)
            t.SetEnd(b)
            t.SetWidth(mm(width))
            t.SetLayer(pcbnew.F_Cu)
            t.SetNet(netinfo[net])
            board.Add(t)

    for x, y in vias:
        v = pcbnew.PCB_VIA(board)
        board.Add(v)
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetPosition(pt(x, y))
        v.SetDrill(mm(0.3))
        v.SetWidth(mm(0.6))
        v.SetNet(netinfo["GND"])

    # ground pours
    for layer in zones:
        z = pcbnew.ZONE(board)
        z.SetLayer(layer_id[layer])
        z.SetNet(netinfo["GND"])
        z.SetLocalClearance(mm(0.3))
        z.SetMinThickness(mm(0.25))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetThermalReliefGap(mm(0.4))
        z.SetThermalReliefSpokeWidth(mm(0.5))
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        ol = z.Outline()
        ol.NewOutline()
        for x, y in ((0, 0), (w, 0), (w, h), (0, h)):
            ol.Append(mm(x + OX), mm(y + OY))
        board.Add(z)

    # silkscreen
    align = {"left": pcbnew.GR_TEXT_H_ALIGN_LEFT, "center": pcbnew.GR_TEXT_H_ALIGN_CENTER,
             "right": pcbnew.GR_TEXT_H_ALIGN_RIGHT}
    for layer, items in (("F.SilkS", texts), ("B.SilkS", back_text)):
        for s, x, y, size, angle, just in items:
            t = pcbnew.PCB_TEXT(board)
            t.SetText(s)
            t.SetLayer(layer_id[layer])
            t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
            t.SetTextThickness(mm(0.15 if size >= 1.0 else 0.12))
            t.SetHorizJustify(align[just])
            t.SetKeepUpright(False)
            t.SetTextAngleDegrees(angle)
            if layer.startswith("B."):
                t.SetMirrored(True)
            t.SetPosition(pt(x, y))
            board.Add(t)

    pcbnew.SaveBoard(str(path), board)
    return fps


# --- the two boards ---------------------------------------------------------------

def build_board(name, title, comments, parts, flags, notes, paper, layout):
    d = PCB / name
    d.mkdir(exist_ok=True)
    write_lib_tables(d)
    sch = d / (name + ".kicad_sch")
    root = build_schematic(sch, name, title, comments, parts, flags, notes, paper)
    comps, nets = read_netlist(sch)
    pcb = d / (name + ".kicad_pcb")
    write_project(d, name, root)
    build_layout(pcb, comps, nets, **layout)
    write_project(d, name, root)   # SaveBoard rewrites the project file; put the rules back
    run("pcb", "drc", "--refill-zones", "--save-board", "-o", Path(tempfile.gettempdir()) / (name + "_drc.rpt"), pcb)
    print("built", name)


def main():
    build_symbol_lib(KICAD_SYM, SYMLIB)
    run("sym", "upgrade", "--force", SYMLIB)
    build_footprint_lib(KICAD_FP, FPLIB)

    build_board("scan_controller", "Scan-mirror controller",
                ["ESP32-DevKitC + BTT TMC2209 for the scan mirror's NEMA 8",
                 "Pin map: firmware/include/board_pins.h", boards.REPO],
                boards.CONTROLLER_PARTS, boards.CONTROLLER_FLAGS, boards.CONTROLLER_NOTES, "A3",
                dict(size=boards.CONTROLLER_SIZE, place=boards.CONTROLLER_PLACE, routes=boards.CONTROLLER_ROUTES,
                     vias=boards.CONTROLLER_VIAS, texts=boards.CONTROLLER_TEXT, holes=boards.CONTROLLER_HOLES,
                     thickness=1.6, refs=boards.CONTROLLER_REFS))
    build_board("hall_breakout", "Hall-sensor breakout",
                ["A3144 home sensor on the scanner head's +X wall", boards.REPO],
                boards.HALL_PARTS, boards.HALL_FLAGS, boards.HALL_NOTES, "A4",
                dict(size=boards.HALL_SIZE, place=boards.HALL_PLACE, routes=boards.HALL_ROUTES,
                     vias=boards.HALL_VIAS, texts=boards.HALL_TEXT, holes=boards.HALL_HOLES, thickness=0.8,
                     refs=boards.HALL_REFS,
                     back_text=boards.HALL_BACK_TEXT, relief=boards.HALL_RELIEF, m2=boards.HALL_M2,
                     zones=("B.Cu",)))


if __name__ == "__main__":
    main()
