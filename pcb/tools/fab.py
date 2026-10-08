#!/usr/bin/env python3
"""Check the boards and write their JLCPCB files, with kicad-cli (KiCad 10).

    python pcb/tools/fab.py                 ERC, DRC, then fab/ for both boards
    python pcb/tools/fab.py --check-only    ERC and DRC only (what CI runs)
    python pcb/tools/fab.py hall_breakout   one board

ERC and DRC count warnings as failures too. DRC also checks that the board
matches its schematic (--schematic-parity). For each board, fab/ gets:

    <board>_gerbers.zip   Gerbers and Excellon drill files: upload to JLCPCB
    bom_jlc.csv           parts JLCPCB solders (SMT assembly), with LCSC numbers
    cpl_jlc.csv           their positions and rotations
    bom.csv               every part on the board, including the ones you solder
    <board>.pdf           the schematic
    erc.rpt, drc.rpt      the check reports

and pcb/img/<board>.png, a 3D render of the board.
"""
import csv
import io
import os
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

PCB = Path(__file__).resolve().parent.parent
BOARDS = ["scan_controller", "hall_breakout"]
LAYERS = "F.Cu,B.Cu,F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts"
RENDER = {"scan_controller": (1200, 820, "--rotate", "-35,0,25"),
          "hall_breakout": (520, 470, "--background", "opaque")}


def find_cli():
    if os.environ.get("KICAD_CLI"):
        return os.environ["KICAD_CLI"]
    found = shutil.which("kicad-cli")
    if found:
        return found
    for base in (os.environ.get("LOCALAPPDATA", ""), os.environ.get("ProgramFiles", "")):
        for sub in ("Programs/KiCad/10.0/bin/kicad-cli.exe", "KiCad/10.0/bin/kicad-cli.exe"):
            if base and Path(base, sub).exists():
                return str(Path(base, sub))
    sys.exit("kicad-cli not found: install KiCad 10 or set KICAD_CLI")


CLI = find_cli()


def cli(*args, check=True):
    r = subprocess.run([CLI] + [str(a) for a in args], capture_output=True, text=True)
    if check and r.returncode != 0:
        sys.stdout.write(r.stdout + r.stderr)
    return r.returncode


def checks(board, out):
    d = PCB / board
    ok = True
    for kind, args in (("erc", ["sch", "erc", d / (board + ".kicad_sch")]),
                       ("drc", ["pcb", "drc", "--schematic-parity", d / (board + ".kicad_pcb")])):
        rpt = out / (kind + ".rpt")
        rc = cli(*args[:2], "--severity-all", "--exit-code-violations", "-o", rpt, *args[2:])
        text = rpt.read_text(encoding="utf-8", errors="replace") if rpt.exists() else ""
        if rc != 0:
            ok = False
            print("%s: %s FAILED" % (board, kind.upper()))
            print(text)
        else:
            print("%s: %s clean" % (board, kind.upper()))
    return ok


def exports(board, fab):
    d = PCB / board
    pcb, sch = d / (board + ".kicad_pcb"), d / (board + ".kicad_sch")
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        g = tmp / "gerbers"
        g.mkdir()
        cli("pcb", "export", "gerbers", "--layers", LAYERS, "-o", g, pcb)
        cli("pcb", "export", "drill", "--format", "excellon", "--excellon-units", "mm",
            "--excellon-separate-th", "--generate-map", "--map-format", "gerberx2", "-o", g, pcb)
        zpath = fab / (board + "_gerbers.zip")
        with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
            for f in sorted(g.iterdir()):
                z.write(f, f.name)

        # every part, grouped, with the fields the schematic carries
        cli("sch", "export", "bom", "--fields",
            "Reference,Value,Footprint,${QUANTITY},LCSC,MPN,Assembly,${DNP}",
            "--labels", "Designator,Value,Footprint,Qty,LCSC,MPN,Assembly,DNP",
            "--group-by", "Value,Footprint,LCSC,MPN,Assembly,${DNP}", "--ref-range-delimiter", "",
            "-o", tmp / "bom.csv", sch)
        rows = list(csv.DictReader(io.StringIO((tmp / "bom.csv").read_text(encoding="utf-8"))))
        with open(fab / "bom.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

        # JLCPCB's assembly BOM and placement file: only the parts it solders
        jlc = [r for r in rows if r["Assembly"] == "JLC" and not r["DNP"].strip()]
        refs = {ref.strip() for r in jlc for ref in r["Designator"].split(",")}
        with open(fab / "bom_jlc.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
            for r in jlc:
                w.writerow([r["Value"], r["Designator"], r["Footprint"].split(":")[-1], r["LCSC"]])
        cli("pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "both", "--exclude-dnp",
            "-o", tmp / "pos.csv", pcb)
        pos = list(csv.DictReader(io.StringIO((tmp / "pos.csv").read_text(encoding="utf-8"))))
        with open(fab / "cpl_jlc.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
            for p in pos:
                if p["Ref"] in refs:
                    w.writerow([p["Ref"], p["PosX"] + "mm", p["PosY"] + "mm",
                                "Top" if p["Side"] == "top" else "Bottom", p["Rot"]])

    cli("sch", "export", "pdf", "-o", fab / (board + ".pdf"), sch)

    # the pictures in pcb/README.md (KiCad's 3D models have to be installed)
    img = PCB / "img"
    img.mkdir(exist_ok=True)
    view = RENDER[board]
    cli("pcb", "render", "--quality", "high", "-w", view[0], "-h", view[1], *view[2:],
        "-o", img / (board + ".png"), pcb)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    check_only = "--check-only" in sys.argv
    ok = True
    for board in args or BOARDS:
        if check_only:
            with tempfile.TemporaryDirectory() as tmp:
                ok &= checks(board, Path(tmp))
            continue
        fab = PCB / board / "fab"
        fab.mkdir(exist_ok=True)
        if checks(board, fab):
            exports(board, fab)
            print("%s: fab files in %s" % (board, fab.relative_to(PCB.parent)))
        else:
            ok = False
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
