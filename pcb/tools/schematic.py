"""Write a one-sheet KiCad schematic in net-label style.

Every connected pin gets a short wire and a label (or a power symbol) at its
end; pins with the same label are one net. Unused pins get a no-connect flag,
so ERC can tell a forgotten wire from a pin left open on purpose.
"""
import math
import uuid

from sexpr import A, dump, find, parse, top_symbol, walk

NS = uuid.UUID("6f1c2d0e-9b1a-4c57-8a43-5d1f0c0a7e21")
POWER = ("GND", "+3V3", "+5V")
STUB = 2.54


def uid(*parts):
    """Stable UUIDs, so rerunning the generator gives the same files."""
    return str(uuid.uuid5(NS, "/".join(parts)))


def _font(size=1.27, justify=None):
    e = [A("effects"), [A("font"), [A("size"), size, size]]]
    if justify:
        e.append([A("justify")] + [A(j) for j in justify])
    return e


def _r(v):
    return round(v, 4)


class Schematic:
    def __init__(self, project, lib_text, title, date, rev, comments=(), paper="A3"):
        self.project = project
        self.root = uid(project, "root")
        self.lib_text = lib_text
        self.used = {}
        self.items = []
        self.n_pwr = 0
        self.n_flg = 0
        self.title_block = [A("title_block"), [A("title"), title], [A("date"), date], [A("rev"), rev],
                            [A("company"), "CoraBeanz"]]
        for i, c in enumerate(comments, 1):
            self.title_block.append([A("comment"), i, c])
        self.paper = paper

    # --- library symbols ------------------------------------------------------
    def _lib(self, name):
        if name not in self.used:
            raw = top_symbol(self.lib_text, name)
            self.used[name] = raw
        return self.used[name]

    def pins(self, name):
        """{pin number: (x, y, angle, name)} in library coordinates (y up)."""
        out = {}
        for p in walk(parse(self._lib(name)), "pin"):
            at = find(p, "at")
            out[find(p, "number")[1]] = (float(at[1]), float(at[2]), float(at[3]), find(p, "name")[1])
        return out

    # --- items ----------------------------------------------------------------
    def _instance(self, lib_name, ref, value, x, y, rot, props, in_bom, on_board, dnp, key):
        u = uid(self.project, "sym", key)
        node = [A("symbol"), [A("lib_id"), "rig:" + lib_name], [A("at"), _r(x), _r(y), rot], [A("unit"), 1],
                [A("exclude_from_sim"), A("no")], [A("in_bom"), A("yes" if in_bom else "no")],
                [A("on_board"), A("yes" if on_board else "no")], [A("dnp"), A("yes" if dnp else "no")],
                [A("uuid"), u]]
        for name, val, px, py, ang, hide, just in props:
            p = [A("property"), name, val, [A("at"), _r(px), _r(py), ang]]
            if hide:
                p.append([A("hide"), A("yes")])
            p.append(_font(justify=just))
            node.append(p)
        for num in self.pins(lib_name):
            node.append([A("pin"), num, [A("uuid"), uid(self.project, "pin", key, num)]])
        node.append([A("instances"), [A("project"), self.project,
                                      [A("path"), "/" + self.root, [A("reference"), ref], [A("unit"), 1]]]])
        self.items.append(node)
        return u

    def wire(self, x0, y0, x1, y1):
        self.items.append([A("wire"), [A("pts"), [A("xy"), _r(x0), _r(y0)], [A("xy"), _r(x1), _r(y1)]],
                           [A("stroke"), [A("width"), 0], [A("type"), A("default")]],
                           [A("uuid"), uid(self.project, "wire", "%.3f,%.3f,%.3f,%.3f" % (x0, y0, x1, y1))]])

    def label(self, net, x, y, angle):
        just = ("left", "bottom") if angle in (0, 90) else ("right", "bottom")
        self.items.append([A("label"), net, [A("at"), _r(x), _r(y), angle], [A("fields_autoplaced"), A("yes")],
                           _font(justify=just),
                           [A("uuid"), uid(self.project, "label", net, "%.3f,%.3f" % (x, y))]])

    def no_connect(self, x, y):
        self.items.append([A("no_connect"), [A("at"), _r(x), _r(y)],
                           [A("uuid"), uid(self.project, "nc", "%.3f,%.3f" % (x, y))]])

    def text(self, s, x, y, size=1.27):
        self.items.append([A("text"), s, [A("exclude_from_sim"), A("no")], [A("at"), _r(x), _r(y), 0],
                           _font(size, ("left", "bottom")), [A("uuid"), uid(self.project, "text", s)]])

    def power(self, net, x, y, direction):
        """A power symbol at the end of a wire that leaves its pin going direction.
        On a sideways wire it points along the wire, so it stays clear of the pins
        2.54 mm above and below; on an upright one GND points down and supply
        arrows up, unless the wire comes from that side."""
        self.n_pwr += 1
        ref = "#PWR%02d" % self.n_pwr
        if direction in ("left", "right"):
            rot = {("GND", "left"): 270, ("GND", "right"): 90}.get((net, direction),
                                                                   90 if direction == "left" else 270)
            dx, dy = (-6.35 if direction == "left" else 6.35), 0
        else:
            rot = 180 if direction == ("up" if net == "GND" else "down") else 0
            dx, dy = 0, (3.81 if net == "GND" else -3.556) * (-1 if rot == 180 else 1)
        self._instance(net, ref, net, x, y, rot,
                       [("Reference", ref, x, y, 0, True, None),
                        # field angles turn with the symbol: 90 on a sideways symbol reads level
                        ("Value", net, x + dx, y + dy, 90 if rot in (90, 270) else 0, False, None),
                        ("Footprint", "", x, y, 0, True, None), ("Datasheet", "", x, y, 0, True, None),
                        ("Description", "", x, y, 0, True, None)],
                       False, True, False, ref)

    def flag(self, net, x, y):
        """PWR_FLAG: tells ERC this net is powered from off the sheet (a connector)."""
        self.n_flg += 1
        ref = "#FLG%02d" % self.n_flg
        self._instance("PWR_FLAG", ref, "PWR_FLAG", x, y, 0,
                       [("Reference", ref, x, y - 1.905, 0, True, None),
                        ("Value", "PWR_FLAG", x, y - 3.81, 0, False, None),
                        ("Footprint", "", x, y, 0, True, None), ("Datasheet", "", x, y, 0, True, None),
                        ("Description", "", x, y, 0, True, None)],
                       False, True, False, ref)
        self.wire(x, y, x, y + STUB)
        if net in POWER:
            self.power(net, x, y + STUB, "down")
        else:
            self.label(net, x, y + STUB, 0)

    def part(self, ref, lib_name, value, x, y, nets, footprint, fields=(), dnp=False, in_bom=True,
             ref_at=None, val_at=None, datasheet=""):
        """Place a symbol at (x, y), unrotated, and wire every pin to its net."""
        pins = self.pins(lib_name)
        ys = [y - p[1] for p in pins.values()]
        vertical = all(p[2] in (90, 270) for p in pins.values())
        if vertical:   # two-pin part standing up: text to its right
            ref_at = ref_at or (x + 2.54, y - 1.27)
            val_at = val_at or (x + 2.54, y + 1.27)
            just = ("left",)
        else:          # text above and below, clear of the wires
            ref_at = ref_at or (x, min(ys) - 3.81)
            val_at = val_at or (x, max(ys) + 3.81)
            just = None
        props = [("Reference", ref, ref_at[0], ref_at[1], 0, False, just),
                 ("Value", value, val_at[0], val_at[1], 0, False, just),
                 ("Footprint", footprint, x, y, 0, True, None),
                 ("Datasheet", datasheet, x, y, 0, True, None),
                 ("Description", "", x, y, 0, True, None)]
        props += [(k, v, x, y, 0, True, None) for k, v in fields]
        self._instance(lib_name, ref, value, x, y, 0, props, in_bom, True, dnp, ref)
        unknown = set(nets) - set(pins)
        if unknown:
            raise KeyError("%s has no pins %s" % (ref, sorted(unknown)))
        for num, (px, py, ang, _name) in pins.items():
            ex, ey = x + px, y - py
            ox, oy = -round(math.cos(math.radians(ang))), round(math.sin(math.radians(ang)))
            if num not in nets:
                self.no_connect(ex, ey)
                continue
            net = nets[num]
            sx, sy = ex + ox * STUB, ey + oy * STUB
            self.wire(ex, ey, sx, sy)
            direction = {(-1, 0): "left", (1, 0): "right", (0, -1): "up", (0, 1): "down"}[(ox, oy)]
            if net in POWER:
                self.power(net, sx, sy, direction)
            else:
                self.label(net, sx, sy, {"left": 180, "right": 0, "up": 90, "down": 270}[direction])

    # --- file -------------------------------------------------------------------
    def write(self, path):
        libs = []
        for name, raw in sorted(self.used.items()):
            libs.append(raw.replace('(symbol "%s"' % name, '(symbol "rig:%s"' % name, 1))
        head = [A("kicad_sch"), [A("version"), 20260101], [A("generator"), "eeschema"],
                [A("generator_version"), "10.0"], [A("uuid"), self.root], [A("paper"), self.paper],
                self.title_block]
        body = dump(head)[:-2]  # drop the closing paren; more follows
        out = [body, "\t(lib_symbols"] + ["\t\t" + b for b in libs] + ["\t)"]
        out += [dump(i, 1) for i in self.items]
        out.append(dump([A("sheet_instances"), [A("path"), "/", [A("page"), "1"]]], 1))
        out.append("\t(embedded_fonts no)\n)")
        path.write_text("\n".join(out) + "\n", encoding="utf-8")
