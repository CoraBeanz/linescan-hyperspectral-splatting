"""Expression-bound Part features.

Every dimension in the rig is an `E`: a number plus the FreeCAD expression that
computes it from the `Params` spreadsheet. The helpers below create ordinary
Part workbench primitives (Box, Cylinder, Cone, Prism, Wedge) and set both the
number and the expression, so the saved .FCStd is fully parametric: change a
cell in Params, press Recompute, and every part that depends on it moves or
resizes. Booleans (Cut, MultiFuse) are live features too.

Conventions:
  * Units are millimetres and degrees, stored as plain numbers in the sheet.
    (FreeCAD's sin/cos take degrees; asin returns an angle, so the sheet
    divides it by `1 deg` to keep everything unitless.)
  * A `Frame` is an origin plus an optional rotation about X by an E angle.
    It lets one part mix axis-aligned features with features tilted to the
    camera's 22 deg axis while every placement stays expression-bound.
"""

import math

import FreeCAD as App

SHEET = "Params"


def _fmt(x):
    if abs(x - round(x)) < 1e-9:
        s = str(int(round(x)))
    else:
        s = ("%.6f" % x).rstrip("0").rstrip(".")
    return "(" + s + ")" if x < 0 else s


class E:
    """A float that also carries the FreeCAD expression computing it."""

    __slots__ = ("v", "s", "atom")

    def __init__(self, v, s=None, atom=True):
        self.v = float(v)
        self.s = _fmt(self.v) if s is None else s
        self.atom = atom

    @staticmethod
    def of(x):
        return x if isinstance(x, E) else E(x)

    @property
    def const(self):
        return SHEET + "." not in self.s

    def _p(self):
        return self.s if self.atom else "(" + self.s + ")"

    def __repr__(self):
        return "E(%g, %r)" % (self.v, self.s)

    def __float__(self):
        return self.v

    def __add__(self, o):
        o = E.of(o)
        if o.const and o.v == 0:
            return self
        if self.const and self.v == 0:
            return o
        if self.const and o.const:
            return E(self.v + o.v)
        return E(self.v + o.v, self._p() + " + " + o._p(), False)

    __radd__ = __add__

    def __sub__(self, o):
        o = E.of(o)
        if o.const and o.v == 0:
            return self
        if self.const and o.const:
            return E(self.v - o.v)
        return E(self.v - o.v, self._p() + " - " + o._p(), False)

    def __rsub__(self, o):
        return E.of(o) - self

    def __mul__(self, o):
        o = E.of(o)
        if o.const and o.v == 1:
            return self
        if self.const and self.v == 1:
            return o
        if (o.const and o.v == 0) or (self.const and self.v == 0):
            return E(0)
        if self.const and o.const:
            return E(self.v * o.v)
        return E(self.v * o.v, self._p() + " * " + o._p(), False)

    __rmul__ = __mul__

    def __truediv__(self, o):
        o = E.of(o)
        if o.const and o.v == 1:
            return self
        if self.const and o.const:
            return E(self.v / o.v)
        return E(self.v / o.v, self._p() + " / " + o._p(), False)

    def __rtruediv__(self, o):
        return E.of(o) / self

    def __neg__(self):
        if self.const:
            return E(-self.v)
        return E(-self.v, "-" + self._p(), False)


def sin(a):
    a = E.of(a)
    return E(math.sin(math.radians(a.v)), "sin(" + a.s + ")") if not a.const else E(math.sin(math.radians(a.v)))


def cos(a):
    a = E.of(a)
    return E(math.cos(math.radians(a.v)), "cos(" + a.s + ")") if not a.const else E(math.cos(math.radians(a.v)))


def tan(a):
    a = E.of(a)
    return E(math.tan(math.radians(a.v)), "tan(" + a.s + ")") if not a.const else E(math.tan(math.radians(a.v)))


def emax(a, b):
    """max() for E. Value is exact; the expression uses FreeCAD's max()."""
    a, b = E.of(a), E.of(b)
    if a.const and b.const:
        return E(max(a.v, b.v))
    return E(max(a.v, b.v), "max(" + a.s + "; " + b.s + ")")


class Params:
    """Reads aliases from the spreadsheet and hands them out as E values."""

    def __init__(self, sheet):
        self.sheet = sheet

    def __getattr__(self, name):
        if name.startswith("_") or name == "sheet":
            raise AttributeError(name)
        v = self.sheet.get(name)
        return E(float(v), SHEET + "." + name)


class Frame:
    """Origin (E, E, E) plus a rotation about +X by angle `a` (E, degrees)."""

    def __init__(self, origin=(0, 0, 0), a=0):
        self.o = tuple(E.of(c) for c in origin)
        self.a = E.of(a)

    def pt(self, x, y, z):
        """Local point -> container coordinates (tuple of E)."""
        x, y, z = E.of(x), E.of(y), E.of(z)
        if self.a.const and self.a.v == 0:
            return (self.o[0] + x, self.o[1] + y, self.o[2] + z)
        c, s = cos(self.a), sin(self.a)
        return (self.o[0] + x, self.o[1] + y * c - z * s, self.o[2] + y * s + z * c)

    def sub(self, x, y, z):
        """A child frame offset by (x, y, z) in local coordinates."""
        return Frame(self.pt(x, y, z), self.a)

    @property
    def tilted(self):
        return not (self.a.const and self.a.v == 0)


WORLD = Frame()

_AXIS_ROT = {
    "z": App.Rotation(),
    "x": App.Rotation(App.Vector(0, 1, 0), 90),   # local +Z -> +X
    "y": App.Rotation(App.Vector(1, 0, 0), -90),  # local +Z -> +Y
    "-x": App.Rotation(App.Vector(0, 1, 0), -90),
    "-y": App.Rotation(App.Vector(1, 0, 0), 90),
    "-z": App.Rotation(App.Vector(1, 0, 0), 180),
    # Wedges drawn in their XY plane and extruded along X: local X -> Y,
    # local Y -> Z, local Z -> X (used for the dovetail rail and slot)
    "yzx": App.Rotation(App.Matrix(0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1)),
    # Wedges drawn in the ZY plane and extruded toward -X: local X -> Z,
    # local Y -> Y, local Z -> -X (the window hood's chamfered underside)
    "zy-x": App.Rotation(App.Matrix(0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1)),
}


def bind(obj, prop, e):
    """Set a numeric property to e.v and, if e depends on Params, bind it."""
    e = E.of(e)
    setattr(obj, prop, e.v)
    if not e.const:
        obj.setExpression(prop, e.s)


def place(obj, pos, frame=WORLD, axis="z"):
    """Place obj with its local origin at `pos` (local to `frame`)."""
    p = frame.pt(*pos)
    if frame.tilted:
        if axis != "z":
            raise ValueError("tilted frames only take Z-axis primitives")
        rot = App.Rotation(App.Vector(1, 0, 0), frame.a.v)
    else:
        rot = _AXIS_ROT[axis]
    obj.Placement = App.Placement(App.Vector(p[0].v, p[1].v, p[2].v), rot)
    for comp, e in zip("xyz", p):
        if not e.const:
            obj.setExpression(".Placement.Base." + comp, e.s)
    if frame.tilted and not frame.a.const:
        obj.setExpression(".Placement.Rotation.Angle", frame.a.s)


class Builder:
    """Creates features inside one App::Part container."""

    def __init__(self, doc, container, prefix):
        self.doc = doc
        self.c = container
        self.prefix = prefix
        self.n = 0

    def _new(self, kind, name):
        self.n += 1
        o = self.doc.addObject(kind, "%s_%s" % (self.prefix, name))
        o.Label = "%s_%s" % (self.prefix, name)
        self.c.addObject(o)
        return o

    def box(self, name, x, y, z, L, W, H, frame=WORLD):
        """Axis-aligned box (in `frame`) with min corner (x, y, z), size L x W x H."""
        o = self._new("Part::Box", name)
        bind(o, "Length", L)
        bind(o, "Width", W)
        bind(o, "Height", H)
        place(o, (x, y, z), frame)
        return o

    def cbox(self, name, cx, cy, z, L, W, H, frame=WORLD):
        """Box centred on (cx, cy) in X/Y, from z up by H."""
        L, W = E.of(L), E.of(W)
        return self.box(name, E.of(cx) - L / 2, E.of(cy) - W / 2, z, L, W, H, frame)

    def rbox(self, name, cx, cy, z, L, W, H, r, frame=WORLD):
        """Box centred on (cx, cy) in X/Y, from z up by H, with its four vertical edges rounded to r."""
        cx, cy, L, W, r = E.of(cx), E.of(cy), E.of(L), E.of(W), E.of(r)
        shapes = [self.cbox(name + "_x", cx, cy, z, L - 2 * r, W, H, frame),
                  self.cbox(name + "_y", cx, cy, z, L, W - 2 * r, H, frame)]
        for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
            shapes.append(self.cyl("%s_r%d" % (name, i), "z", (cx + sx * (L / 2 - r), cy + sy * (W / 2 - r), z),
                                   r, H, frame))
        return self.fuse(name, shapes)

    def cyl(self, name, axis, base, r, h, frame=WORLD):
        """Cylinder of radius r and length h starting at `base` along `axis`."""
        o = self._new("Part::Cylinder", name)
        bind(o, "Radius", r)
        bind(o, "Height", h)
        place(o, base, frame, axis)
        return o

    def cone(self, name, axis, base, r1, r2, h, frame=WORLD):
        o = self._new("Part::Cone", name)
        bind(o, "Radius1", r1)
        bind(o, "Radius2", r2)
        bind(o, "Height", h)
        place(o, base, frame, axis)
        return o

    def prism(self, name, axis, base, n, circumradius, h, frame=WORLD):
        """Regular n-gon prism (hex nut traps)."""
        o = self._new("Part::Prism", name)
        o.Polygon = n
        bind(o, "Circumradius", circumradius)
        bind(o, "Height", h)
        place(o, base, frame, axis)
        return o

    def wedge(self, name, base, xmin, xmax, x2min, x2max, ymax, zmax, frame=WORLD, axis="z"):
        """Trapezoid prism: X span [xmin,xmax] at y=0, [x2min,x2max] at y=ymax, Z depth zmax."""
        o = self._new("Part::Wedge", name)
        for prop, v in (("Xmin", xmin), ("Xmax", xmax), ("X2min", x2min), ("X2max", x2max),
                        ("Ymin", 0), ("Ymax", ymax), ("Zmin", 0), ("Zmax", zmax),
                        ("Z2min", 0), ("Z2max", zmax)):
            bind(o, prop, v)
        place(o, base, frame, axis)
        return o

    def fuse(self, name, shapes):
        if len(shapes) == 1:
            return shapes[0]
        o = self._new("Part::MultiFuse", name)
        o.Shapes = list(shapes)
        o.Refine = True
        for s in shapes:
            s.Visibility = False
        return o

    def cut(self, name, base, tools):
        tools = [t for t in tools if t is not None]
        if not tools:
            return base
        tool = tools[0] if len(tools) == 1 else self.fuse(name + "_tools", tools)
        o = self._new("Part::Cut", name)
        o.Base = base
        o.Tool = tool
        o.Refine = True
        base.Visibility = False
        tool.Visibility = False
        return o

    def common(self, name, shapes):
        o = self._new("Part::MultiCommon", name)
        o.Shapes = list(shapes)
        for s in shapes:
            s.Visibility = False
        return o
