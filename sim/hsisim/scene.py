"""Scenes: simple solids with textured faces, lit by a lamp, and rays cast into them.

A scene is a list of solids (an infinite table plane, boxes, spheres). Every face carries a
texture: layers of patterns (solid, checker, a bitmap, random dots), each optionally kept
to a rectangle of the face, in the face's own 2-D coordinates (metres from its centre).
A texture says which material each point is; the spectral library (spectra.py) says what
that material reflects. Surfaces are matte (Lambertian).

Lighting is either "uniform", as from a big diffuse source or a light tent, where every
surface gets the same light and a scan sees plain reflectance, or a "lamp", a point source
whose light falls off with distance and the angle of incidence, with hard shadows. Either
way a white reference held flat at the scan target sees shading 1.

Everything is in metres, in the arm's base_link frame (z up, the table top at the plan's
target height).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import spectra

EPS = 1e-7


# -- patterns and textures -------------------------------------------------------------------


@dataclass
class Solid:
    material: str

    def ids(self, u, v):
        return np.full(u.shape, spectra.material_id(self.material), np.int32)

    def to_dict(self):
        return dict(type="solid", material=self.material)


@dataclass
class Checker:
    size: float
    materials: tuple
    origin: tuple = (0.0, 0.0)

    def ids(self, u, v):
        a, b = (spectra.material_id(m) for m in self.materials)
        k = (np.floor((u - self.origin[0]) / self.size) + np.floor((v - self.origin[1]) / self.size)).astype(int)
        return np.where(k % 2 == 0, a, b).astype(np.int32)

    def to_dict(self):
        return dict(type="checker", size=self.size, materials=list(self.materials), origin=list(self.origin))


@dataclass
class Bitmap:
    """Rows of text, '#' for `on` and anything else for `off`, `cell` metres per character,
    centred on `centre`; the first row is at +v (the top, seen from above)."""
    rows: tuple
    cell: float
    on: str
    off: str
    centre: tuple = (0.0, 0.0)

    def ids(self, u, v):
        grid = np.array([[ch == "#" for ch in r] for r in self.rows])
        nr, nc = grid.shape
        c = np.floor((u - self.centre[0]) / self.cell + nc / 2).astype(int)
        r = np.floor(nr / 2 - (v - self.centre[1]) / self.cell).astype(int)
        inside = (r >= 0) & (r < nr) & (c >= 0) & (c < nc)
        lit = np.zeros(u.shape, bool)
        lit[inside] = grid[r[inside], c[inside]]
        return np.where(lit, spectra.material_id(self.on), spectra.material_id(self.off)).astype(np.int32)

    def to_dict(self):
        return dict(type="bitmap", rows=list(self.rows), cell=self.cell, on=self.on, off=self.off,
                    centre=list(self.centre))


@dataclass
class Speckle:
    """Round dots of `dot` material on `base`: one chance per `cell` x `cell` square, at a
    random place in it, so the pattern has detail at every scale a scan line sees."""
    cell: float
    radius: float
    base: str
    dot: str
    density: float = 0.6
    seed: int = 0

    def ids(self, u, v):
        i, j = np.floor(u / self.cell).astype(np.int64), np.floor(v / self.cell).astype(np.int64)
        out = np.full(u.shape, spectra.material_id(self.base), np.int32)
        for di in (-1, 0, 1):           # a dot near a cell's edge reaches into its neighbours
            for dj in (-1, 0, 1):
                ii, jj = i + di, j + dj
                h = _hash(ii, jj, self.seed)
                has = (h & 0xFFFF) / 65536.0 < self.density
                cx = (ii + 0.15 + 0.7 * ((h >> 16) & 0xFF) / 255.0) * self.cell
                cy = (jj + 0.15 + 0.7 * ((h >> 24) & 0xFF) / 255.0) * self.cell
                hit = has & ((u - cx) ** 2 + (v - cy) ** 2 <= self.radius ** 2)
                out[hit] = spectra.material_id(self.dot)
        return out

    def to_dict(self):
        return dict(type="speckle", cell=self.cell, radius=self.radius, base=self.base, dot=self.dot,
                    density=self.density, seed=self.seed)


def _hash(i, j, seed):
    """A fixed pseudo-random 32-bit number per integer cell (splitmix-style mixing)."""
    x = (i.astype(np.uint64) * np.uint64(0x9E3779B97F4A7C15)) ^ (j.astype(np.uint64) * np.uint64(0xBF58476D1CE4E5B9))
    x ^= np.uint64(seed * 0x94D049BB133111EB & 0xFFFFFFFFFFFFFFFF)
    x ^= x >> np.uint64(31)
    x *= np.uint64(0xD6E8FEB86659FD93)
    x ^= x >> np.uint64(32)
    return (x & np.uint64(0xFFFFFFFF)).astype(np.int64)


@dataclass
class Texture:
    """Layers of (pattern, rect): later layers sit on top; rect = (u0, v0, u1, v1) or None."""
    layers: list = field(default_factory=list)

    @classmethod
    def solid(cls, material):
        return cls([(Solid(material), None)])

    def add(self, pattern, rect=None):
        self.layers.append((pattern, rect))
        return self

    def ids(self, u, v):
        out = np.zeros(u.shape, np.int32)
        for pattern, rect in self.layers:
            if rect is None:
                out[:] = pattern.ids(u, v)
                continue
            m = (u >= rect[0]) & (u <= rect[2]) & (v >= rect[1]) & (v <= rect[3])
            if m.any():
                out[m] = pattern.ids(u[m], v[m])
        return out

    def materials(self):
        names = set()
        for pattern, _ in self.layers:
            d = pattern.to_dict()
            for k in ("material", "on", "off", "base", "dot"):
                if k in d:
                    names.add(d[k])
            names.update(d.get("materials", []))
        return names

    def to_dict(self):
        return [dict(pattern.to_dict(), rect=None if rect is None else list(rect)) for pattern, rect in self.layers]


# -- solids ------------------------------------------------------------------------------------


def yaw(deg):
    """Rotation about +z (up)."""
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Plane:
    """The infinite horizontal plane z = height: the table. Texture coordinates are (x, y)."""

    def __init__(self, height, texture, name="table"):
        self.height, self.texture, self.name = float(height), texture, name

    def intersect(self, o, d):
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (self.height - o[:, 2]) / d[:, 2]
        t = np.where(np.isfinite(t) & (t > EPS), t, np.inf)
        n = np.zeros_like(d)
        n[:, 2] = np.where(d[:, 2] < 0, 1.0, -1.0)
        return t, n

    def ids(self, p, n):
        return self.texture.ids(p[:, 0], p[:, 1])

    def bound(self):
        return None  # infinite

    def materials(self):
        return self.texture.materials()

    def to_dict(self):
        return dict(type="plane", name=self.name, height=self.height, texture=self.texture.to_dict())


class Box:
    """A box `size` (x, y, z) centred on `centre`, turned by `rotation` (box -> world).

    Faces: "top" (+z), "bottom" (-z) and "sides". Texture coordinates on the top are the
    box's own (x, y); on a side, (along the face, z)."""

    def __init__(self, centre, size, top, sides=None, bottom=None, rotation=None, name="box"):
        self.c = np.asarray(centre, float)
        self.half = 0.5 * np.asarray(size, float)
        self.R = np.eye(3) if rotation is None else np.asarray(rotation, float)
        self.top = top
        self.sides = sides or top
        self.bottom = bottom or self.sides
        self.name = name

    def intersect(self, o, d):
        ol = (o - self.c) @ self.R
        dl = d @ self.R
        near = np.full(len(o), -np.inf)
        far = np.full(len(o), np.inf)
        axis = np.zeros(len(o), np.int8)
        for i in range(3):  # slabs, one axis at a time (1-D arrays are quicker than axis reductions)
            di = np.where(np.abs(dl[:, i]) < 1e-30, 1e-30, dl[:, i])
            a, b = (-self.half[i] - ol[:, i]) / di, (self.half[i] - ol[:, i]) / di
            lo, hi = np.minimum(a, b), np.maximum(a, b)
            later = lo > near
            near = np.where(later, lo, near)
            axis[later] = i
            far = np.minimum(far, hi)
        t = np.where((near <= far) & (near > EPS), near, np.inf)
        nl = np.zeros_like(dl)
        rows = np.arange(len(dl))
        nl[rows, axis] = -np.sign(dl[rows, axis])
        return t, nl @ self.R.T

    def bound(self):
        """(centre, radius) of a sphere around the box."""
        return self.c, float(np.linalg.norm(self.half))

    def extent(self, direction):
        """Half the box's width along a unit direction."""
        return float(np.abs(self.R.T @ np.asarray(direction, float)) @ self.half)

    def ids(self, p, n):
        pl = (p - self.c) @ self.R
        nl = n @ self.R
        out = np.empty(len(p), np.int32)
        top, bottom = nl[:, 2] > 0.5, nl[:, 2] < -0.5
        side = ~(top | bottom)
        if top.any():
            out[top] = self.top.ids(pl[top, 0], pl[top, 1])
        if bottom.any():
            out[bottom] = self.bottom.ids(pl[bottom, 0], -pl[bottom, 1])
        if side.any():
            # along the face: +y on the +x face, going round the box anticlockwise seen from above
            ps, ns = pl[side], nl[side]
            along = np.where(np.abs(ns[:, 0]) > 0.5, np.sign(ns[:, 0]) * ps[:, 1], -np.sign(ns[:, 1]) * ps[:, 0])
            out[side] = self.sides.ids(along, ps[:, 2])
        return out

    def materials(self):
        return self.top.materials() | self.sides.materials() | self.bottom.materials()

    def to_dict(self):
        return dict(type="box", name=self.name, centre=self.c.tolist(), size=(2 * self.half).tolist(),
                    rotation=self.R.tolist(), top=self.top.to_dict(), sides=self.sides.to_dict(),
                    bottom=self.bottom.to_dict())


class Sphere:
    """A ball; texture coordinates are (longitude, latitude) times the radius."""

    def __init__(self, centre, radius, texture, name="sphere"):
        self.c = np.asarray(centre, float)
        self.r = float(radius)
        self.texture, self.name = texture, name

    def intersect(self, o, d):
        oc = o - self.c
        b = np.einsum("ij,ij->i", oc, d)
        cq = np.einsum("ij,ij->i", oc, oc) - self.r ** 2
        disc = b * b - cq
        ok = disc >= 0
        t = np.full(len(o), np.inf)
        t0 = -b[ok] - np.sqrt(disc[ok])
        t[ok] = np.where(t0 > EPS, t0, np.inf)
        p = o + np.where(np.isfinite(t), t, 0.0)[:, None] * d
        return t, (p - self.c) / self.r

    def bound(self):
        return self.c, self.r

    def extent(self, direction):
        return self.r

    def ids(self, p, n):
        q = p - self.c
        lon = np.arctan2(q[:, 1], q[:, 0])
        lat = np.arcsin(np.clip(q[:, 2] / self.r, -1, 1))
        return self.texture.ids(lon * self.r, lat * self.r)

    def materials(self):
        return self.texture.materials()

    def to_dict(self):
        return dict(type="sphere", name=self.name, centre=self.c.tolist(), radius=self.r,
                    texture=self.texture.to_dict())


# -- lighting ----------------------------------------------------------------------------------


@dataclass
class Uniform:
    """The same light on every surface, whatever its direction: a light tent."""

    def shading(self, scene, p, n):
        return np.ones(len(p))

    def to_dict(self):
        return dict(type="uniform")


@dataclass
class Lamp:
    """A small lamp at `position`: light falls as 1/d^2 and with the cosine of the angle of
    incidence; solids cast hard shadows; `ambient` adds a fraction of light from everywhere.
    Normalised to 1 on a level surface at `target`."""
    position: tuple
    target: tuple
    ambient: float = 0.05

    def shading(self, scene, p, n):
        lamp, target = np.asarray(self.position, float), np.asarray(self.target, float)
        L = lamp - p
        dist = np.linalg.norm(L, axis=1)
        l = L / dist[:, None]
        cos = np.clip(np.einsum("ij,ij->i", n, l), 0.0, None)
        L0 = lamp - target
        d0 = np.linalg.norm(L0)
        ref = (L0[2] / d0) / d0 ** 2
        direct = cos / dist ** 2 / ref
        lit = cos > 0
        if lit.any():
            t, _, _ = scene.nearest(p[lit] + 1e-6 * n[lit], l[lit])
            blocked = t < dist[lit] - 1e-6
            direct[np.flatnonzero(lit)[blocked]] = 0.0
        return self.ambient + (1 - self.ambient) * direct

    def to_dict(self):
        return dict(type="lamp", position=list(self.position), target=list(self.target), ambient=self.ambient)


# -- the scene ----------------------------------------------------------------------------------


@dataclass
class Hits:
    t: np.ndarray        # distance along the ray (inf: nothing hit)
    solid: np.ndarray    # index of the solid hit, -1 for none
    point: np.ndarray
    normal: np.ndarray   # unit, facing the ray
    material: np.ndarray  # material id (spectra.NAMES)
    shading: np.ndarray  # light on the surface, 1 = a level surface at the target


class Scene:
    def __init__(self, solids, lighting=None, name="scene", description="", target=(0.0, 0.0, 0.0)):
        self.solids = list(solids)
        self.lighting = lighting or Uniform()
        self.name, self.description = name, description
        self.target = tuple(float(v) for v in target)

    def nearest(self, o, d, only=None):
        """(t, solid index, normal) of the first surface each ray meets. `only`: indices of
        the solids that can be hit, when the caller knows the others are out of reach."""
        o, d = np.asarray(o, float), np.asarray(d, float)
        t = np.full(len(o), np.inf)
        which = np.full(len(o), -1, np.int32)
        normal = np.zeros_like(d)
        for k in range(len(self.solids)) if only is None else only:
            tk, nk = self.solids[k].intersect(o, d)
            closer = tk < t
            t[closer], which[closer], normal[closer] = tk[closer], k, nk[closer]
        return t, which, normal

    def cast(self, o, d, only=None):
        """Hits for rays from origins o along unit directions d (both N x 3)."""
        o, d = np.asarray(o, float), np.asarray(d, float)
        t, which, n = self.nearest(o, d, only)
        hit = which >= 0
        p = o + np.where(hit, t, 0.0)[:, None] * d
        n = np.where((np.einsum("ij,ij->i", n, d) > 0)[:, None], -n, n)
        mat = np.full(len(o), spectra.material_id("void"), np.int32)
        for k, s in enumerate(self.solids):
            m = which == k
            if m.any():
                mat[m] = s.ids(p[m], n[m])
        shade = np.zeros(len(o))
        if hit.any():
            shade[hit] = self.lighting.shading(self, p[hit], n[hit])
        return Hits(t, which, p, n, mat, shade)

    def materials(self):
        names = set()
        for s in self.solids:
            names |= s.materials()
        return sorted(names, key=spectra.NAMES.index)

    def to_dict(self):
        return dict(name=self.name, description=self.description, target=list(self.target),
                    lighting=self.lighting.to_dict(), materials=self.materials(),
                    solids=[s.to_dict() for s in self.solids])
