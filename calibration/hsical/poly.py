"""Small polynomial models with robust weighted fits, stored as plain JSON.

Coordinates are normalised to about -1..1 before the powers are taken, so the
coefficients stay well conditioned and are easy to port to C++ or CUDA:

    value = sum_k c[k] * u**i_k * v**j_k,   u = (x - x0) / xs,   v = (y - y0) / ys
"""

from __future__ import annotations

import numpy as np


class Poly2D:
    """sum of c * u^i * v^j over a fixed list of (i, j) terms."""

    def __init__(self, terms, x0=0.0, xs=1.0, y0=0.0, ys=1.0, coef=None):
        self.terms = [tuple(int(k) for k in t) for t in terms]
        self.x0, self.xs, self.y0, self.ys = float(x0), float(xs), float(y0), float(ys)
        self.coef = np.zeros(len(self.terms)) if coef is None else np.asarray(coef, float)

    def _uv(self, x, y):
        return (np.asarray(x, float) - self.x0) / self.xs, (np.asarray(y, float) - self.y0) / self.ys

    def basis(self, x, y):
        u, v = self._uv(x, y)
        u, v = np.broadcast_arrays(u, v)
        return np.stack([u ** i * v ** j for i, j in self.terms], axis=-1)

    def __call__(self, x, y):
        u, v = self._uv(x, y)
        out = np.zeros(np.broadcast(u, v).shape)
        for c, (i, j) in zip(self.coef, self.terms):
            out = out + c * u ** i * v ** j
        return out

    def dx(self, x, y):
        u, v = self._uv(x, y)
        out = np.zeros(np.broadcast(u, v).shape)
        for c, (i, j) in zip(self.coef, self.terms):
            if i:
                out = out + c * i * u ** (i - 1) * v ** j
        return out / self.xs

    def dy(self, x, y):
        u, v = self._uv(x, y)
        out = np.zeros(np.broadcast(u, v).shape)
        for c, (i, j) in zip(self.coef, self.terms):
            if j:
                out = out + c * j * u ** i * v ** (j - 1)
        return out / self.ys

    def fit(self, x, y, z, sigma=None, clip=4.0, iters=5):
        """Weighted least squares with iterative outlier rejection.

        Returns the boolean mask of points kept."""
        z = np.asarray(z, float)
        sigma = np.ones_like(z) if sigma is None else np.asarray(sigma, float)
        keep = np.isfinite(z) & np.isfinite(sigma) & (sigma > 0)
        A = self.basis(x, y)
        for _ in range(iters):
            w = 1.0 / sigma[keep]
            self.coef = np.linalg.lstsq(A[keep] * w[:, None], z[keep] * w, rcond=None)[0]
            r = (A @ self.coef - z) / sigma
            scale = 1.4826 * np.median(np.abs(r[keep])) if keep.sum() > len(self.terms) else 1.0
            new = keep & (np.abs(r) <= clip * max(scale, 1.0))
            if new.sum() == keep.sum():
                break
            keep = new
        return keep

    def to_dict(self):
        return dict(terms=[list(t) for t in self.terms], x0=self.x0, xs=self.xs, y0=self.y0,
                    ys=self.ys, coef=[float(c) for c in self.coef])

    @classmethod
    def from_dict(cls, d):
        return cls(d["terms"], d["x0"], d["xs"], d["y0"], d["ys"], d["coef"])


class Poly1D(Poly2D):
    """A Poly2D in x only (j = 0 everywhere), with a numerical inverse."""

    def __init__(self, degree=3, x0=0.0, xs=1.0, coef=None):
        super().__init__([(i, 0) for i in range(degree + 1)], x0, xs, 0.0, 1.0, coef)

    @property
    def degree(self):
        return len(self.terms) - 1

    def __call__(self, x, y=0.0):
        return super().__call__(x, 0.0)

    def deriv(self, x):
        return self.dx(x, 0.0)

    def fit(self, x, z, sigma=None, clip=4.0, iters=5):
        return super().fit(x, np.zeros_like(np.asarray(x, float)), z, sigma, clip, iters)

    def inverse(self, z, lo, hi, n=8001):
        """x where the polynomial equals z, searched over [lo, hi] (monotone there)."""
        xs = np.linspace(lo, hi, n)
        zs = self(xs)
        if zs[-1] < zs[0]:
            xs, zs = xs[::-1], zs[::-1]
        x = np.interp(z, zs, xs)
        for _ in range(3):  # polish with Newton steps
            x = x - (self(x) - z) / self.deriv(x)
        return x

    def to_dict(self):
        return dict(degree=self.degree, x0=self.x0, xs=self.xs, coef=[float(c) for c in self.coef])

    @classmethod
    def from_dict(cls, d):
        return cls(d["degree"], d["x0"], d["xs"], d["coef"])
