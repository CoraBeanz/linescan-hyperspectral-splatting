"""Rigid transforms as 4x4 numpy arrays, and the small conversions the rest of headcal needs.

A pose maps local coordinates into its parent frame: p_parent = T @ [p_local, 1], the URDF's
and the trainer's convention. Small changes are written as a 6-vector (rho, phi) applied in
the pose's own frame: T @ exp(rho, phi), with phi a rotation vector.
"""

from __future__ import annotations

import math

import numpy as np


def skew(w):
    return np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])


def rot_exp(w):
    """Rotation matrix of the rotation vector w (axis times angle, Rodrigues)."""
    w = np.asarray(w, float)
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3) + skew(w)
    k = skew(w / th)
    return np.eye(3) + math.sin(th) * k + (1.0 - math.cos(th)) * k @ k


def rot_log(r):
    """Rotation vector of a rotation matrix (inverse of rot_exp), stable near 0 and 180 deg."""
    r = np.asarray(r, float)
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    s = 0.5 * np.linalg.norm(v)
    c = 0.5 * (np.trace(r) - 1.0)
    th = math.atan2(s, c)
    if th < 1e-9:
        return 0.5 * v
    if s < 1e-6:
        i = int(np.argmax(np.diag(r)))
        a = np.zeros(3)
        a[i] = math.sqrt(max(0.0, 0.5 * (r[i, i] + 1.0)))
        for j in range(3):
            if j != i:
                a[j] = (r[j, i] + r[i, j]) / (4.0 * a[i])
        a /= np.linalg.norm(a)
        if a @ v < 0:
            a = -a
        return th * a
    return th / (2.0 * s) * v


def rotation_about(axis, angle):
    a = np.asarray(axis, float)
    return rot_exp(a / np.linalg.norm(a) * angle)


def rot_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def make(r=None, t=None):
    out = np.eye(4)
    if r is not None:
        out[:3, :3] = r
    if t is not None:
        out[:3, 3] = t
    return out


def inv(t):
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return out


def exp6(xi):
    """4x4 transform of a 6-vector (rho, phi): a rotation by phi and then a shift by rho
    (rho is applied as is, not through the SE(3) left Jacobian: simpler, and all the same
    for parameter updates)."""
    xi = np.asarray(xi, float)
    return make(rot_exp(xi[3:]), xi[:3])


def log6(t):
    return np.concatenate([t[:3, 3], rot_log(t[:3, :3])])


def perturb(t, xi):
    """t changed by xi in its own frame."""
    return t @ exp6(xi)


def apply(t, p):
    """Transform points p (..., 3)."""
    p = np.asarray(p, float)
    return p @ t[:3, :3].T + t[:3, 3]


def angle_between(r_a, r_b):
    """Angle (rad) of the rotation that takes r_a to r_b."""
    c = 0.5 * (np.trace(np.asarray(r_a).T @ np.asarray(r_b)) - 1.0)
    return math.acos(max(-1.0, min(1.0, c)))


def rpy(r):
    """URDF roll, pitch, yaw of a rotation matrix: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    r = np.asarray(r, float)
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if math.cos(pitch) > 1e-9:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:  # gimbal lock: put it all in roll
        yaw = 0.0
        roll = math.atan2(-r[1, 2], r[1, 1]) if pitch > 0 else -math.atan2(-r[1, 2], r[1, 1])
    return roll, pitch, yaw


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def quat_wxyz(r):
    """Rotation matrix -> (w, x, y, z), w >= 0."""
    m = np.asarray(r, float)
    tr = np.trace(m)
    if tr > m[0, 0] and tr > m[1, 1] and tr > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + tr)
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.array(q) / np.linalg.norm(q)
    return q if q[0] >= 0 else -q


def quat_matrix(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def pose_array(t):
    """4x4 -> [tx, ty, tz, qw, qx, qy, qz] (the trainer's layout)."""
    return np.concatenate([np.asarray(t, float)[:3, 3], quat_wxyz(np.asarray(t)[:3, :3])])


def pose_from_array(v):
    return make(quat_matrix(v[3:7]), v[:3])


def mean_pose(poses, weights=None):
    """Mean of 4x4 poses: positions averaged, rotations by their sign-aligned quaternions."""
    poses = list(poses)
    w = np.ones(len(poses)) if weights is None else np.asarray(weights, float)
    t = np.average([p[:3, 3] for p in poses], axis=0, weights=w)
    qs = np.array([quat_wxyz(p[:3, :3]) for p in poses])
    qs[(qs @ qs[0]) < 0] *= -1
    q = np.average(qs, axis=0, weights=w)
    return make(quat_matrix(q), t)


def kabsch(src, dst, weights=None):
    """The rigid transform T minimising sum w |T src - dst|^2 (points as rows)."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    w = np.ones(len(src)) if weights is None else np.asarray(weights, float)
    w = w / w.sum()
    cs, cd = w @ src, w @ dst
    h = (src - cs).T @ ((dst - cd) * w[:, None])
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    return make(r, cd - r @ cs)
