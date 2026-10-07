"""The line-splat trainer's instrument model in Python (splat/include/linesplat/scan_model.hpp),
for writing datasets it can read and checking them.

The head carries a fixed line camera (the objective and the slit) looking into the scan mirror.
The scene sees the camera reflected in the mirror, the "virtual camera", which turns twice as
fast as the mirror. The trainer gets the real camera's pose in the head and the mirror's axis
and face, and works out the virtual camera of every line from its mirror angle, the same way
virtual_camera_in_head() does here. Poses are 4x4 matrices mapping local coordinates to the
parent frame; on disk a pose is [tx, ty, tz, qw, qx, qy, qz].
"""

import math
from dataclasses import dataclass

import numpy as np

# splat's SlitOptics: the spectrograph numbers the blur of a binned pixel follows from
SLIT_LENGTH_MM = 5.0
SLIT_WIDTH_MM = 0.05
OPTICS_BLUR_MM = 0.008      # 1 sigma at the slit; a guess until measured


def rotation_about(axis, angle):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * kx @ kx


def quat_wxyz(r):
    """Rotation matrix -> (w, x, y, z) with w >= 0 (Shepperd, as splat's rotation_to_quat)."""
    m = np.asarray(r, float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
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


def quat_matrix(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def pose_array(t):
    """4x4 -> [tx, ty, tz, qw, qx, qy, qz]."""
    return np.concatenate([np.asarray(t, float)[:3, 3], quat_wxyz(np.asarray(t)[:3, :3])])


def pose_matrix(v):
    t = np.eye(4)
    t[:3, :3] = quat_matrix(*v[3:7])
    t[:3, 3] = v[:3]
    return t


@dataclass
class HeadModel:
    camera_in_head: np.ndarray    # 4x4: the real line camera (x along the slit, z the view)
    axis_point: np.ndarray        # a point on the mirror's axis, head frame (m)
    axis_dir: np.ndarray          # unit direction of the axis
    normal_at_zero: np.ndarray    # unit normal of the mirror's face at angle 0
    face_offset: float            # axis to face along the normal (m)

    def to_json(self):
        return dict(camera_in_head=[float(v) for v in pose_array(self.camera_in_head)],
                    mirror=dict(axis_point=[float(v) for v in self.axis_point],
                                axis_dir=[float(v) for v in self.axis_dir],
                                normal_at_zero=[float(v) for v in self.normal_at_zero],
                                face_offset=float(self.face_offset)))


def cad_head_model():
    """splat's cad_head_model(): the head as designed in cad/."""
    mm, s = 1e-3, math.sqrt(0.5)
    y_shaft, z_shaft = -2.3 * mm, 24.1 * mm
    mirror_e, mirror_to_obj = (2.5 + 1.6 + 3.0) * mm, 21.0 * mm
    cam = np.eye(4)
    cam[:3, :3] = np.column_stack([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    cam[:3, 3] = [0.0, y_shaft + mirror_e * s, z_shaft + mirror_e * s + mirror_to_obj]
    return HeadModel(cam, np.array([0.0, y_shaft, z_shaft]), np.array([1.0, 0.0, 0.0]), np.array([0.0, s, s]),
                     mirror_e)


def head_from_urdf(robot, head="scan_head_link", camera="spectrograph_optical_frame",
                   mirror_joint="scan_mirror_joint", face="scan_mirror_face"):
    """The head model from a URDF (so101_scan_description.kinematics.Robot): the objective as
    built, and the mirror's axis and face at joint zero."""
    cam = robot.fk(camera, {}, base=head)
    j = robot.joints[mirror_joint]
    if j["parent"] != head:
        raise ValueError("%s doesn't hang off %s" % (mirror_joint, head))
    axis_point = j["origin"][:3, 3].copy()
    axis_dir = j["origin"][:3, :3] @ np.asarray(j["axis"], float)
    f = robot.fk(face, {mirror_joint: 0.0}, base=head)
    normal = f[:3, 2] / np.linalg.norm(f[:3, 2])
    offset = float(normal @ (f[:3, 3] - axis_point))
    if offset < 0:
        normal, offset = -normal, -offset
    return HeadModel(cam, axis_point, axis_dir / np.linalg.norm(axis_dir), normal, offset)


def virtual_camera_in_head(head, angle):
    """The camera the scene sees at a mirror angle: the real one reflected in the mirror, with
    its y axis negated to keep it a proper rotation (splat's virtual_camera_in_head)."""
    n = rotation_about(head.axis_dir, angle) @ head.normal_at_zero
    n = n / np.linalg.norm(n)
    face = head.axis_point + head.face_offset * n
    cam = head.camera_in_head
    x = cam[:3, 0] - 2.0 * (n @ cam[:3, 0]) * n
    z = cam[:3, 2] - 2.0 * (n @ cam[:3, 2]) * n
    v = np.eye(4)
    v[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    v[:3, 3] = cam[:3, 3] - 2.0 * (n @ (cam[:3, 3] - face)) * n
    return v


def intrinsics(width, scene_distance_m, scan_line_half_length_m):
    """The trainer's LineIntrinsics for `width` pixels spread evenly along the slit: the slit's
    image at the scene distance spans the scan line, and the blur is the pixel and the slit's
    width (as boxes) plus the optics (splat's intrinsics_from_optics)."""
    pitch_mm = SLIT_LENGTH_MM / width
    blur = OPTICS_BLUR_MM / pitch_mm
    slit = SLIT_WIDTH_MM / pitch_mm
    return dict(width=int(width), f_px=width * scene_distance_m / (2.0 * scan_line_half_length_m),
                cu_px=0.5 * width, v_slit_px=0.0, sigma_u_px=math.sqrt(blur * blur + 1.0 / 12.0),
                sigma_v_px=math.sqrt(blur * blur + slit * slit / 12.0), near_m=0.01)


def pixel_rays(intr, camera_in_world, u=None):
    """Origin and unit directions (world) of the rays through pixel centres u (default: every
    pixel's), on the slit (v = v_slit)."""
    if u is None:
        u = np.arange(intr["width"]) + 0.5
    d = np.stack([(np.asarray(u, float) - intr["cu_px"]) / intr["f_px"],
                  np.full(np.shape(u), intr["v_slit_px"] / intr["f_px"]), np.ones(np.shape(u))], axis=-1)
    d = d @ camera_in_world[:3, :3].T
    return camera_in_world[:3, 3], d / np.linalg.norm(d, axis=-1, keepdims=True)
