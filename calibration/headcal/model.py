"""The scanner head's geometry: what the calibration estimates, how each camera sees a point, and
how the result is written back as URDF joint origins.

Frames (as in ros2/so101_scan_description/urdf/scan_head.xacro):

  wrist_roll_link -> scan_head_link      the mount: the head on the wrist-roll servo horn
  scan_head_link  -> mirror joint frame  at mirror angle 0: x along the shaft (the joint turns
                                         about it), y the mirror face's normal
  mirror link     -> mirror face         the face sits face_offset along the normal (+y)
  scan_head_link  -> spectrograph_optical_frame   the objective as built: z its view, x along
                                         the slit
  scan_head_link  -> pose_camera_optical_frame    the pose camera (OpenCV axes: x right, y down,
                                         z forward)

The line camera the scene sees is the objective reflected in the mirror. At mirror angle t
(the angle the bridge reports: 0 at the 45 degree rest) the mirror link is mirror @ Rx(t), so a
wrong home angle is just a turn of the mirror frame about x, and the calibration finds it as
that. The URDF writes the same reflection as a chain with a mimic joint; joint_origins() works
out that chain's fixed parts for any calibrated geometry.

A point X in the head frame lands on the slit at
    m = x/z, w = y/z  in the reflected camera's frame,   h = m/k + k1 (m/k)^3,
where h runs from -1 to +1 along the slit (binned pixel i of W covers h in
[2i/W - 1, 2(i+1)/W - 1]), k = scan_line_half_length / scene_distance (the tangent of half the
field along the slit), and k1 is the objective's distortion along the slit. A point is on the
line when w = 0 (the slit is centred in the objective's field: any offset is the same thing
as a turn of the mirror, and is found as one).
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, replace

import numpy as np

from . import geometry as g

FORMAT = "headcal head calibration v1"

# the joints whose origins a calibration sets, in the order the URDF has them
CALIBRATED_JOINTS = ("scan_head_mount", "scan_mirror_joint", "spectrograph_optical_joint",
                     "line_camera_fold_joint", "line_camera_optical_joint", "pose_camera_joint")
FLIP_Y = np.diag([1.0, -1.0, 1.0])


@dataclass
class HeadGeometry:
    mount: np.ndarray                 # wrist_roll_link -> scan_head_link
    mirror: np.ndarray                # scan_head_link -> mirror joint frame at angle 0
    face_offset: float                # shaft axis to the mirror face, along its normal (m)
    objective: np.ndarray             # scan_head_link -> spectrograph_optical_frame
    pose_camera: np.ndarray           # scan_head_link -> pose_camera_optical_frame
    pose_camera_lens: float = 0.0045  # pose_camera_link -> its optical frame, along z (m)
    scene_distance: float = 0.15      # objective to the in-focus scan line (m)
    half_line: float = 0.020937       # half the scan line's length at scene_distance (m)
    slit_k1: float = 0.0              # distortion along the slit (the trainer's pinhole has none)

    @property
    def k(self):
        return self.half_line / self.scene_distance

    def copy(self):
        return replace(self, mount=self.mount.copy(), mirror=self.mirror.copy(), objective=self.objective.copy(),
                       pose_camera=self.pose_camera.copy())

    # --- from a URDF ----------------------------------------------------------------------------

    @classmethod
    def from_urdf(cls, robot):
        """The geometry a URDF describes (so101_scan_description.kinematics.Robot)."""
        j = robot.joints
        if np.abs(np.asarray(j["scan_mirror_joint"]["axis"]) - [1, 0, 0]).max() > 1e-9:
            raise ValueError("scan_mirror_joint must turn about its own x axis")
        face = j["scan_mirror_face_joint"]["origin"]
        if np.abs(face[:3, 2] - [0, 1, 0]).max() > 1e-6 or np.abs(face[[0, 2], 3]).max() > 1e-9:
            raise ValueError("the mirror face must sit on the mirror link's +y, facing +y")
        line = j["scan_line_joint"]["origin"]
        box = robot.root.find("link[@name='scan_line_frame']/visual/geometry/box")
        lens = j["pose_camera_optical_joint"]["origin"][2, 3]
        return cls(mount=j["scan_head_mount"]["origin"].copy(),
                   mirror=j["scan_mirror_joint"]["origin"].copy(),
                   face_offset=float(face[1, 3]),
                   objective=robot.fk("spectrograph_optical_frame", {}, base="scan_head_link"),
                   pose_camera=robot.fk("pose_camera_optical_frame", {}, base="scan_head_link"),
                   pose_camera_lens=float(lens),
                   scene_distance=float(line[2, 3]),
                   half_line=float(box.get("size").split()[0]) / 2.0)

    # --- the line camera --------------------------------------------------------------------------

    def line_cameras(self, t):
        """Reflected (virtual) line cameras in the head frame at mirror angles t: (R (N,3,3),
        centres (N,3)). Their x runs along the slit and z looks at the scene."""
        t = np.atleast_1d(np.asarray(t, float))
        rm, pm = self.mirror[:3, :3], self.mirror[:3, 3]
        n = np.stack([np.zeros_like(t), np.cos(t), np.sin(t)], axis=-1) @ rm.T   # face normals
        face = pm + self.face_offset * n
        ro, po = self.objective[:3, :3], self.objective[:3, 3]

        def reflect(d):
            return d[None, :] - 2.0 * (n @ d)[:, None] * n

        x = reflect(ro[:, 0])
        z = reflect(ro[:, 2])
        y = np.cross(z, x)
        r = np.stack([x, y, z], axis=-1)
        c = po[None, :] - 2.0 * np.einsum("ij,ij->i", n, po[None, :] - face)[:, None] * n
        return r, c

    def line_camera(self, t):
        """4x4 virtual camera -> head at one mirror angle."""
        r, c = self.line_cameras([t])
        return g.make(r[0], c[0])

    def slit_project(self, x_head, t):
        """(h, w) of head-frame points x_head (N,3) seen at mirror angles t (N,): h along the slit
        (-1..1), w = y/z across it (0 on the line)."""
        r, c = self.line_cameras(t)
        p = np.einsum("nji,nj->ni", r, np.asarray(x_head, float) - c)
        m, w = p[:, 0] / p[:, 2], p[:, 1] / p[:, 2]
        hu = m / self.k
        return hu + self.slit_k1 * hu ** 3, w

    def slit_ray(self, h, t):
        """Unit direction in the head frame, and origin, of the ray through slit coordinate h at
        mirror angle t (arrays broadcast together)."""
        h, t = np.broadcast_arrays(np.asarray(h, float), np.asarray(t, float))
        hu = h.copy()
        for _ in range(4):  # invert h = hu + k1 hu^3
            hu = hu - (hu + self.slit_k1 * hu ** 3 - h) / (1.0 + 3.0 * self.slit_k1 * hu ** 2)
        r, c = self.line_cameras(t.ravel())
        d = np.stack([hu.ravel() * self.k, np.zeros(hu.size), np.ones(hu.size)], axis=-1)
        d = np.einsum("nij,nj->ni", r, d)
        d /= np.linalg.norm(d, axis=-1, keepdims=True)
        return d.reshape(h.shape + (3,)), c.reshape(h.shape + (3,))

    # --- written back to the URDF -------------------------------------------------------------------

    def joint_origins(self):
        """{joint: 4x4 origin} for CALIBRATED_JOINTS, as scan_head.xacro chains them: the mirror
        link turns by t about x, the fold joint (2 face_offset along the face normal) turns by t
        again (mimic), and line_camera_optical_joint is the objective reflected in the face at
        angle 0, in the mirror joint frame, with y flipped to stay right-handed."""
        o = g.inv(self.mirror) @ self.objective
        q = g.make(FLIP_Y @ o[:3, :3] @ FLIP_Y, FLIP_Y @ o[:3, 3])
        return {"scan_head_mount": self.mount,
                "scan_mirror_joint": self.mirror,
                "spectrograph_optical_joint": self.objective,
                "line_camera_fold_joint": g.make(t=[0.0, 2.0 * self.face_offset, 0.0]),
                "line_camera_optical_joint": q,
                "pose_camera_joint": self.pose_camera @ g.make(t=[0.0, 0.0, -self.pose_camera_lens])}

    def origin_strings(self):
        """{joint: (xyz, rpy)} written as the URDF writes them."""
        return {name: (fmt(t[:3, 3]), fmt(g.rpy(t[:3, :3]))) for name, t in self.joint_origins().items()}

    def calibration_yaml(self, header=""):
        """head_calibration.yaml for the xacro argument head_calibration."""
        lines = ["# " + ln if ln else "#" for ln in header.splitlines()]
        lines += ["format: %s" % FORMAT, "joints:"]
        for name, (xyz, rpy) in self.origin_strings().items():
            lines.append('  %s: {xyz: "%s", rpy: "%s"}' % (name, xyz, rpy))
        lines.append("face_offset: %.9g" % self.face_offset)
        lines.append("scan_line_half_length: %.9g" % self.half_line)
        lines.append("slit_k1: %.9g   # not in the URDF: the trainer's line camera is a pinhole" % self.slit_k1)
        return "\n".join(lines) + "\n"

    def patch_urdf(self, urdf_xml):
        """The URDF with this geometry: the calibrated joint origins and the scan line's length."""
        root = ET.fromstring(urdf_xml)
        for name, (xyz, rpy) in self.origin_strings().items():
            j = root.find("joint[@name='%s']" % name)
            if j is None:
                raise ValueError("the URDF has no joint %s" % name)
            o = j.find("origin")
            if o is None:
                o = ET.SubElement(j, "origin")
            o.set("xyz", xyz)
            o.set("rpy", rpy)
        face = root.find("joint[@name='scan_mirror_face_joint']/origin")
        face.set("xyz", fmt([0.0, self.face_offset, 0.0]))
        box = root.find("link[@name='scan_line_frame']/visual/geometry/box")
        size = box.get("size").split()
        box.set("size", " ".join([fmt([2.0 * self.half_line])] + size[1:]))
        return ET.tostring(root, encoding="unicode")

    def trainer_head_model(self):
        """The trainer's HeadModel (dataset.json "head"), in the head frame."""
        m = self.mirror
        return dict(camera_in_head=[float(v) for v in g.pose_array(self.objective)],
                    mirror=dict(axis_point=[float(v) for v in m[:3, 3]], axis_dir=[float(v) for v in m[:3, 0]],
                                normal_at_zero=[float(v) for v in m[:3, 1]], face_offset=float(self.face_offset)))

    def to_json(self):
        return dict(mount=g.pose_array(self.mount).tolist(), mirror=g.pose_array(self.mirror).tolist(),
                    face_offset=self.face_offset, objective=g.pose_array(self.objective).tolist(),
                    pose_camera=g.pose_array(self.pose_camera).tolist(), pose_camera_lens=self.pose_camera_lens,
                    scene_distance=self.scene_distance, half_line=self.half_line, slit_k1=self.slit_k1)

    @classmethod
    def from_json(cls, d):
        p = g.pose_from_array
        return cls(mount=p(d["mount"]), mirror=p(d["mirror"]), face_offset=d["face_offset"],
                   objective=p(d["objective"]), pose_camera=p(d["pose_camera"]),
                   pose_camera_lens=d["pose_camera_lens"], scene_distance=d["scene_distance"],
                   half_line=d["half_line"], slit_k1=d.get("slit_k1", 0.0))


def fmt(values):
    """Numbers as the URDF holds them: to the nanometre and nanoradian, no exponents."""
    out = []
    for v in values:
        s = "%.9f" % v
        s = s.rstrip("0").rstrip(".") if "." in s else s
        out.append("0" if s in ("-0", "") else s)
    return " ".join(out)


# --- the pose camera -----------------------------------------------------------------------------------

@dataclass
class PinholeCamera:
    """OpenCV's pinhole camera with plumb-bob distortion (k1, k2, p1, p2, k3); pixel centres
    at integer coordinates."""
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    dist: np.ndarray = field(default_factory=lambda: np.zeros(5))

    @property
    def matrix(self):
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    def project(self, p_cam):
        """Pixels (N,2) of camera-frame points (N,3)."""
        p = np.asarray(p_cam, float)
        x, y = p[:, 0] / p[:, 2], p[:, 1] / p[:, 2]
        k1, k2, p1, p2, k3 = self.dist
        r2 = x * x + y * y
        rad = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        xd = x * rad + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        yd = y * rad + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        return np.stack([self.fx * xd + self.cx, self.fy * yd + self.cy], axis=-1)

    def rays(self, uv):
        """Unit directions (N,3) in the camera frame through pixels uv (N,2)."""
        import cv2
        pts = cv2.undistortPoints(np.asarray(uv, np.float64).reshape(-1, 1, 2), self.matrix, np.asarray(self.dist),
                                  criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 1e-10))
        d = np.concatenate([pts.reshape(-1, 2), np.ones((len(pts), 1))], axis=1)
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    def to_json(self):
        return dict(width=self.width, height=self.height, fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
                    dist=[float(v) for v in self.dist])

    @classmethod
    def from_json(cls, d):
        return cls(d["width"], d["height"], d["fx"], d["fy"], d["cx"], d["cy"], np.asarray(d["dist"], float))

    def camera_info_yaml(self, name="pose_camera"):
        """ROS camera_info (camera_calibration_parsers' YAML)."""
        k = self.matrix.ravel()
        p = [self.fx, 0, self.cx, 0, 0, self.fy, self.cy, 0, 0, 0, 1, 0]

        def row(v):
            return "[" + ", ".join("%.9g" % x for x in v) + "]"

        return ("image_width: %d\nimage_height: %d\ncamera_name: %s\n"
                "camera_matrix:\n  rows: 3\n  cols: 3\n  data: %s\n"
                "distortion_model: plumb_bob\n"
                "distortion_coefficients:\n  rows: 1\n  cols: 5\n  data: %s\n"
                "rectification_matrix:\n  rows: 3\n  cols: 3\n  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
                "projection_matrix:\n  rows: 3\n  cols: 4\n  data: %s\n"
                % (self.width, self.height, name, row(k), row(self.dist), row(p)))


def slit_pixels(h, width):
    """Binned pixel coordinate (pixel i spans [i, i+1]) of slit coordinate h."""
    return (np.asarray(h) + 1.0) * width / 2.0


def slit_coordinate(u, width):
    return 2.0 * np.asarray(u) / width - 1.0


def deg(rad):
    return math.degrees(rad)
