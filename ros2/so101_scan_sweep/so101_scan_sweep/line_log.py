"""Where the instrument was at each scan line, and the files a scan run writes.

The pose of a line comes from the arm's joint positions at the line's timestamp (interpolated
between /joint_states samples) and the mirror angle of the line, put through the URDF, the
same one robot_state_publisher uses for TF. lines.csv keeps the joint positions too, so the
poses can be recomputed later with a better calibration.
"""

import bisect
import csv
import math
import threading

import numpy as np

from so101_scan_sweep.plan import ARM_JOINTS

CSV_COLUMNS = (["viewpoint", "sweep_id", "index", "stamp_ns", "mirror_angle"]
               + ["head_" + k for k in ("x", "y", "z", "qx", "qy", "qz", "qw")]
               + ["cam_" + k for k in ("x", "y", "z", "qx", "qy", "qz", "qw")]
               + ARM_JOINTS)


def quaternion(r):
    """Rotation matrix -> (x, y, z, w), w >= 0."""
    m = np.asarray(r, float)
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    return tuple(q if q[3] >= 0 else -q)


class JointBuffer:
    """The last few seconds of arm joint positions, to read them at any instant in between."""

    def __init__(self, joints=ARM_JOINTS, seconds=10.0):
        self.joints = list(joints)
        self.keep_ns = int(seconds * 1e9)
        self.stamps = []
        self.values = []
        self.lock = threading.Lock()

    def add(self, stamp_ns, names, positions):
        """One /joint_states message; ignored unless it has every arm joint."""
        index = {n: i for i, n in enumerate(names)}
        if any(j not in index for j in self.joints) or len(positions) < len(names):
            return False
        row = [positions[index[j]] for j in self.joints]
        with self.lock:
            if self.stamps and stamp_ns <= self.stamps[-1]:
                return False
            self.stamps.append(stamp_ns)
            self.values.append(row)
            cut = bisect.bisect_left(self.stamps, stamp_ns - self.keep_ns)
            if cut:
                del self.stamps[:cut]
                del self.values[:cut]
        return True

    @property
    def latest_ns(self):
        with self.lock:
            return self.stamps[-1] if self.stamps else None

    def at(self, stamp_ns):
        """{joint: position} at stamp_ns and how far (ns) the nearest sample is; None if empty.
        Between two samples it interpolates; outside them it holds the nearest."""
        with self.lock:
            if not self.stamps:
                return None, None
            i = bisect.bisect_left(self.stamps, stamp_ns)
            if i == 0 or i == len(self.stamps):
                k = 0 if i == 0 else len(self.stamps) - 1
                return dict(zip(self.joints, self.values[k])), abs(self.stamps[k] - stamp_ns)
            t0, t1 = self.stamps[i - 1], self.stamps[i]
            a = (stamp_ns - t0) / float(t1 - t0)
            v = [(1 - a) * x0 + a * x1 for x0, x1 in zip(self.values[i - 1], self.values[i])]
            return dict(zip(self.joints, v)), min(stamp_ns - t0, t1 - stamp_ns)


class LinePoser:
    """Head and line-camera poses in base_link for a line, from the URDF."""

    def __init__(self, robot, base="base_link", head="scan_head_link", camera="line_camera_optical_frame",
                 mirror_joint="scan_mirror_joint"):
        self.robot = robot
        self.base, self.head, self.camera, self.mirror_joint = base, head, camera, mirror_joint
        line = robot.fk("scan_line_frame", {}, base=camera)
        self.scene_distance = float(line[2, 3])
        box = robot.root.find("link[@name='scan_line_frame']/visual/geometry/box")
        self.half_line = float(box.get("size").split()[0]) / 2 if box is not None else 0.02

    def poses(self, joints, mirror_angle):
        q = dict(joints)
        q[self.mirror_joint] = mirror_angle
        return self.robot.fk(self.head, q, self.base), self.robot.fk(self.camera, q, self.base)

    def line_ends(self, camera_pose):
        """The in-focus scan line's two ends in base_link."""
        out = []
        for x in (-self.half_line, self.half_line):
            out.append((camera_pose @ np.array([x, 0.0, self.scene_distance, 1.0]))[:3])
        return out


def pose_fields(t):
    return list(t[:3, 3]) + list(quaternion(t[:3, :3]))


class LinesCsv:
    def __init__(self, path):
        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(CSV_COLUMNS)
        self.rows = 0

    def write(self, viewpoint, sweep_id, index, stamp_ns, mirror_angle, head, camera, joints):
        self.writer.writerow(
            [viewpoint, sweep_id, index, stamp_ns, "%.9f" % mirror_angle]
            + ["%.9f" % v for v in pose_fields(head) + pose_fields(camera)]
            + ["%.9f" % joints[j] for j in ARM_JOINTS])
        self.rows += 1

    def close(self):
        self.file.close()
