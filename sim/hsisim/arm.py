"""Where the head really is for each scan line, and what the arm and the mirror report.

Poses come from the URDF (ros2/so101_scan_description) through its own forward
kinematics, as scan_sweep computes them, so lines.csv here holds what the real scan would
log. The truth differs from the log the way the rig's will:

  servo calibration   each joint reads a fixed offset from its true angle
  servo readings      the STS3215s report whole encoder ticks (4096 a turn), with jitter
  arm flex            the printed links sag and the gears have play: the head sits a
                      little off where the joints say, differently at every viewpoint
  mirror homing       the hall sensor homes the mirror a fixed small angle off
  mirror steps        each microstep lands a little off its nominal angle

The splat trainer corrects one head pose per sweep, which covers all of these, so a
simulated scan tests exactly that. errors=0 turns them all off, the encoders' rounding too,
so the log is the truth.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from . import repo  # noqa: F401
from so101_scan_description.kinematics import axis_angle, transform  # noqa: E402
from so101_scan_sweep.line_log import LinePoser  # noqa: E402
from so101_scan_sweep.plan import ARM_JOINTS  # noqa: E402

TICK = 2 * np.pi / 4096  # STS3215 encoder step, rad


@dataclass
class PoseErrors:
    joint_offset_deg: float = 0.25   # servo calibration, per joint (1 sigma)
    joint_noise_ticks: float = 0.3   # reading jitter, encoder ticks
    head_mm: float = 0.5             # arm flex at each viewpoint, per axis
    head_deg: float = 0.2
    mirror_home_deg: float = 0.05    # homing offset
    mirror_step_deg: float = 0.005   # per line
    ticks: bool = True               # the servos report whole encoder ticks

    def scaled(self, k):
        out = {key: v * k for key, v in asdict(self).items() if key != "ticks"}
        return PoseErrors(**out, ticks=self.ticks and k > 0)


class Arm:
    """The simulated arm and mirror: true poses, and what the rig would log."""

    def __init__(self, robot, errors: PoseErrors, rng):
        self.robot = robot
        self.poser = LinePoser(robot)
        self.errors = errors
        self.rng = rng
        self.joint_offset = dict(zip(ARM_JOINTS, rng.normal(0.0, np.radians(errors.joint_offset_deg), len(ARM_JOINTS))))
        self.mirror_offset = float(rng.normal(0.0, np.radians(errors.mirror_home_deg)))

    def flex(self):
        """A fresh arm-flex error for one viewpoint: head frame -> where it really is."""
        e = self.errors
        rot = self.rng.normal(0.0, np.radians(e.head_deg), 3)
        angle = float(np.linalg.norm(rot))
        R = axis_angle(rot / angle, angle) if angle > 0 else np.eye(3)
        return transform(R, self.rng.normal(0.0, e.head_mm * 1e-3, 3))

    def read_joints(self, q_true):
        """The joint angles the servos report for true angles q_true."""
        out = {}
        for j in ARM_JOINTS:
            q = q_true[j] + self.joint_offset[j] + self.rng.normal(0.0, self.errors.joint_noise_ticks) * TICK
            out[j] = float(np.round(q / TICK) * TICK) if self.errors.ticks else float(q)
        return out

    def mirror_true(self, angle_logged):
        """The mirror's true angle when the log says angle_logged."""
        return angle_logged + self.mirror_offset + float(self.rng.normal(0.0, np.radians(self.errors.mirror_step_deg)))

    def poses(self, joints, mirror_angle, flex=None):
        """(head, line camera) 4 x 4 poses in base_link; flex moves the head (true poses)."""
        head = self.robot.fk(self.poser.head, dict(joints), self.poser.base)
        if flex is not None:
            head = head @ flex
        cam_in_head = self.robot.fk(self.poser.camera, {self.poser.mirror_joint: mirror_angle}, self.poser.head)
        return head, head @ cam_in_head

    def to_dict(self):
        return dict(errors=asdict(self.errors), joint_offset_rad=self.joint_offset,
                    mirror_offset_rad=self.mirror_offset)
