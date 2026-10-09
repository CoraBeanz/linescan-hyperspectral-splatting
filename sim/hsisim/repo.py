"""The rest of the repo, imported rather than copied.

The simulator reuses five parts of the repo, so it can't drift from them:

  calibration/hsical            the synthetic spectrograph and IMX219 (hsical.synth), the
                                design map traced in Optiland, and the calibration itself
  calibration/headcal           the hand-eye calibration's tag board, the head's geometry and
                                how far a hand-built head is off it, and the pose camera's lens
                                and still format (only for the pose camera and --head-errors;
                                they need OpenCV)
  ros2/so101_scan_description   kinematics.py: forward kinematics straight from the URDF
  ros2/so101_scan_sweep         line_log.py, plan.py and frame_lock.py: the lines.csv writer, scan
                                plans, and how the mirror's line clock locks to the camera
  ros2/so101_scan_camera        sources.py, matcher.py and session.py: the camera's rolling-shutter
                                timing, which frames belong to which line, and the frames.csv columns

None of them needs ROS to import. robot.urdf is built from the ROS 2 package's xacro
files with the `xacro` package from PyPI. xacro resolves $(find pkg) through ROS 2's
ament_index_python, which isn't on PyPI; when it's missing, a stand-in maps this repo's
own ROS 2 packages to their source folders, which have the layout of an installed share
folder as far as the URDF needs (urdf/, meshes/).
"""

from __future__ import annotations

import importlib
import sys
import types
from functools import lru_cache
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
CALIBRATION = REPO / "calibration"
ROS2 = REPO / "ros2"
XACRO = ROS2 / "so101_scan_description" / "urdf" / "so101_scan.urdf.xacro"
PLANS = ROS2 / "so101_scan_sweep" / "plans"
HEADCAL_PLANS = CALIBRATION / "headcal" / "plans"

_PATHS = (CALIBRATION, ROS2 / "so101_scan_description", ROS2 / "so101_scan_sweep", ROS2 / "so101_scan_camera")


def _ament_stand_in():
    """A minimal ament_index_python.packages that finds the repo's ROS 2 packages."""

    def get_package_share_directory(name):
        path = ROS2 / name
        if not (path / "package.xml").is_file():
            raise LookupError(f"package '{name}' is not one of this repo's ROS 2 packages")
        return str(path)

    pkg = types.ModuleType("ament_index_python")
    packages = types.ModuleType("ament_index_python.packages")
    packages.get_package_share_directory = get_package_share_directory
    pkg.packages = packages
    pkg.__stand_in__ = True
    sys.modules["ament_index_python"] = pkg
    sys.modules["ament_index_python.packages"] = packages


def setup():
    """Make hsical, the ROS 2 packages' Python modules and ament_index_python importable."""
    for p in _PATHS:
        if str(p) not in sys.path:
            sys.path.append(str(p))
    try:
        importlib.import_module("ament_index_python.packages")
    except ImportError:
        _ament_stand_in()


setup()


def robot_description(use_mock_hardware=True, head_calibration=None):
    """The URDF that robot_state_publisher would publish, as an XML string.

    Mock hardware by default: the kinematics are the same either way, and the simulated
    arm has no servo bus. head_calibration: a head_calibration.yaml (headcal's) whose head
    replaces the CAD model's, as the launch file's head_calibration argument does."""
    if not head_calibration:
        return _cad_description(bool(use_mock_hardware))
    return _xacro({"use_mock_hardware": str(use_mock_hardware).lower(),
                   "head_calibration": str(Path(head_calibration).resolve())})


@lru_cache(maxsize=2)
def _cad_description(use_mock_hardware):
    return _xacro({"use_mock_hardware": str(use_mock_hardware).lower()})


def _xacro(mappings):
    import xacro

    return xacro.process_file(str(XACRO), mappings=mappings).toprettyxml(indent="  ")


def robot(head_calibration=None):
    """so101_scan_description.kinematics.Robot built from robot_description()."""
    from so101_scan_description.kinematics import Robot

    return Robot(robot_description(head_calibration=head_calibration))
