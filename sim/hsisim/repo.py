"""The rest of the repo, imported rather than copied.

The simulator reuses three parts of the repo, so it can't drift from them:

  calibration/hsical            the synthetic spectrograph and IMX219 (hsical.synth), the
                                design map traced in Optiland, and the calibration itself
  ros2/so101_scan_description   kinematics.py: forward kinematics straight from the URDF
  ros2/so101_scan_sweep         line_log.py and plan.py: the lines.csv writer and scan plans

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

_PATHS = (CALIBRATION, ROS2 / "so101_scan_description", ROS2 / "so101_scan_sweep")


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


@lru_cache(maxsize=4)
def robot_description(use_mock_hardware=True):
    """The URDF that robot_state_publisher would publish, as an XML string.

    Mock hardware by default: the kinematics are the same either way, and the simulated
    arm has no servo bus."""
    import xacro

    doc = xacro.process_file(str(XACRO), mappings={"use_mock_hardware": str(use_mock_hardware).lower()})
    return doc.toprettyxml(indent="  ")


def robot():
    """so101_scan_description.kinematics.Robot built from robot_description()."""
    from so101_scan_description.kinematics import Robot

    return Robot(robot_description())
