"""The parts of this repo headcal reuses, imported rather than copied.

  ros2/so101_scan_description   kinematics.py: forward kinematics and IK straight from the URDF,
                                and the xacro files the URDF is built from

None of it needs ROS. A URDF is built with the `xacro` package from PyPI, which resolves
$(find pkg) through ament_index_python; when that isn't installed, a stand-in maps this repo's
own ROS 2 packages to their source folders (the layout an installed share folder has, as far
as the URDF needs). Solving reads the arm from the robot.urdf a scan folder holds, and builds
the CAD model's URDF from this checkout as the head's starting point.
"""

from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
ROS2 = REPO / "ros2"
DESCRIPTION = ROS2 / "so101_scan_description"
XACRO = DESCRIPTION / "urdf" / "so101_scan.urdf.xacro"

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def _ament_stand_in():
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
    if str(DESCRIPTION) not in sys.path:
        sys.path.append(str(DESCRIPTION))


setup()


def kinematics():
    """so101_scan_description.kinematics (Robot, solve_ik)."""
    return importlib.import_module("so101_scan_description.kinematics")


def robot(urdf_xml):
    return kinematics().Robot(urdf_xml)


def line_log():
    """so101_scan_sweep.line_log: the lines.csv writer and the line poser scan_sweep uses (for
    the synthetic scans; needs PyYAML)."""
    sweep = str(ROS2 / "so101_scan_sweep")
    if sweep not in sys.path:
        sys.path.append(sweep)
    return importlib.import_module("so101_scan_sweep.line_log")


def build_urdf(head_calibration=None):
    """The scan arm's URDF (mock hardware, as robot_state_publisher would publish it), from the
    xacro files in this checkout; with head_calibration, built with that head_calibration.yaml."""
    try:
        import xacro
    except ImportError as e:
        raise RuntimeError("building a URDF needs xacro: pip install xacro") from e
    try:
        importlib.import_module("ament_index_python.packages")
    except ImportError:
        _ament_stand_in()
    mappings = {"use_mock_hardware": "true"}
    if head_calibration:
        mappings["head_calibration"] = str(Path(head_calibration).resolve())
    return xacro.process_file(str(XACRO), mappings=mappings).toprettyxml(indent="  ")
