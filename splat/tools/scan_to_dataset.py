#!/usr/bin/env python3
"""Turn a scan folder from the rig into a dataset splat_train reads, without ROS.

    python3 splat/tools/scan_to_dataset.py ~/so101_scan/scans/ring_20261007-031500 ~/datasets/ring
    ./splat_train ~/datasets/ring ~/datasets/ring/train

The scan folder is what scan_sweep and line_camera write (lines.csv, scan.json, robot.urdf,
frames/, binned/, reference/). This runs ros2/so101_scan_camera's scan_to_dataset from this
checkout, which needs only numpy; --help lists its options, and its docstring says how values,
bands, pixels and poses are made.
"""

import os
import sys

ROS2 = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir, "ros2")
sys.path[:0] = [os.path.normpath(os.path.join(ROS2, p)) for p in ("so101_scan_camera", "so101_scan_description")]

from so101_scan_camera.scan_to_dataset import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
