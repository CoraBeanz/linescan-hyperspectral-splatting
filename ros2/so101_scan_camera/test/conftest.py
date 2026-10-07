"""Fixtures the camera tests share."""

import os
import tempfile
from pathlib import Path

import pytest
import xacro

PKG = Path(__file__).resolve().parent.parent
DESCRIPTION = PKG.parent / "so101_scan_description"


@pytest.fixture(scope="session")
def urdf():
    """The scan arm's URDF from the source tree, whether or not the description package is installed."""
    prefix = Path(tempfile.mkdtemp())
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True)
    (index / "so101_scan_description").touch()
    (prefix / "share" / "so101_scan_description").symlink_to(DESCRIPTION)
    old = os.environ.get("AMENT_PREFIX_PATH", "")
    os.environ["AMENT_PREFIX_PATH"] = str(prefix) + os.pathsep + old
    try:
        return xacro.process_file(str(DESCRIPTION / "urdf" / "so101_scan.urdf.xacro"),
                                  mappings={"use_mock_hardware": "true"}).toxml()
    finally:
        os.environ["AMENT_PREFIX_PATH"] = old
