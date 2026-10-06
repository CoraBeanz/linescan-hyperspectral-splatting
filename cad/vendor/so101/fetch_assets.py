"""Download the SO-101 follower meshes that so101_new_calib.urdf points at.

The meshes come from TheRobotStudio's SO-ARM100 repository (Apache-2.0),
folder Simulation/SO101/assets. They are not committed here because they are
17 MB; the decimated copies inside ../../so101_hsi_rig.FCStd are enough to view
the assembly. Run this before cad/build_rig.py:

    python3 cad/vendor/so101/fetch_assets.py

Each file is checked against the SHA-256 it had when the rig was modelled
(main branch, 2026-10-06), so a changed upstream file is reported, not used
silently.
"""

import hashlib
import sys
import urllib.request
from pathlib import Path

BASE = "https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/Simulation/SO101/assets/"
SHA256 = {
    "base_motor_holder_so101_v1.stl": "8cd2f241037ea377af1191fffe0dd9d9006beea6dcc48543660ed41647072424",
    "base_so101_v2.stl": "bb12b7026575e1f70ccc7240051f9d943553bf34e5128537de6cd86fae33924d",
    "motor_holder_so101_base_v1.stl": "31242ae6fb59d8b15c66617b88ad8e9bded62d57c35d11c0c43a70d2f4caa95b",
    "motor_holder_so101_wrist_v1.stl": "887f92e6013cb64ea3a1ab8675e92da1e0beacfd5e001f972523540545e08011",
    "moving_jaw_so101_v1.stl": "785a9dded2f474bc1d869e0d3dae398a3dcd9c0c345640040472210d2861fa9d",
    "rotation_pitch_so101_v1.stl": "9be900cc2a2bf718102841ef82ef8d2873842427648092c8ed2ca1e2ef4ffa34",
    "sts3215_03a_no_horn_v1.stl": "75ef3781b752e4065891aea855e34dc161a38a549549cd0970cedd07eae6f887",
    "sts3215_03a_v1.stl": "a37c871fb502483ab96c256baf457d36f2e97afc9205313d9c5ab275ef941cd0",
    "under_arm_so101_v1.stl": "d01d1f2de365651dcad9d6669e94ff87ff7652b5bb2d10752a66a456a86dbc71",
    "upper_arm_so101_v1.stl": "475056e03a17e71919b82fd88ab9a0b898ab50164f2a7943652a6b2941bb2d4f",
    "waveshare_mounting_plate_so101_v2.stl": "e197e24005a07d01bbc06a8c42311664eaeda415bf859f68fa247884d0f1a6e9",
    "wrist_roll_follower_so101_v1.stl": "4b17b410a12d64ec39554abc3e8054d8a97384b2dc4a8d95a5ecb2a93670f5f4",
    "wrist_roll_pitch_so101_v2.stl": "6c7ec5525b4d8b9e397a30ab4bb0037156a5d5f38a4adf2c7d943d6c56eda5ae",
}

ASSETS = Path(__file__).resolve().parent / "assets"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fetch(force=False):
    ASSETS.mkdir(exist_ok=True)
    bad = []
    for name, digest in SHA256.items():
        dst = ASSETS / name
        if force or not dst.exists() or sha256(dst) != digest:
            print("downloading", name)
            with urllib.request.urlopen(BASE + name, timeout=60) as r:
                dst.write_bytes(r.read())
        if sha256(dst) != digest:
            bad.append(name)
    if bad:
        print("SHA-256 mismatch (upstream changed?):", ", ".join(bad))
        return False
    print("SO-101 meshes ready in", ASSETS)
    return True


if __name__ == "__main__":
    sys.exit(0 if fetch("--force" in sys.argv) else 1)
