from glob import glob

from setuptools import setup

package_name = "so101_scan_sweep"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/plans", glob("plans/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CoraBeanz",
    maintainer_email="vincentryanbaker@gmail.com",
    description="Scan sweeps for the SO-101 hyperspectral scanner",
    license="TODO",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "scan_mirror_bridge = so101_scan_sweep.bridge:main",
            "fake_scan_mirror = so101_scan_sweep.fake_mirror:main",
            "scan_sweep = so101_scan_sweep.sweep:main",
            "make_plan = so101_scan_sweep.make_plan:main",
            "move_arm = so101_scan_sweep.move_arm:main",
        ],
    },
)
