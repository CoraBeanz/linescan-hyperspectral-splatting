from glob import glob

from setuptools import setup

package_name = "so101_scan_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CoraBeanz",
    maintainer_email="vincentryanbaker@gmail.com",
    description="The spectrograph camera of the SO-101 hyperspectral scanner",
    license="TODO",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "line_camera = so101_scan_camera.camera_node:main",
            "capture_reference = so101_scan_camera.capture_reference:main",
            "scan_to_dataset = so101_scan_camera.scan_to_dataset:main",
            "fake_calibration = so101_scan_camera.synthetic:main",
        ],
    },
)
