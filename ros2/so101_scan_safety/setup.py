from setuptools import setup

package_name = "so101_scan_safety"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CoraBeanz",
    maintainer_email="vincentryanbaker@gmail.com",
    description="Safety tools for the SO-101 scan arm: plan checks, the e-stop and a safety-state watcher",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "check_plan = so101_scan_safety.check:main",
            "arm_estop = so101_scan_safety.estop:main",
        ],
    },
)
