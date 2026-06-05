from glob import glob

from setuptools import find_packages, setup

package_name = "blk360_stop_scan"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kkk3449",
    maintainer_email="kimsang.m@g.skku.edu",
    description="BLK360 stop-scan sequencer for active mapping.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stop_scan_sequencer = blk360_stop_scan.stop_scan_sequencer:main",
            "mock_blk360_scanner = blk360_stop_scan.mock_blk360_scanner:main",
            "exploration_monitor = blk360_stop_scan.exploration_monitor:main",
        ],
    },
)
