from setuptools import setup

package_name = "semantic_nav_bt"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/mission_bt.launch.py"]),
    ],
    install_requires=["setuptools", "py_trees"],
    zip_safe=True,
    maintainer="Sangmin Kim",
    maintainer_email="kimsang.m@g.skku.edu",
    description="TOSM semantic-goal mission behavior tree",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_bt = semantic_nav_bt.mission_node:main",
            "semantic_cli = semantic_nav_bt.semantic_cli:main",
            "ui_server = semantic_nav_bt.ui_server:main",
            "mediator_server = semantic_nav_bt.mediator_server:main",
            "vda5050_master = semantic_nav_bt.vda5050_master:main",
            "vda5050_agv_adapter = semantic_nav_bt.vda5050_agv_adapter:main",
        ],
    },
)
