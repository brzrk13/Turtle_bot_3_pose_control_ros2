import os
from glob import glob

from setuptools import find_packages, setup

package_name = "pose_controller"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="green",
    maintainer_email="green6dine@gmail.com",
    description="Closed-loop pose controller for TurtleBot3 (MoveToPose service).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pose_controller_node = pose_controller.pose_controller_node:main",
            "move_client = pose_controller.move_client:main",
        ],
    },
)
