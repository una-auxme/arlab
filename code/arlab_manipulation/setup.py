"""
Setup script for the arlab_manipulation ROS2 Python package.

This script defines package metadata, dependencies, launch files, and
console scripts for ROS2 execution.

Maintainer:
    Sofia Öttl <sofia.oettl@uni-a.de>
"""

from glob import glob

from setuptools import setup

package_name = "arlab_manipulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sofia Öttl",
    maintainer_email="sofia.oettl@uni-a.de",
    description=(
        "ARLab Manipulation stack: Python nodes, services, and actions "
        "for pick-and-place orchestration using octomap and gripping parameters."
        ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "orchestrator = arlab_manipulation.orchestrator:main",
            "gripping_parameter = arlab_manipulation.services.gripping_parameter:main",
        ],
    },
)
