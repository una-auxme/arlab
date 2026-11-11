"""ROS 2 ament_python setup for the rebuilt decision-making package."""

import os
from setuptools import find_packages, setup

package_name = "arlab_decision_making"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "architecture.svg"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Arlab Team",
    maintainer_email="maintainers@invalid.example",
    description="Prototype HFSM supervisor with orthogonal monitors and py_trees tasks",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "decision_maker = arlab_decision_making.main:main",
            "decision_maker_demo = arlab_decision_making.demo:main",
        ],
    },
)
