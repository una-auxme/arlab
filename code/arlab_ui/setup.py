import os
from glob import glob

from setuptools import find_packages, setup

package_name = "arlab_ui"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name + "/config",
            ["config/buttons.json"],
        ),
        (
            "share/" + package_name + "/launch",
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    include_package_data=True,
    package_data={
        package_name: ["camera_placeholder.png"],
    },
    zip_safe=True,
    maintainer="ARLab HRI Team",
    maintainer_email="tobias.neher@uni-a.de",
    description="HRI display UI prototype for Zirbi.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "zirbi_display = arlab_ui.zirbi_display:main",
        ],
    },
)
