from setuptools import find_packages, setup

package_name = "arlab_dialog"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/dialog_settings.yaml"]),
        ("share/" + package_name + "/launch", ["launch/dialog.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="arlab",
    maintainer_email="tobias.neher@uni-a.de",
    description="todo",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "dialog = arlab_dialog.dialog_node:main",
        ],
    },
)
