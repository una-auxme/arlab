from setuptools import find_packages, setup

package_name = "arlab_movement"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ('share/' + package_name + '/launch', ['launch/arlab_movement.launch.py']),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lukas Asam",
    maintainer_email="lukas.asam@uni-a.de",
    description="Package for AR Lab robot movement and navigation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["dummy = arlab_movement.dummy:main",
            "arlab_movement_test = arlab_movement.arlab_movement_test:main",
            "arlab_movement_orchestrator = arlab_movement.arlab_movement_orchestrator:main"
        ], 
    },
)
