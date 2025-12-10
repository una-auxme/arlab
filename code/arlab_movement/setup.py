import os
from glob import glob
from setuptools import find_packages, setup

package_name = "arlab_movement"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ('share/' + package_name + '/launch', ['launch/deprecated_arlab_movement.launch.py']),
        ('share/' + package_name + '/launch', ['launch/deprecated_arlab_simulation.launch.py']),
        ('share/' + package_name + '/launch', ['launch/custom_spawn_turtlebot3.launch.py']),
        ('share/' + package_name + '/launch', ['launch/nav2.launch.py']),
        ('share/' + package_name + '/launch', ['launch/slam_async.launch.py']),
        ('share/' + package_name + '/launch', ['launch/arlab_navigation.launch.py']),
        ('share/' + package_name + '/launch', ['launch/map_load_nav.launch.py']),
        # entries for arlab_simulation
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
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
            "map_save_publisher = arlab_movement.map_save_publisher:main",
            "arlab_movement_test = arlab_movement.arlab_movement_test:main",
            "arlab_movement_orchestrator = arlab_movement.arlab_movement_orchestrator:main",
            "navigation_stack_manager = arlab_movement.navigation_stack_manager:main",
        ],
    },
)
