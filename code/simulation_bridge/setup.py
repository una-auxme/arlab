from setuptools import find_packages, setup
"""Package setup for project.

colcon / ament_python use this file to build and install the package.
The key things defined here are:
  - which Python packages (sub-directories with __init__.py) to install,
  - which non-Python data files (launch files, config, maps) to install into
    the ROS 2 share directory so they can be found at runtime via
    ament_index_python.packages.get_package_share_directory(), and
  - the console script entry-point that creates the
    ``coverage_flight_node`` executable.
"""

import os

package_name = 'simulation_bridge'

# ---------------------------------------------------------------------------
# Data files
# ---------------------------------------------------------------------------
# Every file that is NOT a Python source must be listed here explicitly so
# that colcon copies it into the install/share tree.

data_files = [
    # Mandatory: register the package with the ament resource index.
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    # Mandatory: make package.xml accessible to ROS 2 tooling.
    ('share/' + package_name, ['package.xml']),
]

# Automatically collect all files from launch/, config/, and maps/
# directories so we don't have to update this list every time a file is added.
for directory in ('launch'):
    dir_path = os.path.join(os.path.dirname(__file__), directory)
    if not os.path.isdir(dir_path):
        continue
    # Collect every file in the directory (no recursive search needed here).
    files = [
        os.path.join(directory, filename)
        for filename in os.listdir(dir_path)
        if os.path.isfile(os.path.join(dir_path, filename))
    ]
    if files:
        data_files.append(('share/' + package_name + '/' + directory, files))

# ---------------------------------------------------------------------------
# Package definition
# ---------------------------------------------------------------------------

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wpc',
    maintainer_email='wpc@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mia_hand_bridge = simulation_bridge.mia_hand_bridge:main',
            'ur5e_bridge = simulation_bridge.ur5e_bridge:main'
        ],
    },
)
