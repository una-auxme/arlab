from setuptools import find_packages, setup

package_name = "arlab_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "video_node = arlab_perception.video_node:main",
            "camera_data = arlab_perception.camera:main",
            "lidar_data = arlab_perception.lidar:main",
        ],
    },
)
