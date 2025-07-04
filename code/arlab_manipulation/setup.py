from setuptools import find_packages, setup

package_name = "arlab_manipulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/srv", ["src/arlab/code/arlab_manipulation/srv/GetGrippingForce.srv"]),
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
            "MoveItGoalPublisher = arlab_manipulation.MoveItGoalPublisher:main",
            "GetGrippingForce = arlab_manipulation.GetGrippingForce:main",
        ],
    },
)

