from setuptools import find_packages, setup
from glob import glob

package_name = "arlab_manipulation"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
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
            "PosePublisher = arlab_manipulation.PosePublisher:main",
            "GetGrippingForce = arlab_manipulation.GetGrippingForce:main",
        ],
    },
)
