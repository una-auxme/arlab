from setuptools import find_packages, setup

package_name = "arlab_asyncio_executor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="peter",
    maintainer_email="peter.viechter@student.uni-augsburg.de",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
