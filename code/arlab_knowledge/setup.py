from setuptools import find_packages, setup

package_name = "arlab_knowledge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Peter Viechter, Daniel Gabler",
    maintainer_email="peter.viechter@uni-augsburg.de",
    description="Arlab knowledge base: database for handling "
    "the robot's surroundings, events and maps",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["database_node = arlab_knowledge.database_node:main"],
        "console_scripts": ["knowledge_visualization = arlab_knowledge.knowledge_visualization:main"],
    },
)
