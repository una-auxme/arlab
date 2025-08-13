from setuptools import find_packages, setup

package_name = "arlab_templates"

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
            "template_node = arlab_templates.template_node:main",
            "template_pubsub = arlab_templates.template_pubsub:main",
            "template_service_client = arlab_templates.template_service_client:main",
            "template_service = arlab_templates.template_service:main",
            "template_local_safety.py = arlab_templates.template_local_safety:main",
        ],
    },
)
