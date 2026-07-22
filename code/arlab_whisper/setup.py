from setuptools import find_packages, setup

package_name = "arlab_whisper"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/whisper.yaml"]),
        ("share/" + package_name + "/launch", ["launch/whisper.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="arlab",
    maintainer_email="tobias.neher@uni-a.de",
    description="This package allows speech recognition using Faster-Whisper with wakeword detection. It continuously monitors microphone input, detects configurable wakewords, transcribes them and publishes the result",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "whisper = arlab_whisper.whisper_node:main",
        ],
    },
)
