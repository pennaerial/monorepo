from glob import glob
import os

from setuptools import find_packages, setup

package_name = "udp_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yuzhiliu8",
    maintainer_email="yuzhiliu8@gmail.com",
    description="UDP middleware node bridging isolated ROS2 graphs across a local network",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "bridge_node = udp_bridge.bridge_node:main",
        ],
    },
)
