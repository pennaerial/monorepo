import os
from glob import glob

from generate_parameter_library_py.setup_helper import generate_parameter_module
from setuptools import find_packages, setup

package_name = "payload"

generate_parameter_module(
    "payload_mission_parameters",
    "payload/payload_mission_parameters.yaml",
)

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
        (
            os.path.join("share", package_name, "missions"),
            glob(os.path.join("missions", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf", "*.urdf.xacro")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yuzhiliu8",
    maintainer_email="yuzhiliu8@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "payload_mission = payload.payload_mission:main",
        ],
    },
)
