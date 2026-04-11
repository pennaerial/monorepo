from setuptools import find_packages, setup
import os
from glob import glob

package_name = "uav"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["missions/*.yaml", "fleets/*.yaml"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "missions"),
            glob(os.path.join(package_name, "missions", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "fleets"),
            glob(os.path.join(package_name, "fleets", "*.yaml")),
        ),
    ],
    install_requires=["setuptools", "apriltag"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "uav_mission = uav.runtime.uav_mission:main",
            "payload_mission = uav.runtime.payload_mission:main",
            "payload_tracking_node = uav.vision_nodes.PayloadTrackingNode:main",
            "payload_april_tag_node = uav.vision_nodes.PayloadAprilTagNode:main",
            "payload_color_orbit_node = uav.vision_nodes.PayloadColorOrbitNode:main",
            "payload_drive_out_node = uav.vision_nodes.PayloadDriveOutNode:main",
            "payload_color_square_node = uav.vision_nodes.PayloadColorSquareNode:main",
            "camera = uav.CameraNode:main",
        ],
    },
)
