from glob import glob
import os
import sys

from setuptools import find_packages, setup
from setuptools.command.build_py import build_py
from pathlib import Path

from vehicle_common.mode_loader import mode_registry

UAV_MODE_REGISTRY_PATH = Path(__file__).parent / "uav" / "uav_mode_registry.json"


class BuildUAVModeRegistry(build_py):
    def run(self):
        print("RUNNING UAV CUSTOM BUILD STEP")
        try:
            mode_registry.discover_modes(["uav.modes"])
            mode_registry.write_json(UAV_MODE_REGISTRY_PATH)
        except Exception as e:
            sys.exit(f"uav mode registry build failed: {e}")
        build_py.run(self)  # continue normal build


package_name = "uav"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    cmdclass={"build_py": BuildUAVModeRegistry},  # define our custom build step here
    package_data={
        package_name: ["missions/*.yaml", "fleets/*.yaml", "uav_mode_registry.json"]
    },
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
            os.path.join("share", package_name, "launch", "ci"),
            glob(os.path.join("launch", "ci", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "config", "camera_calibrations"),
            glob(os.path.join("config", "camera_calibrations", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "missions"),
            glob(os.path.join("missions", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "fleets"),
            glob(os.path.join("fleets", "*.yaml")),
        ),
        (
            os.path.join("share", package_name),
            [str(UAV_MODE_REGISTRY_PATH)],
        ),
    ],
    install_requires=["setuptools", "apriltag", "pydantic>=2,<3"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "uav_mission = uav.uav_mission:main",
            "payload_tracking_node = uav.vision_nodes.PayloadTrackingNode:main",
            "payload_april_tag_node = uav.vision_nodes.PayloadAprilTagNode:main",
            "payload_color_orbit_node = uav.vision_nodes.PayloadColorOrbitNode:main",
            "payload_drive_out_node = uav.vision_nodes.PayloadDriveOutNode:main",
            "payload_color_square_node = uav.vision_nodes.PayloadColorSquareNode:main",
            "camera = uav.CameraNode:main",
        ],
    },
)
