import os
from glob import glob

from setuptools import find_packages, setup
from setuptools.command.build_py import build_py
from pathlib import Path
import sys

from vehicle_common.mode_loader import mode_registry

PAYLOAD_MODE_REGISTRY_PATH = (
    Path(__file__).parent / "payload" / "payload_mode_registry.json"
)


class BuildPayloadModeRegistry(build_py):
    def run(self) -> None:
        print(
            "RUNNING PAYLOAD CUSTOM BUILD STEP"
        )  # This only shows up in build logs, not in terminal during build
        try:
            mode_registry.discover_modes(["uav.modes"])
            mode_registry.write_json(PAYLOAD_MODE_REGISTRY_PATH)
        except Exception as e:
            sys.exit(f"uav mode registry build failed: {e}")
        build_py.run(self)  # continue normal build


package_name = "payload"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    cmdclass={
        "build_py": BuildPayloadModeRegistry
    },  # define our custom build step here
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "missions"),
            glob(os.path.join("missions", "*.yaml")),
        ),
        (
            os.path.join("share", package_name),
            [str(PAYLOAD_MODE_REGISTRY_PATH)],
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
