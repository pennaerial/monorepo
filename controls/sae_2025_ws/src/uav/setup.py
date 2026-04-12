from glob import glob
import os
from pathlib import Path
import sys

from setuptools import find_packages, setup
from setuptools.command.build_py import build_py as _build_py
from setuptools.command.develop import develop as _develop
from setuptools.command.install import install as _install

package_name = "uav"
_SCHEMA_REGISTRY_REFRESHED = False


def _refresh_mode_schema_registry() -> None:
    global _SCHEMA_REGISTRY_REFRESHED
    if _SCHEMA_REGISTRY_REFRESHED:
        return

    package_root = Path(__file__).resolve().parent
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

    from uav.runtime.schema_generator import refresh_schema_registry

    output = refresh_schema_registry()
    print(f"Wrote mode schema registry to '{output}'.")
    _SCHEMA_REGISTRY_REFRESHED = True


class _RefreshSchemaRegistryMixin:
    def run(self):
        _refresh_mode_schema_registry()
        super().run()


class build_py(_RefreshSchemaRegistryMixin, _build_py):
    pass


class develop(_RefreshSchemaRegistryMixin, _develop):
    pass


class install(_RefreshSchemaRegistryMixin, _install):
    pass


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["missions/*.yaml", "fleets/*.yaml", "runtime/*.json"]},
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
    cmdclass={
        "build_py": build_py,
        "develop": develop,
        "install": install,
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
