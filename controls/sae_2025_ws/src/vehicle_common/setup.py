from setuptools import find_packages, setup

package_name = "vehicle_common"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        "vehicle_common.runtime": ["mode_registry.json"],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
            "test_plugin_loader = vehicle_common.runtime.plugin_loader:main"
        ],
    },
)
