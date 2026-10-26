from setuptools import find_packages, setup

package_name = "iarc"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pydantic>=2,<3"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="IARC flight modes and path planning",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
)
