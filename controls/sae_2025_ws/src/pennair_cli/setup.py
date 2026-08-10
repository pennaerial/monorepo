from setuptools import find_packages, setup

package_name = "pennair_cli"

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
        "ros2cli.command": [  # for registering top level commands. 'ros2 pennair'
            "pennair = pennair_cli.command.pennair:PennairCommand",
        ],
        "pennair.verb": [  # register verbs under pennair command. 'ros2 pennair <verb>'
            "greeting = pennair_cli.verb.greeting:GreetingVerb",
        ],
    },
)
