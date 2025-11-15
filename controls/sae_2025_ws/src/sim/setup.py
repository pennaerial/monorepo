from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('sim/world_gen/worlds/*.sdf')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pennair',
    maintainer_email='pennair@example.com',
    description='Simulation framework for UAV competitions',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scoring_node = sim.in_house.scoring_node:main',
            'simulation = sim.simulation:main',
        ],
    },
)
