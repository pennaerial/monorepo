from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=['uav.UAV'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'srv'), glob(os.path.join('srv', '*.srv')))  # Add srv files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temp = uav.temp:main',
            'camera_feed = uav.camera_feed:main',
            'global_position_offboard_control = uav.global_position_offboard_control:main',
            'flight = uav.flight:main',
            'test_flight = uav.test_flight:main'
        ],
    },
)
