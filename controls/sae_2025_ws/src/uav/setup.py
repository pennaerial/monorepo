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
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    
    
    entry_points={
        'console_scripts': [
            'temp = uav.temp:main',
            'vision_pipeline = uav.vision_pipeline:main',
            'global_position_offboard_control = uav.global_position_offboard_control:main',
            'mission = uav.mission:main',
            'payload_tracking_node = uav.vision_nodes.PayloadTrackingNode:main',
            'camera = uav.CameraNode:main',
        ],
    },
)
