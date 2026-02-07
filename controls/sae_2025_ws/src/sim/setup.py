from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sim'

models_root = os.path.join('sim', 'world_gen', 'models')

model_data_files = []
if os.path.isdir(models_root):
    for dirpath, dirnames, filenames in os.walk(models_root):
        if not filenames:
            continue  # nothing to install from this directory

        rel_dir = os.path.relpath(dirpath, 'sim')
        dest_dir = os.path.join('share', package_name, rel_dir)

        files = [os.path.join(dirpath, f) for f in filenames]
        model_data_files.append((dest_dir, files))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Standard ROS 2 bits
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.yaml')),

        # Worlds and simulations
        (os.path.join('share', package_name, 'worlds'), glob('sim/world_gen/worlds/*.sdf')),
        (os.path.join('share', package_name, 'simulations'), glob('sim/simulations/*.yaml')),

        # All model files, tree preserved
        *model_data_files,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pennair',
    maintainer_email='board@pennaerial.com',
    description='Simulation framework for UAV projects',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hoop_course = sim.world_gen.HoopCourse:main',
            'custom_world_node = sim.world_gen.CustomWorldNode:main',
            'helipad_world_node = sim.world_gen.HelipadWorldNode:main',
            'hoop_score = sim.scoring.HoopScore:main',
        ],
    },
)
