import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'unitree_r1_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE.unitree']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rodrigo Pérez-Rodríguez',
    maintainer_email='rodrigo.perez@urjc.es',
    description='The Unitree R1 in Gazebo, driven as the real robot through unitree_sdk2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r1_sdk_sim = unitree_r1_gazebo.sdk_sim:main',
            'r1_state_publisher = unitree_r1_gazebo.state_publisher:main',
            'fetch_r1_model = unitree_r1_gazebo.model:fetch_main',
        ],
    },
)
