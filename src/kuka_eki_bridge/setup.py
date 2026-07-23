"""Setup script for kuka_eki_bridge package."""
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'kuka_eki_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Ament index registration
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Config files
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Example XML files
        ('share/' + package_name + '/examples', glob('examples/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardex',
    maintainer_email='eduardex@todo.com',
    description='ROS2 bridge for KUKA EthernetKRL (EKI) via TCP/IP and XML.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eki_xml_server_node = kuka_eki_bridge.eki_xml_server_node:main',
            'eki_axis_stream_node = kuka_eki_bridge.eki_axis_stream_node:main',
            'eki_axis_command_loop_node = kuka_eki_bridge.eki_axis_command_loop_node:main',
            'eki_axis_move_node = kuka_eki_bridge.eki_axis_move_node:main',
        ],
    },
)
