"""Setup script for kuka_telemetry_logger package.

Passive, read-only telemetry recorder for the KUKA EthernetKRL ROS2 bridge.
This package does NOT depend on, nor modify, any existing package.
"""
from glob import glob
from setuptools import find_packages, setup

package_name = 'kuka_telemetry_logger'

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
        # Offline analysis script (plain Python, no ROS2 required)
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardex',
    maintainer_email='adrian.vargas@ucb.edu.bo',
    description=(
        'Passive ROS2 subscriber that records KUKA telemetry to CSV + SQLite. '
        'Publishes nothing.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_logger = '
            'kuka_telemetry_logger.telemetry_logger_node:main',
        ],
    },
)
