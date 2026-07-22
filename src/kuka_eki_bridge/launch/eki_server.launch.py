"""
Launch file for the KUKA EKI XML Server node.

Starts the eki_xml_server node with parameters loaded from
config/eki_server.yaml, and supports command-line overrides.

Usage:
  ros2 launch kuka_eki_bridge eki_server.launch.py
  ros2 launch kuka_eki_bridge eki_server.launch.py port:=59152 bind_host:=0.0.0.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the EKI XML Server."""

    # ── Locate the package share directory ───────────────────────────
    pkg_share = get_package_share_directory('kuka_eki_bridge')
    default_config = os.path.join(pkg_share, 'config', 'eki_server.yaml')

    # ── Declare launch arguments (overridable from command line) ─────
    declare_bind_host = DeclareLaunchArgument(
        'bind_host',
        default_value='0.0.0.0',
        description='IP address to bind the TCP server.',
    )

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='59152',
        description='TCP port for the EKI server.',
    )

    declare_response_xml_path = DeclareLaunchArgument(
        'response_xml_path',
        default_value='',
        description='Path to a custom Sensor response XML file.',
    )

    # ── Define the ROS2 node ─────────────────────────────────────────
    eki_server_node = Node(
        package='kuka_eki_bridge',
        executable='eki_xml_server_node',
        name='eki_xml_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            default_config,
            {
                'bind_host': LaunchConfiguration('bind_host'),
                'port': LaunchConfiguration('port'),
                'response_xml_path': LaunchConfiguration('response_xml_path'),
            },
        ],
    )

    return LaunchDescription([
        declare_bind_host,
        declare_port,
        declare_response_xml_path,
        eki_server_node,
    ])
