"""
Launch file for the KUKA EKI Axis Command Loop node.

Starts eki_axis_command_loop_node with parameters from
config/axis_command_loop.yaml.

This launch file is independent from the existing:
  - eki_server.launch.py
  - axis_stream.launch.py
  - axis_command.launch.py

Usage:
  ros2 launch kuka_eki_bridge axis_command_loop.launch.py
  ros2 launch kuka_eki_bridge axis_command_loop.launch.py port:=59153
  ros2 launch kuka_eki_bridge axis_command_loop.launch.py safe_mode:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the axis command loop node."""

    pkg_share = get_package_share_directory('kuka_eki_bridge')
    default_config = os.path.join(
        pkg_share, 'config', 'axis_command_loop.yaml'
    )

    # ── Launch arguments ─────────────────────────────────────────────
    declare_bind_host = DeclareLaunchArgument(
        'bind_host',
        default_value='0.0.0.0',
        description='IP address to bind the TCP server.',
    )

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='59153',
        description='TCP port for the command loop server.',
    )

    declare_safe_mode = DeclareLaunchArgument(
        'safe_mode',
        default_value='true',
        description='If true, EnableMove is always forced to 0 in KUKA commands.',
    )

    declare_log_feedback = DeclareLaunchArgument(
        'log_feedback_values',
        default_value='true',
        description='Print a compact line per cycle with axis values.',
    )

    declare_log_cmd_xml = DeclareLaunchArgument(
        'log_command_xml',
        default_value='true',
        description='Log the <Command> XML sent to the KUKA.',
    )

    declare_log_raw_xml = DeclareLaunchArgument(
        'log_raw_xml',
        default_value='false',
        description='Log the raw <Robot> XML received from the KUKA.',
    )

    # ── Node ─────────────────────────────────────────────────────────
    command_loop_node = Node(
        package='kuka_eki_bridge',
        executable='eki_axis_command_loop_node',
        name='eki_axis_command_loop',
        output='screen',
        emulate_tty=True,
        parameters=[
            default_config,
            {
                'bind_host': LaunchConfiguration('bind_host'),
                'port': LaunchConfiguration('port'),
                'safe_mode': LaunchConfiguration('safe_mode'),
                'log_feedback_values': LaunchConfiguration('log_feedback_values'),
                'log_command_xml': LaunchConfiguration('log_command_xml'),
                'log_raw_xml': LaunchConfiguration('log_raw_xml'),
            },
        ],
    )

    return LaunchDescription([
        declare_bind_host,
        declare_port,
        declare_safe_mode,
        declare_log_feedback,
        declare_log_cmd_xml,
        declare_log_raw_xml,
        command_loop_node,
    ])
