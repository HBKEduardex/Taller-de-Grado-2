"""
Launch file for the KUKA EKI Axis Stream node.

Starts eki_axis_stream_node with parameters from config/axis_stream.yaml.
This is independent from eki_server.launch.py.

Usage:
  ros2 launch kuka_eki_bridge axis_stream.launch.py
  ros2 launch kuka_eki_bridge axis_stream.launch.py port:=59152
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the axis stream node."""

    pkg_share = get_package_share_directory('kuka_eki_bridge')
    default_config = os.path.join(pkg_share, 'config', 'axis_stream.yaml')

    # ── Launch arguments ─────────────────────────────────────────────
    declare_bind_host = DeclareLaunchArgument(
        'bind_host',
        default_value='0.0.0.0',
        description='IP address to bind the TCP server.',
    )

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='59152',
        description='TCP port for the axis stream server.',
    )

    declare_send_response = DeclareLaunchArgument(
        'send_response',
        default_value='false',
        description='Send a minimal Sensor response to the KUKA.',
    )

    declare_publish_joint_states = DeclareLaunchArgument(
        'publish_joint_states',
        default_value='false',
        description='Publish JointState to /joint_states.',
    )

    # ── Node ─────────────────────────────────────────────────────────
    axis_stream_node = Node(
        package='kuka_eki_bridge',
        executable='eki_axis_stream_node',
        name='eki_axis_stream',
        output='screen',
        emulate_tty=True,
        parameters=[
            default_config,
            {
                'bind_host': LaunchConfiguration('bind_host'),
                'port': LaunchConfiguration('port'),
                'send_response': LaunchConfiguration('send_response'),
                'publish_joint_states': LaunchConfiguration(
                    'publish_joint_states'
                ),
            },
        ],
    )

    return LaunchDescription([
        declare_bind_host,
        declare_port,
        declare_send_response,
        declare_publish_joint_states,
        axis_stream_node,
    ])
