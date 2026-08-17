"""
telemetry_logger.launch.py — start ONLY the passive telemetry logger.

This launch file starts nothing else. It does not start the EKI bridge, it
does not start the GUI, and it does not touch the robot. Start your existing
system exactly as you always do, then start this logger alongside it.

Usage:
  ros2 launch kuka_telemetry_logger telemetry_logger.launch.py
  ros2 launch kuka_telemetry_logger telemetry_logger.launch.py verbose:=true
  ros2 launch kuka_telemetry_logger telemetry_logger.launch.py log_dir:=/tmp/kuka_logs
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the passive KUKA telemetry logger node."""
    share = get_package_share_directory('kuka_telemetry_logger')
    default_config = os.path.join(share, 'config', 'telemetry_logger.yaml')

    telemetry_topic_arg = DeclareLaunchArgument(
        'telemetry_topic',
        default_value='/kuka/axis_move/feedback_json',
        description='Telemetry topic to observe (same one the GUI uses).',
    )
    log_dir_arg = DeclareLaunchArgument(
        'log_dir',
        default_value='logs',
        description='Directory for the CSV and SQLite files.',
    )
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Print every received message.',
    )

    logger_node = Node(
        package='kuka_telemetry_logger',
        executable='telemetry_logger',
        name='kuka_telemetry_logger',
        output='screen',
        emulate_tty=True,
        parameters=[
            default_config,
            {
                'telemetry_topic': LaunchConfiguration('telemetry_topic'),
                'log_dir': LaunchConfiguration('log_dir'),
                'verbose': LaunchConfiguration('verbose'),
            },
        ],
    )

    return LaunchDescription([
        telemetry_topic_arg,
        log_dir_arg,
        verbose_arg,
        LogInfo(msg='Starting PASSIVE KUKA telemetry logger (no publishers)...'),
        logger_node,
    ])
