"""
gui_control.launch.py — Launch only the kuka_gui_control GUI node.

Does NOT start the axis_command_loop. Useful when the command loop
is already running in a separate terminal.

Usage:
  ros2 launch kuka_gui_control gui_control.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch only the GUI control node."""

    pkg_share = get_package_share_directory('kuka_gui_control')
    default_config = os.path.join(pkg_share, 'config', 'gui_control.yaml')

    gui_node = Node(
        package='kuka_gui_control',
        executable='gui_control_node',
        name='kuka_gui_control_node',
        output='screen',
        emulate_tty=True,
        parameters=[default_config],
    )

    return LaunchDescription([gui_node])
