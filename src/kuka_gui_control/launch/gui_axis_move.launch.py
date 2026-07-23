"""
gui_axis_move.launch.py — Launch the GUI for the XmlAxisMove mode.

Starts kuka_gui_control/gui_axis_move_node with gui_axis_move.yaml config.

Note: This only starts the GUI. It does NOT start the TCP bridge.
If you want to start both, use gui_with_axis_move_bridge.launch.py instead.

Usage:
  ros2 launch kuka_gui_control gui_axis_move.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the GUI control node for AxisMove."""

    gui_share = get_package_share_directory('kuka_gui_control')
    gui_config = os.path.join(gui_share, 'config', 'gui_axis_move.yaml')

    gui_node = Node(
        package='kuka_gui_control',
        executable='gui_axis_move_node',
        name='kuka_gui_axis_move_node',
        output='screen',
        emulate_tty=True,
        parameters=[gui_config],
    )

    return LaunchDescription([
        LogInfo(msg='Starting GUI control node for AxisMove...'),
        gui_node,
    ])
