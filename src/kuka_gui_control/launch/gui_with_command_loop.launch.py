"""
gui_with_command_loop.launch.py — Launch the GUI + axis_command_loop together.

Starts:
  1. kuka_eki_bridge/axis_command_loop  (TCP server, port 59153)
  2. kuka_gui_control/gui_control_node  (PyQt5 GUI)

Requirements:
  - kuka_eki_bridge must be built and sourced.
  - kuka_gui_control must be built and sourced.

Usage:
  ros2 launch kuka_gui_control gui_with_command_loop.launch.py

If kuka_eki_bridge is not found, you will see:
  [ERROR] Package 'kuka_eki_bridge' not found.
Make sure you have built and sourced both packages:
  cd ~/Documents/TG2
  colcon build --packages-select kuka_eki_bridge kuka_gui_control
  source install/setup.bash
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Launch axis_command_loop and the GUI control node."""

    # ── kuka_eki_bridge — axis_command_loop ──────────────────────────
    try:
        bridge_share = get_package_share_directory('kuka_eki_bridge')
    except Exception:
        raise RuntimeError(
            '\n\n'
            '[ERROR] Package "kuka_eki_bridge" not found.\n'
            'Make sure you have built and sourced both packages:\n'
            '  cd ~/Documents/TG2\n'
            '  colcon build --packages-select kuka_eki_bridge kuka_gui_control\n'
            '  source install/setup.bash\n'
        )

    command_loop_launch = os.path.join(
        bridge_share, 'launch', 'axis_command_loop.launch.py'
    )

    if not os.path.isfile(command_loop_launch):
        raise RuntimeError(
            f'\n\n'
            f'[ERROR] axis_command_loop.launch.py not found at:\n'
            f'  {command_loop_launch}\n'
            f'Rebuild kuka_eki_bridge:\n'
            f'  colcon build --packages-select kuka_eki_bridge\n'
            f'  source install/setup.bash\n'
        )

    include_command_loop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(command_loop_launch)
    )

    # ── kuka_gui_control — GUI node ───────────────────────────────────
    gui_share = get_package_share_directory('kuka_gui_control')
    gui_config = os.path.join(gui_share, 'config', 'gui_control.yaml')

    gui_node = Node(
        package='kuka_gui_control',
        executable='gui_control_node',
        name='kuka_gui_control_node',
        output='screen',
        emulate_tty=True,
        parameters=[gui_config],
    )

    return LaunchDescription([
        LogInfo(msg='Starting axis_command_loop (kuka_eki_bridge)...'),
        include_command_loop,
        LogInfo(msg='Starting GUI control node (kuka_gui_control)...'),
        gui_node,
    ])
