"""
gui_with_axis_move_bridge.launch.py — Launch the GUI + eki_axis_move bridge.

Starts:
  1. kuka_eki_bridge/axis_move.launch.py (TCP server, port 59153)
  2. kuka_gui_control/gui_axis_move_node (PyQt5 GUI)

Requirements:
  - kuka_eki_bridge must be built and sourced.
  - kuka_gui_control must be built and sourced.

Usage:
  ros2 launch kuka_gui_control gui_with_axis_move_bridge.launch.py

If kuka_eki_bridge is not found, you will see an error.
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
    """Launch axis_move bridge and the GUI control node."""

    # ── kuka_eki_bridge — axis_move ──────────────────────────────────
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

    bridge_launch = os.path.join(
        bridge_share, 'launch', 'axis_move.launch.py'
    )

    if not os.path.isfile(bridge_launch):
        raise RuntimeError(
            f'\n\n'
            f'[ERROR] axis_move.launch.py not found at:\n'
            f'  {bridge_launch}\n'
            f'Rebuild kuka_eki_bridge:\n'
            f'  colcon build --packages-select kuka_eki_bridge\n'
            f'  source install/setup.bash\n'
        )

    include_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch)
    )

    # ── kuka_gui_control — GUI node ───────────────────────────────────
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
        LogInfo(msg='Starting axis_move bridge (kuka_eki_bridge)...'),
        include_bridge,
        LogInfo(msg='Starting GUI control node (kuka_gui_control)...'),
        gui_node,
    ])
