"""
gui_dual_kuka_rviz.launch.py — Launch the Dual GUI + eki_axis_move bridge.

Starts:
  1. kuka_eki_bridge/axis_move.launch.py (TCP server, port 59153)
  2. kuka_gui_control/gui_dual_node     (PyQt5 Dual GUI)

Usage:
  ros2 launch kuka_gui_control gui_dual_kuka_rviz.launch.py
  ros2 launch kuka_gui_control gui_dual_kuka_rviz.launch.py safe_mode:=false allow_motion_commands:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch axis_move bridge and the Dual GUI control node."""

    # ── Launch arguments ─────────────────────────────────────────────
    safe_mode_arg = DeclareLaunchArgument(
        'safe_mode', default_value='true',
        description='Bridge safe mode (blocks EnableMove if true)'
    )
    allow_motion_arg = DeclareLaunchArgument(
        'allow_motion_commands', default_value='false',
        description='Allow motion commands through the bridge'
    )
    force_udp_arg = DeclareLaunchArgument(
        'force_udp_dds', default_value='true',
        description=(
            'Desactiva el transporte SHM de Fast DDS y deja solo UDPv4. '
            'Necesario para que los comandos lleguen al bridge de MoveIt2 '
            'que corre como root dentro del contenedor Docker.'
        )
    )

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
        PythonLaunchDescriptionSource(bridge_launch),
        launch_arguments={
            'safe_mode': LaunchConfiguration('safe_mode'),
            'allow_motion_commands': LaunchConfiguration('allow_motion_commands'),
        }.items(),
    )

    # ── kuka_gui_control — Dual GUI node ─────────────────────────────
    gui_share = get_package_share_directory('kuka_gui_control')
    gui_config = os.path.join(
        gui_share, 'config', 'gui_dual_kuka_rviz.yaml'
    )

    # ── Fast DDS: solo UDPv4 (sin memoria compartida) ────────────────
    # La GUI corre en el host como uid 1000 y el bridge de MoveIt2 corre
    # como root dentro del contenedor. Comparten /dev/shm (--ipc=host),
    # pero los segmentos SHM del contenedor son root:root 0644, así que
    # la GUI no puede escribir en ellos y los mensajes se pierden en
    # silencio (el tópico se ve, los datos no llegan).
    fastdds_profile = os.path.join(
        gui_share, 'config', 'fastdds_udp_only.xml'
    )
    set_fastdds_profile = SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILES_FILE',
        value=fastdds_profile,
        condition=IfCondition(LaunchConfiguration('force_udp_dds')),
    )

    gui_node = Node(
        package='kuka_gui_control',
        executable='gui_dual_node',
        name='kuka_gui_dual_node',
        output='screen',
        emulate_tty=True,
        parameters=[gui_config],
    )

    return LaunchDescription([
        safe_mode_arg,
        allow_motion_arg,
        force_udp_arg,
        set_fastdds_profile,
        LogInfo(msg=f'Fast DDS profile (UDP only): {fastdds_profile}'),
        LogInfo(msg='Starting axis_move bridge (kuka_eki_bridge)...'),
        include_bridge,
        LogInfo(msg='Starting Dual GUI (KUKA + RViz)...'),
        gui_node,
    ])
