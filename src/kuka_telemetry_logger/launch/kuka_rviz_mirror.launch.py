"""
kuka_rviz_mirror.launch.py — start ONLY the KUKA -> RViz joint-state mirror.

This launch file starts nothing else. It does NOT start RViz, MoveIt, the
robot_state_publisher, the joint_state_publisher, the EKI bridge, any GUI, any
TCP server, or the telemetry logger. Start your existing system exactly as you
always do, then start this mirror alongside it.

    TERMINAL 1  your usual visualiser (RViz + MoveIt) and, separately,
                the original TCP/IP GUI + eki_axis_move
    TERMINAL 2  ros2 launch kuka_telemetry_logger kuka_rviz_mirror.launch.py
    TERMINAL 3  ros2 run kuka_telemetry_logger telemetry_logger

Prerequisite for the picture to reach RViz: the visualiser must be running the
plain `joint_state_publisher` configured with source_list ["/fake_joint_states"],
i.e. kuka_kr6_moveit_config demo.launch.py with use_gui:=false. With
use_gui:=true that launch starts joint_state_publisher_gui WITHOUT source_list,
and nothing would consume /fake_joint_states. Nothing in that package is
modified by this launch file — this is only a note about how to start it.

Usage:
  ros2 launch kuka_telemetry_logger kuka_rviz_mirror.launch.py
  ros2 launch kuka_telemetry_logger kuka_rviz_mirror.launch.py verbose:=true
  ros2 launch kuka_telemetry_logger kuka_rviz_mirror.launch.py \
      joint_states_topic:=/fake_joint_states

NOTE: the `kuka_rviz_mirror` executable must be registered in setup.py
console_scripts first. See README_RVIZ_MIRROR.md — that change is documented
there but deliberately NOT applied.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the KUKA -> RViz joint-state mirror node, and nothing else."""
    telemetry_topic_arg = DeclareLaunchArgument(
        'telemetry_topic',
        default_value='/kuka/axis_move/feedback_json',
        description=(
            'Telemetry topic to observe (the same one the GUI and the '
            'telemetry logger use).'
        ),
    )
    joint_states_topic_arg = DeclareLaunchArgument(
        'joint_states_topic',
        default_value='/fake_joint_states',
        description=(
            'Output topic. Feeds the EXISTING joint_state_publisher, which '
            'republishes on /joint_states. Never set this to /joint_states: '
            'that topic already has a publisher.'
        ),
    )
    report_every_arg = DeclareLaunchArgument(
        'report_every',
        default_value='100',
        description='Print a compact diagnostic report every N messages.',
    )
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Print every degree -> radian conversion (noisy).',
    )

    mirror_node = Node(
        package='kuka_telemetry_logger',
        executable='kuka_rviz_mirror',
        name='kuka_rviz_mirror',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'telemetry_topic': LaunchConfiguration('telemetry_topic'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'report_every': LaunchConfiguration('report_every'),
            'verbose': LaunchConfiguration('verbose'),
        }],
    )

    return LaunchDescription([
        telemetry_topic_arg,
        joint_states_topic_arg,
        report_every_arg,
        verbose_arg,
        LogInfo(msg=(
            'Starting KUKA RViz mirror (visualisation only — no commands, '
            'no planning, no /joint_states publisher)...'
        )),
        mirror_node,
    ])
