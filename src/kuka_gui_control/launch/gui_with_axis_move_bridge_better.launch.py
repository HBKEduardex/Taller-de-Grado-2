"""
gui_with_axis_move_bridge_better.launch.py — GUI + bridge BATCH, un solo comando.

Copia aditiva de gui_with_axis_move_bridge.launch.py. El original NO se toca y
sigue arrancando el bridge baseline; este arranca el bridge con soporte de
LOTES, que es el que la GUI necesita para ENVIAR TRAYECTORIA OPTIMIZADA.

Arranca:
  1. kuka_eki_bridge/axis_move_better.launch.py  (servidor TCP, puerto 59153)
  2. kuka_gui_control/gui_axis_move_node         (GUI PyQt5 original)

Ejecuta ESTE o el original, nunca los dos: comparten puerto 59153, nombre de
nodo y topicos.

En el controlador hacen falta los archivos _better cargados:
  XmlDualMove_better.xml, sps_submit_better.sub, config_submit_better.dat,
  y XmlDualMove_better.src seleccionado como programa de robot.

Uso:
  ros2 launch kuka_gui_control gui_with_axis_move_bridge_better.launch.py

Con movimiento habilitado:
  ros2 launch kuka_gui_control gui_with_axis_move_bridge_better.launch.py \
      safe_mode:=false allow_motion_commands:=true

Para ver el XML crudo del KUKA en la consola del bridge:
  ros2 launch kuka_gui_control gui_with_axis_move_bridge_better.launch.py \
      log_raw_robot_xml:=true

Nota: la GUI original no lee gui_axis_move.yaml (usa su DEFAULT_CONFIG
interno), pero el parametro se pasa igual que en el launch original para no
introducir ninguna diferencia de comportamiento.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Arranca el bridge con lotes y la GUI original."""

    # ── Argumentos ────────────────────────────────────────────────────
    safe_mode_arg = DeclareLaunchArgument(
        'safe_mode', default_value='true',
        description='Modo seguro del bridge (fuerza EnableMove a 0 si true)',
    )
    allow_motion_arg = DeclareLaunchArgument(
        'allow_motion_commands', default_value='false',
        description='Permite comandos de movimiento a traves del bridge',
    )
    log_raw_xml_arg = DeclareLaunchArgument(
        'log_raw_robot_xml', default_value='false',
        description='Vuelca el <Robot> crudo recibido del KUKA en consola',
    )

    # ── kuka_eki_bridge — axis_move_better ────────────────────────────
    try:
        bridge_share = get_package_share_directory('kuka_eki_bridge')
    except Exception:
        raise RuntimeError(
            '\n\n'
            '[ERROR] No se encuentra el paquete "kuka_eki_bridge".\n'
            'Compila y sourcea los dos paquetes:\n'
            '  cd ~/Documents/TG2\n'
            '  colcon build --packages-select kuka_eki_bridge kuka_gui_control\n'
            '  source install/setup.bash\n'
        )

    bridge_launch = os.path.join(
        bridge_share, 'launch', 'axis_move_better.launch.py'
    )

    if not os.path.isfile(bridge_launch):
        raise RuntimeError(
            f'\n\n'
            f'[ERROR] No se encuentra axis_move_better.launch.py en:\n'
            f'  {bridge_launch}\n'
            f'Recompila kuka_eki_bridge:\n'
            f'  colcon build --packages-select kuka_eki_bridge\n'
            f'  source install/setup.bash\n'
        )

    include_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch),
        launch_arguments={
            'safe_mode': LaunchConfiguration('safe_mode'),
            'allow_motion_commands': LaunchConfiguration(
                'allow_motion_commands'),
            'log_raw_robot_xml': LaunchConfiguration('log_raw_robot_xml'),
        }.items(),
    )

    # ── kuka_gui_control — GUI original ───────────────────────────────
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
        safe_mode_arg,
        allow_motion_arg,
        log_raw_xml_arg,
        LogInfo(msg='Arrancando bridge axis_move BETTER (modo LOTE)...'),
        include_bridge,
        LogInfo(msg='Arrancando GUI original (kuka_gui_control)...'),
        gui_node,
    ])
