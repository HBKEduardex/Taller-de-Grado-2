"""
gui_dual_node.py — Entry point for the Dual KUKA + RViz GUI.

Wires together:
  - RosAxisMoveBridge       (KUKA TCP/IP — existing, unmodified)
  - RosMoveitMirrorBridge   (RViz/MoveIt2 — new, shares ROS2 node)
  - DualCommandModel        (extended data model)
  - DualKukaRvizWindow      (PyQt5 dual GUI)

The RViz bridge reuses the rclpy Node from RosAxisMoveBridge
so there is only ONE rclpy.init() call.

Usage:
  ros2 run kuka_gui_control gui_dual_node
  ros2 launch kuka_gui_control gui_dual_kuka_rviz.launch.py
"""

import signal
import sys

try:
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QTimer
except ImportError as e:
    print(
        '\\n[ERROR] PyQt5 not found.\\n'
        '  Install with:  sudo apt install python3-pyqt5\\n'
    )
    sys.exit(1)

from kuka_gui_control.joint_command_model import (
    AXES,
    CARTESIAN_AXES,
    DEFAULT_CARTESIAN_HOME,
    DEFAULT_HOME,
    DEFAULT_LIMITS,
)
from kuka_gui_control.ros_axis_move_bridge import RosAxisMoveBridge
from kuka_gui_control.ros_moveit_mirror_bridge import RosMoveitMirrorBridge
from kuka_gui_control.dual_command_model import DualCommandModel
from kuka_gui_control.dual_kuka_rviz_window import DualKukaRvizWindow


# ---------------------------------------------------------------------------
# Default configuration
# ---------------------------------------------------------------------------

DEFAULT_CONFIG = {
    # ── KUKA TCP/IP topics ───────────────────────────────────────────
    'command_topic':          '/kuka/axis_move/target_json',
    'feedback_topic':         '/kuka/axis_move/feedback_json',
    'raw_command_xml_topic':  '/kuka/axis_move/raw_command_xml',
    'raw_feedback_xml_topic': '/kuka/axis_move/raw_robot_xml',

    # ── RViz / MoveIt2 topics ────────────────────────────────────────
    'rviz_joint_command_topic':     '/kuka_bridge/joint_command_deg',
    'rviz_cartesian_command_topic': '/kuka_bridge/cartesian_command_deg',
    'rviz_status_topic':            '/kuka_bridge/status',
    'rviz_joint_state_topic':       '/kuka_bridge/joint_state_deg',
    'rviz_cartesian_state_topic':   '/kuka_bridge/cartesian_state_deg',

    # ── Publishing behaviour ─────────────────────────────────────────
    'auto_publish_hz':        2.0,
    'step_deg':               1.0,
    'enable_move_default':    False,
    'require_confirmation_for_first_send': True,

    'allow_auto_mode':        True,
    'allow_auto_motion':      False,

    'publish_joints_to_kuka':  True,
    'publish_joints_to_rviz':  True,
    'publish_cartesian_to_kuka': True,
    'publish_cartesian_to_rviz': True,

    # ── Home / limits ────────────────────────────────────────────────
    'home_joints_deg':        dict(DEFAULT_HOME),
    'home_cartesian':         dict(DEFAULT_CARTESIAN_HOME),
    'soft_limits_deg': {
        a: list(DEFAULT_LIMITS[a]) for a in AXES
    },

    # ── UI options ───────────────────────────────────────────────────
    'feedback_timeout_sec':   2.0,
    'show_raw_json':          True,
    'show_raw_xml':           True,

    # ── Secuencias de trayectorias (capa AÑADIDA) ────────────────────
    # Carpeta donde se guardan los .json generados. Vacío = resolución
    # automática: <raíz del repositorio>/trajectories, nunca dentro de
    # install/. También se puede forzar con KUKA_TRAJECTORIES_DIR.
    'trajectories_dir':                     '',
    'trajectory_generation_timeout_sec':    60.0,
    'trajectory_arrival_tolerance_deg':     0.5,
    'trajectory_resend_period_sec':         0.5,
    'trajectory_point_timeout_sec':         15.0,
    'trajectory_min_point_period_sec':      0.2,
    'trajectory_gripper_settle_sec':        2.0,
    'trajectory_max_delta_deg':             10.0,
    'trajectory_kuka_ptp_velocity_normal_pct':  30.0,
    'trajectory_kuka_ptp_velocity_reduced_pct': 5.0,

    # ── Modo LOTE (capa AÑADIDA) ─────────────────────────────────────
    # Puntos por lote. Debe ser <= XD_BATCH_MAX de config_submit_better.dat
    # y <= max_batch_size de axis_move_better.yaml (ambos 20).
    'trajectory_batch_max_size':            20,
    # Reponer el siguiente sub-lote cuando quedan por consumir menos de esta
    # fracción del lote en curso.
    'trajectory_batch_refill_threshold':    0.5,
    # Sin progreso del contador de puntos consumidos durante este tiempo,
    # se aborta la secuencia.
    'trajectory_batch_stall_timeout_sec':   20.0,
    # Reenvío del lote mientras el KUKA no acusa haberlo recibido.
    'trajectory_batch_resend_period_sec':   0.5,
}


# ---------------------------------------------------------------------------
# Lectura de parámetros ROS2
# ---------------------------------------------------------------------------

# Parámetros escalares: nombre en el YAML → clave en cfg
_SCALAR_PARAMS = {
    # RViz / MoveIt2 topics
    'rviz_joint_command_topic':     'rviz_joint_command_topic',
    'rviz_cartesian_command_topic': 'rviz_cartesian_command_topic',
    'rviz_status_topic':            'rviz_status_topic',
    'rviz_joint_state_topic':       'rviz_joint_state_topic',
    'rviz_cartesian_state_topic':   'rviz_cartesian_state_topic',

    # Publishing behaviour
    'auto_publish_hz':                     'auto_publish_hz',
    'step_deg':                            'step_deg',
    'enable_move_default':                 'enable_move_default',
    'require_confirmation_for_first_send': 'require_confirmation_for_first_send',
    'allow_auto_mode':                     'allow_auto_mode',
    'allow_auto_motion':                   'allow_auto_motion',
    'publish_joints_to_kuka':              'publish_joints_to_kuka',
    'publish_joints_to_rviz':              'publish_joints_to_rviz',
    'publish_cartesian_to_kuka':           'publish_cartesian_to_kuka',
    'publish_cartesian_to_rviz':           'publish_cartesian_to_rviz',

    # UI options
    'feedback_timeout_sec': 'feedback_timeout_sec',
    'show_raw_json':        'show_raw_json',
    'show_raw_xml':         'show_raw_xml',

    # Secuencias de trayectorias (capa AÑADIDA)
    'trajectories_dir':                  'trajectories_dir',
    'trajectory_generation_timeout_sec': 'trajectory_generation_timeout_sec',
    'trajectory_arrival_tolerance_deg':  'trajectory_arrival_tolerance_deg',
    'trajectory_resend_period_sec':      'trajectory_resend_period_sec',
    'trajectory_point_timeout_sec':      'trajectory_point_timeout_sec',
    'trajectory_min_point_period_sec':   'trajectory_min_point_period_sec',
    'trajectory_gripper_settle_sec':     'trajectory_gripper_settle_sec',
    'trajectory_max_delta_deg':          'trajectory_max_delta_deg',
    'trajectory_kuka_ptp_velocity_normal_pct':
        'trajectory_kuka_ptp_velocity_normal_pct',
    'trajectory_kuka_ptp_velocity_reduced_pct':
        'trajectory_kuka_ptp_velocity_reduced_pct',

    # Modo LOTE
    'trajectory_batch_max_size':          'trajectory_batch_max_size',
    'trajectory_batch_refill_threshold':  'trajectory_batch_refill_threshold',
    'trajectory_batch_stall_timeout_sec': 'trajectory_batch_stall_timeout_sec',
    'trajectory_batch_resend_period_sec': 'trajectory_batch_resend_period_sec',
}


def load_config_from_node(node, cfg: dict) -> dict:
    """
    Sobreescribir `cfg` con los parámetros ROS2 declarados en el nodo.

    Hasta ahora el launch pasaba `parameters=[gui_dual_kuka_rviz.yaml]` pero
    nadie leía esos valores, así que el YAML no tenía ningún efecto.

    Los tópicos TCP/IP hacia el KUKA real NO se leen aquí a propósito: se
    crean en el constructor del bridge, antes de que exista el nodo, y no
    deben tocarse.
    """
    log = node.get_logger()

    for param_name, cfg_key in _SCALAR_PARAMS.items():
        default = cfg[cfg_key]
        try:
            node.declare_parameter(param_name, default)
            cfg[cfg_key] = node.get_parameter(param_name).value
        except Exception as e:
            log.warn(f'No se pudo leer el parámetro "{param_name}": {e}')

    # Home articular: home_joints_deg.A1 ... .A6 (grados)
    home = dict(cfg['home_joints_deg'])
    for axis in AXES:
        name = f'home_joints_deg.{axis}'
        try:
            node.declare_parameter(name, float(home[axis]))
            home[axis] = float(node.get_parameter(name).value)
        except Exception as e:
            log.warn(f'No se pudo leer el parámetro "{name}": {e}')
    cfg['home_joints_deg'] = home

    # Home cartesiano: home_cartesian.X ... .C (X, Y, Z en mm; A, B, C en grados)
    cart_home = dict(cfg['home_cartesian'])
    for key in CARTESIAN_AXES:
        name = f'home_cartesian.{key}'
        try:
            node.declare_parameter(name, float(cart_home[key]))
            cart_home[key] = float(node.get_parameter(name).value)
        except Exception as e:
            log.warn(f'No se pudo leer el parámetro "{name}": {e}')
    cfg['home_cartesian'] = cart_home

    # Límites blandos: soft_limits_deg.A1 = [min, max]
    limits = {a: list(cfg['soft_limits_deg'][a]) for a in AXES}
    for axis in AXES:
        name = f'soft_limits_deg.{axis}'
        try:
            node.declare_parameter(name, [float(v) for v in limits[axis]])
            value = node.get_parameter(name).value
            if value is not None and len(value) == 2:
                limits[axis] = [float(value[0]), float(value[1])]
        except Exception as e:
            log.warn(f'No se pudo leer el parámetro "{name}": {e}')
    cfg['soft_limits_deg'] = limits

    log.info(
        'Configuración cargada:\n'
        f'  HOME articular  : '
        f'{[round(cfg["home_joints_deg"][a], 3) for a in AXES]} deg\n'
        f'  HOME cartesiano : '
        f'{[round(cfg["home_cartesian"][k], 3) for k in CARTESIAN_AXES]} '
        f'(X,Y,Z en mm; A,B,C en deg)\n'
        f'  Comando joints  → {cfg["rviz_joint_command_topic"]}\n'
        f'  Comando cart.   → {cfg["rviz_cartesian_command_topic"]}'
    )
    return cfg


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main(args=None):
    """
    Launch the Dual KUKA + RViz GUI control application.

    Steps:
      1. Create the QApplication.
      2. Create and start RosAxisMoveBridge (KUKA TCP/IP).
      3. Create RosMoveitMirrorBridge (RViz — shares the ROS2 node).
      4. Build the DualCommandModel and DualKukaRvizWindow.
      5. Install Ctrl+C handler.
      6. Run the Qt event loop.
    """
    # ── Qt application ───────────────────────────────────────────────
    app = QApplication(sys.argv)
    app.setApplicationName('kuka_gui_dual')
    app.setOrganizationName('TG2')

    # ── Configuration ────────────────────────────────────────────────
    cfg = dict(DEFAULT_CONFIG)

    # ── KUKA bridge (creates rclpy.init + node) ──────────────────────
    kuka_bridge = RosAxisMoveBridge(
        command_topic=cfg['command_topic'],
        feedback_topic=cfg['feedback_topic'],
        raw_command_xml_topic=cfg['raw_command_xml_topic'],
        raw_robot_xml_topic=cfg['raw_feedback_xml_topic'],
    )

    # ── Start KUKA bridge ────────────────────────────────────────────
    kuka_bridge.start()

    # ── Configuración desde el YAML del launch ───────────────────────
    # Debe hacerse después de start() porque el nodo rclpy nace ahí, y antes
    # de construir el modelo y el bridge de RViz, que consumen estos valores.
    cfg = load_config_from_node(kuka_bridge._node, cfg)

    # ── Data model ───────────────────────────────────────────────────
    home = cfg.get('home_joints_deg', dict(DEFAULT_HOME))
    cartesian_home = cfg.get('home_cartesian', dict(DEFAULT_CARTESIAN_HOME))
    limits_raw = cfg.get('soft_limits_deg', {})
    limits = {
        a: tuple(limits_raw.get(a, DEFAULT_LIMITS[a]))
        for a in AXES
    }

    model = DualCommandModel(
        home=home,
        cartesian_home=cartesian_home,
        limits=limits,
        enable_move_default=cfg.get('enable_move_default', False),
        step_deg=cfg.get('step_deg', 1.0),
        publish_joints_to_kuka=cfg.get('publish_joints_to_kuka', True),
        publish_joints_to_rviz=cfg.get('publish_joints_to_rviz', True),
        publish_cartesian_to_kuka=cfg.get('publish_cartesian_to_kuka', False),
        publish_cartesian_to_rviz=cfg.get('publish_cartesian_to_rviz', True),
    )

    # ── RViz bridge (reuses the KUKA bridge's rclpy node) ────────────
    rviz_bridge = RosMoveitMirrorBridge(
        node=kuka_bridge._node,
        joint_command_topic=cfg['rviz_joint_command_topic'],
        cartesian_command_topic=cfg['rviz_cartesian_command_topic'],
        status_topic=cfg['rviz_status_topic'],
        joint_state_topic=cfg['rviz_joint_state_topic'],
        cartesian_state_topic=cfg['rviz_cartesian_state_topic'],
    )

    # ── Create window ────────────────────────────────────────────────
    window = DualKukaRvizWindow(
        model=model,
        kuka_bridge=kuka_bridge,
        rviz_bridge=rviz_bridge,
        config=cfg,
    )
    window.show()

    # Re-emit ROS status now that window is connected
    kuka_bridge.ros_status_changed.emit(True)

    # ── Ctrl+C handler ───────────────────────────────────────────────
    def _sigint_handler(sig, frame):
        print('\n[GUI Dual] Ctrl+C received — closing...')
        window.close()

    signal.signal(signal.SIGINT, _sigint_handler)

    # Pulse the Python event loop every 200 ms so signal.signal works
    _pulse = QTimer()
    _pulse.start(200)
    _pulse.timeout.connect(lambda: None)

    # ── Run ──────────────────────────────────────────────────────────
    exit_code = app.exec_()

    # ── Clean shutdown ───────────────────────────────────────────────
    if kuka_bridge.is_running:
        kuka_bridge.stop()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
