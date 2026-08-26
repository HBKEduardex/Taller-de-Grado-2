"""
gui_axis_move_node.py — Entry point for the KUKA AxisMove GUI.

Wires together:
  - RosAxisMoveBridge (ROS2 in background thread)
  - JointCommandModel (pure-Python data model)
  - AxisMoveGuiWindow (PyQt5 UI)

Handles clean shutdown on Ctrl+C and window close without raising
'publisher's context is invalid' or KeyboardInterrupt join() errors.

This module does NOT modify gui_control_node.py.

Usage:
  ros2 run kuka_gui_control gui_axis_move_node
  ros2 launch kuka_gui_control gui_axis_move.launch.py
"""

import signal
import sys

try:
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QTimer
except ImportError as e:
    print(
        '\n[ERROR] PyQt5 not found.\n'
        '  Install with:  sudo apt install python3-pyqt5\n'
    )
    sys.exit(1)

from kuka_gui_control.joint_command_model import (
    JointCommandModel,
    AXES,
    DEFAULT_HOME,
    DEFAULT_LIMITS,
)
from kuka_gui_control.ros_axis_move_bridge import RosAxisMoveBridge
from kuka_gui_control.gui_axis_move_window import AxisMoveGuiWindow


# ---------------------------------------------------------------------------
# Default configuration (overridden by ROS2 parameters from YAML)
# ---------------------------------------------------------------------------

DEFAULT_CONFIG = {
    'command_topic':          '/kuka/axis_move/target_json',
    'feedback_topic':         '/kuka/axis_move/feedback_json',
    'raw_command_xml_topic':  '/kuka/axis_move/raw_command_xml',
    'raw_feedback_xml_topic': '/kuka/axis_move/raw_robot_xml',

    'auto_publish_hz':        2.0,
    'step_deg':               1.0,
    'enable_move_default':    False,
    'require_confirmation_for_first_send': True,

    'allow_auto_mode':        True,
    'allow_auto_motion':      False,

    'feedback_timeout_sec':   2.0,
    'show_raw_json':          True,
    'show_raw_xml':           True,

    # ── Secuencias de trayectorias (capa AÑADIDA) ────────────────────
    # Carpeta donde se guardan los .json generados. Vacío = resolución
    # automática: <raíz del repositorio>/trajectories, nunca dentro de
    # install/. Se puede forzar con la variable de entorno
    # KUKA_TRAJECTORIES_DIR sin tocar este archivo.
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
    #
    # HISTORIA: estuvo en 1 como DIAGNOSTICO mientras se perseguia el KSS01422.
    # Se sospecho del tamano del telegrama (~2195 bytes con 20 puntos), pero la
    # causa real resulto ser otra: los arrays destino de EKI_GetReal en el SPS
    # entraban sin inicializar, contra KST Ethernet KRL 3.0 6.2.5 p.39. Con eso
    # corregido el tamano deja de ser sospechoso, y 20 es justo el valor para el
    # que esta dimensionado todo el camino: XD_BATCH_MAX=20 en
    # config_submit_better.dat, arrays [20] en el SPS y en el .src,
    # max_batch_size=20 en axis_move_better.yaml, <BUFFERING Limit="128"> y
    # BUFFSIZE por defecto de 16384 bytes (el lote de 20 pesa ~1740).
    #
    # Con 1 punto por lote el modo LOTE no existe: es un viaje de red por punto,
    # exactamente el modo base. Por eso el robot seguia yendo a tirones.
    # Si algo fallase, bisecar hacia abajo (10, 5) y leer XD_SPS_RUNNING_ID.
    'trajectory_batch_max_size':            20,
    # Reponer el siguiente sub-lote cuando quedan por consumir menos de esta
    # fracción del lote en curso.
    'trajectory_batch_refill_threshold':    0.5,
    # Sin progreso del contador de puntos consumidos durante este tiempo,
    # se aborta la secuencia.
    'trajectory_batch_stall_timeout_sec':   20.0,
    # Reenvío del lote mientras el KUKA no acusa haberlo recibido.
    'trajectory_batch_resend_period_sec':   0.5,

    'home_joints_deg':        dict(DEFAULT_HOME),
    'soft_limits_deg': {
        a: list(DEFAULT_LIMITS[a]) for a in AXES
    },
    'max_delta_deg': {
        a: 2.0 for a in AXES
    },
}


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main(args=None):
    """
    Launch the KUKA AxisMove GUI control application.

    Steps:
      1. Create the QApplication.
      2. Start the RosAxisMoveBridge (initialises rclpy in a background thread).
      3. Build the model and main window.
      4. Install a Ctrl+C handler that exits cleanly.
      5. Run the Qt event loop.
      6. On exit: stop the bridge, clean up rclpy.
    """
    # ── Qt application ───────────────────────────────────────────────
    app = QApplication(sys.argv)
    app.setApplicationName('kuka_gui_axis_move')
    app.setOrganizationName('TG2')

    # ── Configuration (use defaults; could be extended to read ROS2 params) ──
    cfg = dict(DEFAULT_CONFIG)

    # ── ROS2 bridge (created but NOT started yet) ─────────────────────
    bridge = RosAxisMoveBridge(
        command_topic=cfg['command_topic'],
        feedback_topic=cfg['feedback_topic'],
        raw_command_xml_topic=cfg['raw_command_xml_topic'],
        raw_robot_xml_topic=cfg['raw_feedback_xml_topic'],
    )

    # ── Data model ───────────────────────────────────────────────────
    home = cfg.get('home_joints_deg', dict(DEFAULT_HOME))
    limits_raw = cfg.get('soft_limits_deg', {})
    limits = {
        a: tuple(limits_raw.get(a, DEFAULT_LIMITS[a]))
        for a in AXES
    }

    model = JointCommandModel(
        home=home,
        limits=limits,
        enable_move_default=cfg.get('enable_move_default', False),
        step_deg=cfg.get('step_deg', 1.0),
    )

    # ── Main window ──────────────────────────────────────────────────
    window = AxisMoveGuiWindow(
        model=model,
        bridge=bridge,
        config=cfg,
    )
    window.show()

    # ── Start ROS2 AFTER window exists (so signal connection is live) ─
    bridge.start()

    # ── Ctrl+C handler ───────────────────────────────────────────────
    def _sigint_handler(sig, frame):
        print('\n[GUI] Ctrl+C received — closing...')
        window.close()

    signal.signal(signal.SIGINT, _sigint_handler)

    # Pulse the Python event loop every 200 ms so signal.signal works
    _pulse = QTimer()
    _pulse.start(200)
    _pulse.timeout.connect(lambda: None)

    # ── Run ──────────────────────────────────────────────────────────
    exit_code = app.exec_()

    # ── Clean shutdown ───────────────────────────────────────────────
    if bridge.is_running:
        bridge.stop()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
