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

    'publish_to_kuka_default':  True,
    'publish_to_rviz_default':  True,
    'cartesian_to_rviz':        True,

    # ── Home / limits ────────────────────────────────────────────────
    'home_joints_deg':        dict(DEFAULT_HOME),
    'soft_limits_deg': {
        a: list(DEFAULT_LIMITS[a]) for a in AXES
    },

    # ── UI options ───────────────────────────────────────────────────
    'feedback_timeout_sec':   2.0,
    'show_raw_json':          True,
    'show_raw_xml':           True,
}


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

    # ── Data model ───────────────────────────────────────────────────
    home = cfg.get('home_joints_deg', dict(DEFAULT_HOME))
    limits_raw = cfg.get('soft_limits_deg', {})
    limits = {
        a: tuple(limits_raw.get(a, DEFAULT_LIMITS[a]))
        for a in AXES
    }

    model = DualCommandModel(
        home=home,
        limits=limits,
        enable_move_default=cfg.get('enable_move_default', False),
        step_deg=cfg.get('step_deg', 1.0),
        publish_to_kuka=cfg.get('publish_to_kuka_default', True),
        publish_to_rviz=cfg.get('publish_to_rviz_default', True),
        cartesian_to_rviz=cfg.get('cartesian_to_rviz', True),
    )

    # ── Main window (created BEFORE bridge.start) ────────────────────
    # RViz bridge will be created after kuka_bridge starts (needs node)
    # Use a placeholder; we'll set the real one after start.
    window = None

    # ── Start KUKA bridge ────────────────────────────────────────────
    kuka_bridge.start()

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
