"""
gui_control_node.py — Entry point for the kuka_gui_control GUI.

Wires together:
  - RosGuiBridge (ROS2 in background thread)
  - JointCommandModel (pure-Python data model)
  - KukaGuiMainWindow (PyQt5 UI)

Handles clean shutdown on Ctrl+C and window close without raising
'publisher's context is invalid' or KeyboardInterrupt join() errors.

Usage:
  ros2 run kuka_gui_control gui_control_node
  ros2 launch kuka_gui_control gui_control.launch.py
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
from kuka_gui_control.ros_gui_bridge import RosGuiBridge
from kuka_gui_control.gui_window import KukaGuiMainWindow


# ---------------------------------------------------------------------------
# Default configuration (overridden by ROS2 parameters in production)
# ---------------------------------------------------------------------------

DEFAULT_CONFIG = {
    'command_topic':          '/kuka/axis_command/target_json',
    'feedback_topic':         '/kuka/axis_command_loop/feedback_json',
    'raw_feedback_xml_topic': '/kuka/axis_command_loop/raw_robot_xml',
    'auto_publish_hz':        5.0,
    'step_deg':               1.0,
    'enable_move_default':    True,
    'feedback_timeout_sec':   2.0,
    'home_joints_deg':        dict(DEFAULT_HOME),
    'soft_limits_deg': {
        a: list(DEFAULT_LIMITS[a]) for a in AXES
    },
}


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main(args=None):
    """
    Launch the KUKA GUI control application.

    Steps:
      1. Create the QApplication.
      2. Start the RosGuiBridge (initialises rclpy in a background thread).
      3. Build the model and main window.
      4. Install a Ctrl+C handler that exits cleanly.
      5. Run the Qt event loop.
      6. On exit: stop the bridge, clean up rclpy.
    """
    # ── Qt application ───────────────────────────────────────────────
    app = QApplication(sys.argv)
    app.setApplicationName('kuka_gui_control')
    app.setOrganizationName('TG2')

    # ── Configuration (use defaults; ROS2 params are read in the bridge) ──
    cfg = dict(DEFAULT_CONFIG)

    # ── ROS2 bridge ──────────────────────────────────────────────────
    bridge = RosGuiBridge(
        command_topic=cfg['command_topic'],
        feedback_topic=cfg['feedback_topic'],
        raw_xml_topic=cfg['raw_feedback_xml_topic'],
    )
    bridge.start()

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
        enable_move_default=cfg.get('enable_move_default', True),
        step_deg=cfg.get('step_deg', 1.0),
    )

    # ── Main window ──────────────────────────────────────────────────
    window = KukaGuiMainWindow(
        model=model,
        bridge=bridge,
        config=cfg,
    )
    window.show()

    # ── Ctrl+C handler ───────────────────────────────────────────────
    # Allow Ctrl+C from the terminal to close the Qt app cleanly.
    def _sigint_handler(sig, frame):
        print('\n[GUI] Ctrl+C received — closing...')
        window.close()

    signal.signal(signal.SIGINT, _sigint_handler)

    # Pulse the Python event loop every 200 ms so signal.signal works
    # (Qt normally blocks signal delivery until the C++ event loop yields)
    _pulse = QTimer()
    _pulse.start(200)
    _pulse.timeout.connect(lambda: None)

    # ── Run ──────────────────────────────────────────────────────────
    exit_code = app.exec_()

    # ── Clean shutdown ───────────────────────────────────────────────
    # bridge.stop() is already called by closeEvent, but be safe:
    if bridge.is_running:
        bridge.stop()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
