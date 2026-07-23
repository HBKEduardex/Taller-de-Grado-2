"""
ros_axis_move_bridge.py — Thread-safe bridge between rclpy and PyQt5 for AxisMove.

Extended version of ros_gui_bridge.py that subscribes to 3 topics:
  - feedback_json (KUKA feedback)
  - raw_command_xml (XML sent to KUKA)
  - raw_robot_xml (XML received from KUKA)

This module does NOT modify ros_gui_bridge.py.
"""

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from PyQt5.QtCore import QObject, pyqtSignal
except ImportError as e:
    raise ImportError(
        'PyQt5 is required. Install with: sudo apt install python3-pyqt5'
    ) from e


# ---------------------------------------------------------------------------
# Internal ROS2 node
# ---------------------------------------------------------------------------

class _AxisMoveBridgeNode(Node):
    """
    Lightweight ROS2 node owned by the RosAxisMoveBridge.

    Publishes to the command topic and subscribes to feedback + raw XML topics.
    """

    def __init__(
        self,
        command_topic: str,
        feedback_topic: str,
        raw_command_xml_topic: str,
        raw_robot_xml_topic: str,
        on_feedback: callable,
        on_raw_command_xml: callable,
        on_raw_robot_xml: callable,
    ):
        super().__init__('kuka_gui_axis_move_node')

        self._on_feedback = on_feedback
        self._on_raw_command_xml = on_raw_command_xml
        self._on_raw_robot_xml = on_raw_robot_xml

        # Publisher
        self._pub = self.create_publisher(String, command_topic, 10)
        self.get_logger().info(f'Publishing to: {command_topic}')

        # Feedback subscriber
        self.create_subscription(
            String, feedback_topic,
            self._feedback_callback, 10,
        )
        self.get_logger().info(f'Subscribed to: {feedback_topic}')

        # Raw command XML subscriber
        self.create_subscription(
            String, raw_command_xml_topic,
            self._raw_command_xml_callback, 10,
        )
        self.get_logger().info(f'Subscribed to: {raw_command_xml_topic}')

        # Raw robot XML subscriber
        self.create_subscription(
            String, raw_robot_xml_topic,
            self._raw_robot_xml_callback, 10,
        )
        self.get_logger().info(f'Subscribed to: {raw_robot_xml_topic}')

    def publish_command(self, json_str: str) -> None:
        """Publish a command JSON string on the command topic."""
        msg = String()
        msg.data = json_str
        self._pub.publish(msg)

    def _feedback_callback(self, msg: String) -> None:
        try:
            self._on_feedback(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Feedback callback error: {e}')

    def _raw_command_xml_callback(self, msg: String) -> None:
        try:
            self._on_raw_command_xml(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Raw command XML callback error: {e}')

    def _raw_robot_xml_callback(self, msg: String) -> None:
        try:
            self._on_raw_robot_xml(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Raw robot XML callback error: {e}')


# ---------------------------------------------------------------------------
# Public bridge (QObject with signals)
# ---------------------------------------------------------------------------

class RosAxisMoveBridge(QObject):
    """
    Thread-safe bridge between rclpy and PyQt5 for AxisMove mode.

    Signals (emitted from the ROS2 thread, connected to Qt slots):
      feedback_received(str)         — JSON string from /feedback_json
      raw_command_xml_received(str)  — XML string from /raw_command_xml
      raw_robot_xml_received(str)    — XML string from /raw_robot_xml
      ros_status_changed(bool)       — True when ROS2 is active

    Usage:
        bridge = RosAxisMoveBridge(
            command_topic='/kuka/axis_move/target_json',
            feedback_topic='/kuka/axis_move/feedback_json',
            raw_command_xml_topic='/kuka/axis_move/raw_command_xml',
            raw_robot_xml_topic='/kuka/axis_move/raw_robot_xml',
        )
        bridge.feedback_received.connect(my_slot)
        bridge.start()
        ...
        bridge.publish_command(json_string)
        ...
        bridge.stop()
    """

    feedback_received = pyqtSignal(str)
    raw_command_xml_received = pyqtSignal(str)
    raw_robot_xml_received = pyqtSignal(str)
    ros_status_changed = pyqtSignal(bool)

    def __init__(
        self,
        command_topic: str = '/kuka/axis_move/target_json',
        feedback_topic: str = '/kuka/axis_move/feedback_json',
        raw_command_xml_topic: str = '/kuka/axis_move/raw_command_xml',
        raw_robot_xml_topic: str = '/kuka/axis_move/raw_robot_xml',
        parent=None,
    ):
        super().__init__(parent)
        self._command_topic = command_topic
        self._feedback_topic = feedback_topic
        self._raw_command_xml_topic = raw_command_xml_topic
        self._raw_robot_xml_topic = raw_robot_xml_topic

        self._node: _AxisMoveBridgeNode = None
        self._thread: threading.Thread = None
        self._running = False

    # ── Lifecycle ────────────────────────────────────────────────────

    def start(self) -> None:
        """Initialize rclpy and start the spin loop in a background thread."""
        if self._running:
            return

        rclpy.init()
        self._node = _AxisMoveBridgeNode(
            command_topic=self._command_topic,
            feedback_topic=self._feedback_topic,
            raw_command_xml_topic=self._raw_command_xml_topic,
            raw_robot_xml_topic=self._raw_robot_xml_topic,
            on_feedback=self._on_feedback,
            on_raw_command_xml=self._on_raw_command_xml,
            on_raw_robot_xml=self._on_raw_robot_xml,
        )

        self._running = True
        self._thread = threading.Thread(
            target=self._spin_loop,
            daemon=True,
            name='ros_axis_move_bridge_thread',
        )
        self._thread.start()
        self.ros_status_changed.emit(True)

    def stop(self) -> None:
        """Stop the spin loop and cleanly shut down rclpy."""
        if not self._running:
            return

        self._running = False

        # Destroy node first to unblock spin
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None

        # Shutdown rclpy
        try:
            rclpy.shutdown()
        except Exception:
            pass

        # Join the thread
        if self._thread is not None and self._thread.is_alive():
            try:
                self._thread.join(timeout=3.0)
            except Exception:
                pass
            self._thread = None

        self.ros_status_changed.emit(False)

    def _spin_loop(self) -> None:
        """Background thread: run rclpy executor."""
        try:
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(self._node)
            while self._running and rclpy.ok():
                executor.spin_once(timeout_sec=0.05)
        except Exception:
            pass

    # ── Publishing ───────────────────────────────────────────────────

    def publish_command(self, json_str: str) -> None:
        """
        Publish a JSON command string to the command topic.

        Thread-safe: may be called from the Qt main thread.
        """
        if self._node is not None and self._running:
            try:
                self._node.publish_command(json_str)
            except Exception:
                pass

    # ── Callbacks (called from ROS2 thread → emit Qt signals) ────────

    def _on_feedback(self, data: str) -> None:
        """Relay feedback JSON from ROS2 thread to Qt via signal."""
        self.feedback_received.emit(data)

    def _on_raw_command_xml(self, data: str) -> None:
        """Relay raw command XML from ROS2 thread to Qt via signal."""
        self.raw_command_xml_received.emit(data)

    def _on_raw_robot_xml(self, data: str) -> None:
        """Relay raw robot XML from ROS2 thread to Qt via signal."""
        self.raw_robot_xml_received.emit(data)

    # ── State ────────────────────────────────────────────────────────

    @property
    def is_running(self) -> bool:
        return self._running
