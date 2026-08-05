"""
ros_moveit_mirror_bridge.py — ROS2 bridge for mirroring commands to RViz/MoveIt2.

Publishes joint and cartesian commands to the kuka_gui_moveit_bridge package.
Subscribes to RViz/MoveIt2 feedback topics.

This module does NOT call rclpy.init(). It receives an existing rclpy Node
and creates publishers/subscribers on it.

Topics published:
  /kuka_bridge/joint_command_deg      (Float64MultiArray)  [A1..A6 in degrees]
  /kuka_bridge/cartesian_command_deg  (Float64MultiArray)  [X,Y,Z in m, A,B,C in deg]

Topics subscribed:
  /kuka_bridge/status                 (String)
  /kuka_bridge/joint_state_deg        (Float64MultiArray)
  /kuka_bridge/cartesian_state_deg    (Float64MultiArray)
"""

import json
from typing import List, Optional

from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray

try:
    from PyQt5.QtCore import QObject, pyqtSignal
except ImportError as e:
    raise ImportError(
        'PyQt5 is required. Install with: sudo apt install python3-pyqt5'
    ) from e


# ---------------------------------------------------------------------------
# Axis labels for JSON conversion
# ---------------------------------------------------------------------------

_JOINT_LABELS = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']
_CART_LABELS = ['X', 'Y', 'Z', 'A', 'B', 'C']


# ---------------------------------------------------------------------------
# Bridge
# ---------------------------------------------------------------------------

class RosMoveitMirrorBridge(QObject):
    """
    Thread-safe bridge between RViz/MoveIt2 topics and PyQt5.

    Signals (emitted from the ROS2 thread, connected to Qt slots):
      rviz_status_received(str)            — status text from MoveIt
      rviz_joint_state_received(str)       — JSON {A1:.., A2:.., ...}
      rviz_cartesian_state_received(str)   — JSON {X:.., Y:.., ...}

    All publishing and subscribing reuses the given rclpy Node.
    """

    rviz_status_received = pyqtSignal(str)
    rviz_joint_state_received = pyqtSignal(str)
    rviz_cartesian_state_received = pyqtSignal(str)

    def __init__(
        self,
        node: Node,
        joint_command_topic: str = '/kuka_bridge/joint_command_deg',
        cartesian_command_topic: str = '/kuka_bridge/cartesian_command_deg',
        status_topic: str = '/kuka_bridge/status',
        joint_state_topic: str = '/kuka_bridge/joint_state_deg',
        cartesian_state_topic: str = '/kuka_bridge/cartesian_state_deg',
        parent=None,
    ):
        super().__init__(parent)
        self._node = node

        # ── Publishers ───────────────────────────────────────────────
        self._pub_joints = node.create_publisher(
            Float64MultiArray, joint_command_topic, 10,
        )
        node.get_logger().info(
            f'[MoveIt Mirror] Publishing joints to: {joint_command_topic}'
        )

        self._pub_cartesian = node.create_publisher(
            Float64MultiArray, cartesian_command_topic, 10,
        )
        node.get_logger().info(
            f'[MoveIt Mirror] Publishing cartesian to: {cartesian_command_topic}'
        )

        # ── Subscribers ──────────────────────────────────────────────
        node.create_subscription(
            String, status_topic,
            self._on_status, 10,
        )
        node.get_logger().info(
            f'[MoveIt Mirror] Subscribed to status: {status_topic}'
        )

        node.create_subscription(
            Float64MultiArray, joint_state_topic,
            self._on_joint_state, 10,
        )
        node.get_logger().info(
            f'[MoveIt Mirror] Subscribed to joint state: {joint_state_topic}'
        )

        node.create_subscription(
            Float64MultiArray, cartesian_state_topic,
            self._on_cartesian_state, 10,
        )
        node.get_logger().info(
            f'[MoveIt Mirror] Subscribed to cartesian state: {cartesian_state_topic}'
        )

    # ── Publishing ───────────────────────────────────────────────────

    def publish_joints(self, values: List[float]) -> None:
        """
        Publish joint targets to RViz/MoveIt2.

        Args:
            values: list of 6 floats [A1, A2, A3, A4, A5, A6] in degrees.
        """
        if len(values) != 6:
            return
        msg = Float64MultiArray()
        msg.data = [float(v) for v in values]
        try:
            self._pub_joints.publish(msg)
            self._node.get_logger().info(f'[MoveIt Mirror] Published joint command: {values}')
        except Exception as e:
            self._node.get_logger().error(f'[MoveIt Mirror] Failed to publish joints: {e}')

    def publish_cartesian(self, values: List[float]) -> None:
        """
        Publish cartesian target to RViz/MoveIt2.

        Args:
            values: list of 6 floats [X, Y, Z, A, B, C].
                    X, Y, Z in meters; A, B, C in degrees.
        """
        if len(values) != 6:
            return
        msg = Float64MultiArray()
        msg.data = [float(v) for v in values]
        try:
            self._pub_cartesian.publish(msg)
            self._node.get_logger().info(f'[MoveIt Mirror] Published cartesian command: {values}')
        except Exception as e:
            self._node.get_logger().error(f'[MoveIt Mirror] Failed to publish cartesian: {e}')

    # ── Callbacks (ROS2 thread → Qt signals) ─────────────────────────

    def _on_status(self, msg: String) -> None:
        """Relay MoveIt status text to Qt."""
        self.rviz_status_received.emit(msg.data)

    def _on_joint_state(self, msg: Float64MultiArray) -> None:
        """Convert Float64MultiArray to JSON dict and emit."""
        data = list(msg.data)
        result = {}
        for i, label in enumerate(_JOINT_LABELS):
            if i < len(data):
                result[label] = round(data[i], 6)
            else:
                result[label] = None
        self.rviz_joint_state_received.emit(json.dumps(result))

    def _on_cartesian_state(self, msg: Float64MultiArray) -> None:
        """Convert Float64MultiArray to JSON dict and emit."""
        data = list(msg.data)
        result = {}
        for i, label in enumerate(_CART_LABELS):
            if i < len(data):
                result[label] = round(data[i], 6)
            else:
                result[label] = None
        self.rviz_cartesian_state_received.emit(json.dumps(result))
