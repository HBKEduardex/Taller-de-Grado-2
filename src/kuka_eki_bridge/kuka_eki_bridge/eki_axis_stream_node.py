"""
eki_axis_stream_node.py — ROS2 node for continuous KUKA axis streaming.

This node starts a TCP server that receives a continuous stream of
<Robot> XML messages from the KUKA containing $AXIS_ACT and $POS_ACT
values. It does NOT modify the existing eki_xml_server_node.

Features:
  - Handles TCP stream fragmentation and concatenation.
  - Extracts individual <Robot>...</Robot> messages from the byte stream.
  - Logs compact axis/position values to the console.
  - Publishes raw XML to /kuka/axis_stream/raw_xml (std_msgs/String).
  - Optionally publishes sensor_msgs/JointState to /joint_states.
  - Supports reconnection when the KUKA disconnects.
  - Does NOT send movement commands to the robot.

Usage:
  ros2 launch kuka_eki_bridge axis_stream.launch.py
  ros2 topic echo /kuka/axis_stream/raw_xml
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from kuka_eki_bridge.eki_axis_stream_server import EkiAxisStreamServer
from kuka_eki_bridge.axis_xml_utils import (
    parse_axis_stream_xml,
    format_compact_line,
    axis_degrees_to_radians,
    pretty_xml,
)

# Joint names for the KUKA KR6 R900 (or similar 6-axis KUKA)
KUKA_JOINT_NAMES = [
    'joint_a1',
    'joint_a2',
    'joint_a3',
    'joint_a4',
    'joint_a5',
    'joint_a6',
]


class EkiAxisStreamNode(Node):
    """ROS2 node for continuous axis position streaming from KUKA."""

    def __init__(self):
        super().__init__('eki_axis_stream')

        # ── Declare parameters ───────────────────────────────────────
        self.declare_parameter('bind_host', '0.0.0.0')
        self.declare_parameter('port', 59152)
        self.declare_parameter('receive_buffer_size', 8192)
        self.declare_parameter('log_raw_xml', False)
        self.declare_parameter('log_pretty_xml', False)
        self.declare_parameter('log_axis_values', True)
        self.declare_parameter('publish_raw_xml', True)
        self.declare_parameter('publish_joint_states', False)
        self.declare_parameter('send_response', False)

        # ── Read parameters ──────────────────────────────────────────
        self._bind_host = (
            self.get_parameter('bind_host').get_parameter_value().string_value
        )
        self._port = (
            self.get_parameter('port').get_parameter_value().integer_value
        )
        self._recv_size = (
            self.get_parameter('receive_buffer_size')
            .get_parameter_value().integer_value
        )
        self._log_raw = (
            self.get_parameter('log_raw_xml')
            .get_parameter_value().bool_value
        )
        self._log_pretty = (
            self.get_parameter('log_pretty_xml')
            .get_parameter_value().bool_value
        )
        self._log_axis = (
            self.get_parameter('log_axis_values')
            .get_parameter_value().bool_value
        )
        self._pub_raw = (
            self.get_parameter('publish_raw_xml')
            .get_parameter_value().bool_value
        )
        self._pub_joints = (
            self.get_parameter('publish_joint_states')
            .get_parameter_value().bool_value
        )
        self._send_resp = (
            self.get_parameter('send_response')
            .get_parameter_value().bool_value
        )

        # ── Banner ───────────────────────────────────────────────────
        self.get_logger().info('╔══════════════════════════════════════════════╗')
        self.get_logger().info('║   KUKA EKI Axis Stream — ROS2 Node          ║')
        self.get_logger().info('╚══════════════════════════════════════════════╝')
        self.get_logger().info(f'  Bind host:          {self._bind_host}')
        self.get_logger().info(f'  Port:               {self._port}')
        self.get_logger().info(f'  Buffer size:        {self._recv_size}')
        self.get_logger().info(f'  Log raw XML:        {self._log_raw}')
        self.get_logger().info(f'  Log pretty XML:     {self._log_pretty}')
        self.get_logger().info(f'  Log axis values:    {self._log_axis}')
        self.get_logger().info(f'  Publish raw XML:    {self._pub_raw}')
        self.get_logger().info(f'  Publish JointState: {self._pub_joints}')
        self.get_logger().info(f'  Send response:      {self._send_resp}')
        self.get_logger().info('──────────────────────────────────────────────')

        # ── Create publishers ────────────────────────────────────────
        if self._pub_raw:
            self._raw_pub = self.create_publisher(
                String, '/kuka/axis_stream/raw_xml', 10
            )
            self.get_logger().info(
                'Publishing raw XML to: /kuka/axis_stream/raw_xml'
            )

        if self._pub_joints:
            try:
                from sensor_msgs.msg import JointState
                self._JointState = JointState
                self._joint_pub = self.create_publisher(
                    JointState, '/joint_states', 10
                )
                self.get_logger().info(
                    'Publishing JointState to: /joint_states'
                )
            except ImportError:
                self.get_logger().error(
                    'sensor_msgs not available — disabling JointState publisher.'
                )
                self._pub_joints = False

        # ── Message counter ──────────────────────────────────────────
        self._msg_count = 0

        # ── Create and start the TCP server ──────────────────────────
        self._server = EkiAxisStreamServer(
            host=self._bind_host,
            port=self._port,
            logger=self.get_logger(),
            on_message=self._on_robot_message,
            receive_buffer_size=self._recv_size,
            send_response=self._send_resp,
        )

        try:
            self._server.start()
        except RuntimeError as e:
            self.get_logger().fatal(f'Server failed to start: {e}')
            raise SystemExit(1)

    def _on_robot_message(self, xml_string: str) -> None:
        """
        Callback for each complete <Robot>...</Robot> message.

        Args:
            xml_string: Complete XML message string.
        """
        self._msg_count += 1

        # ── Log raw XML ──────────────────────────────────────────────
        if self._log_raw:
            self.get_logger().info(f'[RAW] {xml_string}')

        # ── Log pretty XML ───────────────────────────────────────────
        if self._log_pretty:
            formatted = pretty_xml(xml_string)
            if formatted:
                self.get_logger().info(f'[PRETTY]\n{formatted}')

        # ── Parse the XML ────────────────────────────────────────────
        parsed = parse_axis_stream_xml(xml_string)

        if parsed is None:
            self.get_logger().warn(
                f'[MSG #{self._msg_count}] Malformed XML — skipping.'
            )
            return

        # ── Log compact axis values ──────────────────────────────────
        if self._log_axis:
            line = format_compact_line(parsed)
            self.get_logger().info(line)

        # ── Publish raw XML ──────────────────────────────────────────
        if self._pub_raw:
            msg = String()
            msg.data = xml_string
            self._raw_pub.publish(msg)

        # ── Publish JointState ───────────────────────────────────────
        if self._pub_joints and 'axis' in parsed:
            self._publish_joint_state(parsed['axis'])

    def _publish_joint_state(self, axis: dict) -> None:
        """
        Publish a sensor_msgs/JointState message from axis values.

        Converts degrees to radians.

        Args:
            axis: Dictionary with keys A1-A6 in degrees.
        """
        js = self._JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = KUKA_JOINT_NAMES
        js.position = axis_degrees_to_radians(axis)
        js.velocity = []
        js.effort = []
        self._joint_pub.publish(js)

    def destroy_node(self):
        """Clean shutdown: stop the TCP server before destroying the node."""
        self.get_logger().info('Shutting down axis stream node...')
        if hasattr(self, '_server'):
            self._server.stop()
        super().destroy_node()


def main(args=None):
    """Entry point for the eki_axis_stream_node."""
    rclpy.init(args=args)
    node = EkiAxisStreamNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt — shutting down.')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
