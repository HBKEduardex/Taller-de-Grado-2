"""
eki_axis_command_loop_node.py — ROS2 node for bidirectional KUKA axis command loop.

This node:
  - Opens a TCP server on port 59153.
  - Receives continuous <Robot> XML feedback from the KUKA (AxisActual, PositionActual).
  - Publishes feedback as JSON to /kuka/axis_command_loop/feedback_json.
  - Publishes raw robot XML to /kuka/axis_command_loop/raw_robot_xml.
  - Subscribes to /kuka/axis_command/target_json from the GUI.
  - Validates incoming targets against soft limits and safe_mode.
  - Stores the last valid target.
  - Responds to each KUKA message with a <Command> XML containing the target.
  - Publishes the sent command XML to /kuka/axis_command_loop/raw_command_xml.

This node does NOT modify any existing node.

Usage:
  ros2 launch kuka_eki_bridge axis_command_loop.launch.py
"""

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from kuka_eki_bridge.eki_axis_command_loop_server import EkiAxisCommandLoopServer
from kuka_eki_bridge.axis_command_loop_xml_utils import (
    build_command_xml,
    format_command_loop_log,
)

# ---------------------------------------------------------------------------
# Joint names in order
# ---------------------------------------------------------------------------
_AXES = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']


class EkiAxisCommandLoopNode(Node):
    """ROS2 node for the bidirectional KUKA axis command loop."""

    def __init__(self):
        super().__init__('eki_axis_command_loop')

        # ── Declare parameters ───────────────────────────────────────
        self.declare_parameter('bind_host', '0.0.0.0')
        self.declare_parameter('port', 59153)
        self.declare_parameter('receive_buffer_size', 8192)

        self.declare_parameter('target_topic', '/kuka/axis_command/target_json')
        self.declare_parameter('feedback_topic', '/kuka/axis_command_loop/feedback_json')
        self.declare_parameter('raw_robot_xml_topic', '/kuka/axis_command_loop/raw_robot_xml')
        self.declare_parameter('raw_command_xml_topic', '/kuka/axis_command_loop/raw_command_xml')

        self.declare_parameter('log_feedback_values', True)
        self.declare_parameter('log_raw_xml', False)
        self.declare_parameter('log_command_xml', True)

        self.declare_parameter('safe_mode', True)
        self.declare_parameter('enable_move_default', False)

        # Default target (home position)
        self.declare_parameter('default_target_deg.A1', 0.0)
        self.declare_parameter('default_target_deg.A2', -90.0)
        self.declare_parameter('default_target_deg.A3', 90.0)
        self.declare_parameter('default_target_deg.A4', 0.0)
        self.declare_parameter('default_target_deg.A5', 0.0)
        self.declare_parameter('default_target_deg.A6', 0.0)

        # Soft limits: [min, max] per joint — stored as flat params
        self.declare_parameter('soft_limits_deg.A1', [-160.0, 160.0])
        self.declare_parameter('soft_limits_deg.A2', [-180.0, 35.0])
        self.declare_parameter('soft_limits_deg.A3', [-110.0, 146.0])
        self.declare_parameter('soft_limits_deg.A4', [-175.0, 175.0])
        self.declare_parameter('soft_limits_deg.A5', [-110.0, 110.0])
        self.declare_parameter('soft_limits_deg.A6', [-340.0, 340.0])

        # ── Read parameters ──────────────────────────────────────────
        self._host = self.get_parameter('bind_host').get_parameter_value().string_value
        self._port = self.get_parameter('port').get_parameter_value().integer_value
        self._recv_size = self.get_parameter('receive_buffer_size').get_parameter_value().integer_value

        self._target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        self._feedback_topic = self.get_parameter('feedback_topic').get_parameter_value().string_value
        self._raw_robot_xml_topic = self.get_parameter('raw_robot_xml_topic').get_parameter_value().string_value
        self._raw_command_xml_topic = self.get_parameter('raw_command_xml_topic').get_parameter_value().string_value

        self._log_feedback = self.get_parameter('log_feedback_values').get_parameter_value().bool_value
        self._log_raw_xml = self.get_parameter('log_raw_xml').get_parameter_value().bool_value
        self._log_command_xml = self.get_parameter('log_command_xml').get_parameter_value().bool_value

        self._safe_mode = self.get_parameter('safe_mode').get_parameter_value().bool_value
        self._enable_move_default = self.get_parameter('enable_move_default').get_parameter_value().bool_value

        # Default target
        self._default_target = {
            a: self.get_parameter(f'default_target_deg.{a}').get_parameter_value().double_value
            for a in _AXES
        }

        # Soft limits
        self._soft_limits = {}
        for a in _AXES:
            limits = self.get_parameter(f'soft_limits_deg.{a}').get_parameter_value().double_array_value
            if len(limits) == 2:
                self._soft_limits[a] = (limits[0], limits[1])
            else:
                self._soft_limits[a] = (-360.0, 360.0)

        # ── Internal state ───────────────────────────────────────────
        self._target_lock = threading.Lock()
        self._last_valid_target = dict(self._default_target)
        self._last_enable_move: bool = self._enable_move_default

        # ── Banner ───────────────────────────────────────────────────
        self.get_logger().info('╔══════════════════════════════════════════════╗')
        self.get_logger().info('║  KUKA EKI Axis Command Loop — ROS2 Node     ║')
        self.get_logger().info('╚══════════════════════════════════════════════╝')
        self.get_logger().info(f'  Bind host:     {self._host}')
        self.get_logger().info(f'  Port:          {self._port}')
        self.get_logger().info(f'  Safe mode:     {self._safe_mode}')
        self.get_logger().info(f'  Target topic:  {self._target_topic}')
        self.get_logger().info(f'  Feedback topic:{self._feedback_topic}')
        self.get_logger().info('  Default target: ' +
            ' '.join(f'{a}={v:.1f}' for a, v in self._default_target.items()))
        self.get_logger().info('──────────────────────────────────────────────')

        # ── Publishers ───────────────────────────────────────────────
        self._pub_feedback = self.create_publisher(String, self._feedback_topic, 10)
        self._pub_raw_robot = self.create_publisher(String, self._raw_robot_xml_topic, 10)
        self._pub_raw_cmd = self.create_publisher(String, self._raw_command_xml_topic, 10)

        self.get_logger().info(f'Publishing feedback to: {self._feedback_topic}')
        self.get_logger().info(f'Publishing raw robot XML to: {self._raw_robot_xml_topic}')
        self.get_logger().info(f'Publishing raw command XML to: {self._raw_command_xml_topic}')

        # ── Subscriber ───────────────────────────────────────────────
        self._sub_target = self.create_subscription(
            String,
            self._target_topic,
            self._on_target_json,
            10,
        )
        self.get_logger().info(f'Subscribed to target topic: {self._target_topic}')

        # ── TCP server ───────────────────────────────────────────────
        self._server = EkiAxisCommandLoopServer(
            host=self._host,
            port=self._port,
            logger=self.get_logger(),
            on_feedback=self._on_robot_message,
            get_command_xml=self._build_command,
            receive_buffer_size=self._recv_size,
            log_raw_xml=self._log_raw_xml,
            log_command_xml=self._log_command_xml,
        )

        try:
            self._server.start()
        except RuntimeError as e:
            self.get_logger().fatal(f'Server failed to start: {e}')
            raise SystemExit(1)

    # ── Target subscriber callback ───────────────────────────────────

    def _on_target_json(self, msg: String) -> None:
        """
        Process a new target JSON published by the GUI.

        Validates the target against soft limits and, if valid, stores it
        as the last known good target. Invalid targets are rejected with
        a warning.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Invalid JSON on target topic: {e}')
            return

        # Extract enable_move
        enable_move = bool(data.get('enable_move', self._enable_move_default))

        # Extract axis values
        candidate: dict = {}
        missing = False
        for a in _AXES:
            val = data.get(a)
            if val is None:
                self.get_logger().warn(
                    f'Target JSON missing axis "{a}" — rejecting.'
                )
                missing = True
                break
            try:
                candidate[a] = float(val)
            except (TypeError, ValueError):
                self.get_logger().warn(
                    f'Target JSON non-numeric value for "{a}" — rejecting.'
                )
                missing = True
                break

        if missing:
            return

        # Validate soft limits
        if self._safe_mode:
            for a, val in candidate.items():
                lo, hi = self._soft_limits.get(a, (-360.0, 360.0))
                if not (lo <= val <= hi):
                    self.get_logger().warn(
                        f'Target {a}={val:.2f} out of soft limits '
                        f'[{lo:.1f}, {hi:.1f}] — target REJECTED.'
                    )
                    return

        # Accept the target
        with self._target_lock:
            self._last_valid_target = candidate
            self._last_enable_move = enable_move

        self.get_logger().debug(
            f'New target accepted: ' +
            ' '.join(f'{a}={v:.2f}' for a, v in candidate.items())
        )

    # ── Feedback callback (called from TCP thread) ───────────────────

    def _on_robot_message(self, parsed: dict, raw_xml: str) -> None:
        """
        Handle parsed feedback from the KUKA.

        Called from the TCP server thread — must be thread-safe for
        ROS2 publishers (rclpy is thread-safe for publishing).
        """
        seq = parsed.get('seq', 0)
        mode = parsed.get('mode', 'Unknown')
        status = parsed.get('status', 0)
        axis_actual = parsed.get('axis_actual', {})
        pos_actual = parsed.get('position_actual', {})

        # ── Publish raw robot XML ────────────────────────────────────
        raw_msg = String()
        raw_msg.data = raw_xml
        self._pub_raw_robot.publish(raw_msg)

        # ── Build and publish feedback JSON ──────────────────────────
        feedback = {
            'seq': seq,
            'mode': mode,
            'status': status,
            'axis_actual': axis_actual,
            'position_actual': pos_actual,
        }
        fb_msg = String()
        fb_msg.data = json.dumps(feedback)
        self._pub_feedback.publish(fb_msg)

        # ── Compact log ──────────────────────────────────────────────
        if self._log_feedback:
            with self._target_lock:
                target_snap = dict(self._last_valid_target)
                enable_move_snap = self._last_enable_move

            effective_enable = 0 if self._safe_mode else (1 if enable_move_snap else 0)

            line = format_command_loop_log(
                seq=seq,
                mode=mode,
                actual=axis_actual,
                target=target_snap,
                enable_move=effective_enable,
            )
            self.get_logger().info(line)

    # ── Command provider (called from TCP thread) ────────────────────

    def _build_command(self, seq: int) -> str:
        """
        Build the <Command> XML to send back to the KUKA.

        Called from the TCP server thread with the sequence number
        from the received <Robot> message.
        """
        with self._target_lock:
            target_snap = dict(self._last_valid_target)
            enable_move_snap = self._last_enable_move

        xml = build_command_xml(
            seq=seq,
            target=target_snap,
            enable_move=enable_move_snap,
            safe_mode=self._safe_mode,
        )

        # Publish the sent command XML
        cmd_msg = String()
        cmd_msg.data = xml
        self._pub_raw_cmd.publish(cmd_msg)

        return xml

    # ── Cleanup ──────────────────────────────────────────────────────

    def destroy_node(self):
        """Clean shutdown: stop TCP server before destroying the node."""
        self.get_logger().info('Shutting down axis command loop node...')
        if hasattr(self, '_server'):
            self._server.stop()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    """Entry point for eki_axis_command_loop_node."""
    rclpy.init(args=args)
    node = EkiAxisCommandLoopNode()

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
