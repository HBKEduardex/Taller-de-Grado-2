"""
eki_axis_move_node.py — ROS2 node for KUKA XmlAxisMove bidirectional control.

This node:
  - Opens a TCP server on the configured port (default 59153).
  - Receives continuous <Robot> XML feedback from the KUKA (AxisActual,
    PositionActual, MoveReady, LimitsOK, DeltaOK, MoveExecuted).
  - Publishes feedback as JSON to /kuka/axis_move/feedback_json.
  - Publishes raw robot XML to /kuka/axis_move/raw_robot_xml.
  - Subscribes to /kuka/axis_move/target_json from the GUI.
  - Validates incoming targets against soft limits, max delta, seq rules.
  - Responds to each KUKA message with a <Command> XML containing the target.
  - Publishes the sent command XML to /kuka/axis_move/raw_command_xml.

Multi-layer safety:
  1. safe_mode=true → EnableMove always 0
  2. allow_motion_commands=false → EnableMove always 0
  3. Soft limits validation
  4. Max delta validation (target vs. current feedback)
  5. Seq must be > 0 (if require_seq_positive)
  6. Seq must be new (if reject_repeated_seq)
  7. Command timeout (stale commands get EnableMove=0)

Command pacing (telemetry optimisation):
  The KUKA SPS runs ~15 EKI_Get calls every time it finds a <Command> in its
  receive buffer, and that read is roughly half of the telemetry period. So a
  <Command> is only put on the wire when it actually carries something new:

    - the GUI requested a different command  -> sent immediately
    - EnableMove would be 1                  -> sent immediately (never withheld)
    - otherwise                              -> repeated once per
                                                command_heartbeat_period_sec

  Set command_heartbeat_period_sec to 0.0 to restore the previous behaviour of
  replying to every single <Robot> frame.

  Telemetry itself is NOT throttled: every <Robot> frame is still parsed and
  still published to the feedback topic, one for one.

This node does NOT modify any existing node.

Usage:
  ros2 launch kuka_eki_bridge axis_move.launch.py
"""

import json
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from kuka_eki_bridge.eki_axis_move_server import EkiAxisMoveServer
from kuka_eki_bridge.axis_move_xml_utils import (
    build_axis_move_command_xml,
    format_axis_move_log,
)

# ---------------------------------------------------------------------------
# Joint names in order
# ---------------------------------------------------------------------------
_AXES = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']
_CARTESIAN_AXES = ['X', 'Y', 'Z', 'A', 'B', 'C']

class EkiAxisMoveNode(Node):
    """ROS2 node for the KUKA XmlDualMove bidirectional control."""

    def __init__(self):
        super().__init__('eki_axis_move')

        # ── Declare parameters ───────────────────────────────────────
        self.declare_parameter('bind_host', '0.0.0.0')
        self.declare_parameter('port', 59153)
        self.declare_parameter('receive_buffer_size', 8192)

        self.declare_parameter('target_topic', '/kuka/axis_move/target_json')
        self.declare_parameter('feedback_topic', '/kuka/axis_move/feedback_json')
        self.declare_parameter('raw_robot_xml_topic', '/kuka/axis_move/raw_robot_xml')
        self.declare_parameter('raw_command_xml_topic', '/kuka/axis_move/raw_command_xml')

        self.declare_parameter('log_feedback_values', True)
        self.declare_parameter('log_raw_robot_xml', False)
        self.declare_parameter('log_command_xml', True)

        self.declare_parameter('safe_mode', True)
        self.declare_parameter('allow_motion_commands', False)
        self.declare_parameter('default_enable_move', False)

        # Default target (home position)
        self.declare_parameter('default_target_deg.A1', 0.0)
        self.declare_parameter('default_target_deg.A2', -90.0)
        self.declare_parameter('default_target_deg.A3', 90.0)
        self.declare_parameter('default_target_deg.A4', 0.0)
        self.declare_parameter('default_target_deg.A5', 0.0)
        self.declare_parameter('default_target_deg.A6', 0.0)

        # Soft limits: [min, max] per joint
        self.declare_parameter('soft_limits_deg.A1', [-160.0, 160.0])
        self.declare_parameter('soft_limits_deg.A2', [-180.0, 35.0])
        self.declare_parameter('soft_limits_deg.A3', [-110.0, 146.0])
        self.declare_parameter('soft_limits_deg.A4', [-175.0, 175.0])
        self.declare_parameter('soft_limits_deg.A5', [-110.0, 110.0])
        self.declare_parameter('soft_limits_deg.A6', [-340.0, 340.0])

        # Max delta per joint
        self.declare_parameter('max_delta_deg.A1', 2.0)
        self.declare_parameter('max_delta_deg.A2', 2.0)
        self.declare_parameter('max_delta_deg.A3', 2.0)
        self.declare_parameter('max_delta_deg.A4', 2.0)
        self.declare_parameter('max_delta_deg.A5', 2.0)
        self.declare_parameter('max_delta_deg.A6', 2.0)

        # Max delta for Cartesian
        self.declare_parameter('max_delta_pos_mm', 10.0)
        self.declare_parameter('max_delta_ori_deg', 5.0)

        # Seq validation
        self.declare_parameter('require_seq_positive', True)
        self.declare_parameter('reject_repeated_seq', True)
        self.declare_parameter('command_timeout_sec', 2.0)

        # Command pacing — see the module docstring. Default 0.5 s (~2 Hz).
        # 0.0 restores the legacy "reply to every <Robot> frame" behaviour.
        self.declare_parameter('command_heartbeat_period_sec', 0.5)

        # ── Read parameters ──────────────────────────────────────────
        self._host = self.get_parameter('bind_host').get_parameter_value().string_value
        self._port = self.get_parameter('port').get_parameter_value().integer_value
        self._recv_size = self.get_parameter('receive_buffer_size').get_parameter_value().integer_value

        self._target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        self._feedback_topic = self.get_parameter('feedback_topic').get_parameter_value().string_value
        self._raw_robot_xml_topic = self.get_parameter('raw_robot_xml_topic').get_parameter_value().string_value
        self._raw_command_xml_topic = self.get_parameter('raw_command_xml_topic').get_parameter_value().string_value

        self._log_feedback = self.get_parameter('log_feedback_values').get_parameter_value().bool_value
        self._log_raw_xml = self.get_parameter('log_raw_robot_xml').get_parameter_value().bool_value
        self._log_command_xml = self.get_parameter('log_command_xml').get_parameter_value().bool_value

        self._safe_mode = self.get_parameter('safe_mode').get_parameter_value().bool_value
        self._allow_motion = self.get_parameter('allow_motion_commands').get_parameter_value().bool_value
        self._default_enable_move = self.get_parameter('default_enable_move').get_parameter_value().bool_value

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

        # Max delta
        self._max_delta = {
            a: self.get_parameter(f'max_delta_deg.{a}').get_parameter_value().double_value
            for a in _AXES
        }

        self._max_delta_pos = self.get_parameter('max_delta_pos_mm').get_parameter_value().double_value
        self._max_delta_ori = self.get_parameter('max_delta_ori_deg').get_parameter_value().double_value

        # Seq validation
        self._require_seq_positive = self.get_parameter('require_seq_positive').get_parameter_value().bool_value
        self._reject_repeated_seq = self.get_parameter('reject_repeated_seq').get_parameter_value().bool_value
        self._command_timeout = self.get_parameter('command_timeout_sec').get_parameter_value().double_value
        self._heartbeat_period = self.get_parameter(
            'command_heartbeat_period_sec').get_parameter_value().double_value

        # ── Internal state ───────────────────────────────────────────
        self._target_lock = threading.Lock()
        self._last_valid_target = dict(self._default_target)
        self._last_valid_cartesian = {a: 0.0 for a in _CARTESIAN_AXES}
        self._last_mode = 'AxisTarget'
        self._last_enable_move: bool = self._default_enable_move
        self._last_cmd_seq: int = 0
        self._last_sent_seq: int = -1
        self._last_cmd_time: float = 0.0

        # Command pacing state. Touched only from the TCP server thread,
        # same as _last_sent_seq, so it needs no extra lock.
        self._last_command_signature: Optional[tuple] = None
        self._last_command_send_time: float = 0.0

        # Latest feedback from KUKA (for delta validation)
        self._feedback_lock = threading.Lock()
        self._last_feedback_actual: dict = dict(self._default_target)
        self._last_feedback_pos: dict = {a: 0.0 for a in _CARTESIAN_AXES}

        # ── Banner ───────────────────────────────────────────────────
        self.get_logger().info('╔══════════════════════════════════════════════╗')
        self.get_logger().info('║  KUKA EKI Axis Move — ROS2 Node             ║')
        self.get_logger().info('╚══════════════════════════════════════════════╝')
        self.get_logger().info(f'  Bind host:              {self._host}')
        self.get_logger().info(f'  Port:                   {self._port}')
        self.get_logger().info(f'  Safe mode:              {self._safe_mode}')
        self.get_logger().info(f'  Allow motion commands:  {self._allow_motion}')
        self.get_logger().info(f'  Target topic:           {self._target_topic}')
        self.get_logger().info(f'  Feedback topic:         {self._feedback_topic}')
        self.get_logger().info('  Default target: ' +
            ' '.join(f'{a}={v:.1f}' for a, v in self._default_target.items()))
        self.get_logger().info('  Max delta: ' +
            ' '.join(f'{a}={v:.1f}' for a, v in self._max_delta.items()))
        self.get_logger().info(
            f'  Command heartbeat:      {self._heartbeat_period:.3f} s'
            + ('  (0 = reply to every frame)' if self._heartbeat_period <= 0.0 else ''))
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
        self._server = EkiAxisMoveServer(
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
        enable_move = bool(data.get('enable_move', self._default_enable_move))

        # Extract seq and mode
        cmd_seq = int(data.get('seq', 0))
        mode = str(data.get('mode', 'AxisTarget'))

        # Support both flat layout (legacy) and nested layout (new)
        candidate_axis: dict = {}
        missing_axis = False
        
        axis_source = data.get('axis_target', data)
        for a in _AXES:
            val = axis_source.get(a)
            if val is None:
                missing_axis = True
                break
            try:
                candidate_axis[a] = float(val)
            except (TypeError, ValueError):
                missing_axis = True
                break

        # Cartesian parsing
        candidate_cartesian: dict = {}
        missing_cartesian = False
        cart_source = data.get('cartesian_target', {})
        for a in _CARTESIAN_AXES:
            val = cart_source.get(a)
            if val is None:
                missing_cartesian = True
                break
            try:
                candidate_cartesian[a] = float(val)
            except (TypeError, ValueError):
                missing_cartesian = True
                break
                
        # Validate missing based on mode
        if mode == 'AxisTarget' and missing_axis:
            self.get_logger().warn(f'Target JSON missing axis values for AxisTarget mode — rejecting.')
            return
        elif mode == 'CartesianTarget' and missing_cartesian:
            self.get_logger().warn(f'Target JSON missing cartesian values for CartesianTarget mode — rejecting.')
            return

        # Validate soft limits for Axis mode
        if mode == 'AxisTarget':
            for a, val in candidate_axis.items():
                lo, hi = self._soft_limits.get(a, (-360.0, 360.0))
                if not (lo <= val <= hi):
                    self.get_logger().warn(
                        f'Target {a}={val:.2f} out of soft limits '
                        f'[{lo:.1f}, {hi:.1f}] — target REJECTED.'
                    )
                    return

        # Accept the target
        with self._target_lock:
            if not missing_axis:
                self._last_valid_target = candidate_axis
            if not missing_cartesian:
                self._last_valid_cartesian = candidate_cartesian
            self._last_mode = mode
            self._last_enable_move = enable_move
            self._last_cmd_seq = cmd_seq
            self._last_cmd_time = time.monotonic()

        self.get_logger().info(
            f'[TARGET RECEIVED] seq={cmd_seq} mode={mode} enable={enable_move}'
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
        move_ready = parsed.get('move_ready', False)
        limits_ok = parsed.get('limits_ok', False)
        delta_ok = parsed.get('delta_ok', False)
        move_executed = parsed.get('move_executed', False)

        # Store latest feedback for delta validation
        with self._feedback_lock:
            self._last_feedback_actual = dict(axis_actual) if axis_actual else dict(self._default_target)
            self._last_feedback_pos = dict(pos_actual) if pos_actual else {a: 0.0 for a in _CARTESIAN_AXES}

        # ── Publish raw robot XML ────────────────────────────────────
        raw_msg = String()
        raw_msg.data = raw_xml
        self._pub_raw_robot.publish(raw_msg)

        # ── Build and publish feedback JSON ──────────────────────────
        feedback = {
            'seq': seq,
            'mode': mode,
            'status': status,
            'move_ready': move_ready,
            'limits_ok': limits_ok,
            'delta_ok': delta_ok,
            'move_executed': move_executed,
            'axis_actual': axis_actual,
            'position_actual': pos_actual,
            'bridge_safe_mode': self._safe_mode,
            'bridge_allow_motion': self._allow_motion,
        }
        fb_msg = String()
        fb_msg.data = json.dumps(feedback)
        self._pub_feedback.publish(fb_msg)

    # ── Command provider (called from TCP thread) ────────────────────

    def _build_command(self, parsed_feedback: dict) -> Optional[str]:
        """
        Build the <Command> XML to send back to the KUKA.

        Called from the TCP server thread with the parsed feedback dict.
        Applies all validation layers before allowing EnableMove=1.

        Returns None when this frame does not need a reply, which the TCP
        server already treats as "send nothing" — the socket stays open and
        the KUKA never waits for a reply, since EKI_Send is fire-and-forget.
        """
        with self._target_lock:
            target_snap = dict(self._last_valid_target)
            cart_snap = dict(self._last_valid_cartesian)
            mode_snap = self._last_mode
            enable_move_snap = self._last_enable_move
            cmd_seq = self._last_cmd_seq
            cmd_time = self._last_cmd_time

        # Get current feedback for delta validation
        with self._feedback_lock:
            feedback_actual = dict(self._last_feedback_actual)
            feedback_pos = dict(self._last_feedback_pos)

        # Signature of what the GUI is asking for right now. Compared further
        # down to decide whether this <Robot> frame earns a <Command> reply.
        signature = self._command_signature(
            cmd_seq, mode_snap, enable_move_snap, target_snap, cart_snap)

        # ── Validation layers ────────────────────────────────────────
        effective_enable = enable_move_snap
        reasons: list = []

        # Layer 1: safe_mode
        if self._safe_mode:
            effective_enable = False
            reasons.append('safe_mode')

        # Layer 2: allow_motion_commands
        if not self._allow_motion:
            effective_enable = False
            reasons.append('allow_motion_commands=false')

        # Layer 3: seq validation
        if effective_enable and self._require_seq_positive and cmd_seq <= 0:
            effective_enable = False
            reasons.append(f'seq={cmd_seq}<=0')

        # Layer 4: repeated seq
        if effective_enable and self._reject_repeated_seq and cmd_seq == self._last_sent_seq:
            effective_enable = False
            reasons.append(f'repeated_seq={cmd_seq}')

        # Layer 5: command timeout
        if effective_enable and self._command_timeout > 0:
            age = time.monotonic() - cmd_time
            if cmd_time == 0.0 or age > self._command_timeout:
                effective_enable = False
                reasons.append(f'timeout({age:.1f}s)')

        # Layer 6: Robot readiness
        if effective_enable:
            # We don't necessarily abort EnableMove if MoveReady=0 here,
            # as KUKA uses EnableMove=1 to *trigger* MoveReady.
            # However, if limits or delta are natively violated, block it.
            fb_limits = parsed_feedback.get('limits_ok', True)
            if not fb_limits:
                effective_enable = False
                reasons.append('robot_limits_violation')

        # Layer 7: max delta (Axis)
        if effective_enable and mode_snap == 'AxisTarget':
            for a in _AXES:
                target_val = target_snap.get(a, 0.0)
                actual_val = feedback_actual.get(a, target_val)
                delta = abs(target_val - actual_val)
                max_d = self._max_delta.get(a, 2.0)
                if delta > max_d:
                    effective_enable = False
                    reasons.append(f'{a} delta={delta:.2f}>{max_d:.1f}')
                    break

        # Layer 8: max delta (Cartesian)
        if effective_enable and mode_snap == 'CartesianTarget':
            for a in ['X', 'Y', 'Z']:
                t_val = cart_snap.get(a, 0.0)
                a_val = feedback_pos.get(a, t_val)
                delta = abs(t_val - a_val)
                if delta > self._max_delta_pos:
                    effective_enable = False
                    reasons.append(f'{a} delta={delta:.2f}>{self._max_delta_pos:.1f}')
                    break
            for a in ['A', 'B', 'C']:
                t_val = cart_snap.get(a, 0.0)
                a_val = feedback_pos.get(a, t_val)
                delta = abs(t_val - a_val)
                if delta > self._max_delta_ori:
                    effective_enable = False
                    reasons.append(f'{a} delta={delta:.2f}>{self._max_delta_ori:.1f}')
                    break

        # ── Send decision ────────────────────────────────────────────
        # Every reply we skip saves the KUKA SPS one full 15 x EKI_Get read,
        # which is what caps telemetry at ~6 Hz. Nothing is skipped that
        # carries new information or that would enable motion.
        now = time.monotonic()
        is_new_command = signature != self._last_command_signature
        heartbeat_due = (
            now - self._last_command_send_time) >= self._heartbeat_period

        if is_new_command:
            send_reason = 'new_command'
        elif effective_enable:
            # A frame that would put EnableMove=1 on the wire is never
            # withheld, not even for a few milliseconds.
            send_reason = 'enable_move'
        elif heartbeat_due:
            send_reason = 'heartbeat'
        else:
            send_reason = None

        if send_reason is None:
            self.get_logger().debug(
                f'[COMMAND SUPPRESSED] seq={cmd_seq} '
                f'since_last_send={now - self._last_command_send_time:.3f}s'
            )
            self._log_cycle(parsed_feedback, target_snap, effective_enable)
            return None

        # Track sent seq
        if effective_enable:
            self._last_sent_seq = cmd_seq
        else:
            # Log WHY EnableMove is blocked
            blocked_msg = (
                f'[ENABLE BLOCKED] seq={cmd_seq} enable_snap={enable_move_snap} '
                f'cmd_time={cmd_time:.1f} reasons={reasons}'
            )
            if is_new_command:
                self.get_logger().warn(blocked_msg)
            else:
                # A heartbeat repeats an already-sent seq on purpose, so
                # reject_repeated_seq forcing EnableMove=0 is the designed
                # outcome here — not an anomaly worth warning about twice a
                # second.
                self.get_logger().debug(blocked_msg)

        # ── Build XML ────────────────────────────────────────────────
        xml = build_axis_move_command_xml(
            seq=cmd_seq,
            target=target_snap,
            enable_move=effective_enable,
            safe_mode=False,  # Already handled above
            allow_motion=True,  # Already handled above
            cartesian_target=cart_snap,
            mode=mode_snap,
        )

        # Publish the sent command XML
        cmd_msg = String()
        cmd_msg.data = xml
        self._pub_raw_cmd.publish(cmd_msg)

        self._last_command_signature = signature
        self._last_command_send_time = now
        self.get_logger().debug(
            f'[COMMAND SEND] reason={send_reason} seq={cmd_seq} '
            f'enable={1 if effective_enable else 0}'
        )

        # ── Compact log ──────────────────────────────────────────────
        self._log_cycle(parsed_feedback, target_snap, effective_enable)

        return xml

    # ── Helpers ──────────────────────────────────────────────────────

    def _command_signature(
        self,
        seq: int,
        mode: str,
        enable_move: bool,
        target: dict,
        cartesian: dict,
    ) -> tuple:
        """
        Build a comparable signature of the command the GUI is requesting.

        Values are rounded to 4 decimals because build_axis_move_command_xml()
        formats them with '%.4f'. Two snapshots with the same signature would
        therefore produce a byte-identical <Command>, which makes "nothing
        changed" exact rather than approximate.

        Seq is part of the signature on purpose: joint_command_model.next_seq()
        bumps it on every GUI publish, so a fresh seq is the GUI explicitly
        asking for the command to be delivered again.
        """
        return (
            int(seq),
            str(mode),
            bool(enable_move),
            tuple(round(float(target.get(a, 0.0)), 4) for a in _AXES),
            tuple(round(float(cartesian.get(a, 0.0)), 4)
                  for a in _CARTESIAN_AXES),
        )

    def _log_cycle(
        self,
        parsed_feedback: dict,
        target_snap: dict,
        effective_enable: bool,
    ) -> None:
        """
        Emit the per-cycle compact log line.

        Unchanged content and level, and still emitted once per received
        <Robot> frame whether or not a <Command> went out, so the console
        keeps showing the real telemetry rate.
        """
        if not self._log_feedback:
            return

        line = format_axis_move_log(
            seq=parsed_feedback.get('seq', 0),
            mode=parsed_feedback.get('mode', 'Unknown'),
            actual=parsed_feedback.get('axis_actual', {}),
            target=target_snap,
            limits_ok=parsed_feedback.get('limits_ok', False),
            delta_ok=parsed_feedback.get('delta_ok', False),
            enable_move=1 if effective_enable else 0,
            safe_mode=self._safe_mode,
        )
        self.get_logger().info(line)

    # ── Cleanup ──────────────────────────────────────────────────────

    def destroy_node(self):
        """Clean shutdown: stop TCP server before destroying the node."""
        self.get_logger().info('Shutting down axis move node...')
        if hasattr(self, '_server'):
            self._server.stop()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    """Entry point for eki_axis_move_node."""
    rclpy.init(args=args)
    node = EkiAxisMoveNode()

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
