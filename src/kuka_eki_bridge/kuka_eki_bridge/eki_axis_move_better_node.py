"""
eki_axis_move_better_node.py — bridge node with BATCH support.

Fork of eki_axis_move_node.py.  The original file is NOT modified and keeps
serving the baseline exactly as before.

This is a NEW FILE, not a new ROS node: same node name, same topics, same TCP
port, same EKI channel.  You run EITHER the baseline node OR this one, never
both — the TCP server owns port 59153, so they are mutually exclusive by
construction, exactly like XmlDualMove.src and XmlDualMove_better.src.

What is added, all of it optional and absent-by-default:
  - the target JSON may carry batch fields (batch_seq, batch_points_deg,
    batch_ptp_velocity_pct, abort_batch).  A command WITHOUT them produces a
    byte-identical <Command> to the baseline, so SEND, jog, HOME and the
    gripper behave exactly as they do today;
  - a batch command emits <Batch/> rows the SPS drains with EKI_GetRealArray;
  - the feedback JSON gains batch_seq / batch_consumed / batch_active, read
    from the SAME periodic telemetry frame that already carries RxCounter.

Requires XmlDualMove_better.xml + sps_submit_better.sub + the extended
$CONFIG on the controller.  Those are supersets, so the baseline program
still runs against them unchanged.

Original header follows.
--------------------------------------------------------------------------
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

EKI receive-buffer guard:
  EthernetKRL does not discard old elements when its receive memory fills up —
  it CLOSES the TCP connection (KST Ethernet KRL 3.0, p. 97-98: "Se ha cerrado
  la conexion Ethernet para evitar la recepcion de mas datos"). The default
  limit is 16 elements per memory, and XmlDualMove.xml declares no <INTERNAL>
  block, so that default applies.

  The SPS only reads its receive buffer while XD_CMD_RECEIVED is FALSE. When
  the KRL program stops consuming, commands pile up and the link dies. This
  has already happened: several telemetry logs end abruptly after exactly 16
  consecutive rounds in which the SPS skipped its read block.

  So the node keeps a deliberately conservative estimate of how many commands
  the KUKA has not read yet, and stops sending well before the limit:

    +1  every <Command> actually written to the socket   (a certainty)
    -1  every frame whose inter-arrival time says the SPS ran its read block
     0  whenever the KUKA (re)connects — the buffers die with the connection

  Detection errors are biased towards safety: a missed read keeps the estimate
  too high, which only makes the node send less.

  Since the KUKA now publishes Robot/RxCounter — the SPS's own count of
  complete commands taken out of the buffer — the estimate is driven by that
  explicit acknowledgement instead of by timing. The timing heuristic is kept
  only as a fallback for a controller that has not been updated yet, so the
  bridge stays usable against both.

Gripper:
  Commands carry <GripperCommand>: -1 do nothing, 0 open, 1 close. It defaults
  to -1 and is forced to -1 whenever safe_mode is on or allow_motion_commands
  is off, because the gripper is a physical action just like motion. The GUI
  does not drive it yet; a target JSON may carry an optional "gripper_command"
  field.

This node does NOT modify any existing node.

Usage:
  ros2 launch kuka_eki_bridge axis_move.launch.py
"""

import json
import math
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from kuka_eki_bridge.eki_axis_move_server import EkiAxisMoveServer
from kuka_eki_bridge.axis_move_better_xml_utils import (
    build_axis_move_command_xml,
    build_axis_move_batch_command_xml,
    build_abort_batch_command_xml,
    format_axis_move_log,
    parse_axis_move_xml as parse_axis_move_xml_better,
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

        # EKI receive-buffer guard — see the module docstring.
        # consume_threshold_sec separates the two measured regimes: a round
        # where the SPS skipped its read block (~91 ms) from one where it ran
        # it (~168 ms). Retune it if the SPS loop is ever made cheaper.
        self.declare_parameter('consume_threshold_sec', 0.130)
        self.declare_parameter('max_pending_heartbeat', 4)
        self.declare_parameter('max_pending_command', 10)
        self.declare_parameter('publish_bridge_diagnostics', True)
        self.declare_parameter(
            'bridge_diagnostics_topic',
            '/kuka/axis_move/bridge_diagnostics_json')

        # ── BATCH MODE ───────────────────────────────────────────────
        # Hard ceiling applied to anything arriving on the target topic.
        # The GUI has its own, lower, configurable limit; this one is the
        # last line of defence and is sized against XmlDualMove_better.xml
        # (<BUFFERING Limit="128">) and XD_BATCH_MAX in the mailbox.
        self.declare_parameter('max_batch_size', 20)
        # A batch packet is ONE command against the EKI guard, but it drops
        # N entries into each of the six Command/Batch/@Ax memories. Refuse
        # to add another batch while this many commands are still unread.
        self.declare_parameter('max_pending_batch', 2)
        # Ventana durante la que este nodo NO manda comandos de punto suelto
        # despues de poner un lote en el cable. Cubre el retardo entre
        # "lote enviado" y Robot/BatchActive=1 (telemetria ~200 ms, el .src
        # mira el buzon cada 100 ms) y los huecos entre lotes consecutivos.
        self.declare_parameter('batch_command_hold_sec', 2.0)
        # Plazo MAXIMO para que el robot reclame un lote que ya viaja. No es
        # un ajuste de rendimiento: es el tope que impide que este nodo se
        # quede mudo para siempre si un lote es rechazado en el KUKA. Por
        # encima del stall watchdog del ejecutor (20 s), para que sea SIEMPRE
        # el ejecutor quien decida que un lote ha fracasado, no este reloj.
        self.declare_parameter('batch_ack_timeout_sec', 30.0)

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
        self._consume_threshold = self.get_parameter(
            'consume_threshold_sec').get_parameter_value().double_value
        self._max_pending_heartbeat = self.get_parameter(
            'max_pending_heartbeat').get_parameter_value().integer_value
        self._max_pending_command = self.get_parameter(
            'max_pending_command').get_parameter_value().integer_value
        self._publish_diagnostics = self.get_parameter(
            'publish_bridge_diagnostics').get_parameter_value().bool_value
        self._diagnostics_topic = self.get_parameter(
            'bridge_diagnostics_topic').get_parameter_value().string_value
        self._max_batch_size = self.get_parameter(
            'max_batch_size').get_parameter_value().integer_value
        self._max_pending_batch = self.get_parameter(
            'max_pending_batch').get_parameter_value().integer_value

        # ── Internal state ───────────────────────────────────────────
        self._target_lock = threading.Lock()
        self._last_valid_target = dict(self._default_target)
        self._last_valid_cartesian = {a: 0.0 for a in _CARTESIAN_AXES}
        self._last_mode = 'AxisTarget'
        self._last_enable_move: bool = self._default_enable_move
        self._last_cmd_seq: int = 0
        self._last_sent_seq: int = -1
        self._last_cmd_time: float = 0.0
        # -1 = no gripper action. Never anything else unless a target JSON
        # explicitly asks for 0 or 1 and both safety gates are open.
        self._last_gripper_command: int = -1

        # ── BATCH state (guarded by _target_lock like everything above) ──
        # None means "no batch pending": the node then behaves exactly like
        # the baseline and builds an ordinary single-point <Command>.
        self._pending_batch: Optional[list] = None
        self._pending_batch_seq: int = 0
        self._pending_batch_velocity: float = 0.0
        self._batch_sent_seq: int = 0
        self._last_batch_send_at: float = 0.0
        self._last_batch_progress_at: float = 0.0
        self._batch_hold_sec = float(
            self.get_parameter('batch_command_hold_sec')
            .get_parameter_value().double_value)
        self._batch_ack_timeout = float(
            self.get_parameter('batch_ack_timeout_sec')
            .get_parameter_value().double_value)
        # True cuando se ha visto que el interprete del robot RECLAMO el lote
        # que este nodo mando por ultima vez (Robot/BatchActive=1 o el
        # contador de puntos avanzando). Hasta entonces el lote esta en el
        # buzon pero nadie lo ha cogido.
        self._batch_ack_seen: bool = False
        self._abort_batch_request: bool = False
        # Mirror of the KUKA's own batch telemetry.
        self._robot_batch_seq = None
        self._robot_batch_consumed = None
        self._robot_batch_active = None
        # 0.0 = comando no procedente de ENVIAR TRAYECTORIA. Los comandos
        # manuales no imponen ninguna velocidad PTP al controlador.
        self._last_trajectory_ptp_velocity_pct: float = 0.0

        # Command pacing state. Touched only from the TCP server thread,
        # same as _last_sent_seq, so it needs no extra lock.
        self._last_command_signature: Optional[tuple] = None
        self._last_command_send_time: float = 0.0

        # EKI receive-buffer guard state. Same threading story as
        # _last_sent_seq: only the TCP server thread touches these.
        self._pending_commands: int = 0
        self._last_frame_monotonic: float = 0.0
        self._last_frame_delta: float = 0.0
        # Robot/RxCounter mirror. None until the KUKA publishes one; while it
        # stays None the timing fallback drives the estimate instead.
        self._last_rx_counter = None
        self._rx_ack_available: bool = False
        self._guard_block_count: int = 0
        self._last_guard_warn_time: float = 0.0

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
        self.get_logger().info(
            f'  Batch hold:             {self._batch_hold_sec:.3f} s tras el '
            f'acuse, sin limite hasta el acuse (tope '
            f'{self._batch_ack_timeout:.1f} s)')
        self.get_logger().info(
            f'  EKI buffer guard:       heartbeat stops at '
            f'{self._max_pending_heartbeat}, all commands stop at '
            f'{self._max_pending_command} (EKI limit is 16)')
        self.get_logger().info(
            f'  Consume threshold:      {self._consume_threshold * 1000:.0f} ms')
        self.get_logger().info('──────────────────────────────────────────────')

        # ── Publishers ───────────────────────────────────────────────
        self._pub_feedback = self.create_publisher(String, self._feedback_topic, 10)
        self._pub_raw_robot = self.create_publisher(String, self._raw_robot_xml_topic, 10)
        self._pub_raw_cmd = self.create_publisher(String, self._raw_command_xml_topic, 10)

        self.get_logger().info(f'Publishing feedback to: {self._feedback_topic}')
        self.get_logger().info(f'Publishing raw robot XML to: {self._raw_robot_xml_topic}')
        self.get_logger().info(f'Publishing raw command XML to: {self._raw_command_xml_topic}')

        # Guard telemetry goes to its own NEW topic. The feedback topic the
        # GUI and the logger consume is deliberately left untouched.
        self._pub_diagnostics = None
        if self._publish_diagnostics:
            self._pub_diagnostics = self.create_publisher(
                String, self._diagnostics_topic, 10)
            self.get_logger().info(
                f'Publishing bridge diagnostics to: {self._diagnostics_topic}')

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
            on_connect=self._on_kuka_connected,
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

        # Campo exclusivo de ENVIAR TRAYECTORIA. Su ausencia conserva el
        # contrato anterior para SEND manual. Si está presente debe ser un
        # porcentaje PTP articular finito dentro de (0, 100].
        trajectory_velocity_pct = 0.0
        if 'trajectory_ptp_velocity_pct' in data:
            raw_velocity = data.get('trajectory_ptp_velocity_pct')
            try:
                trajectory_velocity_pct = float(raw_velocity)
            except (TypeError, ValueError):
                self.get_logger().warn(
                    'Target JSON trajectory_ptp_velocity_pct no es numérico '
                    '— target REJECTED.')
                return
            if (not math.isfinite(trajectory_velocity_pct)
                    or not 0.0 < trajectory_velocity_pct <= 100.0):
                self.get_logger().warn(
                    'Target JSON trajectory_ptp_velocity_pct fuera de '
                    '(0, 100] — target REJECTED.')
                return
            if mode != 'AxisTarget':
                self.get_logger().warn(
                    'trajectory_ptp_velocity_pct solo es válido para '
                    'AxisTarget — target REJECTED.')
                return

        # Optional gripper request. Absent, malformed or out-of-range always
        # degrades to -1 ("do nothing") — a bad payload must never be able to
        # open or close the gripper.
        gripper = -1
        raw_gripper = data.get('gripper_command', -1)
        try:
            candidate_gripper = int(raw_gripper)
        except (TypeError, ValueError):
            candidate_gripper = -1
            self.get_logger().warn(
                f'Target JSON gripper_command={raw_gripper!r} is not an '
                f'integer — treated as -1 (no action).'
            )
        if candidate_gripper in (-1, 0, 1):
            gripper = candidate_gripper
        else:
            self.get_logger().warn(
                f'Target JSON gripper_command={candidate_gripper} out of '
                f'range — treated as -1 (no action).'
            )

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

        # ── BATCH fields (all optional; absent = baseline behaviour) ──
        batch_points, batch_seq, batch_velocity, abort_batch, batch_error = \
            self._parse_batch_fields(data)
        if batch_error:
            self.get_logger().warn(f'Batch rejected: {batch_error}')
            # A malformed batch is dropped whole. It must never degrade into
            # a partial batch, and it must never fall through to the
            # single-point path either.
            return

        # Accept the target
        with self._target_lock:
            if not missing_axis:
                self._last_valid_target = candidate_axis
            if not missing_cartesian:
                self._last_valid_cartesian = candidate_cartesian
            self._last_mode = mode
            self._last_enable_move = enable_move
            self._last_gripper_command = gripper
            self._last_trajectory_ptp_velocity_pct = trajectory_velocity_pct
            self._last_cmd_seq = cmd_seq
            self._last_cmd_time = time.monotonic()

            if abort_batch:
                self._abort_batch_request = True
                # An abort also drops anything not yet on the wire.
                self._pending_batch = None
                self._pending_batch_seq = 0
            elif batch_points is not None:
                self._pending_batch = batch_points
                self._pending_batch_seq = batch_seq
                self._pending_batch_velocity = batch_velocity

        self.get_logger().info(
            f'[TARGET RECEIVED] seq={cmd_seq} mode={mode} '
            f'enable={enable_move} gripper={gripper} '
            f'trajectory_ptp_velocity_pct={trajectory_velocity_pct:g}'
        )

    # ── BATCH: target JSON parsing ───────────────────────────────────

    def _parse_batch_fields(self, data: dict):
        """
        Extract the optional batch fields from a target JSON.

        Returns:
            (points, batch_seq, velocity_pct, abort, error_message)

        points is None when the message carries no batch, which is the
        baseline case and must stay indistinguishable from today. Any
        malformed batch returns an error string and is dropped whole: a
        partial batch would move the robot along a path nobody validated.
        """
        abort = bool(data.get('abort_batch', False))

        raw_points = data.get('batch_points_deg')
        if raw_points is None:
            return None, 0, 0.0, abort, ''

        if not isinstance(raw_points, list) or not raw_points:
            return None, 0, 0.0, abort, 'batch_points_deg must be a non-empty list'
        if len(raw_points) > self._max_batch_size:
            return None, 0, 0.0, abort, (
                f'batch of {len(raw_points)} points exceeds max_batch_size='
                f'{self._max_batch_size}')

        try:
            batch_seq = int(data.get('batch_seq', 0))
        except (TypeError, ValueError):
            return None, 0, 0.0, abort, 'batch_seq is not an integer'
        if batch_seq <= 0:
            return None, 0, 0.0, abort, f'batch_seq={batch_seq} must be > 0'

        velocity = 0.0
        raw_velocity = data.get('batch_ptp_velocity_pct', 0.0)
        try:
            velocity = float(raw_velocity)
        except (TypeError, ValueError):
            return None, 0, 0.0, abort, 'batch_ptp_velocity_pct is not numeric'
        if velocity != 0.0 and not 0.0 < velocity <= 100.0:
            return None, 0, 0.0, abort, (
                f'batch_ptp_velocity_pct={velocity} outside (0, 100]')

        points = []
        for index, entry in enumerate(raw_points):
            point = {}
            if isinstance(entry, dict):
                source = entry
            elif isinstance(entry, (list, tuple)) and len(entry) == len(_AXES):
                source = dict(zip(_AXES, entry))
            else:
                return None, 0, 0.0, abort, (
                    f'batch point {index} is neither a dict nor 6 values')

            for axis in _AXES:
                try:
                    value = float(source[axis])
                except (KeyError, TypeError, ValueError):
                    return None, 0, 0.0, abort, (
                        f'batch point {index} missing or invalid {axis}')
                if not math.isfinite(value):
                    return None, 0, 0.0, abort, (
                        f'batch point {index} {axis} is not finite')
                # Same soft limits as any single point. No exemptions.
                low, high = self._soft_limits.get(axis, (-360.0, 360.0))
                if not (low <= value <= high):
                    return None, 0, 0.0, abort, (
                        f'batch point {index} {axis}={value:.2f} outside soft '
                        f'limits [{low:.1f}, {high:.1f}]')
                point[axis] = value
            points.append(point)

        return points, batch_seq, velocity, abort, ''

    # ── Feedback callback (called from TCP thread) ───────────────────

    def _on_robot_message(self, parsed: dict, raw_xml: str) -> None:
        """
        Handle parsed feedback from the KUKA.

        Called from the TCP server thread — must be thread-safe for
        ROS2 publishers (rclpy is thread-safe for publishing).

        Runs once per received <Robot> frame, which is what makes it the right
        place to watch the inter-arrival time: a round that took the long path
        is a round in which the SPS read one command out of its buffer.
        """
        self._observe_consumption(parsed)

        seq = parsed.get('seq', 0)
        mode = parsed.get('mode', 'Unknown')
        status = parsed.get('status', 0)
        axis_actual = parsed.get('axis_actual', {})
        pos_actual = parsed.get('position_actual', {})
        move_ready = parsed.get('move_ready', False)
        limits_ok = parsed.get('limits_ok', False)
        delta_ok = parsed.get('delta_ok', False)
        move_executed = parsed.get('move_executed', False)
        rx_counter = parsed.get('rx_counter')

        # EkiAxisMoveServer is shared with the baseline node and parses with
        # axis_move_xml_utils, which knows nothing about the batch tags: its
        # result never carries batch_seq / batch_consumed / batch_active, so
        # reading them here would always yield None even though the KUKA does
        # send <BatchSeq>, <BatchConsumed> and <BatchActive>.
        # Re-parsing the same frame with the batch-aware parser is what
        # recovers them, and it touches neither the shared server nor the
        # baseline parser. One extra parse per frame at ~2 Hz.
        better_parsed = parse_axis_move_xml_better(raw_xml)
        if better_parsed is not None:
            parsed['batch_seq'] = better_parsed.get('batch_seq')
            parsed['batch_consumed'] = better_parsed.get('batch_consumed')
            parsed['batch_active'] = better_parsed.get('batch_active')

        batch_seq_fb = parsed.get('batch_seq')
        batch_consumed_fb = parsed.get('batch_consumed')
        batch_active_fb = parsed.get('batch_active')

        # Progreso del lote. Con aproximacion activa XD_BATCH_ACTIVE se pone
        # a FALSE en el AVANCE, hasta $ADVANCE puntos antes de que el robot
        # haya llegado, asi que por si solo no sirve para saber si el lote
        # sigue en vuelo. El contador si: XmlDualMove_better.src lo incrementa
        # junto a cada PTP del lote. Mientras suba, quedan puntos por delante.
        if (isinstance(batch_consumed_fb, int)
                and isinstance(self._robot_batch_consumed, int)
                and batch_consumed_fb > self._robot_batch_consumed):
            self._last_batch_progress_at = time.monotonic()
            self._batch_ack_seen = True

        # ACUSE DEL LOTE. Robot/BatchSeq NO sirve para esto: la SPS lo escribe
        # en cuanto acepta el paquete, antes de que el interprete del robot lo
        # haya mirado siquiera. Lo unico que prueba que el robot COGIO el lote
        # es XD_BATCH_ACTIVE, que el .src pone a TRUE justo al abrir su puerta.
        if batch_active_fb is True:
            self._batch_ack_seen = True

        self._robot_batch_seq = batch_seq_fb
        self._robot_batch_consumed = batch_consumed_fb
        self._robot_batch_active = batch_active_fb

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
            'rx_counter': rx_counter,
            'bridge_safe_mode': self._safe_mode,
            'bridge_allow_motion': self._allow_motion,
            # ── BATCH telemetry ──────────────────────────────────────
            # None on a baseline controller, which is exactly how the GUI
            # tells "batch not supported here" from "batch idle".
            'batch_seq': batch_seq_fb,
            'batch_consumed': batch_consumed_fb,
            'batch_active': batch_active_fb,
        }
        fb_msg = String()
        fb_msg.data = json.dumps(feedback)
        self._pub_feedback.publish(fb_msg)

    # ── EKI receive-buffer guard ─────────────────────────────────────

    def _observe_consumption(self, parsed: dict) -> None:
        """
        Track how many commands the KUKA still has unread.

        Preferred path — explicit acknowledgement. Robot/RxCounter is the
        SPS's own count of COMPLETE commands taken out of the EKI buffer, so
        the difference between two frames is exactly how many were consumed.
        No timing assumption is involved.

        Fallback path — timing. Used only until the first RxCounter arrives,
        which keeps this node working against a controller that has not been
        updated yet. A long round means the SPS ran its read block.

        Both paths are conservative: they never decrement below zero and never
        decrement more than what was actually observed.
        """
        now = time.monotonic()
        previous = self._last_frame_monotonic
        self._last_frame_monotonic = now
        if previous > 0.0:
            self._last_frame_delta = now - previous

        rx_counter = parsed.get('rx_counter')

        if rx_counter is not None:
            if not self._rx_ack_available:
                self._rx_ack_available = True
                self.get_logger().info(
                    '[GUARD] Robot/RxCounter present — buffer estimate now '
                    'driven by explicit acknowledgement, not by timing.')

            if self._last_rx_counter is None:
                self._last_rx_counter = rx_counter
                return

            consumed = rx_counter - self._last_rx_counter
            self._last_rx_counter = rx_counter

            if consumed < 0:
                # The SPS zeroes RxCounter when it reopens the channel, so a
                # backwards step means the buffer was destroyed with it.
                self._pending_commands = 0
                self.get_logger().info(
                    '[GUARD] RxCounter went backwards — KUKA reopened the '
                    'channel; buffer estimate reset to 0.')
            elif consumed > 0:
                self._pending_commands = max(
                    0, self._pending_commands - consumed)
            return

        # ---- Fallback: no RxCounter in this telemetry ----
        if previous <= 0.0:
            return
        if (self._last_frame_delta >= self._consume_threshold
                and self._pending_commands > 0):
            self._pending_commands -= 1

    def _on_kuka_connected(self) -> None:
        """
        Called by the TCP server each time the KUKA (re)connects.

        The EKI receive buffers are torn down with the TCP connection, so this
        is the only moment their occupancy is known for certain: zero.
        """
        self._pending_commands = 0
        self._last_frame_monotonic = 0.0
        self._last_rx_counter = None
        self._last_command_signature = None
        self._last_command_send_time = 0.0
        # A half-delivered batch must never survive a reconnect.
        with self._target_lock:
            self._pending_batch = None
            self._pending_batch_seq = 0
            self._abort_batch_request = False
        self._batch_sent_seq = 0
        self._last_batch_send_at = 0.0
        self._last_batch_progress_at = 0.0
        self._batch_ack_seen = False
        self._robot_batch_seq = None
        self._robot_batch_consumed = None
        self._robot_batch_active = None
        self.get_logger().info(
            '[GUARD] KUKA connected — EKI buffer estimate reset to 0, '
            'batch state cleared.')

    def _publish_diagnostics_msg(
        self,
        send_reason: Optional[str],
        guard_blocked: Optional[str],
        cmd_seq: int,
        effective_enable: bool,
    ) -> None:
        """Publish guard state on its own topic (never on the feedback topic)."""
        if self._pub_diagnostics is None:
            return

        msg = String()
        msg.data = json.dumps({
            'pending_commands': self._pending_commands,
            'max_pending_heartbeat': self._max_pending_heartbeat,
            'max_pending_command': self._max_pending_command,
            'send_reason': send_reason,
            'guard_blocked': guard_blocked,
            'delta_ms': round(self._last_frame_delta * 1000.0, 3),
            'consume_threshold_ms': round(self._consume_threshold * 1000.0, 1),
            'rx_counter': self._last_rx_counter,
            'rx_ack_available': self._rx_ack_available,
            'cmd_seq': cmd_seq,
            'effective_enable': bool(effective_enable),
            'guard_block_count': self._guard_block_count,
        })
        self._pub_diagnostics.publish(msg)

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
            batch_snap = self._pending_batch
            batch_seq_snap = self._pending_batch_seq
            batch_velocity_snap = self._pending_batch_velocity
            abort_snap = self._abort_batch_request
            # Both are one-shot: consumed here so they go out exactly once,
            # the same contract the gripper already uses.
            self._pending_batch = None
            self._pending_batch_seq = 0
            self._abort_batch_request = False

            target_snap = dict(self._last_valid_target)
            cart_snap = dict(self._last_valid_cartesian)
            mode_snap = self._last_mode
            enable_move_snap = self._last_enable_move
            gripper_snap = self._last_gripper_command
            trajectory_velocity_snap = \
                self._last_trajectory_ptp_velocity_pct
            cmd_seq = self._last_cmd_seq
            cmd_time = self._last_cmd_time

        # Get current feedback for delta validation
        with self._feedback_lock:
            feedback_actual = dict(self._last_feedback_actual)
            feedback_pos = dict(self._last_feedback_pos)

        # ── BATCH: an abort outranks everything ──────────────────────
        # It carries EnableMove=0 and BatchCount=0, so it can neither move
        # anything nor start a batch; it only raises the flag the robot
        # interpreter checks between points.
        if abort_snap:
            xml = build_abort_batch_command_xml(
                seq=cmd_seq, hold_target=feedback_actual)
            self._publish_command_xml(xml)
            self.get_logger().warn(
                '[BATCH] AbortBatch sent — the KUKA will finish the PTP in '
                'flight and not start the next point.')
            return xml

        # ── BATCH: a pending batch replaces the single-point command ──
        if batch_snap is not None:
            return self._build_batch_command(
                cmd_seq, batch_seq_snap, batch_snap, batch_velocity_snap,
                enable_move_snap, feedback_actual)

        # ── BATCH: no single-point command may talk over a running batch ──
        # XmlDualMove_better.src re-reads XD_ENABLE_MOVE before EVERY point of
        # the batch, so ANY <Command> carrying EnableMove=0 aborts the batch
        # mid-flight. And a heartbeat during a batch is GUARANTEED to carry
        # EnableMove=0: layer 4 zeroes it because the seq is repeated, and
        # layer 7 zeroes it because it compares a target frozen at the start
        # of the batch against a robot that has since moved away from it.
        # That is what stopped every batch after a handful of points.
        # Nothing is lost by staying quiet: an abort and the next batch are
        # both handled above, and the single-point path is stood down inside
        # the robot program while a batch is pending anyway.
        if self._batch_in_flight():
            return None

        # Signature of what the GUI is asking for right now. Compared further
        # down to decide whether this <Robot> frame earns a <Command> reply.
        signature = self._command_signature(
            cmd_seq, mode_snap, enable_move_snap, target_snap, cart_snap,
            gripper_snap, trajectory_velocity_snap)

        # ── Validation layers ────────────────────────────────────────
        effective_enable = enable_move_snap
        reasons: list = []

        # The gripper is a physical action, so it clears exactly the same two
        # gates as motion before it may leave this node. The builder is called
        # with the gates already applied, so this is where they must bite.
        if self._safe_mode or not self._allow_motion:
            effective_gripper = -1
        elif gripper_snap in (0, 1):
            effective_gripper = int(gripper_snap)
        else:
            effective_gripper = -1

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

        guard_blocked: Optional[str] = None

        if self._pending_commands >= self._max_pending_command:
            # Hard stop. Past this point the EKI receive memory is close
            # enough to its limit that one more command risks the connection,
            # and a dead link serves nobody — least of all a motion command.
            send_reason = None
            guard_blocked = 'max_pending_command'
        elif is_new_command:
            send_reason = 'new_command'
        elif effective_enable:
            # A frame that would put EnableMove=1 on the wire is never
            # withheld, not even for a few milliseconds.
            send_reason = 'enable_move'
        elif self._pending_commands >= self._max_pending_heartbeat:
            # The heartbeat carries nothing new, so it is the first thing to
            # give up when the KUKA has stopped reading.
            send_reason = None
            guard_blocked = 'max_pending_heartbeat'
        elif heartbeat_due:
            send_reason = 'heartbeat'
        else:
            send_reason = None

        if guard_blocked is not None:
            self._guard_block_count += 1
            # Rate-limited: a stall can last minutes and this must not flood
            # the console at telemetry rate.
            if now - self._last_guard_warn_time >= 5.0:
                self._last_guard_warn_time = now
                self.get_logger().warn(
                    f'[GUARD] Withholding commands ({guard_blocked}): '
                    f'pending={self._pending_commands}, EKI closes the link at '
                    f'16 unread. The KRL program looks like it has stopped '
                    f'consuming commands. blocked_total={self._guard_block_count}'
                )

        if send_reason is None:
            self.get_logger().debug(
                f'[COMMAND SUPPRESSED] seq={cmd_seq} '
                f'guard={guard_blocked} pending={self._pending_commands} '
                f'since_last_send={now - self._last_command_send_time:.3f}s'
            )
            self._publish_diagnostics_msg(
                None, guard_blocked, cmd_seq, effective_enable)
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
            gripper_command=effective_gripper,
            trajectory_ptp_velocity_pct=trajectory_velocity_snap,
        )

        # Publish the sent command XML
        cmd_msg = String()
        cmd_msg.data = xml
        self._pub_raw_cmd.publish(cmd_msg)

        self._last_command_signature = signature
        self._last_command_send_time = now
        # One more element written into every EKI receive memory. Only a
        # confirmed read or a reconnect takes it back down.
        self._pending_commands += 1
        self.get_logger().debug(
            f'[COMMAND SEND] reason={send_reason} seq={cmd_seq} '
            f'enable={1 if effective_enable else 0} '
            f'gripper={effective_gripper} '
            f'trajectory_ptp_velocity_pct={trajectory_velocity_snap:g} '
            f'pending={self._pending_commands}'
        )
        self._publish_diagnostics_msg(
            send_reason, None, cmd_seq, effective_enable)

        # ── Compact log ──────────────────────────────────────────────
        self._log_cycle(parsed_feedback, target_snap, effective_enable)

        return xml

    # ── BATCH: command builder ───────────────────────────────────────

    def _publish_command_xml(self, xml: str) -> None:
        """Put a <Command> on the wire and mirror it on the raw topic."""
        message = String()
        message.data = xml
        self._pub_raw_cmd.publish(message)
        self._last_command_send_time = time.monotonic()
        self._pending_commands += 1

    def _batch_in_flight(self) -> bool:
        """
        True mientras el KUKA esta ejecutando un lote, o aun no lo ha cogido.

        ANTES del acuse la espera NO puede ser un reloj. Un lote puede quedarse
        esperando en el buzon todo el tiempo que el interprete del robot tarde
        en llegar a su puerta, y ese tiempo no esta acotado: si el segmento
        viene detras de una accion de garra, el interprete esta dentro de
        GRPg_SetStateAndCheck y no mira el buzon hasta que la pinza termina
        fisicamente. Con la ventana fija de 2 s se colaba un heartbeat en ese
        hueco; el heartbeat lleva EnableMove=0 por diseno, la SPS lo copia a
        XD_ENABLE_MOVE y la puerta del lote de XmlDualMove_better.src exige
        XD_ENABLE_MOVE == TRUE. El lote quedaba muerto sin haber empezado, y
        con batchSeq <> handledBatchSeq el camino de punto suelto tambien se
        queda en pie: el programa no ejecutaba NADA. Ese era el 0/10 de T4.

        DESPUES del acuse la ventana de tiempo si vale, y sigue igual: cubre
        los huecos cortos entre lotes consecutivos de un mismo segmento.
        """
        if self._robot_batch_active is True:
            return True
        if self._batch_sent_seq <= 0:
            return False

        if not self._batch_ack_seen:
            # El lote viaja pero el robot todavia no lo ha reclamado. Callar
            # es lo unico seguro: un heartbeat aqui cierra la puerta del lote.
            # El tope solo existe para no quedarse mudo si el lote se rechaza;
            # el ejecutor aborta a los 20 s mucho antes de llegar a el, y su
            # AbortBatch se atiende por encima de este guardia.
            if self._batch_ack_timeout <= 0.0:
                return True
            return ((time.monotonic() - self._last_batch_send_at)
                    < self._batch_ack_timeout)

        if self._batch_hold_sec > 0.0:
            # La ventana se renueva con cada punto que el robot consume, de
            # modo que un lote largo no se queda sin cobertura a mitad.
            last = max(self._last_batch_send_at, self._last_batch_progress_at)
            if (time.monotonic() - last) < self._batch_hold_sec:
                return True
        return False

    def _build_batch_command(
        self,
        cmd_seq: int,
        batch_seq: int,
        points: list,
        velocity_pct: float,
        enable_move: bool,
        feedback_actual: dict,
    ) -> Optional[str]:
        """
        Emit a batch <Command>, or nothing if a gate or the guard says no.

        The two safety gates are applied by the builder itself, exactly as
        they are for a single point: with safe_mode on or allow_motion off,
        EnableMove goes out as 0 and XmlDualMove_better.src refuses to start
        the batch.

        The EKI guard is checked with a TIGHTER threshold than a single
        point: one batch packet is one command against RxCounter, but it
        drops N entries into each of the six Command/Batch/@Ax memories.
        """
        if self._pending_commands >= self._max_pending_batch:
            self.get_logger().warn(
                f'[BATCH] Withholding batch {batch_seq}: '
                f'{self._pending_commands} commands still unread by the SPS '
                f'(max_pending_batch={self._max_pending_batch}).')
            return None

        xml = build_axis_move_batch_command_xml(
            seq=cmd_seq,
            batch_seq=batch_seq,
            points_deg=points,
            enable_move=enable_move,
            safe_mode=self._safe_mode,
            allow_motion=self._allow_motion,
            batch_ptp_velocity_pct=velocity_pct,
            abort_batch=False,
            # The hold target is the CURRENT position: inert under both the
            # baseline and the better program.
            hold_target=feedback_actual,
            max_batch_size=self._max_batch_size,
        )
        if xml is None:
            self.get_logger().error(
                f'[BATCH] Batch {batch_seq} failed to build — nothing sent.')
            return None

        self._publish_command_xml(xml)
        # Un batch_seq NUEVO vuelve a estar sin acusar. Un reenvio del MISMO
        # batch_seq no borra un acuse ya visto.
        if batch_seq != self._batch_sent_seq:
            self._batch_ack_seen = False
        self._batch_sent_seq = batch_seq
        self._last_batch_send_at = time.monotonic()
        # El objetivo de punto suelto se re-ancla en la posicion REAL del
        # robot. El JSON de lote trae como axis_target el PRIMER punto del
        # lote, que es relleno inerte; dejarlo ahi hacia que el primer
        # heartbeat posterior al lote midiese un delta enorme contra un punto
        # que el robot ya dejo atras.
        with self._target_lock:
            if len(feedback_actual) == len(_AXES):
                self._last_valid_target = dict(feedback_actual)
        self.get_logger().info(
            f'[BATCH SEND] batch_seq={batch_seq} points={len(points)} '
            f'ptp_vel={velocity_pct:g}% enable={enable_move} '
            f'pending={self._pending_commands}')
        return xml

    # ── Helpers ──────────────────────────────────────────────────────

    def _command_signature(
        self,
        seq: int,
        mode: str,
        enable_move: bool,
        target: dict,
        cartesian: dict,
        gripper_command: int,
        trajectory_ptp_velocity_pct: float,
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

        gripper_command is in the signature so that -1 -> 0, -1 -> 1 or 0 -> 1
        counts as a new command and goes out immediately. A heartbeat repeats
        the same seq and the same gripper value, so its signature is unchanged
        and the KUKA ignores it.
        """
        return (
            int(seq),
            str(mode),
            bool(enable_move),
            int(gripper_command),
            round(float(trajectory_ptp_velocity_pct), 4),
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
