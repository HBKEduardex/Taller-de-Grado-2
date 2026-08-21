"""
rviz_mirror_node.py — KUKA -> RViz JOINT-STATE MIRROR (VISUALISATION ONLY).

What this node does
-------------------
It subscribes, read-only, to the very same telemetry topic the existing GUI
and the existing telemetry logger consume, takes the MEASURED joint angles
A1..A6 (which the KUKA sends in DEGREES), converts them to radians and
republishes them as a sensor_msgs/JointState on /fake_joint_states.

    KUKA REAL (jog / KRL program / XmlDualMove)
        -> SPS + EthernetKRL
        -> node `eki_axis_move`             (kuka_eki_bridge, UNMODIFIED)
        -> /kuka/axis_move/feedback_json    (std_msgs/String, JSON payload)
             |
             +--> GUI `kuka_gui_axis_move_node`     (UNMODIFIED)
             +--> `kuka_telemetry_logger`  -> CSV + SQLite   (UNMODIFIED)
             |
             +--> THIS NODE
                     -> axis_actual.A1..A6  [degrees]
                     -> math.radians()
                     -> sensor_msgs/JointState
                     -> /fake_joint_states
                          |
                          +--> `joint_state_publisher`  (source_list, EXISTING)
                                 -> /joint_states
                                      -> robot_state_publisher / RViz / MoveIt

Why /fake_joint_states and NOT /joint_states
--------------------------------------------
/joint_states already has a publisher: the `joint_state_publisher` node
started by kuka_kr6_moveit_config/launch/demo.launch.py, which is configured
with source_list: ["/fake_joint_states"]. Publishing a second, competing
source directly on /joint_states would fight that node. Feeding
/fake_joint_states instead reuses the chain that is already wired up.

What this node NEVER does
-------------------------
  * It opens NO TCP socket and speaks NO EthernetKRL / EKI.
  * It sends NO command to the robot: no target_json, no joint_command_deg,
    no cartesian_command_deg, no EnableMove.
  * It creates NO ActionClient and calls NO MoveIt planning
    (no MoveGroup, no MoveItCpp, no plan(), no execute(),
    no FollowJointTrajectory, no trajectory_msgs).
  * It creates exactly ONE publisher: /fake_joint_states.

The measured position is a STATE, not a goal. RViz shows where the robot IS;
nothing here plans a path to get there.

Event-driven, 1:1
-----------------
There is no timer in this node. One received feedback message produces at
most one JointState. If the KUKA telemetry arrives at 7.68 Hz, this node
publishes at 7.68 Hz. No interpolation, no smoothing, no filtering, no
extra samples are ever generated.

(Note, for honesty about the whole chain: the downstream
`joint_state_publisher` republishes /joint_states on its own 10 Hz timer —
that is existing, unmodified behaviour and is outside this node.)

Timestamps
----------
The telemetry message carries NO timestamp of its own: it is a
std_msgs/String (no Header) and the KUKA <SEND> block declares no time
element. Therefore JointState.header.stamp is
`self.get_clock().now().to_msg()`, i.e. the ROS2 RECEPTION/PROCESSING time
on this machine. It is NOT the KUKA clock. Nothing is fabricated.

Usage (AFTER the entry point is registered — see README_RVIZ_MIRROR.md):
    ros2 launch kuka_telemetry_logger kuka_rviz_mirror.launch.py
    ros2 launch kuka_telemetry_logger kuka_rviz_mirror.launch.py verbose:=true
"""

import argparse
import math
import sys
from typing import Any, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# Reused verbatim from the existing logger. These helpers are imported, not
# copied and not modified: the parsing path is therefore identical to the one
# that already produces the CSV columns axis_actual.A1 .. axis_actual.A6.
from kuka_telemetry_logger.message_introspection import (
    expand_json_payload,
    find_sequence,
    flatten,
    ros_message_to_dict,
)

# ---------------------------------------------------------------------------
# Constants discovered by reading the existing workspace — none of these are
# guessed.
#
#   INPUT topic / QoS
#     publisher : kuka_eki_bridge/eki_axis_move_node.py  (node `eki_axis_move`)
#     QoS       : rclpy default -> RELIABLE / VOLATILE / KEEP_LAST(10)
#
#   JSON layout (eki_axis_move_node.py, `feedback` dict)
#     {"seq": .., "mode": .., "status": .., "move_ready": .., "limits_ok": ..,
#      "delta_ok": .., "move_executed": ..,
#      "axis_actual":     {"A1":.., "A2":.., "A3":.., "A4":.., "A5":.., "A6":..},
#      "position_actual": {"X":.., "Y":.., "Z":.., "A":.., "B":.., "C":..},
#      "rx_counter": .., "bridge_safe_mode": .., "bridge_allow_motion": ..}
#     axis_actual values are DEGREES, straight from KUKA $AXIS_ACT.
#
#   JOINT NAMES (empty xacro prefix ->  no namespace on the names)
#     kuka_kr6_support/urdf/kr6r900sixx_macro.xacro   joint_a1 .. joint_a6
#     kuka_kr6_moveit_config/config/kuka_kr6.srdf     same six, same order
#     kuka_gui_moveit_bridge default 'joint_names'    same six, same order
# ---------------------------------------------------------------------------

DEFAULT_TELEMETRY_TOPIC = '/kuka/axis_move/feedback_json'
DEFAULT_OUTPUT_TOPIC = '/fake_joint_states'
DEFAULT_QOS_DEPTH = 10

# Key of the nested object holding the measured axes, and the six axis keys
# inside it — exactly as the bridge writes them.
AXIS_ACTUAL_KEY = 'axis_actual'
AXIS_KEYS = ('A1', 'A2', 'A3', 'A4', 'A5', 'A6')

# The six joints of the KR6 R900 sixx model currently loaded in RViz/MoveIt,
# in the order that matches AXIS_KEYS index by index (A1 -> joint_a1, ...).
DEFAULT_JOINT_NAMES = [
    'joint_a1',
    'joint_a2',
    'joint_a3',
    'joint_a4',
    'joint_a5',
    'joint_a6',
]

# Warnings are throttled so a permanently malformed stream cannot flood rosout.
WARN_EVERY_N_INVALID = 25


class KukaRvizMirrorNode(Node):
    """Mirror the MEASURED KUKA joint angles into /fake_joint_states."""

    def __init__(self, verbose: bool = False):
        super().__init__('kuka_rviz_mirror')

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('telemetry_topic', DEFAULT_TELEMETRY_TOPIC)
        self.declare_parameter('joint_states_topic', DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('qos_depth', DEFAULT_QOS_DEPTH)
        self.declare_parameter('report_every', 100)
        self.declare_parameter('verbose', False)

        self._topic = self.get_parameter(
            'telemetry_topic').get_parameter_value().string_value
        self._out_topic = self.get_parameter(
            'joint_states_topic').get_parameter_value().string_value
        joint_names = list(self.get_parameter(
            'joint_names').get_parameter_value().string_array_value)
        qos_depth = self.get_parameter(
            'qos_depth').get_parameter_value().integer_value
        self._report_every = max(1, self.get_parameter(
            'report_every').get_parameter_value().integer_value)

        # --verbose on the command line OR the `verbose` parameter enables it.
        self._verbose = bool(verbose) or self.get_parameter(
            'verbose').get_parameter_value().bool_value

        if len(joint_names) != 6:
            self.get_logger().warn(
                f'joint_names has {len(joint_names)} entries, expected 6. '
                f'Falling back to the model defaults {DEFAULT_JOINT_NAMES}.')
            joint_names = list(DEFAULT_JOINT_NAMES)
        self._joint_names: List[str] = joint_names

        # ── Runtime statistics (diagnostics only) ────────────────────
        self._messages_received = 0
        self._messages_published = 0
        self._invalid_messages = 0
        self._last_seq: Optional[int] = None
        self._last_degrees: Optional[List[float]] = None
        self._last_radians: Optional[List[float]] = None
        self._first_rx_ns: Optional[int] = None
        self._last_rx_ns: Optional[int] = None
        self._window_start_ns: Optional[int] = None
        self._window_count = 0

        # ── QoS: mirrors the publisher in eki_axis_move_node.py ──────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth if qos_depth > 0 else DEFAULT_QOS_DEPTH,
        )

        # ── The ONE and ONLY publisher of this node ──────────────────
        self._pub_joint_states = self.create_publisher(
            JointState, self._out_topic, qos)

        # ── Subscription (read-only) ─────────────────────────────────
        # NOTE: no create_timer() anywhere in this node. The callback below is
        # the only thing that ever publishes: one feedback in, one state out.
        self.create_subscription(String, self._topic, self._on_feedback, qos)

        # ── Banner ───────────────────────────────────────────────────
        self.get_logger().info('╔══════════════════════════════════════════════╗')
        self.get_logger().info('║   KUKA RViz Mirror — VISUALISATION ONLY      ║')
        self.get_logger().info('╚══════════════════════════════════════════════╝')
        self.get_logger().info(f'  Input topic  : {self._topic}')
        self.get_logger().info('  Input type   : std_msgs/msg/String (JSON)')
        self.get_logger().info(
            f'  QoS          : RELIABLE / VOLATILE / KEEP_LAST({qos.depth})')
        self.get_logger().info(f'  Output topic : {self._out_topic}')
        self.get_logger().info('  Output type  : sensor_msgs/msg/JointState')
        self.get_logger().info(f'  Joint names  : {self._joint_names}')
        self.get_logger().info('  Conversion   : degrees -> radians only '
                               '(no sign flip, no offset, no reorder)')
        self.get_logger().info('  Mode         : event-driven 1:1, no timer, '
                               'no interpolation')
        self.get_logger().info('  Commands     : NONE — this node never moves '
                               'the robot')
        self.get_logger().info(f'  Verbose      : {self._verbose}')
        self.get_logger().info('──────────────────────────────────────────────')
        self.get_logger().info('Waiting for telemetry...')

    # ── Telemetry callback ───────────────────────────────────────────

    def _on_feedback(self, msg: String) -> None:
        """Convert ONE feedback message into at most ONE JointState."""
        now = self.get_clock().now()
        rx_sec, rx_nanosec = now.seconds_nanoseconds()
        rx_ns = rx_sec * 1_000_000_000 + rx_nanosec

        self._messages_received += 1
        if self._first_rx_ns is None:
            self._first_rx_ns = rx_ns
        if self._window_start_ns is None:
            self._window_start_ns = rx_ns
        self._last_rx_ns = rx_ns
        self._window_count += 1

        # ── Decode exactly like the existing logger does ─────────────
        msg_dict = ros_message_to_dict(msg)
        payload_obj, expanded = expand_json_payload(msg_dict, 'data')

        if not expanded:
            self._reject('payload is not a JSON object')
            return

        # ── Sequence: diagnostics ONLY (never used to move or to stamp) ──
        seq = find_sequence(flatten(payload_obj))
        if seq is not None:
            self._last_seq = seq

        # ── Extract A1..A6 ───────────────────────────────────────────
        axis_actual = payload_obj.get(AXIS_ACTUAL_KEY)
        if not isinstance(axis_actual, dict):
            self._reject(f"'{AXIS_ACTUAL_KEY}' missing or not an object")
            return

        degrees: List[float] = []
        for key in AXIS_KEYS:
            if key not in axis_actual:
                self._reject(f"'{AXIS_ACTUAL_KEY}.{key}' missing")
                return
            value = _as_finite_float(axis_actual[key])
            if value is None:
                self._reject(
                    f"'{AXIS_ACTUAL_KEY}.{key}' is not a finite number "
                    f'({axis_actual[key]!r})')
                return
            degrees.append(value)

        # ── The whole transformation: degrees -> radians ─────────────
        # No sign change, no offset, no reordering. The URDF already encodes
        # the KUKA rotation senses in its <axis> vectors
        # (joint_a1 "0 0 -1", joint_a4 / joint_a6 "-1 0 0"), which is why the
        # existing kuka_gui_moveit_bridge also maps A[i] -> joint_names[i]
        # with a plain deg_to_rad() and nothing else.
        radians = [math.radians(d) for d in degrees]

        # ── Build the JointState ─────────────────────────────────────
        js = JointState()
        # ROS2 reception/processing time on THIS machine. The KUKA sends no
        # timestamp, so none is invented. See the module docstring.
        js.header.stamp = now.to_msg()
        js.name = list(self._joint_names)
        js.position = radians
        js.velocity = []
        js.effort = []

        self._pub_joint_states.publish(js)
        self._messages_published += 1
        self._last_degrees = degrees
        self._last_radians = radians

        # ── Console output ──────────────────────────────────────────
        if self._verbose:
            self.get_logger().info(
                f'[{self._messages_received}] seq={self._last_seq} '
                f'deg={_fmt_list(degrees)} -> rad={_fmt_list(radians)}')

        if self._messages_received % self._report_every == 0:
            self._print_report()

    # ── Rejection path ───────────────────────────────────────────────

    def _reject(self, reason: str) -> None:
        """Count an unusable message and warn, throttled. Publishes nothing."""
        self._invalid_messages += 1
        if self._invalid_messages == 1 or \
                self._invalid_messages % WARN_EVERY_N_INVALID == 0:
            self.get_logger().warn(
                f'Dropped telemetry sample ({reason}). '
                f'invalid_messages={self._invalid_messages}. '
                f'Nothing was published for this sample.')

    # ── Diagnostics ──────────────────────────────────────────────────

    def _window_rate_hz(self) -> Optional[float]:
        """Measured input rate over the last report window (never assumed)."""
        if self._window_start_ns is None or self._last_rx_ns is None:
            return None
        if self._window_count < 2:
            return None
        elapsed = (self._last_rx_ns - self._window_start_ns) / 1e9
        if elapsed <= 0.0:
            return None
        return (self._window_count - 1) / elapsed

    def _average_rate_hz(self) -> Optional[float]:
        """Measured input rate since the first message."""
        if self._first_rx_ns is None or self._last_rx_ns is None:
            return None
        if self._messages_received < 2:
            return None
        elapsed = (self._last_rx_ns - self._first_rx_ns) / 1e9
        if elapsed <= 0.0:
            return None
        return (self._messages_received - 1) / elapsed

    def _print_report(self) -> None:
        """Periodic compact report."""
        rate = self._window_rate_hz()
        self.get_logger().info('[KUKA RViz Mirror]')
        self.get_logger().info(f'  Received:    {self._messages_received}')
        self.get_logger().info(f'  Published:   {self._messages_published}')
        self.get_logger().info(f'  Invalid:     {self._invalid_messages}')
        self.get_logger().info(
            f'  Input rate:  '
            f'{"n/a" if rate is None else f"{rate:.2f} Hz"} (measured)')
        self.get_logger().info(
            f'  Last Seq:    '
            f'{self._last_seq if self._last_seq is not None else "N/A"}')
        self.get_logger().info(
            f'  A1..A6 deg:  {_fmt_list(self._last_degrees)}')
        self.get_logger().info(
            f'  A1..A6 rad:  {_fmt_list(self._last_radians)}')
        self.get_logger().info(f'  Output:      {self._out_topic}')

        # Start a fresh measurement window.
        self._window_start_ns = self._last_rx_ns
        self._window_count = 1

    def _print_final_summary(self) -> None:
        """Full summary printed once at shutdown."""
        rate = self._average_rate_hz()
        self.get_logger().info('══════════ FINAL SUMMARY ══════════')
        self.get_logger().info(f'  Input topic:        {self._topic}')
        self.get_logger().info(f'  Output topic:       {self._out_topic}')
        self.get_logger().info(
            f'  Messages received:  {self._messages_received}')
        self.get_logger().info(
            f'  Messages published: {self._messages_published}')
        self.get_logger().info(
            f'  Invalid messages:   {self._invalid_messages}')
        self.get_logger().info(
            f'  Average input rate: '
            f'{"n/a" if rate is None else f"{rate:.3f} Hz"} (measured)')
        self.get_logger().info(
            f'  Last sequence:      '
            f'{self._last_seq if self._last_seq is not None else "N/A"}')
        self.get_logger().info(
            f'  Last A1..A6 deg:    {_fmt_list(self._last_degrees)}')
        self.get_logger().info(
            f'  Last A1..A6 rad:    {_fmt_list(self._last_radians)}')
        self.get_logger().info(
            '  Robot commands:     0 (this node never commands the robot)')
        self.get_logger().info('═══════════════════════════════════')

    def destroy_node(self):
        self._print_final_summary()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _as_finite_float(value: Any) -> Optional[float]:
    """
    Return value as a finite float, or None when it is unusable.

    Rejects None, booleans, NaN, +/-Inf and anything non-numeric. A numeric
    string is accepted because JSON producers sometimes quote numbers; the
    current bridge writes real JSON numbers, so this is only a safety net.
    """
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, (int, float)):
        number = float(value)
    elif isinstance(value, str):
        try:
            number = float(value.strip())
        except ValueError:
            return None
    else:
        return None
    if not math.isfinite(number):
        return None
    return number


def _fmt_list(values: Optional[List[float]]) -> str:
    """Format a list of floats compactly for console output."""
    if not values:
        return 'n/a'
    return '[' + ', '.join(f'{v:.4f}' for v in values) + ']'


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    """Entry point for the KUKA -> RViz joint-state mirror."""
    argv = list(sys.argv[1:] if args is None else args)

    parser = argparse.ArgumentParser(
        prog='kuka_rviz_mirror',
        description=(
            'Mirror the MEASURED KUKA joint angles (A1..A6, degrees) into '
            'sensor_msgs/JointState on /fake_joint_states, in radians. '
            'Visualisation only: it never commands the robot.'
        ),
        add_help=False,
    )
    parser.add_argument(
        '--verbose', action='store_true',
        help='Print every conversion (noisy).')
    parser.add_argument(
        '-h', '--help', action='store_true', dest='show_help',
        help='Show this help message and exit.')
    known, remaining = parser.parse_known_args(argv)

    if known.show_help:
        parser.print_help()
        print(
            '\nROS2 parameters (pass with --ros-args -p name:=value):\n'
            '  telemetry_topic     (str)      default /kuka/axis_move/feedback_json\n'
            '  joint_states_topic  (str)      default /fake_joint_states\n'
            '  joint_names         (str[])    default '
            '[joint_a1..joint_a6]\n'
            '  qos_depth           (int)      default 10\n'
            '  report_every        (int)      default 100\n'
            '  verbose             (bool)     default false\n'
        )
        return

    rclpy.init(args=remaining)
    node = KukaRvizMirrorNode(verbose=known.verbose)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt — stopping mirror...')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
