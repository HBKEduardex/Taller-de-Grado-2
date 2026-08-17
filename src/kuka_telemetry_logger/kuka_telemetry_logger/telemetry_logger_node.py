"""
telemetry_logger_node.py — PASSIVE KUKA telemetry recorder.

What this node does
-------------------
It subscribes, read-only, to the exact same telemetry topic the existing GUI
consumes and writes every received message to a CSV file and a SQLite
database.

    KUKA SPS / XmlDualMove
        -> EthernetKRL (TCP 59153)
        -> node `eki_axis_move`            (kuka_eki_bridge, UNMODIFIED)
        -> /kuka/axis_move/feedback_json   (std_msgs/String, JSON payload)
             |
             +--> GUI  `kuka_gui_axis_move_node`   (UNMODIFIED)
             |
             +--> THIS NODE  ->  CSV + SQLite

What this node NEVER does
-------------------------
  * It creates NO publisher of any kind.
  * It never sends EnableMove, targets, or any command.
  * It never touches the TCP socket or EthernetKRL directly.
  * It never modifies any existing node, topic, message, or config.

Multiple subscribers on one topic is normal in ROS2 (DDS), so the GUI is
completely unaffected by this node running alongside it.

Timestamps
----------
The telemetry message carries NO timestamp of its own: it is a
std_msgs/String (no Header), and the KUKA `XmlDualMove.xml` <SEND> block does
not declare any time element. Therefore:

  * receive_* columns are generated HERE, on message arrival.
  * source_stamp_* columns stay NULL. They are never fabricated.

If the message type ever gains a header.stamp, the source_stamp_* columns
start filling in automatically — no code change needed.

Usage (AFTER you decide to build the package — see README.md):
    ros2 run kuka_telemetry_logger telemetry_logger
    ros2 run kuka_telemetry_logger telemetry_logger --verbose
"""

import argparse
import os
import sys
import time
from datetime import datetime
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from kuka_telemetry_logger.message_introspection import (
    expand_json_payload,
    find_sequence,
    find_source_stamp,
    flatten,
    ros_message_to_dict,
)
from kuka_telemetry_logger.storage import (
    TelemetryCsvWriter,
    TelemetrySqliteWriter,
)

# ---------------------------------------------------------------------------
# Defaults discovered by reading the existing workspace — do not guess these.
#
#   publisher : kuka_eki_bridge/eki_axis_move_node.py  (node `eki_axis_move`)
#   GUI sub   : kuka_gui_control/ros_axis_move_bridge.py
#   QoS       : rclpy default -> RELIABLE / VOLATILE / KEEP_LAST(10)
# ---------------------------------------------------------------------------

DEFAULT_TELEMETRY_TOPIC = '/kuka/axis_move/feedback_json'
DEFAULT_RAW_ROBOT_XML_TOPIC = '/kuka/axis_move/raw_robot_xml'
DEFAULT_QOS_DEPTH = 10


class KukaTelemetryLoggerNode(Node):
    """Passive subscriber that records KUKA telemetry to CSV + SQLite."""

    def __init__(self, verbose: bool = False):
        super().__init__('kuka_telemetry_logger')

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('telemetry_topic', DEFAULT_TELEMETRY_TOPIC)
        self.declare_parameter('log_raw_robot_xml', True)
        self.declare_parameter('raw_robot_xml_topic', DEFAULT_RAW_ROBOT_XML_TOPIC)
        self.declare_parameter('log_dir', 'logs')
        self.declare_parameter('file_prefix', 'kuka_telemetry')
        self.declare_parameter('qos_depth', DEFAULT_QOS_DEPTH)
        self.declare_parameter('flush_every', 20)
        self.declare_parameter('report_every', 100)
        self.declare_parameter('verbose', False)

        self._topic = self.get_parameter(
            'telemetry_topic').get_parameter_value().string_value
        self._log_raw_xml = self.get_parameter(
            'log_raw_robot_xml').get_parameter_value().bool_value
        self._raw_xml_topic = self.get_parameter(
            'raw_robot_xml_topic').get_parameter_value().string_value
        log_dir = self.get_parameter(
            'log_dir').get_parameter_value().string_value
        prefix = self.get_parameter(
            'file_prefix').get_parameter_value().string_value
        qos_depth = self.get_parameter(
            'qos_depth').get_parameter_value().integer_value
        flush_every = self.get_parameter(
            'flush_every').get_parameter_value().integer_value
        self._report_every = max(
            1, self.get_parameter('report_every').get_parameter_value().integer_value)

        # --verbose on the command line OR the `verbose` parameter enables it.
        self._verbose = bool(verbose) or self.get_parameter(
            'verbose').get_parameter_value().bool_value

        # ── Output files ─────────────────────────────────────────────
        log_dir = os.path.abspath(os.path.expanduser(log_dir))
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(log_dir, f'{prefix}_{stamp}.csv')
        db_path = os.path.join(log_dir, f'{prefix}_{stamp}.db')

        self._csv = TelemetryCsvWriter(csv_path, flush_every=flush_every)
        self._db = TelemetrySqliteWriter(db_path, commit_every=flush_every)

        # ── Runtime statistics ───────────────────────────────────────
        self._count = 0
        self._raw_xml_count = 0
        self._first_rx_ns: Optional[int] = None
        self._last_rx_ns: Optional[int] = None
        self._last_wall_iso: Optional[str] = None
        self._min_delta_ms: Optional[float] = None
        self._max_delta_ms: Optional[float] = None
        self._sum_delta_ms: float = 0.0
        self._delta_samples: int = 0
        self._last_seq: Optional[int] = None
        self._gap_events: int = 0
        self._estimated_missing_total: int = 0
        self._gap_log: List[Dict[str, int]] = []
        self._seq_field_present: bool = False
        self._source_stamp_present: bool = False
        self._message_type: str = 'std_msgs/msg/String'
        self._start_wall = datetime.now().astimezone().isoformat()

        # ── Subscriptions (READ-ONLY — no publisher is ever created) ─
        # QoS mirrors the publisher in eki_axis_move_node.py, which uses the
        # rclpy default profile with depth 10.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth if qos_depth > 0 else DEFAULT_QOS_DEPTH,
        )

        self.create_subscription(String, self._topic, self._on_telemetry, qos)

        if self._log_raw_xml:
            self.create_subscription(
                String, self._raw_xml_topic, self._on_raw_robot_xml, qos)

        # ── Session provenance ───────────────────────────────────────
        self._db.set_session_info({
            'node_name': self.get_name(),
            'telemetry_topic': self._topic,
            'message_type': self._message_type,
            'raw_robot_xml_topic': (
                self._raw_xml_topic if self._log_raw_xml else ''),
            'qos_reliability': 'RELIABLE',
            'qos_durability': 'VOLATILE',
            'qos_history': 'KEEP_LAST',
            'qos_depth': str(qos.depth),
            'session_start_wall_iso8601': self._start_wall,
            'csv_path': csv_path,
            'db_path': db_path,
            'source_timestamp_available': 'unknown_until_first_message',
        })

        # ── Banner ───────────────────────────────────────────────────
        self.get_logger().info('╔══════════════════════════════════════════════╗')
        self.get_logger().info('║  KUKA Telemetry Logger — PASSIVE SUBSCRIBER  ║')
        self.get_logger().info('╚══════════════════════════════════════════════╝')
        self.get_logger().info(f'  Telemetry topic : {self._topic}')
        self.get_logger().info(f'  Message type    : {self._message_type}')
        if self._log_raw_xml:
            self.get_logger().info(f'  Raw XML topic   : {self._raw_xml_topic}')
        self.get_logger().info(
            f'  QoS             : RELIABLE / VOLATILE / KEEP_LAST({qos.depth})')
        self.get_logger().info(f'  CSV             : {csv_path}')
        self.get_logger().info(f'  SQLite          : {db_path}')
        self.get_logger().info(f'  Verbose         : {self._verbose}')
        self.get_logger().info('  Publishers      : NONE (this node never sends)')
        self.get_logger().info('──────────────────────────────────────────────')
        self.get_logger().info('Waiting for telemetry...')

    # ── Telemetry callback ───────────────────────────────────────────

    def _on_telemetry(self, msg: String) -> None:
        """Record one telemetry message. This callback never publishes."""
        now = self.get_clock().now()
        rx_sec, rx_nanosec = now.seconds_nanoseconds()
        rx_ns = rx_sec * 1_000_000_000 + rx_nanosec
        wall_iso = datetime.now().astimezone().isoformat()

        self._count += 1
        index = self._count

        # ── Real inter-arrival time (measured, never assumed) ────────
        delta_ms: Optional[float] = None
        if self._last_rx_ns is not None:
            delta_ms = (rx_ns - self._last_rx_ns) / 1_000_000.0
            self._sum_delta_ms += delta_ms
            self._delta_samples += 1
            if self._min_delta_ms is None or delta_ms < self._min_delta_ms:
                self._min_delta_ms = delta_ms
            if self._max_delta_ms is None or delta_ms > self._max_delta_ms:
                self._max_delta_ms = delta_ms
        else:
            self._first_rx_ns = rx_ns
        self._last_rx_ns = rx_ns
        self._last_wall_iso = wall_iso

        # ── Decode the message ──────────────────────────────────────
        msg_dict = ros_message_to_dict(msg)
        payload_obj, expanded = expand_json_payload(msg_dict, 'data')
        payload_flat = flatten(payload_obj)

        # ── Source timestamp: only if it genuinely exists ───────────
        src_sec, src_nsec, src_ns = find_source_stamp(payload_flat)
        if src_ns is not None:
            self._source_stamp_present = True

        # ── Sequence analysis ───────────────────────────────────────
        seq = find_sequence(payload_flat)
        prev_seq = self._last_seq
        delta_seq: Optional[int] = None
        sequence_gap = 0
        estimated_missing = 0

        if seq is not None:
            self._seq_field_present = True
            if prev_seq is not None:
                delta_seq = seq - prev_seq
                if delta_seq > 1:
                    # Reported as-is. A gap is NOT automatically network loss:
                    # a KUKA restart, a reconnect, or a skipped SPS cycle look
                    # identical from here. It is labelled, not interpreted.
                    sequence_gap = delta_seq
                    estimated_missing = delta_seq - 1
                    self._gap_events += 1
                    self._estimated_missing_total += estimated_missing
                    self._gap_log.append({
                        'receive_index': index,
                        'from_seq': prev_seq,
                        'to_seq': seq,
                        'delta_seq': delta_seq,
                    })
                    self.get_logger().warn(
                        f'[SEQ GAP] {prev_seq} -> {seq} '
                        f'(delta_seq={delta_seq}, '
                        f'estimated_missing={estimated_missing})'
                    )
            self._last_seq = seq

        # ── Build the metadata record ───────────────────────────────
        meta: Dict[str, Any] = {
            'receive_index': index,
            'topic_name': self._topic,
            'message_type': self._message_type,
            'receive_wall_time_iso8601': wall_iso,
            'receive_ros_time_sec': rx_sec,
            'receive_ros_time_nanosec': rx_nanosec,
            'receive_ros_time_ns': rx_ns,
            'delta_receive_ms': delta_ms,
            'source_stamp_sec': src_sec,
            'source_stamp_nanosec': src_nsec,
            'source_stamp_ns': src_ns,
            'sequence': seq,
            'prev_sequence': prev_seq,
            'delta_seq': delta_seq,
            'sequence_gap': sequence_gap if seq is not None else None,
            'estimated_missing': estimated_missing if seq is not None else None,
        }

        # ── Persist ─────────────────────────────────────────────────
        try:
            self._csv.write(meta, payload_flat)
            self._db.write(meta, payload_flat, payload_obj)
        except Exception as exc:                      # never kill the node
            self.get_logger().error(f'Write failed at index {index}: {exc}')
            return

        # ── Console output ──────────────────────────────────────────
        if self._verbose:
            self.get_logger().info(
                f'[{index}] rx={wall_iso} '
                f'dt={"n/a" if delta_ms is None else f"{delta_ms:.2f}ms"} '
                f'seq={seq} payload={payload_obj}'
            )

        if index % self._report_every == 0:
            self._print_report()

        if index == 1 and not expanded:
            self.get_logger().warn(
                'First message payload was not a JSON object — logging the '
                'raw std_msgs/String fields instead. Columns come from the '
                'real message either way.'
            )

    # ── Raw XML callback ─────────────────────────────────────────────

    def _on_raw_robot_xml(self, msg: String) -> None:
        """Archive the raw <Robot> XML frame exactly as the KUKA sent it."""
        now = self.get_clock().now()
        sec, nanosec = now.seconds_nanoseconds()
        self._raw_xml_count += 1
        try:
            self._db.write_raw_xml(
                receive_index=self._raw_xml_count,
                topic_name=self._raw_xml_topic,
                wall_iso=datetime.now().astimezone().isoformat(),
                ros_ns=sec * 1_000_000_000 + nanosec,
                xml=msg.data,
            )
        except Exception as exc:
            self.get_logger().error(f'Raw XML write failed: {exc}')

    # ── Diagnostics ──────────────────────────────────────────────────

    def _elapsed_sec(self) -> float:
        if self._first_rx_ns is None or self._last_rx_ns is None:
            return 0.0
        return (self._last_rx_ns - self._first_rx_ns) / 1e9

    def _average_rate_hz(self) -> Optional[float]:
        elapsed = self._elapsed_sec()
        if elapsed <= 0.0 or self._count < 2:
            return None
        return (self._count - 1) / elapsed

    def _print_report(self) -> None:
        """Periodic compact report (never dumps full messages)."""
        rate = self._average_rate_hz()
        mean_dt = (
            self._sum_delta_ms / self._delta_samples
            if self._delta_samples else None
        )

        self.get_logger().info('[Telemetry Logger]')
        self.get_logger().info(f'  Messages:        {self._count}')
        self.get_logger().info(
            f'  Rate:            '
            f'{"n/a" if rate is None else f"{rate:.2f} Hz"}')
        self.get_logger().info(
            f'  Elapsed:         {self._elapsed_sec():.2f} s')
        self.get_logger().info(
            f'  Delta rx (ms):   '
            f'mean={_fmt(mean_dt)} min={_fmt(self._min_delta_ms)} '
            f'max={_fmt(self._max_delta_ms)}')
        self.get_logger().info(
            f'  Last Seq:        '
            f'{self._last_seq if self._seq_field_present else "N/A (no seq field)"}')
        self.get_logger().info(
            f'  Seq gaps:        {self._gap_events} '
            f'(estimated missing: {self._estimated_missing_total})')
        self.get_logger().info(
            f'  Last msg time:   {self._last_wall_iso}')
        self.get_logger().info(
            f'  Source stamp:    '
            f'{"present" if self._source_stamp_present else "NOT PRESENT (NULL)"}')
        if self._log_raw_xml:
            self.get_logger().info(
                f'  Raw XML frames:  {self._raw_xml_count}')
        self.get_logger().info(f'  CSV:             {self._csv.path}')
        self.get_logger().info(f'  DB:              {self._db.path}')

    def _print_final_summary(self) -> None:
        """Full summary printed once at shutdown."""
        rate = self._average_rate_hz()
        mean_dt = (
            self._sum_delta_ms / self._delta_samples
            if self._delta_samples else None
        )

        self.get_logger().info('══════════ FINAL SUMMARY ══════════')
        self.get_logger().info(f'  Topic:                {self._topic}')
        self.get_logger().info(f'  Message type:         {self._message_type}')
        self.get_logger().info(f'  Messages received:    {self._count}')
        self.get_logger().info(
            f'  Time since first msg: {self._elapsed_sec():.3f} s')
        self.get_logger().info(
            f'  Average rate:         '
            f'{"n/a" if rate is None else f"{rate:.3f} Hz"}')
        self.get_logger().info(
            f'  Mean delta:           {_fmt(mean_dt)} ms')
        self.get_logger().info(
            f'  Min delta:            {_fmt(self._min_delta_ms)} ms')
        self.get_logger().info(
            f'  Max delta:            {_fmt(self._max_delta_ms)} ms')
        self.get_logger().info(
            f'  Last sequence:        '
            f'{self._last_seq if self._seq_field_present else "N/A"}')
        self.get_logger().info(f'  Sequence gap events:  {self._gap_events}')
        self.get_logger().info(
            f'  Estimated missing:    {self._estimated_missing_total} '
            f'(labelled only — NOT confirmed as network loss)')
        self.get_logger().info(
            f'  Last message time:    {self._last_wall_iso}')
        self.get_logger().info(
            f'  Source timestamp:     '
            f'{"present" if self._source_stamp_present else "NOT PRESENT — columns are NULL"}')
        if self._log_raw_xml:
            self.get_logger().info(
                f'  Raw XML frames:       {self._raw_xml_count}')
        if self._csv.unmapped_keys_seen:
            self.get_logger().warn(
                f'  Late-appearing fields (kept in unmapped_json): '
                f'{sorted(self._csv.unmapped_keys_seen)}')
        for gap in self._gap_log[:20]:
            self.get_logger().info(
                f'    gap @ index {gap["receive_index"]}: '
                f'{gap["from_seq"]} -> {gap["to_seq"]} '
                f'(delta_seq={gap["delta_seq"]})')
        if len(self._gap_log) > 20:
            self.get_logger().info(
                f'    ... and {len(self._gap_log) - 20} more gaps '
                f'(all of them are in the CSV / DB)')
        self.get_logger().info(f'  CSV:  {self._csv.path}')
        self.get_logger().info(f'  DB:   {self._db.path}')
        self.get_logger().info('═══════════════════════════════════')

    # ── Cleanup ──────────────────────────────────────────────────────

    def close_files(self) -> None:
        """Flush and close both sinks, then record final session info."""
        try:
            self._db.set_session_info({
                'session_end_wall_iso8601': datetime.now().astimezone().isoformat(),
                'messages_recorded': self._count,
                'raw_xml_frames_recorded': self._raw_xml_count,
                'measured_average_rate_hz': (
                    f'{self._average_rate_hz():.6f}'
                    if self._average_rate_hz() is not None else ''),
                'measured_min_delta_ms': _fmt(self._min_delta_ms),
                'measured_max_delta_ms': _fmt(self._max_delta_ms),
                'sequence_field_present': str(self._seq_field_present),
                'source_timestamp_available': str(self._source_stamp_present),
                'sequence_gap_events': self._gap_events,
                'estimated_missing_total': self._estimated_missing_total,
            })
        except Exception:
            pass

        try:
            self._csv.close()
        except Exception:
            pass
        try:
            self._db.close()
        except Exception:
            pass

    def destroy_node(self):
        self._print_final_summary()
        self.close_files()
        super().destroy_node()


def _fmt(value: Optional[float]) -> str:
    """Format an optional millisecond value for console output."""
    return 'n/a' if value is None else f'{value:.3f}'


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    """Entry point for the passive telemetry logger."""
    argv = list(sys.argv[1:] if args is None else args)

    parser = argparse.ArgumentParser(
        prog='telemetry_logger',
        description=(
            'Passive ROS2 subscriber that records KUKA telemetry to CSV + '
            'SQLite. It never publishes anything.'
        ),
        add_help=False,
    )
    parser.add_argument(
        '--verbose', action='store_true',
        help='Print every received message (noisy).')
    parser.add_argument(
        '-h', '--help', action='store_true', dest='show_help',
        help='Show this help message and exit.')
    known, remaining = parser.parse_known_args(argv)

    if known.show_help:
        parser.print_help()
        print(
            '\nROS2 parameters (pass with --ros-args -p name:=value):\n'
            '  telemetry_topic      (str)  default /kuka/axis_move/feedback_json\n'
            '  raw_robot_xml_topic  (str)  default /kuka/axis_move/raw_robot_xml\n'
            '  log_raw_robot_xml    (bool) default true\n'
            '  log_dir              (str)  default logs\n'
            '  file_prefix          (str)  default kuka_telemetry\n'
            '  qos_depth            (int)  default 10\n'
            '  flush_every          (int)  default 20\n'
            '  report_every         (int)  default 100\n'
            '  verbose              (bool) default false\n'
        )
        return

    rclpy.init(args=remaining)
    node = KukaTelemetryLoggerNode(verbose=known.verbose)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt — closing log files...')
    finally:
        try:
            node.destroy_node()
        except Exception:
            # Make sure the data reaches disk even if teardown misbehaves.
            node.close_files()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        # Give the OS a moment to complete the final fsync.
        time.sleep(0.05)


if __name__ == '__main__':
    main()
