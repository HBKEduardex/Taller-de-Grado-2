"""
storage.py — CSV and SQLite writers for the passive telemetry logger.

Two independent sinks, both fed from the same record:

  TelemetryCsvWriter
      One row per received message. The column header is derived from the
      FIRST message actually received, so the columns always match the real
      message — never a hard-coded guess. Any field that shows up later and
      was not in the first message is preserved in the `unmapped_json`
      overflow column, so no data is ever silently dropped.

  TelemetrySqliteWriter
      telemetry_messages : metadata + the complete message as payload_json.
      telemetry_flat     : the well-known KUKA fields in typed columns.
      raw_robot_xml      : optional raw <Robot> XML frames.
      session_info       : key/value provenance for the run.

  payload_json in telemetry_messages is the source of truth: even if the
  message structure changes later, nothing is lost.
"""

import csv
import json
import os
import sqlite3
from typing import Any, Dict, List, Optional

from kuka_telemetry_logger.message_introspection import csv_value

# ---------------------------------------------------------------------------
# Metadata columns produced by the logger itself (always present, in order)
# ---------------------------------------------------------------------------

META_COLUMNS: List[str] = [
    'receive_index',
    'topic_name',
    'message_type',
    'receive_wall_time_iso8601',
    'receive_ros_time_sec',
    'receive_ros_time_nanosec',
    'receive_ros_time_ns',
    'delta_receive_ms',
    'source_stamp_sec',
    'source_stamp_nanosec',
    'source_stamp_ns',
    'sequence',
    'prev_sequence',
    'delta_seq',
    'sequence_gap',
    'estimated_missing',
]

# ---------------------------------------------------------------------------
# Flat-table columns for the KUKA telemetry discovered in the workspace.
#
# Source: /kuka/axis_move/feedback_json, published by node `eki_axis_move`
# (kuka_eki_bridge/eki_axis_move_node.py). Keys on the left are the flattened
# JSON keys; values on the right are the SQLite column names.
# ---------------------------------------------------------------------------

FLAT_FIELD_MAP = {
    'mode':                'mode',
    'status':              'status',
    'move_ready':          'move_ready',
    'limits_ok':           'limits_ok',
    'delta_ok':            'delta_ok',
    'move_executed':       'move_executed',
    'bridge_safe_mode':    'bridge_safe_mode',
    'bridge_allow_motion': 'bridge_allow_motion',
    'axis_actual.A1':      'axis_actual_a1',
    'axis_actual.A2':      'axis_actual_a2',
    'axis_actual.A3':      'axis_actual_a3',
    'axis_actual.A4':      'axis_actual_a4',
    'axis_actual.A5':      'axis_actual_a5',
    'axis_actual.A6':      'axis_actual_a6',
    'position_actual.X':   'position_actual_x',
    'position_actual.Y':   'position_actual_y',
    'position_actual.Z':   'position_actual_z',
    'position_actual.A':   'position_actual_a',
    'position_actual.B':   'position_actual_b',
    'position_actual.C':   'position_actual_c',
}

_FLAT_TEXT_COLUMNS = {'mode'}
_FLAT_INT_COLUMNS = {
    'status', 'move_ready', 'limits_ok', 'delta_ok', 'move_executed',
    'bridge_safe_mode', 'bridge_allow_motion',
}


# ---------------------------------------------------------------------------
# CSV writer
# ---------------------------------------------------------------------------

class TelemetryCsvWriter:
    """Append-only CSV sink whose header is derived from the first message."""

    def __init__(self, path: str, flush_every: int = 20):
        self._path = path
        self._flush_every = max(1, flush_every)
        self._rows_since_flush = 0

        self._file = open(path, 'w', newline='', encoding='utf-8')
        self._writer = csv.writer(self._file)

        self._payload_columns: Optional[List[str]] = None
        self._header: Optional[List[str]] = None
        self.unmapped_keys_seen: set = set()

    @property
    def path(self) -> str:
        return self._path

    @property
    def header(self) -> Optional[List[str]]:
        return list(self._header) if self._header else None

    def write(self, meta: Dict[str, Any], payload_flat: Dict[str, Any]) -> None:
        """Write one row. The first call also writes the header."""
        if self._header is None:
            self._payload_columns = list(payload_flat.keys())
            self._header = (
                META_COLUMNS + self._payload_columns + ['unmapped_json']
            )
            self._writer.writerow(self._header)

        row = [csv_value(meta.get(col)) for col in META_COLUMNS]
        row.extend(csv_value(payload_flat.get(col)) for col in self._payload_columns)

        # Anything the first message did not contain is preserved verbatim.
        unmapped = {
            key: value
            for key, value in payload_flat.items()
            if key not in self._payload_columns
        }
        if unmapped:
            self.unmapped_keys_seen.update(unmapped.keys())
            row.append(json.dumps(unmapped, default=str))
        else:
            row.append('')

        self._writer.writerow(row)

        self._rows_since_flush += 1
        if self._rows_since_flush >= self._flush_every:
            self.flush()

    def flush(self) -> None:
        """Flush buffered rows to disk (and to the OS)."""
        if self._file.closed:
            return
        self._file.flush()
        try:
            os.fsync(self._file.fileno())
        except OSError:
            pass
        self._rows_since_flush = 0

    def close(self) -> None:
        if not self._file.closed:
            self.flush()
            self._file.close()


# ---------------------------------------------------------------------------
# SQLite writer
# ---------------------------------------------------------------------------

_SCHEMA = """
CREATE TABLE IF NOT EXISTS telemetry_messages (
    id                          INTEGER PRIMARY KEY AUTOINCREMENT,
    receive_index               INTEGER NOT NULL,
    topic_name                  TEXT    NOT NULL,
    message_type                TEXT    NOT NULL,
    receive_wall_time_iso8601   TEXT    NOT NULL,
    receive_ros_time_sec        INTEGER NOT NULL,
    receive_ros_time_nanosec    INTEGER NOT NULL,
    receive_ros_time_ns         INTEGER NOT NULL,
    delta_receive_ms            REAL,
    source_stamp_sec            INTEGER,
    source_stamp_nanosec        INTEGER,
    source_stamp_ns             INTEGER,
    sequence                    INTEGER,
    prev_sequence               INTEGER,
    delta_seq                   INTEGER,
    sequence_gap                INTEGER,
    estimated_missing           INTEGER,
    payload_json                TEXT    NOT NULL
);

CREATE TABLE IF NOT EXISTS telemetry_flat (
    id                          INTEGER PRIMARY KEY AUTOINCREMENT,
    receive_index               INTEGER NOT NULL,
    receive_wall_time_iso8601   TEXT    NOT NULL,
    receive_ros_time_ns         INTEGER NOT NULL,
    delta_receive_ms            REAL,
    sequence                    INTEGER,
    prev_sequence               INTEGER,
    delta_seq                   INTEGER,
    sequence_gap                INTEGER,
    estimated_missing           INTEGER,
    mode                        TEXT,
    status                      INTEGER,
    move_ready                  INTEGER,
    limits_ok                   INTEGER,
    delta_ok                    INTEGER,
    move_executed               INTEGER,
    bridge_safe_mode            INTEGER,
    bridge_allow_motion         INTEGER,
    axis_actual_a1              REAL,
    axis_actual_a2              REAL,
    axis_actual_a3              REAL,
    axis_actual_a4              REAL,
    axis_actual_a5              REAL,
    axis_actual_a6              REAL,
    position_actual_x           REAL,
    position_actual_y           REAL,
    position_actual_z           REAL,
    position_actual_a           REAL,
    position_actual_b           REAL,
    position_actual_c           REAL
);

CREATE TABLE IF NOT EXISTS raw_robot_xml (
    id                          INTEGER PRIMARY KEY AUTOINCREMENT,
    receive_index               INTEGER NOT NULL,
    topic_name                  TEXT    NOT NULL,
    receive_wall_time_iso8601   TEXT    NOT NULL,
    receive_ros_time_ns         INTEGER NOT NULL,
    xml                         TEXT    NOT NULL
);

CREATE TABLE IF NOT EXISTS session_info (
    key                         TEXT PRIMARY KEY,
    value                       TEXT
);

CREATE INDEX IF NOT EXISTS idx_messages_seq
    ON telemetry_messages (sequence);
CREATE INDEX IF NOT EXISTS idx_messages_rx_ns
    ON telemetry_messages (receive_ros_time_ns);
CREATE INDEX IF NOT EXISTS idx_flat_seq
    ON telemetry_flat (sequence);
CREATE INDEX IF NOT EXISTS idx_flat_rx_ns
    ON telemetry_flat (receive_ros_time_ns);
"""

_MESSAGES_COLUMNS = [
    'receive_index', 'topic_name', 'message_type',
    'receive_wall_time_iso8601',
    'receive_ros_time_sec', 'receive_ros_time_nanosec', 'receive_ros_time_ns',
    'delta_receive_ms',
    'source_stamp_sec', 'source_stamp_nanosec', 'source_stamp_ns',
    'sequence', 'prev_sequence', 'delta_seq', 'sequence_gap',
    'estimated_missing', 'payload_json',
]

_FLAT_COLUMNS = [
    'receive_index', 'receive_wall_time_iso8601', 'receive_ros_time_ns',
    'delta_receive_ms',
    'sequence', 'prev_sequence', 'delta_seq', 'sequence_gap',
    'estimated_missing',
] + list(FLAT_FIELD_MAP.values())


class TelemetrySqliteWriter:
    """SQLite sink: full JSON payload plus a typed flat table."""

    def __init__(self, path: str, commit_every: int = 20):
        self._path = path
        self._commit_every = max(1, commit_every)
        self._pending = 0

        self._conn = sqlite3.connect(path, check_same_thread=False)
        self._conn.execute('PRAGMA journal_mode=WAL;')
        self._conn.execute('PRAGMA synchronous=NORMAL;')
        self._conn.executescript(_SCHEMA)
        self._conn.commit()

    @property
    def path(self) -> str:
        return self._path

    def set_session_info(self, info: Dict[str, Any]) -> None:
        """Record provenance for this logging session."""
        self._conn.executemany(
            'INSERT OR REPLACE INTO session_info (key, value) VALUES (?, ?)',
            [(str(k), str(v)) for k, v in info.items()],
        )
        self._conn.commit()

    def write(
        self,
        meta: Dict[str, Any],
        payload_flat: Dict[str, Any],
        payload_obj: Any,
    ) -> None:
        """Insert one message into telemetry_messages and telemetry_flat."""
        msg_row = [meta.get(col) for col in _MESSAGES_COLUMNS[:-1]]
        msg_row.append(json.dumps(payload_obj, default=str))
        self._conn.execute(
            'INSERT INTO telemetry_messages ({}) VALUES ({})'.format(
                ', '.join(_MESSAGES_COLUMNS),
                ', '.join('?' * len(_MESSAGES_COLUMNS)),
            ),
            msg_row,
        )

        flat_row: List[Any] = [
            meta.get('receive_index'),
            meta.get('receive_wall_time_iso8601'),
            meta.get('receive_ros_time_ns'),
            meta.get('delta_receive_ms'),
            meta.get('sequence'),
            meta.get('prev_sequence'),
            meta.get('delta_seq'),
            meta.get('sequence_gap'),
            meta.get('estimated_missing'),
        ]
        for source_key, column in FLAT_FIELD_MAP.items():
            flat_row.append(
                _coerce_for_column(payload_flat.get(source_key), column)
            )

        self._conn.execute(
            'INSERT INTO telemetry_flat ({}) VALUES ({})'.format(
                ', '.join(_FLAT_COLUMNS),
                ', '.join('?' * len(_FLAT_COLUMNS)),
            ),
            flat_row,
        )

        self._pending += 1
        if self._pending >= self._commit_every:
            self.commit()

    def write_raw_xml(
        self,
        receive_index: int,
        topic_name: str,
        wall_iso: str,
        ros_ns: int,
        xml: str,
    ) -> None:
        """Insert one raw <Robot> XML frame."""
        self._conn.execute(
            'INSERT INTO raw_robot_xml '
            '(receive_index, topic_name, receive_wall_time_iso8601, '
            ' receive_ros_time_ns, xml) VALUES (?, ?, ?, ?, ?)',
            (receive_index, topic_name, wall_iso, ros_ns, xml),
        )
        self._pending += 1
        if self._pending >= self._commit_every:
            self.commit()

    def commit(self) -> None:
        self._conn.commit()
        self._pending = 0

    def close(self) -> None:
        try:
            self.commit()
        finally:
            self._conn.close()


def _coerce_for_column(value: Any, column: str) -> Any:
    """Cast a flattened value to the type the flat table declares."""
    if value is None:
        return None
    if column in _FLAT_TEXT_COLUMNS:
        return str(value)
    if column in _FLAT_INT_COLUMNS:
        if isinstance(value, bool):
            return 1 if value else 0
        try:
            return int(value)
        except (TypeError, ValueError):
            return None
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    try:
        return float(value)
    except (TypeError, ValueError):
        return None
