"""
message_introspection.py — generic ROS2 message / JSON payload introspection.

Nothing in this module is specific to a single message type. It converts an
arbitrary ROS2 message into a plain dict, optionally expands a JSON string
payload (which is what the existing KUKA bridge publishes inside
std_msgs/String), flattens nested structures for CSV, and looks for
timestamp / sequence fields WITHOUT inventing them.

Design rule enforced here: if a timestamp or a sequence does not exist in the
received data, the corresponding value is None. It is never fabricated.
"""

import json
from typing import Any, Dict, Optional, Tuple

# ---------------------------------------------------------------------------
# Field-name candidates
# ---------------------------------------------------------------------------

# Sequence field candidates, searched in this order over the flattened keys.
# The KUKA bridge publishes 'seq' (from Robot/Seq).
_SEQ_CANDIDATES = (
    'seq',
    'Seq',
    'sequence',
    'Sequence',
    'seq_id',
    'sequence_number',
    'header.seq',
)

# Timestamp candidates: (seconds_key, nanoseconds_key).
# std_msgs/String has no Header, so none of these will match for the current
# KUKA telemetry — the columns stay NULL, by design.
_STAMP_PAIR_CANDIDATES = (
    ('header.stamp.sec', 'header.stamp.nanosec'),
    ('stamp.sec', 'stamp.nanosec'),
    ('header.stamp.secs', 'header.stamp.nsecs'),
    ('timestamp.sec', 'timestamp.nanosec'),
)

# Single-field nanosecond timestamp candidates.
_STAMP_NS_CANDIDATES = (
    'stamp_ns',
    'timestamp_ns',
    'source_stamp_ns',
    't_ns',
)

# Single-field float-seconds timestamp candidates.
_STAMP_SEC_FLOAT_CANDIDATES = (
    'timestamp',
    'stamp',
    'time',
    'kuka_time',
    'robot_time',
)


# ---------------------------------------------------------------------------
# ROS2 message -> dict
# ---------------------------------------------------------------------------

def ros_message_to_dict(msg: Any) -> Dict[str, Any]:
    """
    Convert any ROS2 message instance into a plain nested dict.

    Uses rosidl_runtime_py when available; otherwise walks
    get_fields_and_field_types() manually. Both paths keep the real field
    names of the message — nothing is renamed.
    """
    try:
        from rosidl_runtime_py.convert import message_to_ordereddict
        return json.loads(json.dumps(message_to_ordereddict(msg), default=str))
    except Exception:
        pass

    return _manual_message_to_dict(msg)


def _manual_message_to_dict(msg: Any) -> Dict[str, Any]:
    """Fallback conversion using the message introspection API."""
    if not hasattr(msg, 'get_fields_and_field_types'):
        return {'value': _coerce_scalar(msg)}

    out: Dict[str, Any] = {}
    for field in msg.get_fields_and_field_types().keys():
        value = getattr(msg, field, None)
        out[field] = _convert_value(value)
    return out


def _convert_value(value: Any) -> Any:
    """Recursively convert a message field value into JSON-safe data."""
    if hasattr(value, 'get_fields_and_field_types'):
        return _manual_message_to_dict(value)
    if isinstance(value, (list, tuple)):
        return [_convert_value(v) for v in value]
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    return _coerce_scalar(value)


def _coerce_scalar(value: Any) -> Any:
    """Return value if it is JSON-serialisable, else its string form."""
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    try:
        import array
        if isinstance(value, array.array):
            return list(value)
    except Exception:
        pass
    return str(value)


# ---------------------------------------------------------------------------
# JSON payload expansion
# ---------------------------------------------------------------------------

def expand_json_payload(
    msg_dict: Dict[str, Any],
    payload_field: str = 'data',
) -> Tuple[Dict[str, Any], bool]:
    """
    If msg_dict[payload_field] is a JSON object string, replace the message
    dict with the decoded object.

    The existing KUKA bridge publishes std_msgs/String whose `data` field is a
    JSON object (seq, mode, status, axis_actual, position_actual, ...). Storing
    that JSON as one opaque string would defeat the purpose of the logger, so
    it is expanded into real fields here.

    Returns:
        (resulting_dict, was_expanded)
        `was_expanded` is False when the field is absent or not a JSON object,
        in which case the original dict is returned untouched.
    """
    raw = msg_dict.get(payload_field)
    if not isinstance(raw, str):
        return msg_dict, False

    text = raw.strip()
    if not text.startswith('{'):
        return msg_dict, False

    try:
        decoded = json.loads(text)
    except (json.JSONDecodeError, ValueError):
        return msg_dict, False

    if not isinstance(decoded, dict):
        return msg_dict, False

    return decoded, True


# ---------------------------------------------------------------------------
# Flattening
# ---------------------------------------------------------------------------

def flatten(
    obj: Any,
    prefix: str = '',
    separator: str = '.',
) -> Dict[str, Any]:
    """
    Flatten a nested dict/list structure into a single-level dict.

    Nested dicts join keys with `separator`:
        {'axis_actual': {'A1': 0.0}}  ->  {'axis_actual.A1': 0.0}

    Lists are indexed:
        {'v': [1, 2]}                 ->  {'v.0': 1, 'v.1': 2}

    Key names are taken verbatim from the source data — no renaming, no
    case conversion.
    """
    flat: Dict[str, Any] = {}

    if isinstance(obj, dict):
        for key, value in obj.items():
            new_prefix = f'{prefix}{separator}{key}' if prefix else str(key)
            flat.update(flatten(value, new_prefix, separator))
    elif isinstance(obj, (list, tuple)):
        if not obj:
            flat[prefix] = ''
        else:
            for index, value in enumerate(obj):
                new_prefix = (
                    f'{prefix}{separator}{index}' if prefix else str(index)
                )
                flat.update(flatten(value, new_prefix, separator))
    else:
        flat[prefix] = obj

    return flat


# ---------------------------------------------------------------------------
# Timestamp / sequence discovery
# ---------------------------------------------------------------------------

def find_source_stamp(
    flat: Dict[str, Any],
) -> Tuple[Optional[int], Optional[int], Optional[int]]:
    """
    Look for a timestamp that came from the SOURCE (the message itself).

    Returns:
        (sec, nanosec, total_ns) or (None, None, None) when the message
        carries no timestamp at all.

    IMPORTANT: this never falls back to the local clock. A missing source
    timestamp must stay NULL so the log honestly reflects that the KUKA does
    not send one.
    """
    for sec_key, nsec_key in _STAMP_PAIR_CANDIDATES:
        if sec_key in flat and nsec_key in flat:
            sec = _as_int(flat[sec_key])
            nsec = _as_int(flat[nsec_key])
            if sec is not None and nsec is not None:
                return sec, nsec, sec * 1_000_000_000 + nsec

    for key in _STAMP_NS_CANDIDATES:
        if key in flat:
            total = _as_int(flat[key])
            if total is not None:
                return total // 1_000_000_000, total % 1_000_000_000, total

    for key in _STAMP_SEC_FLOAT_CANDIDATES:
        if key in flat:
            value = flat[key]
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                total = int(round(float(value) * 1_000_000_000))
                return (
                    total // 1_000_000_000,
                    total % 1_000_000_000,
                    total,
                )

    return None, None, None


def find_sequence(flat: Dict[str, Any]) -> Optional[int]:
    """
    Look for a sequence counter in the flattened message.

    Returns the integer value, or None when the message has no sequence
    field. Never fabricated.
    """
    for key in _SEQ_CANDIDATES:
        if key in flat:
            value = _as_int(flat[key])
            if value is not None:
                return value

    # Also accept a nested sequence such as 'robot.Seq'.
    for key, value in flat.items():
        tail = key.rsplit('.', 1)[-1]
        if tail in _SEQ_CANDIDATES:
            parsed = _as_int(value)
            if parsed is not None:
                return parsed

    return None


def _as_int(value: Any) -> Optional[int]:
    """Best-effort int conversion; None when not convertible."""
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return int(value)
    if isinstance(value, str):
        try:
            return int(value.strip())
        except ValueError:
            return None
    return None


def csv_value(value: Any) -> str:
    """
    Render a flattened value for a CSV cell.

    None becomes an empty cell (SQL NULL semantics), booleans become
    'True'/'False' so they round-trip readably, everything else is str().
    """
    if value is None:
        return ''
    if isinstance(value, bool):
        return 'True' if value else 'False'
    return str(value)
