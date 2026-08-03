"""
joint_command_model.py — Data model for KUKA joint targets and feedback.

Provides:
  - JointCommandModel: stores current targets, feedback, and validates
    against soft limits.
  - Generates the JSON payload for publishing to ROS2.

This module has NO ROS2 dependency and can be unit-tested standalone.
"""

import json
import time
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

AXES: List[str] = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']

DEFAULT_HOME: Dict[str, float] = {
    'A1': 0.0,
    'A2': -90.0,
    'A3': 90.0,
    'A4': 0.0,
    'A5': 0.0,
    'A6': 0.0,
}

DEFAULT_LIMITS: Dict[str, Tuple[float, float]] = {
    'A1': (-160.0, 160.0),
    'A2': (-180.0,  35.0),
    'A3': (-110.0, 146.0),
    'A4': (-175.0, 175.0),
    'A5': (-110.0, 110.0),
    'A6': (-340.0, 340.0),
}


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

class JointCommandModel:
    """
    Data model for the KUKA joint GUI.

    Holds:
      - target values (GUI-edited positions to be sent)
      - feedback values (latest received from /feedback_json topic)
      - soft limits
      - sequence counter
      - mode (manual_send / auto)
      - enable_move flag
    """

    def __init__(
        self,
        home: Optional[Dict[str, float]] = None,
        limits: Optional[Dict[str, Tuple[float, float]]] = None,
        enable_move_default: bool = True,
        step_deg: float = 1.0,
    ):
        self._home: Dict[str, float] = home or dict(DEFAULT_HOME)
        self._limits: Dict[str, Tuple[float, float]] = limits or dict(DEFAULT_LIMITS)
        self._step_deg: float = step_deg

        # Current target values (what will be sent)
        self._target: Dict[str, float] = dict(self._home)

        # Latest feedback from KUKA
        self._feedback: Dict[str, Optional[float]] = {a: None for a in AXES}
        self._last_feedback_time: Optional[float] = None

        # Position actual (Cartesian)
        self._position_actual: Dict[str, Optional[float]] = {
            k: None for k in ['X', 'Y', 'Z', 'A', 'B', 'C']
        }

        # Sequence counter for published messages
        self._seq: int = 0

        # Enable move flag
        self._enable_move: bool = enable_move_default

        # Mode: 'manual_send' or 'auto'
        self._mode: str = 'manual_send'

    # ── Targets ─────────────────────────────────────────────────────

    def set_target(self, axis: str, value: float) -> None:
        """Set a single axis target (no limit check here — use is_in_limits)."""
        if axis in AXES:
            self._target[axis] = value

    def get_target(self, axis: str) -> float:
        """Return target value for an axis."""
        return self._target.get(axis, 0.0)

    def get_all_targets(self) -> Dict[str, float]:
        """Return a copy of all target values."""
        return dict(self._target)

    def load_home(self) -> None:
        """Reset all targets to home position."""
        self._target = dict(self._home)

    def step_target(self, axis: str, direction: int) -> float:
        """
        Increment or decrement a target by step_deg.

        Args:
            axis:      Axis name (A1-A6).
            direction: +1 to increment, -1 to decrement.

        Returns:
            New value after stepping (may be out of limits — caller must check).
        """
        current = self._target.get(axis, 0.0)
        new_val = current + direction * self._step_deg
        self._target[axis] = new_val
        return new_val

    # ── Limits ──────────────────────────────────────────────────────

    def is_in_limits(self, axis: str, value: Optional[float] = None) -> bool:
        """
        Check if a value (or current target if None) is within soft limits.

        Args:
            axis:  Axis name.
            value: Value to check. Defaults to current target for that axis.

        Returns:
            True if within limits.
        """
        if value is None:
            value = self._target.get(axis, 0.0)
        lo, hi = self._limits.get(axis, (-360.0, 360.0))
        return lo <= value <= hi

    def all_targets_in_limits(self) -> bool:
        """Return True if all current targets are within soft limits."""
        return all(self.is_in_limits(a) for a in AXES)

    def get_limits(self, axis: str) -> Tuple[float, float]:
        """Return (min, max) soft limits for an axis."""
        return self._limits.get(axis, (-360.0, 360.0))

    def set_limits(self, limits: Dict[str, Tuple[float, float]]) -> None:
        """Update the soft limits dictionary."""
        self._limits.update(limits)

    # ── Feedback ─────────────────────────────────────────────────────

    def update_feedback(self, data: dict) -> None:
        """
        Update feedback from a parsed JSON dict received from the bridge.

        Expected keys: seq, mode, status, axis_actual, position_actual.
        """
        axis_actual = data.get('axis_actual', {})
        for a in AXES:
            val = axis_actual.get(a)
            if val is not None:
                try:
                    self._feedback[a] = float(val)
                except (TypeError, ValueError):
                    pass

        pos_actual = data.get('position_actual', {})
        for k in ['X', 'Y', 'Z', 'A', 'B', 'C']:
            val = pos_actual.get(k)
            if val is not None:
                try:
                    self._position_actual[k] = float(val)
                except (TypeError, ValueError):
                    pass

        self._last_feedback_time = time.monotonic()

    def get_feedback(self, axis: str) -> Optional[float]:
        """Return last feedback value for an axis, or None if not received."""
        return self._feedback.get(axis)

    def get_error(self, axis: str) -> Optional[float]:
        """
        Return target - feedback for an axis, or None if no feedback.
        """
        fb = self._feedback.get(axis)
        if fb is None:
            return None
        return self._target.get(axis, 0.0) - fb

    def has_recent_feedback(self, timeout_sec: float = 2.0) -> bool:
        """Return True if feedback was received within timeout_sec seconds."""
        if self._last_feedback_time is None:
            return False
        return (time.monotonic() - self._last_feedback_time) < timeout_sec

    def clear_feedback(self) -> None:
        """Clear all feedback (simulate timeout)."""
        self._feedback = {a: None for a in AXES}
        self._last_feedback_time = None

    # ── Mode & enable_move ───────────────────────────────────────────

    def set_mode(self, mode: str) -> None:
        """Set the current mode string: 'manual_send' or 'auto'."""
        self._mode = mode

    def get_mode(self) -> str:
        return self._mode

    def set_enable_move(self, value: bool) -> None:
        self._enable_move = value

    def get_enable_move(self) -> bool:
        return self._enable_move

    # ── Sequence ─────────────────────────────────────────────────────

    def next_seq(self) -> int:
        """Increment and return the next sequence number."""
        self._seq += 1
        return self._seq

    def get_seq(self) -> int:
        return self._seq

    # ── JSON builder ─────────────────────────────────────────────────

    def build_target_json(self) -> str:
        """
        Build the JSON string to publish on the command topic.

        Returns:
            JSON string with seq, source, mode, enable_move, and A1-A6.
        """
        seq = self.next_seq()
        payload = {
            'seq': seq,
            'source': 'kuka_gui_control',
            'mode': self._mode,
            'enable_move': self._enable_move,
        }
        for a in AXES:
            payload[a] = round(self._target.get(a, 0.0), 6)

        return json.dumps(payload)

    # ── Home config ──────────────────────────────────────────────────

    def get_home(self) -> Dict[str, float]:
        return dict(self._home)

    def set_home(self, home: Dict[str, float]) -> None:
        self._home = dict(home)

    @property
    def step_deg(self) -> float:
        return self._step_deg

    @step_deg.setter
    def step_deg(self, value: float) -> None:
        self._step_deg = value
