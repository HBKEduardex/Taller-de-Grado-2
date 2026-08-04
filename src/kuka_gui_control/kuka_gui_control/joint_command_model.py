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

CARTESIAN_AXES: List[str] = ['X', 'Y', 'Z', 'A', 'B', 'C']

DEFAULT_HOME: Dict[str, float] = {
    'A1': 0.0,
    'A2': -90.0,
    'A3': 90.0,
    'A4': 0.0,
    'A5': 0.0,
    'A6': 0.0,
}

DEFAULT_CARTESIAN_HOME: Dict[str, float] = {
    'X': 400.0,
    'Y': 0.0,
    'Z': 600.0,
    'A': 0.0,
    'B': 90.0,
    'C': 0.0,
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
        self._cartesian_home: Dict[str, float] = dict(DEFAULT_CARTESIAN_HOME)
        self._limits: Dict[str, Tuple[float, float]] = limits or dict(DEFAULT_LIMITS)
        self._step_deg: float = step_deg
        self._step_mm: float = 1.0

        # Current target values (what will be sent)
        self._target: Dict[str, float] = dict(self._home)
        self._target_cartesian: Dict[str, float] = dict(self._cartesian_home)

        # Latest feedback from KUKA
        self._feedback: Dict[str, Optional[float]] = {a: None for a in AXES}
        self._last_feedback_time: Optional[float] = None

        # Position actual (Cartesian)
        self._position_actual: Dict[str, Optional[float]] = {
            k: None for k in CARTESIAN_AXES
        }

        # Sequence counter for published messages
        self._seq: int = 0

        # Enable move flag
        self._enable_move: bool = enable_move_default

        # Mode for node: 'manual_send' or 'auto'
        self._node_mode: str = 'manual_send'
        
        # Target Mode for KUKA: 'AxisTarget' or 'CartesianTarget'
        self._target_mode: str = 'AxisTarget'

    # ── Targets ─────────────────────────────────────────────────────

    def set_target(self, axis: str, value: float) -> None:
        """Set a single axis target (no limit check here — use is_in_limits)."""
        if axis in AXES:
            self._target[axis] = value
        elif axis in CARTESIAN_AXES:
            self._target_cartesian[axis] = value

    def get_target(self, axis: str) -> float:
        """Return target value for an axis or cartesian coordinate."""
        if axis in AXES:
            return self._target.get(axis, 0.0)
        return self._target_cartesian.get(axis, 0.0)

    def get_all_targets(self) -> Dict[str, float]:
        """Return a copy of all target values."""
        return dict(self._target)

    def load_home(self) -> None:
        """Reset all targets to home position."""
        self._target = dict(self._home)

    def step_target(self, axis: str, direction: int) -> float:
        """
        Increment or decrement a target by step.

        Args:
            axis:      Axis name (A1-A6 or X-C).
            direction: +1 to increment, -1 to decrement.

        Returns:
            New value after stepping (may be out of limits — caller must check).
        """
        if axis in AXES:
            current = self._target.get(axis, 0.0)
            new_val = current + direction * self._step_deg
            self._target[axis] = new_val
            return new_val
        elif axis in CARTESIAN_AXES:
            current = self._target_cartesian.get(axis, 0.0)
            step = self._step_mm if axis in ['X', 'Y', 'Z'] else self._step_deg
            new_val = current + direction * step
            self._target_cartesian[axis] = new_val
            return new_val
        return 0.0

    # ── Limits ──────────────────────────────────────────────────────

    def is_in_limits(self, axis: str, value: Optional[float] = None) -> bool:
        """
        Check if a value (or current target if None) is within soft limits.
        """
        if axis in CARTESIAN_AXES:
            return True # Cartesian limits not enforced in GUI
            
        if value is None:
            value = self._target.get(axis, 0.0)
            
        lo, hi = self._limits.get(axis, (-360.0, 360.0))
        return lo <= value <= hi

    def all_targets_in_limits(self) -> bool:
        """Return True if all current targets are within soft limits."""
        return all(self.is_in_limits(a) for a in AXES)

    def clamp(self, axis: str, value: float) -> float:
        """Return value clamped to soft limits."""
        if axis in CARTESIAN_AXES:
            return value
        lo, hi = self._limits.get(axis, (-360.0, 360.0))
        return max(lo, min(value, hi))

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
        for k in CARTESIAN_AXES:
            val = pos_actual.get(k)
            if val is not None:
                try:
                    self._position_actual[k] = float(val)
                except (TypeError, ValueError):
                    pass

        self._last_feedback_time = time.monotonic()

    def get_feedback(self, axis: str) -> Optional[float]:
        """Return last feedback value for an axis, or None if not received."""
        if axis in AXES:
            return self._feedback.get(axis)
        return self._position_actual.get(axis)

    def get_error(self, axis: str) -> Optional[float]:
        """
        Return target - feedback for an axis, or None if no feedback.
        """
        if axis in AXES:
            fb = self._feedback.get(axis)
            target = self._target.get(axis, 0.0)
        else:
            fb = self._position_actual.get(axis)
            target = self._target_cartesian.get(axis, 0.0)

        if fb is None:
            return None
        return target - fb

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

    def set_target_mode(self, mode: str) -> None:
        """Set 'AxisTarget' or 'CartesianTarget'."""
        self._target_mode = mode

    def get_target_mode(self) -> str:
        return self._target_mode

    def set_node_mode(self, mode: str) -> None:
        """Set the current node mode string: 'manual_send' or 'auto'."""
        self._node_mode = mode

    def get_node_mode(self) -> str:
        return self._node_mode

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
            'node_mode': self._node_mode,
            'mode': self._target_mode,
            'enable_move': self._enable_move,
            'axis_target': {},
            'cartesian_target': {}
        }
        for a in AXES:
            payload['axis_target'][a] = round(self._target.get(a, 0.0), 6)
            payload[a] = payload['axis_target'][a] # Legacy flat support
            
        for a in CARTESIAN_AXES:
            payload['cartesian_target'][a] = round(self._target_cartesian.get(a, 0.0), 6)

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

    @property
    def step_mm(self) -> float:
        return self._step_mm

    @step_mm.setter
    def step_mm(self, value: float) -> None:
        self._step_mm = value
