"""
dual_command_model.py — Extended data model for Dual KUKA + RViz mode.

Extends JointCommandModel with:
  - RViz joint feedback (from /kuka_bridge/joint_state_deg)
  - Cartesian target and feedback for RViz
  - MoveIt status text
  - Dual publishing flags

This module has NO ROS2 dependency and can be unit-tested standalone.
"""

import json
import time
from typing import Dict, List, Optional, Tuple

from kuka_gui_control.joint_command_model import (
    JointCommandModel,
    AXES,
    DEFAULT_HOME,
    DEFAULT_LIMITS,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

CART_KEYS: List[str] = ['X', 'Y', 'Z', 'A', 'B', 'C']

DEFAULT_CARTESIAN: Dict[str, float] = {
    'X': 0.0, 'Y': 0.0, 'Z': 0.0,
    'A': 0.0, 'B': 0.0, 'C': 0.0,
}


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

class DualCommandModel(JointCommandModel):
    """
    Data model for the Dual KUKA + RViz GUI.

    Inherits all joint target/feedback/limits from JointCommandModel.
    Adds:
      - RViz joint feedback
      - Cartesian target and feedback
      - MoveIt status tracking
      - Dual publishing flags
    """

    def __init__(
        self,
        home: Optional[Dict[str, float]] = None,
        limits: Optional[Dict[str, Tuple[float, float]]] = None,
        enable_move_default: bool = False,
        step_deg: float = 1.0,
        publish_to_kuka: bool = True,
        publish_to_rviz: bool = True,
        cartesian_to_rviz: bool = True,
    ):
        super().__init__(
            home=home,
            limits=limits,
            enable_move_default=enable_move_default,
            step_deg=step_deg,
        )

        # ── Dual publishing flags ────────────────────────────────────
        self._publish_to_kuka: bool = publish_to_kuka
        self._publish_to_rviz: bool = publish_to_rviz
        self._cartesian_to_rviz: bool = cartesian_to_rviz

        # ── RViz joint feedback ──────────────────────────────────────
        self._rviz_feedback: Dict[str, Optional[float]] = {a: None for a in AXES}
        self._last_rviz_feedback_time: Optional[float] = None

        # ── Cartesian target (for RViz only) ─────────────────────────
        self._cart_target: Dict[str, float] = dict(DEFAULT_CARTESIAN)

        # ── Cartesian feedback from RViz ─────────────────────────────
        self._cart_feedback: Dict[str, Optional[float]] = {
            k: None for k in CART_KEYS
        }
        self._last_cart_feedback_time: Optional[float] = None

        # ── MoveIt status ────────────────────────────────────────────
        self._moveit_status: str = 'Sin estado'
        self._last_status_time: Optional[float] = None

    # ── Dual publishing flags ────────────────────────────────────────

    @property
    def publish_to_kuka(self) -> bool:
        return self._publish_to_kuka

    @publish_to_kuka.setter
    def publish_to_kuka(self, value: bool) -> None:
        self._publish_to_kuka = value

    @property
    def publish_to_rviz(self) -> bool:
        return self._publish_to_rviz

    @publish_to_rviz.setter
    def publish_to_rviz(self, value: bool) -> None:
        self._publish_to_rviz = value

    @property
    def cartesian_to_rviz(self) -> bool:
        return self._cartesian_to_rviz

    @cartesian_to_rviz.setter
    def cartesian_to_rviz(self, value: bool) -> None:
        self._cartesian_to_rviz = value

    # ── RViz joint feedback ──────────────────────────────────────────

    def update_rviz_feedback(self, data: dict) -> None:
        """
        Update RViz joint feedback from a parsed JSON dict.

        Expected keys: A1, A2, A3, A4, A5, A6 (values in degrees).
        """
        for a in AXES:
            val = data.get(a)
            if val is not None:
                try:
                    self._rviz_feedback[a] = float(val)
                except (TypeError, ValueError):
                    pass
        self._last_rviz_feedback_time = time.monotonic()

    def get_rviz_feedback(self, axis: str) -> Optional[float]:
        """Return last RViz feedback value for an axis, or None."""
        return self._rviz_feedback.get(axis)

    def get_rviz_error(self, axis: str) -> Optional[float]:
        """Return target - RViz feedback for an axis, or None."""
        fb = self._rviz_feedback.get(axis)
        if fb is None:
            return None
        return self._target.get(axis, 0.0) - fb

    def has_recent_rviz_feedback(self, timeout_sec: float = 2.0) -> bool:
        """Return True if RViz feedback was received within timeout_sec."""
        if self._last_rviz_feedback_time is None:
            return False
        return (time.monotonic() - self._last_rviz_feedback_time) < timeout_sec

    # ── Cartesian target ─────────────────────────────────────────────

    def set_cart_target(self, key: str, value: float) -> None:
        """Set a single cartesian target (X, Y, Z, A, B, or C)."""
        if key in CART_KEYS:
            self._cart_target[key] = value

    def get_cart_target(self, key: str) -> float:
        """Return cartesian target value."""
        return self._cart_target.get(key, 0.0)

    def get_all_cart_targets(self) -> Dict[str, float]:
        """Return a copy of all cartesian targets."""
        return dict(self._cart_target)

    def reset_cart_targets(self) -> None:
        """Reset cartesian targets to zero."""
        self._cart_target = dict(DEFAULT_CARTESIAN)

    # ── Cartesian feedback ───────────────────────────────────────────

    def update_cart_feedback(self, data: dict) -> None:
        """Update cartesian feedback from RViz."""
        for k in CART_KEYS:
            val = data.get(k)
            if val is not None:
                try:
                    self._cart_feedback[k] = float(val)
                except (TypeError, ValueError):
                    pass
        self._last_cart_feedback_time = time.monotonic()

    def get_cart_feedback(self, key: str) -> Optional[float]:
        """Return last cartesian feedback value, or None."""
        return self._cart_feedback.get(key)

    def has_recent_cart_feedback(self, timeout_sec: float = 2.0) -> bool:
        """Return True if cartesian feedback was received within timeout_sec."""
        if self._last_cart_feedback_time is None:
            return False
        return (time.monotonic() - self._last_cart_feedback_time) < timeout_sec

    # ── MoveIt status ────────────────────────────────────────────────

    def update_moveit_status(self, status: str) -> None:
        """Update MoveIt status text."""
        self._moveit_status = status
        self._last_status_time = time.monotonic()

    @property
    def moveit_status(self) -> str:
        return self._moveit_status

    def has_recent_status(self, timeout_sec: float = 5.0) -> bool:
        """Return True if MoveIt status was received within timeout_sec."""
        if self._last_status_time is None:
            return False
        return (time.monotonic() - self._last_status_time) < timeout_sec

    # ── Build arrays for publishing ──────────────────────────────────

    def build_rviz_joint_array(self) -> List[float]:
        """
        Build the Float64MultiArray data for RViz joint command.

        Returns:
            List of 6 floats [A1, A2, A3, A4, A5, A6] in degrees.
        """
        return [
            round(self._target.get(a, 0.0), 6) for a in AXES
        ]

    def build_cartesian_array(self) -> List[float]:
        """
        Build the Float64MultiArray data for RViz cartesian command.

        Returns:
            List of 6 floats [X, Y, Z, A, B, C].
            X, Y, Z in meters; A, B, C in degrees.
        """
        return [
            round(self._cart_target.get(k, 0.0), 6) for k in CART_KEYS
        ]
