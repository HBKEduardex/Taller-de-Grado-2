"""
dual_command_model.py — Extended data model for Dual KUKA + RViz mode.

Extends JointCommandModel with:
  - RViz joint feedback (from /kuka_bridge/joint_state_deg)
  - Cartesian target and feedback for RViz
  - MoveIt status text
  - Dual publishing flags

This module has NO ROS2 dependency and can be unit-tested standalone.
"""

import time
from typing import Dict, List, Optional, Tuple

from kuka_gui_control.joint_command_model import (
    JointCommandModel,
    AXES,
    CARTESIAN_AXES,
    DEFAULT_HOME,
    DEFAULT_LIMITS,
)


class DualCommandModel(JointCommandModel):
    """
    Data model for the Dual KUKA + RViz GUI.

    Inherits all joint target/feedback/limits from JointCommandModel.
    Adds:
      - RViz joint feedback
      - Cartesian feedback from RViz
      - MoveIt status tracking
      - Dual publishing flags
    """

    def __init__(
        self,
        home: Optional[Dict[str, float]] = None,
        limits: Optional[Dict[str, Tuple[float, float]]] = None,
        enable_move_default: bool = False,
        step_deg: float = 1.0,
        publish_joints_to_kuka: bool = True,
        publish_joints_to_rviz: bool = True,
        publish_cartesian_to_kuka: bool = False,
        publish_cartesian_to_rviz: bool = True,
    ):
        super().__init__(
            home=home,
            limits=limits,
            enable_move_default=enable_move_default,
            step_deg=step_deg,
        )

        # ── Dual publishing flags ────────────────────────────────────
        self.publish_joints_to_kuka: bool = publish_joints_to_kuka
        self.publish_joints_to_rviz: bool = publish_joints_to_rviz
        self.publish_cartesian_to_kuka: bool = publish_cartesian_to_kuka
        self.publish_cartesian_to_rviz: bool = publish_cartesian_to_rviz

        # ── RViz joint feedback ──────────────────────────────────────
        self._rviz_feedback: Dict[str, Optional[float]] = {a: None for a in AXES}
        self._last_rviz_feedback_time: Optional[float] = None

        # ── Cartesian feedback from RViz ─────────────────────────────
        self._rviz_cart_feedback: Dict[str, Optional[float]] = {
            k: None for k in CARTESIAN_AXES
        }
        self._last_rviz_cart_feedback_time: Optional[float] = None

        # ── MoveIt status ────────────────────────────────────────────
        self._moveit_status: str = 'Sin estado'
        self._last_status_time: Optional[float] = None

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
        return self.get_target(axis) - fb

    def has_recent_rviz_feedback(self, timeout_sec: float = 2.0) -> bool:
        """Return True if RViz feedback was received within timeout_sec."""
        if self._last_rviz_feedback_time is None:
            return False
        return (time.monotonic() - self._last_rviz_feedback_time) < timeout_sec

    # ── Cartesian feedback ───────────────────────────────────────────

    def update_rviz_cartesian_feedback(self, data: dict) -> None:
        """Update cartesian feedback from RViz."""
        for k in CARTESIAN_AXES:
            val = data.get(k)
            if val is not None:
                try:
                    self._rviz_cart_feedback[k] = float(val)
                except (TypeError, ValueError):
                    pass
        self._last_rviz_cart_feedback_time = time.monotonic()

    def get_rviz_cartesian_feedback(self, key: str) -> Optional[float]:
        """Return last cartesian feedback value, or None."""
        return self._rviz_cart_feedback.get(key)
        
    def get_rviz_cartesian_error(self, key: str) -> Optional[float]:
        """Return target - RViz cartesian feedback for an axis, or None."""
        fb = self._rviz_cart_feedback.get(key)
        if fb is None:
            return None
        return self.get_target(key) - fb

    def has_recent_cart_feedback(self, timeout_sec: float = 2.0) -> bool:
        """Return True if cartesian feedback was received within timeout_sec."""
        if self._last_rviz_cart_feedback_time is None:
            return False
        return (time.monotonic() - self._last_rviz_cart_feedback_time) < timeout_sec

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
            round(self._target_cartesian.get(k, 0.0), 6) for k in CARTESIAN_AXES
        ]
