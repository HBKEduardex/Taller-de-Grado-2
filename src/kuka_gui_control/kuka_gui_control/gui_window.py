"""
gui_window.py — PyQt5 main window for KUKA joint control.

Two-screen flow:
  Screen 1 (Welcome):  Title + description + START button.
  Screen 2 (Control):  Status panel, joint table, joint controls, action buttons.

All ROS2 communication is delegated to RosGuiBridge via Qt signals.
This file has no direct rclpy dependency.
"""

import json
import time
from typing import Dict, Optional

try:
    from PyQt5.QtCore import Qt, QTimer, pyqtSlot
    from PyQt5.QtGui import QColor, QFont, QPalette
    from PyQt5.QtWidgets import (
        QApplication, QFrame, QGridLayout, QGroupBox,
        QHBoxLayout, QLabel, QLineEdit, QMainWindow,
        QMessageBox, QPushButton, QSizePolicy,
        QStackedWidget, QTextEdit, QVBoxLayout, QWidget,
    )
except ImportError as e:
    raise ImportError(
        'PyQt5 is required. Install with:  sudo apt install python3-pyqt5'
    ) from e

from kuka_gui_control.joint_command_model import JointCommandModel, AXES

# ---------------------------------------------------------------------------
# Style constants
# ---------------------------------------------------------------------------

DARK_BG = '#0d1117'
PANEL_BG = '#161b22'
BORDER_CLR = '#30363d'
ACCENT = '#58a6ff'
ACCENT2 = '#3fb950'
WARN_CLR = '#f78166'
TEXT_PRI = '#e6edf3'
TEXT_SEC = '#8b949e'
AUTO_ON_CLR = '#388bfd'
STOP_CLR = '#da3633'

FONT_FAMILY = 'Inter, Segoe UI, DejaVu Sans, sans-serif'

BASE_STYLE = f"""
QMainWindow, QWidget {{
    background-color: {DARK_BG};
    color: {TEXT_PRI};
    font-family: {FONT_FAMILY};
    font-size: 13px;
}}
QGroupBox {{
    border: 1px solid {BORDER_CLR};
    border-radius: 8px;
    margin-top: 10px;
    padding: 8px;
    font-weight: bold;
    color: {ACCENT};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
}}
QLabel {{
    color: {TEXT_PRI};
}}
QLineEdit {{
    background-color: {PANEL_BG};
    border: 1px solid {BORDER_CLR};
    border-radius: 4px;
    color: {TEXT_PRI};
    padding: 3px 6px;
    selection-background-color: {ACCENT};
}}
QLineEdit:focus {{
    border: 1px solid {ACCENT};
}}
QTextEdit {{
    background-color: {PANEL_BG};
    border: 1px solid {BORDER_CLR};
    border-radius: 4px;
    color: {TEXT_SEC};
    font-family: monospace;
    font-size: 11px;
}}
QPushButton {{
    border-radius: 6px;
    padding: 6px 16px;
    font-weight: bold;
    font-size: 13px;
}}
QPushButton:disabled {{
    opacity: 0.4;
    color: {TEXT_SEC};
}}
QFrame[frameShape="4"], QFrame[frameShape="5"] {{
    color: {BORDER_CLR};
}}
"""

def _btn(text: str, color: str, min_width: int = 80) -> QPushButton:
    b = QPushButton(text)
    b.setMinimumWidth(min_width)
    b.setStyleSheet(
        f'QPushButton {{ background-color: {color}; color: #ffffff; }}'
        f'QPushButton:hover {{ background-color: {color}dd; }}'
        f'QPushButton:pressed {{ background-color: {color}99; }}'
        f'QPushButton:disabled {{ background-color: {BORDER_CLR}; color: {TEXT_SEC}; }}'
    )
    return b

def _sep() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.HLine)
    f.setStyleSheet(f'color: {BORDER_CLR};')
    return f


# ---------------------------------------------------------------------------
# Screen 1 — Welcome
# ---------------------------------------------------------------------------

class WelcomeScreen(QWidget):
    """Simple welcome/splash screen with a START button."""

    def __init__(self, on_start, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(24)
        layout.setContentsMargins(60, 60, 60, 60)

        # Logo / title area
        title_label = QLabel('🤖  KUKA Joint Control GUI')
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet(
            f'font-size: 32px; font-weight: bold; color: {ACCENT};'
            f'letter-spacing: 1px;'
        )

        subtitle = QLabel(
            'Sistema de envío de posiciones articulares hacia KUKA\n'
            'mediante ROS2 · kuka_eki_bridge · axis_command_loop'
        )
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setStyleSheet(f'color: {TEXT_SEC}; font-size: 15px;')

        # Info box
        info_frame = QFrame()
        info_frame.setStyleSheet(
            f'background-color: {PANEL_BG}; border: 1px solid {BORDER_CLR};'
            f'border-radius: 10px; padding: 16px;'
        )
        info_layout = QVBoxLayout(info_frame)
        for line in [
            f'<b style="color:{ACCENT2}">Topic de comando:</b>  '
            f'/kuka/axis_command/target_json',
            f'<b style="color:{ACCENT2}">Topic de feedback:</b>  '
            f'/kuka/axis_command_loop/feedback_json',
            f'<b style="color:{WARN_CLR}">⚠  Esta GUI solo envía objetivos articulares.</b>',
            f'<b style="color:{WARN_CLR}">   El movimiento real queda bajo control del KRL del KUKA.</b>',
        ]:
            lbl = QLabel(line)
            lbl.setTextFormat(Qt.RichText)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(f'font-size: 13px; color: {TEXT_PRI};')
            info_layout.addWidget(lbl)

        # START button
        start_btn = _btn('▶   START', ACCENT2, min_width=200)
        start_btn.setMinimumHeight(50)
        start_btn.setStyleSheet(
            f'QPushButton {{ background-color: {ACCENT2}; color: #0d1117;'
            f' font-size: 18px; font-weight: bold; border-radius: 10px; }}'
            f'QPushButton:hover {{ background-color: #56d364; }}'
        )
        start_btn.clicked.connect(on_start)
        start_btn.setObjectName('btn_start')

        layout.addStretch()
        layout.addWidget(title_label)
        layout.addWidget(subtitle)
        layout.addWidget(info_frame)
        layout.addSpacing(12)
        layout.addWidget(start_btn, alignment=Qt.AlignCenter)
        layout.addStretch()


# ---------------------------------------------------------------------------
# Screen 2 — Control
# ---------------------------------------------------------------------------

class ControlScreen(QWidget):
    """Main control panel: status, joint table, controls, buttons."""

    def __init__(self, model: JointCommandModel, bridge, config: dict, parent=None):
        super().__init__(parent)
        self._model = model
        self._bridge = bridge
        self._cfg = config
        self._auto_timer: Optional[QTimer] = None
        self._feedback_check_timer: Optional[QTimer] = None
        self._auto_mode_on = False
        self._feedback_timeout = config.get('feedback_timeout_sec', 2.0)
        self._last_sent_json: str = '—'
        self._last_feedback_json: str = '—'

        # Per-joint widgets (built in _build_joint_table and _build_joint_controls)
        self._target_fields: Dict[str, QLineEdit] = {}
        self._feedback_labels: Dict[str, QLabel] = {}
        self._error_labels: Dict[str, QLabel] = {}
        self._joint_status_labels: Dict[str, QLabel] = {}  # OK / !! OOL

        self._build_ui()
        self._start_feedback_check()

    # ── UI construction ──────────────────────────────────────────────

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        root.addWidget(self._build_status_panel())
        root.addWidget(_sep())
        root.addWidget(self._build_joint_table())
        root.addWidget(_sep())
        root.addWidget(self._build_joint_controls())
        root.addWidget(_sep())
        root.addWidget(self._build_action_buttons())
        root.addWidget(self._build_json_panels())

    # ── Status panel ─────────────────────────────────────────────────

    def _build_status_panel(self) -> QGroupBox:
        box = QGroupBox('Estado del sistema')
        grid = QGridLayout(box)
        grid.setSpacing(6)

        def row(lbl_text, value_text='—', value_color=TEXT_SEC):
            lbl = QLabel(lbl_text)
            lbl.setStyleSheet(f'color: {TEXT_SEC};')
            val = QLabel(value_text)
            val.setStyleSheet(f'color: {value_color}; font-weight: bold;')
            return lbl, val

        r = grid.rowCount

        lbl, self._lbl_ros = row('Estado ROS2:', '● activo', ACCENT2)
        grid.addWidget(lbl, 0, 0); grid.addWidget(self._lbl_ros, 0, 1)

        lbl, self._lbl_kuka = row('Estado KUKA:', '○ sin feedback', WARN_CLR)
        grid.addWidget(lbl, 1, 0); grid.addWidget(self._lbl_kuka, 1, 1)

        lbl, self._lbl_mode = row('Modo:', 'Manual')
        grid.addWidget(lbl, 2, 0); grid.addWidget(self._lbl_mode, 2, 1)

        lbl, self._lbl_seq = row('Secuencia (seq):', '0')
        grid.addWidget(lbl, 3, 0); grid.addWidget(self._lbl_seq, 3, 1)

        lbl = QLabel('Topic comando:')
        lbl.setStyleSheet(f'color: {TEXT_SEC};')
        val = QLabel(self._cfg.get('command_topic', '/kuka/axis_command/target_json'))
        val.setStyleSheet(f'color: {ACCENT}; font-size: 11px;')
        grid.addWidget(lbl, 0, 2); grid.addWidget(val, 0, 3)

        lbl = QLabel('Topic feedback:')
        lbl.setStyleSheet(f'color: {TEXT_SEC};')
        val = QLabel(self._cfg.get('feedback_topic', '/kuka/axis_command_loop/feedback_json'))
        val.setStyleSheet(f'color: {ACCENT}; font-size: 11px;')
        grid.addWidget(lbl, 1, 2); grid.addWidget(val, 1, 3)

        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 2)
        return box

    # ── Joint table ──────────────────────────────────────────────────

    def _build_joint_table(self) -> QGroupBox:
        box = QGroupBox('Posiciones articulares  [ Target · Feedback · Error ]')
        grid = QGridLayout(box)
        grid.setSpacing(6)

        headers = ['Joint', 'Target (°)', 'Feedback (°)', 'Error (°)', 'Estado']
        for col, h in enumerate(headers):
            lbl = QLabel(h)
            lbl.setStyleSheet(
                f'color: {ACCENT}; font-weight: bold; font-size: 12px;'
            )
            lbl.setAlignment(Qt.AlignCenter)
            grid.addWidget(lbl, 0, col)

        for row_idx, axis in enumerate(AXES, start=1):
            # Joint name
            name_lbl = QLabel(axis)
            name_lbl.setAlignment(Qt.AlignCenter)
            name_lbl.setStyleSheet('font-weight: bold;')
            grid.addWidget(name_lbl, row_idx, 0)

            # Target value (read-only display — editing via controls below)
            tgt_lbl = QLabel(f'{self._model.get_target(axis):.2f}')
            tgt_lbl.setAlignment(Qt.AlignCenter)
            tgt_lbl.setObjectName(f'tbl_target_{axis}')
            grid.addWidget(tgt_lbl, row_idx, 1)
            self._target_fields[axis] = tgt_lbl  # reuse label for table display

            # Feedback
            fb_lbl = QLabel('N/A')
            fb_lbl.setAlignment(Qt.AlignCenter)
            fb_lbl.setStyleSheet(f'color: {TEXT_SEC};')
            fb_lbl.setObjectName(f'tbl_feedback_{axis}')
            grid.addWidget(fb_lbl, row_idx, 2)
            self._feedback_labels[axis] = fb_lbl

            # Error
            err_lbl = QLabel('N/A')
            err_lbl.setAlignment(Qt.AlignCenter)
            err_lbl.setStyleSheet(f'color: {TEXT_SEC};')
            err_lbl.setObjectName(f'tbl_error_{axis}')
            grid.addWidget(err_lbl, row_idx, 3)
            self._error_labels[axis] = err_lbl

            # Status
            st_lbl = QLabel('OK')
            st_lbl.setAlignment(Qt.AlignCenter)
            st_lbl.setStyleSheet(f'color: {ACCENT2}; font-weight: bold;')
            st_lbl.setObjectName(f'tbl_status_{axis}')
            grid.addWidget(st_lbl, row_idx, 4)
            self._joint_status_labels[axis] = st_lbl

        for col in range(5):
            grid.setColumnStretch(col, 1)
        return box

    # ── Joint controls ───────────────────────────────────────────────

    def _build_joint_controls(self) -> QGroupBox:
        box = QGroupBox('Controles por joint')
        main_layout = QVBoxLayout(box)
        main_layout.setSpacing(6)

        # step deg selector
        step_row = QHBoxLayout()
        step_row.addWidget(QLabel('Paso (°):'))
        self._step_field = QLineEdit(str(self._model.step_deg))
        self._step_field.setMaximumWidth(70)
        self._step_field.setObjectName('field_step_deg')
        self._step_field.editingFinished.connect(self._on_step_changed)
        step_row.addWidget(self._step_field)
        step_row.addStretch()
        main_layout.addLayout(step_row)

        # Per-axis rows
        self._axis_edit_fields: Dict[str, QLineEdit] = {}

        for axis in AXES:
            row = QHBoxLayout()
            row.setSpacing(6)

            lbl = QLabel(axis)
            lbl.setMinimumWidth(30)
            lbl.setStyleSheet('font-weight: bold;')
            row.addWidget(lbl)

            btn_minus = _btn('−', '#30363d', min_width=32)
            btn_minus.setObjectName(f'btn_minus_{axis}')
            btn_minus.setMaximumWidth(40)
            btn_minus.clicked.connect(lambda checked, a=axis: self._step_axis(a, -1))
            row.addWidget(btn_minus)

            edit = QLineEdit(f'{self._model.get_target(axis):.2f}')
            edit.setObjectName(f'edit_{axis}')
            edit.setMaximumWidth(90)
            edit.setAlignment(Qt.AlignCenter)
            edit.editingFinished.connect(lambda a=axis: self._on_axis_edited(a))
            row.addWidget(edit)
            self._axis_edit_fields[axis] = edit

            btn_plus = _btn('+', '#30363d', min_width=32)
            btn_plus.setObjectName(f'btn_plus_{axis}')
            btn_plus.setMaximumWidth(40)
            btn_plus.clicked.connect(lambda checked, a=axis: self._step_axis(a, +1))
            row.addWidget(btn_plus)

            unit_lbl = QLabel('°')
            unit_lbl.setStyleSheet(f'color: {TEXT_SEC};')
            row.addWidget(unit_lbl)

            lo, hi = self._model.get_limits(axis)
            limit_lbl = QLabel(f'[{lo:.0f} … {hi:.0f}]')
            limit_lbl.setStyleSheet(f'color: {TEXT_SEC}; font-size: 11px;')
            row.addWidget(limit_lbl)
            row.addStretch()

            main_layout.addLayout(row)

        return box

    # ── Action buttons ───────────────────────────────────────────────

    def _build_action_buttons(self) -> QWidget:
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setSpacing(10)

        self._btn_home = _btn('🏠  HOME', '#6e40c9', min_width=110)
        self._btn_home.setObjectName('btn_home')
        self._btn_home.clicked.connect(self._on_home)
        layout.addWidget(self._btn_home)

        self._btn_send = _btn('📤  SEND', ACCENT, min_width=110)
        self._btn_send.setObjectName('btn_send')
        self._btn_send.clicked.connect(self._on_send)
        layout.addWidget(self._btn_send)

        self._btn_auto = _btn('🔄  MODO AUTOMÁTICO  OFF', '#444c56', min_width=200)
        self._btn_auto.setObjectName('btn_auto')
        self._btn_auto.setCheckable(True)
        self._btn_auto.clicked.connect(self._on_toggle_auto)
        layout.addWidget(self._btn_auto)

        self._btn_stop = _btn('⏹  STOP AUTO', STOP_CLR, min_width=130)
        self._btn_stop.setObjectName('btn_stop_auto')
        self._btn_stop.clicked.connect(self._on_stop_auto)
        layout.addWidget(self._btn_stop)

        self._btn_reset = _btn('↺  RESET GUI', '#444c56', min_width=120)
        self._btn_reset.setObjectName('btn_reset_gui')
        self._btn_reset.clicked.connect(self._on_reset_gui)
        layout.addWidget(self._btn_reset)

        layout.addStretch()

        # Warning label for out-of-limits
        self._lbl_warning = QLabel('')
        self._lbl_warning.setStyleSheet(
            f'color: {WARN_CLR}; font-weight: bold; font-size: 12px;'
        )
        layout.addWidget(self._lbl_warning)

        return widget

    # ── JSON panels ──────────────────────────────────────────────────

    def _build_json_panels(self) -> QWidget:
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setSpacing(10)

        # Sent JSON
        grp_sent = QGroupBox('Último JSON enviado')
        sent_layout = QVBoxLayout(grp_sent)
        self._txt_sent = QTextEdit()
        self._txt_sent.setReadOnly(True)
        self._txt_sent.setMaximumHeight(100)
        self._txt_sent.setObjectName('txt_sent_json')
        sent_layout.addWidget(self._txt_sent)
        layout.addWidget(grp_sent)

        # Received feedback JSON
        grp_fb = QGroupBox('Último feedback recibido')
        fb_layout = QVBoxLayout(grp_fb)
        self._txt_feedback = QTextEdit()
        self._txt_feedback.setReadOnly(True)
        self._txt_feedback.setMaximumHeight(100)
        self._txt_feedback.setObjectName('txt_feedback_json')
        fb_layout.addWidget(self._txt_feedback)
        layout.addWidget(grp_fb)

        return widget

    # ── Auto-publish timer ───────────────────────────────────────────

    def start_auto_timer(self, hz: float = 5.0) -> None:
        if self._auto_timer is None:
            self._auto_timer = QTimer(self)
            self._auto_timer.timeout.connect(self._publish_auto)
        interval_ms = max(50, int(1000.0 / hz))
        self._auto_timer.start(interval_ms)

    def stop_auto_timer(self) -> None:
        if self._auto_timer is not None:
            self._auto_timer.stop()

    def _publish_auto(self) -> None:
        if not self._all_in_limits_check():
            return
        self._model.set_mode('auto')
        self._do_publish()

    # ── Feedback check timer ─────────────────────────────────────────

    def _start_feedback_check(self) -> None:
        self._feedback_check_timer = QTimer(self)
        self._feedback_check_timer.timeout.connect(self._update_feedback_status)
        self._feedback_check_timer.start(500)  # every 500 ms

    def _update_feedback_status(self) -> None:
        has_fb = self._model.has_recent_feedback(self._feedback_timeout)
        if has_fb:
            self._lbl_kuka.setText('● conectado')
            self._lbl_kuka.setStyleSheet(f'color: {ACCENT2}; font-weight: bold;')
        else:
            self._lbl_kuka.setText('○ sin feedback')
            self._lbl_kuka.setStyleSheet(f'color: {WARN_CLR}; font-weight: bold;')

        # Also refresh table if no feedback
        if not has_fb:
            for axis in AXES:
                self._feedback_labels[axis].setText('N/A')
                self._feedback_labels[axis].setStyleSheet(f'color: {TEXT_SEC};')
                self._error_labels[axis].setText('N/A')
                self._error_labels[axis].setStyleSheet(f'color: {TEXT_SEC};')

    # ── Slot: feedback received (called from ROS2 via bridge signal) ─

    @pyqtSlot(str)
    def on_feedback_received(self, json_str: str) -> None:
        """Update UI from feedback JSON."""
        try:
            data = json.loads(json_str)
        except json.JSONDecodeError:
            return

        self._model.update_feedback(data)
        self._last_feedback_json = json.dumps(data, indent=2)
        self._txt_feedback.setPlainText(self._last_feedback_json)

        # Update table
        for axis in AXES:
            fb = self._model.get_feedback(axis)
            err = self._model.get_error(axis)

            if fb is not None:
                self._feedback_labels[axis].setText(f'{fb:.2f}')
                self._feedback_labels[axis].setStyleSheet(f'color: {TEXT_PRI};')
            if err is not None:
                abs_err = abs(err)
                color = WARN_CLR if abs_err > 1.0 else ACCENT2
                self._error_labels[axis].setText(f'{err:+.2f}')
                self._error_labels[axis].setStyleSheet(f'color: {color};')

        # Update seq display
        seq = data.get('seq', '?')
        self._lbl_seq.setText(str(seq))

    # ── Slot: ROS status ─────────────────────────────────────────────

    @pyqtSlot(bool)
    def on_ros_status(self, active: bool) -> None:
        if active:
            self._lbl_ros.setText('● activo')
            self._lbl_ros.setStyleSheet(f'color: {ACCENT2}; font-weight: bold;')
        else:
            self._lbl_ros.setText('○ inactivo')
            self._lbl_ros.setStyleSheet(f'color: {WARN_CLR}; font-weight: bold;')

    # ── Button callbacks ─────────────────────────────────────────────

    def _on_home(self) -> None:
        self._model.load_home()
        self._refresh_axis_fields()
        self._refresh_table_targets()
        self._check_limits_ui()
        # If auto mode, timer will publish; otherwise send once
        if not self._auto_mode_on:
            if self._all_in_limits_check():
                self._model.set_mode('manual_send')
                self._do_publish()

    def _on_send(self) -> None:
        if not self._all_in_limits_check():
            return
        self._model.set_mode('manual_send')
        self._do_publish()

    def _on_toggle_auto(self) -> None:
        self._auto_mode_on = not self._auto_mode_on
        if self._auto_mode_on:
            hz = self._cfg.get('auto_publish_hz', 5.0)
            self.start_auto_timer(hz)
            self._btn_auto.setText('🔄  MODO AUTOMÁTICO  ON')
            self._btn_auto.setStyleSheet(
                f'QPushButton {{ background-color: {AUTO_ON_CLR}; color: #ffffff;'
                f' font-weight: bold; border-radius: 6px; }}'
            )
            self._lbl_mode.setText('Automático')
            self._lbl_mode.setStyleSheet(f'color: {AUTO_ON_CLR}; font-weight: bold;')
        else:
            self.stop_auto_timer()
            self._btn_auto.setText('🔄  MODO AUTOMÁTICO  OFF')
            self._btn_auto.setStyleSheet(
                f'QPushButton {{ background-color: #444c56; color: {TEXT_PRI};'
                f' font-weight: bold; border-radius: 6px; }}'
            )
            self._lbl_mode.setText('Manual')
            self._lbl_mode.setStyleSheet(f'color: {TEXT_PRI};')

    def _on_stop_auto(self) -> None:
        if self._auto_mode_on:
            self._auto_mode_on = False
            self.stop_auto_timer()
            self._btn_auto.setText('🔄  MODO AUTOMÁTICO  OFF')
            self._btn_auto.setStyleSheet(
                f'QPushButton {{ background-color: #444c56; color: {TEXT_PRI};'
                f' font-weight: bold; border-radius: 6px; }}'
            )
            self._lbl_mode.setText('Manual')
            self._lbl_mode.setStyleSheet(f'color: {TEXT_PRI};')

    def _on_reset_gui(self) -> None:
        self._model.load_home()
        self._refresh_axis_fields()
        self._refresh_table_targets()
        self._check_limits_ui()
        # Only publish if auto is on
        # (the timer will pick it up automatically)

    # ── Axis stepping & editing ──────────────────────────────────────

    def _step_axis(self, axis: str, direction: int) -> None:
        new_val = self._model.step_target(axis, direction)
        edit = self._axis_edit_fields.get(axis)
        if edit:
            edit.setText(f'{new_val:.2f}')
        # Update table display
        tgt_lbl = self._target_fields.get(axis)
        if tgt_lbl:
            tgt_lbl.setText(f'{new_val:.2f}')
        self._check_limits_ui()

    def _on_axis_edited(self, axis: str) -> None:
        edit = self._axis_edit_fields.get(axis)
        if edit is None:
            return
        text = edit.text().strip().replace(',', '.')
        try:
            value = float(text)
        except ValueError:
            # Restore previous value
            edit.setText(f'{self._model.get_target(axis):.2f}')
            return
        self._model.set_target(axis, value)
        # Update table display
        tgt_lbl = self._target_fields.get(axis)
        if tgt_lbl:
            tgt_lbl.setText(f'{value:.2f}')
        self._check_limits_ui()

    def _on_step_changed(self) -> None:
        text = self._step_field.text().strip().replace(',', '.')
        try:
            step = float(text)
            if step > 0:
                self._model.step_deg = step
        except ValueError:
            self._step_field.setText(str(self._model.step_deg))

    # ── Limit checking & UI updates ──────────────────────────────────

    def _check_limits_ui(self) -> None:
        all_ok = True
        for axis in AXES:
            in_lim = self._model.is_in_limits(axis)
            edit = self._axis_edit_fields.get(axis)
            st_lbl = self._joint_status_labels.get(axis)
            if not in_lim:
                all_ok = False
                if edit:
                    edit.setStyleSheet(
                        f'background-color: #3d1a1a; border: 1px solid {WARN_CLR};'
                        f' color: {WARN_CLR}; border-radius: 4px; padding: 3px 6px;'
                    )
                if st_lbl:
                    lo, hi = self._model.get_limits(axis)
                    st_lbl.setText('!! OOL')
                    st_lbl.setStyleSheet(f'color: {WARN_CLR}; font-weight: bold;')
            else:
                if edit:
                    edit.setStyleSheet('')
                if st_lbl:
                    st_lbl.setText('OK')
                    st_lbl.setStyleSheet(f'color: {ACCENT2}; font-weight: bold;')

        if all_ok:
            self._lbl_warning.setText('')
            self._btn_send.setEnabled(True)
        else:
            self._lbl_warning.setText(
                '⚠  Valores fuera de límites — SEND bloqueado'
            )
            self._btn_send.setEnabled(False)

    def _all_in_limits_check(self) -> bool:
        return self._model.all_targets_in_limits()

    # ── Publish helper ───────────────────────────────────────────────

    def _do_publish(self) -> None:
        json_str = self._model.build_target_json()
        self._bridge.publish_command(json_str)
        self._last_sent_json = json.dumps(json.loads(json_str), indent=2)
        self._txt_sent.setPlainText(self._last_sent_json)
        self._lbl_seq.setText(str(self._model.get_seq()))

    # ── Field refresh helpers ────────────────────────────────────────

    def _refresh_axis_fields(self) -> None:
        for axis in AXES:
            val = self._model.get_target(axis)
            edit = self._axis_edit_fields.get(axis)
            if edit:
                edit.setText(f'{val:.2f}')

    def _refresh_table_targets(self) -> None:
        for axis in AXES:
            val = self._model.get_target(axis)
            tgt_lbl = self._target_fields.get(axis)
            if tgt_lbl:
                tgt_lbl.setText(f'{val:.2f}')

    def cleanup(self) -> None:
        """Stop all timers cleanly."""
        if self._auto_timer:
            self._auto_timer.stop()
        if self._feedback_check_timer:
            self._feedback_check_timer.stop()


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class KukaGuiMainWindow(QMainWindow):
    """
    Main application window.

    Manages a QStackedWidget with WelcomeScreen (index 0) and
    ControlScreen (index 1).
    """

    def __init__(
        self,
        model: JointCommandModel,
        bridge,
        config: dict,
    ):
        super().__init__()
        self._model = model
        self._bridge = bridge
        self._config = config

        self.setWindowTitle('KUKA Joint Control GUI — kuka_gui_control')
        self.setMinimumSize(900, 720)
        self.resize(1100, 800)
        self.setStyleSheet(BASE_STYLE)

        # Stack
        self._stack = QStackedWidget()
        self.setCentralWidget(self._stack)

        # Screen 1
        self._welcome = WelcomeScreen(on_start=self._go_to_control)
        self._stack.addWidget(self._welcome)

        # Screen 2 (created lazily on START)
        self._control: Optional[ControlScreen] = None

        # Connect bridge signals
        bridge.ros_status_changed.connect(self._on_ros_status)

        self._stack.setCurrentIndex(0)

    def _go_to_control(self) -> None:
        """Switch from welcome to control screen."""
        if self._control is None:
            self._control = ControlScreen(
                model=self._model,
                bridge=self._bridge,
                config=self._config,
            )
            self._stack.addWidget(self._control)
            # Wire bridge feedback signal to control screen slot
            self._bridge.feedback_received.connect(self._control.on_feedback_received)

        self._stack.setCurrentWidget(self._control)

    @pyqtSlot(bool)
    def _on_ros_status(self, active: bool) -> None:
        if self._control:
            self._control.on_ros_status(active)

    def closeEvent(self, event) -> None:
        """Clean shutdown on window close."""
        if self._control:
            self._control.cleanup()
        self._bridge.stop()
        event.accept()
