"""
gui_axis_move_window.py — PyQt5 main window for KUKA XmlAxisMove joint control.

Two-screen flow:
  Screen 1 (Welcome):  Title + description + START button.
  Screen 2 (Control):  Status panel, joint table, joint controls, action buttons,
                        JSON/XML viewers.

All ROS2 communication is delegated to RosAxisMoveBridge via Qt signals.
This file has no direct rclpy dependency.

This module does NOT modify gui_window.py.
"""

import json
import time
from typing import Dict, Optional

try:
    from PyQt5.QtCore import Qt, QTimer, pyqtSlot
    from PyQt5.QtGui import (
        QColor, QFont, QIcon, QPainter, QPalette, QPen, QPixmap,
    )
    from PyQt5.QtWidgets import (
        QApplication, QCheckBox, QFrame, QGridLayout, QGroupBox,
        QHBoxLayout, QLabel, QLineEdit, QMainWindow,
        QMessageBox, QPushButton, QSizePolicy, QScrollArea,
        QStackedWidget, QTextEdit, QVBoxLayout, QWidget,
        QTabWidget, QRadioButton, QButtonGroup,
    )
except ImportError as e:
    raise ImportError(
        'PyQt5 is required. Install with:  sudo apt install python3-pyqt5'
    ) from e

from kuka_gui_control.joint_command_model import JointCommandModel, AXES, CARTESIAN_AXES
from kuka_gui_control.trajectory_panel import TrajectorySequencePanel
from kuka_gui_control.trajectory_batch_panel import TrajectoryBatchPanel

# ---------------------------------------------------------------------------
# Style constants
# ---------------------------------------------------------------------------

DARK_BG = '#0d1117'
PANEL_BG = '#161b22'
BORDER_CLR = '#30363d'
ACCENT = '#58a6ff'
ACCENT2 = '#3fb950'
WARN_CLR = '#f78166'
ERROR_CLR = '#da3633'
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
QCheckBox {{
    color: {TEXT_PRI};
    spacing: 6px;
}}
QCheckBox::indicator {{
    width: 18px;
    height: 18px;
}}
"""

BTN_SEND = f"""
QPushButton {{
    background-color: {ACCENT2};
    color: #fff;
    border: none;
    border-radius: 6px;
    padding: 8px 20px;
    font-weight: bold;
}}
QPushButton:hover {{ background-color: #2ea043; }}
QPushButton:disabled {{ background-color: {BORDER_CLR}; color: {TEXT_SEC}; }}
"""

BTN_HOME = f"""
QPushButton {{
    background-color: {ACCENT};
    color: #fff;
    border: none;
    border-radius: 6px;
    padding: 8px 20px;
    font-weight: bold;
}}
QPushButton:hover {{ background-color: #4090e0; }}
"""

BTN_AUTO = f"""
QPushButton {{
    background-color: {AUTO_ON_CLR};
    color: #fff;
    border: none;
    border-radius: 6px;
    padding: 8px 20px;
    font-weight: bold;
}}
QPushButton:hover {{ background-color: #2d7cf0; }}
"""

BTN_STOP = f"""
QPushButton {{
    background-color: {STOP_CLR};
    color: #fff;
    border: none;
    border-radius: 6px;
    padding: 8px 20px;
    font-weight: bold;
}}
QPushButton:hover {{ background-color: #b62324; }}
"""

BTN_RESET = f"""
QPushButton {{
    background-color: {BORDER_CLR};
    color: {TEXT_PRI};
    border: 1px solid {TEXT_SEC};
    border-radius: 6px;
    padding: 8px 20px;
    font-weight: bold;
}}
QPushButton:hover {{ background-color: #3d444d; }}
"""

BTN_START = f"""
QPushButton {{
    background-color: {ACCENT2};
    color: #fff;
    border: none;
    border-radius: 10px;
    padding: 14px 48px;
    font-size: 18px;
    font-weight: bold;
}}
QPushButton:hover {{ background-color: #2ea043; }}
"""

BTN_PM = f"""
QPushButton {{
    background-color: {PANEL_BG};
    color: {TEXT_PRI};
    border: 1px solid {BORDER_CLR};
    border-radius: 4px;
    padding: 4px 10px;
    font-weight: bold;
    font-size: 14px;
    min-width: 28px;
}}
QPushButton:hover {{ border-color: {ACCENT}; }}
"""

BTN_GRIPPER = f"""
QPushButton {{
    background-color: {PANEL_BG};
    color: {TEXT_PRI};
    border: 1px solid {BORDER_CLR};
    border-radius: 6px;
    padding: 8px 18px;
    font-weight: bold;
}}
QPushButton:hover {{ border-color: {ACCENT}; color: {ACCENT}; }}
QPushButton:disabled {{ background-color: {BORDER_CLR}; color: {TEXT_SEC}; }}
"""


# ---------------------------------------------------------------------------
# Gripper glyphs
# ---------------------------------------------------------------------------

def _make_gripper_icon(is_open: bool, color: str) -> QIcon:
    """
    Dibujar un icono sencillo de garra abierta/cerrada con QPainter.

    Se pinta en memoria con PyQt5, que ya es el framework de la GUI: no
    hay ficheros de recursos, ni paquetes, ni dependencias nuevas.
    """
    size = 22
    pixmap = QPixmap(size, size)
    pixmap.fill(Qt.transparent)

    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.Antialiasing, True)
    pen = QPen(QColor(color))
    pen.setWidth(2)
    pen.setCapStyle(Qt.RoundCap)
    painter.setPen(pen)

    # Cuerpo y vástago
    painter.drawLine(6, 3, 16, 3)
    painter.drawLine(11, 3, 11, 8)

    # Dedos: separados si está abierta, juntos si está cerrada
    spread = 7 if is_open else 2
    painter.drawLine(11 - spread, 8, 11, 8)
    painter.drawLine(11 + spread, 8, 11, 8)
    painter.drawLine(11 - spread, 8, 11 - spread, 18)
    painter.drawLine(11 + spread, 8, 11 + spread, 18)

    painter.end()
    return QIcon(pixmap)


# ---------------------------------------------------------------------------
# Main Window
# ---------------------------------------------------------------------------

class AxisMoveGuiWindow(QMainWindow):
    """
    PyQt5 main window for the KUKA XmlAxisMove joint control system.

    Two screens:
      - Welcome (START button)
      - Control (status, table, controls, buttons, logs)
    """

    def __init__(
        self,
        model: JointCommandModel,
        bridge,  # RosAxisMoveBridge
        config: dict,
        parent=None,
    ):
        super().__init__(parent)
        self._model = model
        self._bridge = bridge
        self._config = config

        self._auto_timer: Optional[QTimer] = None
        self._auto_running = False
        self._feedback_timer: Optional[QTimer] = None
        self._send_hold_timer: Optional[QTimer] = None
        self._send_hold_count = 0
        self._first_send_confirmed = False
        self._enable_move_confirmed = False
        self._synced_to_robot = False

        # Config values
        self._auto_hz = config.get('auto_publish_hz', 2.0)
        self._feedback_timeout = config.get('feedback_timeout_sec', 2.0)
        self._allow_auto_mode = config.get('allow_auto_mode', True)
        self._allow_auto_motion = config.get('allow_auto_motion', False)
        self._require_confirm = config.get('require_confirmation_for_first_send', True)
        self._show_raw_json = config.get('show_raw_json', True)
        self._show_raw_xml = config.get('show_raw_xml', True)
        self._max_delta = config.get('max_delta_deg', {})

        self.setWindowTitle('KUKA Joint Control GUI — AxisMove')
        self.setMinimumSize(900, 700)
        self.setStyleSheet(BASE_STYLE)

        # ── Build UI ─────────────────────────────────────────────────
        self._stack = QStackedWidget()
        self.setCentralWidget(self._stack)

        self._build_welcome_screen()
        self._build_control_screen()

        self._stack.setCurrentIndex(0)

        # ── Connect bridge signals ───────────────────────────────────
        self._bridge.feedback_received.connect(self._on_feedback)
        self._bridge.raw_command_xml_received.connect(self._on_raw_command_xml)
        self._bridge.raw_robot_xml_received.connect(self._on_raw_robot_xml)
        self._bridge.ros_status_changed.connect(self._on_ros_status)

        # ── Feedback timeout timer ───────────────────────────────────
        self._feedback_timer = QTimer(self)
        self._feedback_timer.timeout.connect(self._check_feedback_timeout)
        self._feedback_timer.start(500)

    # ===================================================================
    # Screen 1: Welcome
    # ===================================================================

    def _build_welcome_screen(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setAlignment(Qt.AlignCenter)

        title = QLabel('KUKA Joint Control GUI')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f'font-size: 32px; font-weight: bold; color: {ACCENT};')
        layout.addWidget(title)

        layout.addSpacing(10)

        subtitle = QLabel('Sistema de envío de posiciones articulares\nhacia KUKA mediante ROS2 y EthernetKRL.')
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setStyleSheet(f'font-size: 15px; color: {TEXT_SEC};')
        layout.addWidget(subtitle)

        layout.addSpacing(10)

        desc = QLabel(
            'Modo: XmlAxisMove\n'
            'Protocolo: TCP/XML via EthernetKRL\n'
            'Puerto: 59153'
        )
        desc.setAlignment(Qt.AlignCenter)
        desc.setStyleSheet(f'font-size: 13px; color: {TEXT_SEC};')
        layout.addWidget(desc)

        layout.addSpacing(30)

        btn_start = QPushButton('START')
        btn_start.setStyleSheet(BTN_START)
        btn_start.setCursor(Qt.PointingHandCursor)
        btn_start.clicked.connect(self._on_start)
        layout.addWidget(btn_start, alignment=Qt.AlignCenter)

        self._stack.addWidget(page)

    # ===================================================================
    # Screen 2: Control
    # ===================================================================

    def _build_control_screen(self):
        page = QWidget()
        main_layout = QVBoxLayout(page)
        main_layout.setContentsMargins(12, 8, 12, 8)
        main_layout.setSpacing(8)

        # ── Status bar ───────────────────────────────────────────────
        status_group = QGroupBox('Estado del Sistema')
        status_layout = QGridLayout(status_group)
        status_layout.setSpacing(6)

        self._lbl_ros = self._make_status_label('ROS2:', 'Inactivo', TEXT_SEC)
        status_layout.addWidget(self._lbl_ros[0], 0, 0)
        status_layout.addWidget(self._lbl_ros[1], 0, 1)

        self._lbl_kuka = self._make_status_label('KUKA:', 'Sin feedback', TEXT_SEC)
        status_layout.addWidget(self._lbl_kuka[0], 0, 2)
        status_layout.addWidget(self._lbl_kuka[1], 0, 3)

        self._lbl_bridge = self._make_status_label('Bridge:', 'Esperando KUKA', TEXT_SEC)
        status_layout.addWidget(self._lbl_bridge[0], 1, 0)
        status_layout.addWidget(self._lbl_bridge[1], 1, 1)

        self._lbl_safety = self._make_status_label('Seguridad:', 'safe_mode activo', WARN_CLR)
        status_layout.addWidget(self._lbl_safety[0], 1, 2)
        status_layout.addWidget(self._lbl_safety[1], 1, 3)

        main_layout.addWidget(status_group)

        # ── Safety banner ────────────────────────────────────────────
        banner = QLabel('⚠ safe_mode del bridge puede bloquear EnableMove')
        banner.setAlignment(Qt.AlignCenter)
        banner.setStyleSheet(
            f'background-color: #2d1b00; color: {WARN_CLR}; '
            f'padding: 4px; border-radius: 4px; font-size: 12px;'
        )
        main_layout.addWidget(banner)


        # Mode Selector removed in favor of QTabWidget tabs

        # ── Tabs ─────────────────────────────────────────────────────
        self._tabs = QTabWidget()
        
        self._tab_axis = QWidget()
        layout_axis = QVBoxLayout(self._tab_axis)
        
        self._tab_cart = QWidget()
        layout_cart = QVBoxLayout(self._tab_cart)
        
        self._tabs.addTab(self._tab_axis, 'Control por Ejes')
        self._tabs.addTab(self._tab_cart, 'Control Cartesiano')
        
        self._tabs.currentChanged.connect(self._on_tab_changed)
        
        main_layout.addWidget(self._tabs)

        # ── Joint table ──────────────────────────────────────────────
        table_group = QGroupBox('Posiciones Articulares')
        table_layout = QGridLayout(table_group)
        table_layout.setSpacing(4)

        # Header
        for col, hdr in enumerate(['Joint', 'Target (deg)', 'Feedback (deg)', 'Error (deg)']):
            lbl = QLabel(hdr)
            lbl.setStyleSheet(f'font-weight: bold; color: {ACCENT}; font-size: 12px;')
            lbl.setAlignment(Qt.AlignCenter)
            table_layout.addWidget(lbl, 0, col)

        self._table_labels: Dict[str, Dict[str, QLabel]] = {}
        for row, axis in enumerate(AXES, start=1):
            self._table_labels[axis] = {}

            lbl_name = QLabel(axis)
            lbl_name.setAlignment(Qt.AlignCenter)
            lbl_name.setStyleSheet('font-weight: bold;')
            table_layout.addWidget(lbl_name, row, 0)

            lbl_target = QLabel('0.00')
            lbl_target.setAlignment(Qt.AlignCenter)
            table_layout.addWidget(lbl_target, row, 1)
            self._table_labels[axis]['target'] = lbl_target

            lbl_fb = QLabel('N/A')
            lbl_fb.setAlignment(Qt.AlignCenter)
            lbl_fb.setStyleSheet(f'color: {TEXT_SEC};')
            table_layout.addWidget(lbl_fb, row, 2)
            self._table_labels[axis]['feedback'] = lbl_fb

            lbl_err = QLabel('N/A')
            lbl_err.setAlignment(Qt.AlignCenter)
            lbl_err.setStyleSheet(f'color: {TEXT_SEC};')
            table_layout.addWidget(lbl_err, row, 3)
            self._table_labels[axis]['error'] = lbl_err

        layout_axis.addWidget(table_group)

        # ── Joint controls ───────────────────────────────────────────
        controls_group = QGroupBox('Control de Articulaciones')
        controls_layout = QGridLayout(controls_group)
        controls_layout.setSpacing(4)

        self._joint_inputs: Dict[str, QLineEdit] = {}
        self._joint_minus: Dict[str, QPushButton] = {}
        self._joint_plus: Dict[str, QPushButton] = {}

        for row, axis in enumerate(AXES):
            lbl = QLabel(axis)
            lbl.setStyleSheet('font-weight: bold; min-width: 30px;')
            controls_layout.addWidget(lbl, row, 0)

            btn_minus = QPushButton('−')
            btn_minus.setStyleSheet(BTN_PM)
            btn_minus.setCursor(Qt.PointingHandCursor)
            btn_minus.clicked.connect(lambda checked, a=axis: self._on_step(a, -1))
            controls_layout.addWidget(btn_minus, row, 1)
            self._joint_minus[axis] = btn_minus

            inp = QLineEdit(f'{self._model.get_target(axis):.2f}')
            inp.setAlignment(Qt.AlignCenter)
            inp.setFixedWidth(100)
            inp.editingFinished.connect(lambda a=axis: self._on_input_changed(a))
            controls_layout.addWidget(inp, row, 2)
            self._joint_inputs[axis] = inp

            btn_plus = QPushButton('+')
            btn_plus.setStyleSheet(BTN_PM)
            btn_plus.setCursor(Qt.PointingHandCursor)
            btn_plus.clicked.connect(lambda checked, a=axis: self._on_step(a, +1))
            controls_layout.addWidget(btn_plus, row, 3)
            self._joint_plus[axis] = btn_plus

            lbl_deg = QLabel('deg')
            lbl_deg.setStyleSheet(f'color: {TEXT_SEC};')
            controls_layout.addWidget(lbl_deg, row, 4)

        layout_axis.addWidget(controls_group)

        # ── Cartesian table ──────────────────────────────────────────────
        cart_table_group = QGroupBox('Posiciones Cartesianas')
        cart_table_layout = QGridLayout(cart_table_group)
        cart_table_layout.setSpacing(4)

        for col, hdr in enumerate(['Cartesiano', 'Target', 'Feedback', 'Error']):
            lbl = QLabel(hdr)
            lbl.setStyleSheet(f'font-weight: bold; color: {ACCENT}; font-size: 12px;')
            lbl.setAlignment(Qt.AlignCenter)
            cart_table_layout.addWidget(lbl, 0, col)

        for row, axis in enumerate(CARTESIAN_AXES, start=1):
            self._table_labels[axis] = {}
            lbl_name = QLabel(axis)
            lbl_name.setAlignment(Qt.AlignCenter)
            lbl_name.setStyleSheet('font-weight: bold;')
            cart_table_layout.addWidget(lbl_name, row, 0)

            lbl_target = QLabel('0.00')
            lbl_target.setAlignment(Qt.AlignCenter)
            cart_table_layout.addWidget(lbl_target, row, 1)
            self._table_labels[axis]['target'] = lbl_target

            lbl_fb = QLabel('N/A')
            lbl_fb.setAlignment(Qt.AlignCenter)
            lbl_fb.setStyleSheet(f'color: {TEXT_SEC};')
            cart_table_layout.addWidget(lbl_fb, row, 2)
            self._table_labels[axis]['feedback'] = lbl_fb

            lbl_err = QLabel('N/A')
            lbl_err.setAlignment(Qt.AlignCenter)
            lbl_err.setStyleSheet(f'color: {TEXT_SEC};')
            cart_table_layout.addWidget(lbl_err, row, 3)
            self._table_labels[axis]['error'] = lbl_err

        layout_cart.addWidget(cart_table_group)

        # ── Cartesian controls ───────────────────────────────────────────
        cart_controls_group = QGroupBox('Control Cartesiano')
        cart_controls_layout = QGridLayout(cart_controls_group)
        cart_controls_layout.setSpacing(4)

        for row, axis in enumerate(CARTESIAN_AXES):
            lbl = QLabel(axis)
            lbl.setStyleSheet('font-weight: bold; min-width: 30px;')
            cart_controls_layout.addWidget(lbl, row, 0)

            btn_minus = QPushButton('−')
            btn_minus.setStyleSheet(BTN_PM)
            btn_minus.setCursor(Qt.PointingHandCursor)
            btn_minus.clicked.connect(lambda checked, a=axis: self._on_step(a, -1))
            cart_controls_layout.addWidget(btn_minus, row, 1)
            self._joint_minus[axis] = btn_minus

            inp = QLineEdit(f'{self._model.get_target(axis):.2f}')
            inp.setAlignment(Qt.AlignCenter)
            inp.setFixedWidth(100)
            inp.editingFinished.connect(lambda a=axis: self._on_input_changed(a))
            cart_controls_layout.addWidget(inp, row, 2)
            self._joint_inputs[axis] = inp

            btn_plus = QPushButton('+')
            btn_plus.setStyleSheet(BTN_PM)
            btn_plus.setCursor(Qt.PointingHandCursor)
            btn_plus.clicked.connect(lambda checked, a=axis: self._on_step(a, +1))
            cart_controls_layout.addWidget(btn_plus, row, 3)
            self._joint_plus[axis] = btn_plus

            lbl_deg = QLabel('mm' if axis in ['X', 'Y', 'Z'] else 'deg')
            lbl_deg.setStyleSheet(f'color: {TEXT_SEC};')
            cart_controls_layout.addWidget(lbl_deg, row, 4)

        layout_cart.addWidget(cart_controls_group)


        # ── Action buttons ───────────────────────────────────────────
        btn_layout = QHBoxLayout()

        self._btn_home = QPushButton('HOME')
        self._btn_home.setStyleSheet(BTN_HOME)
        self._btn_home.setCursor(Qt.PointingHandCursor)
        self._btn_home.clicked.connect(self._on_home)
        btn_layout.addWidget(self._btn_home)

        self._btn_send = QPushButton('SEND')
        self._btn_send.setStyleSheet(BTN_SEND)
        self._btn_send.setCursor(Qt.PointingHandCursor)
        self._btn_send.clicked.connect(self._on_send)
        btn_layout.addWidget(self._btn_send)

        self._btn_auto = QPushButton('AUTO')
        self._btn_auto.setStyleSheet(BTN_AUTO)
        self._btn_auto.setCursor(Qt.PointingHandCursor)
        self._btn_auto.clicked.connect(self._on_auto)
        btn_layout.addWidget(self._btn_auto)

        self._btn_stop_auto = QPushButton('STOP AUTO')
        self._btn_stop_auto.setStyleSheet(BTN_STOP)
        self._btn_stop_auto.setCursor(Qt.PointingHandCursor)
        self._btn_stop_auto.clicked.connect(self._on_stop_auto)
        self._btn_stop_auto.setEnabled(False)
        btn_layout.addWidget(self._btn_stop_auto)

        self._btn_reset = QPushButton('RESET GUI')
        self._btn_reset.setStyleSheet(BTN_RESET)
        self._btn_reset.setCursor(Qt.PointingHandCursor)
        self._btn_reset.clicked.connect(self._on_reset)
        btn_layout.addWidget(self._btn_reset)

        main_layout.addLayout(btn_layout)

        # ── Garra (acción puntual, no mueve el robot) ─────────────────
        gripper_layout = QHBoxLayout()

        lbl_gripper = QLabel('Garra:')
        lbl_gripper.setStyleSheet(f'color: {TEXT_SEC}; font-weight: bold;')
        gripper_layout.addWidget(lbl_gripper)

        self._btn_gripper_open = QPushButton('  Abrir garra')
        self._btn_gripper_open.setIcon(_make_gripper_icon(True, TEXT_PRI))
        self._btn_gripper_open.setStyleSheet(BTN_GRIPPER)
        self._btn_gripper_open.setCursor(Qt.PointingHandCursor)
        self._btn_gripper_open.setToolTip(
            'Envía GripperCommand=0 (abrir) con EnableMove=false.\n'
            'No modifica los targets articulares ni cartesianos.'
        )
        self._btn_gripper_open.clicked.connect(self._on_gripper_open)
        gripper_layout.addWidget(self._btn_gripper_open)

        self._btn_gripper_close = QPushButton('  Cerrar garra')
        self._btn_gripper_close.setIcon(_make_gripper_icon(False, TEXT_PRI))
        self._btn_gripper_close.setStyleSheet(BTN_GRIPPER)
        self._btn_gripper_close.setCursor(Qt.PointingHandCursor)
        self._btn_gripper_close.setToolTip(
            'Envía GripperCommand=1 (cerrar) con EnableMove=false.\n'
            'No modifica los targets articulares ni cartesianos.'
        )
        self._btn_gripper_close.clicked.connect(self._on_gripper_close)
        gripper_layout.addWidget(self._btn_gripper_close)

        gripper_layout.addStretch(1)
        main_layout.addLayout(gripper_layout)

        # ── Enable Move toggle ───────────────────────────────────────
        enable_layout = QHBoxLayout()
        self._chk_enable_move = QCheckBox('ENABLE MOVE')
        self._chk_enable_move.setChecked(False)
        self._chk_enable_move.setStyleSheet(
            f'QCheckBox {{ color: {WARN_CLR}; font-weight: bold; font-size: 13px; }}'
        )
        self._chk_enable_move.stateChanged.connect(self._on_enable_move_changed)
        enable_layout.addWidget(self._chk_enable_move)

        self._lbl_enable_status = QLabel('enable_move = false')
        self._lbl_enable_status.setStyleSheet(f'color: {TEXT_SEC}; font-size: 12px;')
        enable_layout.addWidget(self._lbl_enable_status)
        enable_layout.addStretch()

        main_layout.addLayout(enable_layout)

        # ── Info panels ──────────────────────────────────────────────
        if self._show_raw_json or self._show_raw_xml:
            info_group = QGroupBox('Comunicación')
            info_layout = QGridLayout(info_group)

            col = 0
            if self._show_raw_json:
                # Last feedback JSON
                info_layout.addWidget(QLabel('Último feedback JSON:'), 0, col)
                self._txt_feedback = QTextEdit()
                self._txt_feedback.setReadOnly(True)
                self._txt_feedback.setMaximumHeight(90)
                info_layout.addWidget(self._txt_feedback, 1, col)

                col += 1
                # Last command JSON
                info_layout.addWidget(QLabel('Último comando JSON:'), 0, col)
                self._txt_command = QTextEdit()
                self._txt_command.setReadOnly(True)
                self._txt_command.setMaximumHeight(90)
                info_layout.addWidget(self._txt_command, 1, col)
                col += 1

            if self._show_raw_xml:
                # Last command XML
                info_layout.addWidget(QLabel('Último XML enviado:'), 0, col)
                self._txt_cmd_xml = QTextEdit()
                self._txt_cmd_xml.setReadOnly(True)
                self._txt_cmd_xml.setMaximumHeight(90)
                info_layout.addWidget(self._txt_cmd_xml, 1, col)
                col += 1

            main_layout.addWidget(info_group)

        # ── Secuencias de trayectorias (capa AÑADIDA) ────────────────
        # Widget compartido con la GUI dual. Reutiliza el mismo modelo, el
        # mismo RosAxisMoveBridge (y por tanto el mismo nodo ROS2) y las
        # mismas funciones de envío que ya usan SEND y los botones de garra.
        self._trajectory_panel = TrajectorySequencePanel(
            model=self._model,
            kuka_bridge=self._bridge,
            config=self._config,
            joint_send_fn=self._validate_and_send,
            gripper_send_fn=self._send_gripper_command,
        )
        main_layout.addWidget(self._trajectory_panel)

        # ── Modo LOTE (capa AÑADIDA, ruta paralela) ─────────────────
        # Panel independiente, debajo del anterior. El panel base y su
        # botón ENVIAR TRAYECTORIA no se han tocado; este usa el pipeline
        # de lotes y sólo funciona con los archivos _better cargados en el
        # controlador. Reutiliza el MISMO RosAxisMoveBridge y la MISMA
        # función de garra.
        self._trajectory_batch_panel = TrajectoryBatchPanel(
            model=self._model,
            kuka_bridge=self._bridge,
            config=self._config,
            gripper_send_fn=self._send_gripper_command,
        )
        main_layout.addWidget(self._trajectory_batch_panel)

        self._stack.addWidget(page)
        self._refresh_table()
        self._refresh_inputs()

    # ===================================================================
    # Helpers
    # ===================================================================

    def _make_status_label(self, name, initial, color):
        lbl_name = QLabel(name)
        lbl_name.setStyleSheet(f'font-weight: bold; color: {TEXT_SEC};')
        lbl_val = QLabel(initial)
        lbl_val.setStyleSheet(f'color: {color}; font-weight: bold;')
        return (lbl_name, lbl_val)


    def _on_tab_changed(self, index: int):
        if index == 0:
            self._model.set_target_mode('AxisTarget')
        else:
            self._model.set_target_mode('CartesianTarget')
        self._refresh_inputs()
        
    def _refresh_table(self):
        """Refresh all table labels from the model."""
        for axis in AXES + CARTESIAN_AXES:
            target = self._model.get_target(axis)
            feedback = self._model.get_feedback(axis)
            error = self._model.get_error(axis)

            self._table_labels[axis]['target'].setText(f'{target:.2f}')

            if feedback is not None:
                self._table_labels[axis]['feedback'].setText(f'{feedback:.2f}')
                self._table_labels[axis]['feedback'].setStyleSheet(f'color: {TEXT_PRI};')
            else:
                self._table_labels[axis]['feedback'].setText('N/A')
                self._table_labels[axis]['feedback'].setStyleSheet(f'color: {TEXT_SEC};')

            if error is not None:
                self._table_labels[axis]['error'].setText(f'{error:.2f}')
                clr = WARN_CLR if abs(error) > 1.0 else TEXT_PRI
                self._table_labels[axis]['error'].setStyleSheet(f'color: {clr};')
            else:
                self._table_labels[axis]['error'].setText('N/A')
                self._table_labels[axis]['error'].setStyleSheet(f'color: {TEXT_SEC};')

    def _refresh_inputs(self):
        """Refresh all input fields from the model and check limits."""
        all_ok = True
        for axis in AXES + CARTESIAN_AXES:
            val = self._model.get_target(axis)
            inp = self._joint_inputs[axis]
            inp.setText(f'{val:.2f}')

            if self._model.is_in_limits(axis):
                inp.setStyleSheet(
                    f'background-color: {PANEL_BG}; border: 1px solid {BORDER_CLR}; '
                    f'border-radius: 4px; color: {TEXT_PRI}; padding: 3px 6px;'
                )
            else:
                all_ok = False
                inp.setStyleSheet(
                    f'background-color: #3d1111; border: 1px solid {ERROR_CLR}; '
                    f'border-radius: 4px; color: {ERROR_CLR}; padding: 3px 6px;'
                )

        self._btn_send.setEnabled(all_ok)

    def _validate_and_send(self):
        """Build JSON and publish if limits are OK."""
        if not self._model.all_targets_in_limits():
            return

        json_str = self._model.build_target_json()
        self._bridge.publish_command(json_str)

        # Update command display
        if self._show_raw_json and hasattr(self, '_txt_command'):
            try:
                pretty = json.dumps(json.loads(json_str), indent=2)
                self._txt_command.setPlainText(pretty)
            except Exception:
                self._txt_command.setPlainText(json_str)

    # ===================================================================
    # Button handlers
    # ===================================================================

    def _on_start(self):
        """Switch to control screen (no command published)."""
        self._stack.setCurrentIndex(1)

    def _on_home(self):
        """Load home position values (publish only if auto is running)."""
        self._model.load_home()
        self._refresh_inputs()
        self._refresh_table()
        if self._auto_running:
            pass  # Auto timer will publish on next tick

    def _on_send(self):
        """Manual send: publish and hold the command for several cycles."""
        if not self._model.all_targets_in_limits():
            return

        if self._require_confirm and not self._first_send_confirmed:
            reply = QMessageBox.question(
                self,
                'Confirmar primer envío',
                'Este es el primer SEND de esta sesión.\n'
                '¿Confirmas que deseas enviar el comando al bridge?\n\n'
                'El movimiento real depende de safe_mode y allow_motion_commands.',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                return
            self._first_send_confirmed = True

        self._model.set_node_mode('manual_send')
        self._validate_and_send()
        self._refresh_table()

        # Hold: keep re-sending for ~3 seconds so the bridge doesn't
        # mark the command as stale before the KUKA can execute it.
        self._send_hold_count = 0
        if self._send_hold_timer is not None:
            self._send_hold_timer.stop()
        self._send_hold_timer = QTimer(self)
        self._send_hold_timer.timeout.connect(self._send_hold_tick)
        self._send_hold_timer.start(200)  # re-send every 200ms

    def _on_gripper_open(self):
        """Abrir garra: un único comando con GripperCommand=0."""
        self._send_gripper_command(0)

    def _on_gripper_close(self):
        """Cerrar garra: un único comando con GripperCommand=1."""
        self._send_gripper_command(1)

    def _send_gripper_command(self, value: int):
        """
        Envío puntual de la garra por el MISMO camino TCP/IP de siempre.

        No toca A1-A6 ni X-C: solo pide un gripper_command para el JSON que
        se construye a continuación, fuerza EnableMove=false en ese envío y
        restaura el estado del checkbox justo después. El nuevo Seq lo da
        build_target_json() a través de next_seq(), igual que SEND, y el
        propio modelo vuelve a -1 al consumirlo: no hacen falta ni
        temporizadores ni hilos.
        """
        previous_enable = self._model.get_enable_move()
        self._model.set_enable_move(False)
        self._model.request_gripper_command(value)
        json_str = self._model.build_target_json()
        self._model.set_enable_move(previous_enable)

        self._bridge.publish_command(json_str)

        if self._show_raw_json and hasattr(self, '_txt_command'):
            try:
                pretty = json.dumps(json.loads(json_str), indent=2)
                self._txt_command.setPlainText(pretty)
            except Exception:
                self._txt_command.setPlainText(json_str)

    def _on_auto(self):
        """Start automatic publishing at auto_publish_hz."""
        if not self._allow_auto_mode:
            QMessageBox.warning(
                self, 'Modo Auto desactivado',
                'allow_auto_mode está desactivado en la configuración.'
            )
            return

        if self._auto_running:
            return

        self._model.set_node_mode('auto')
        interval_ms = max(50, int(1000.0 / self._auto_hz))

        self._auto_timer = QTimer(self)
        self._auto_timer.timeout.connect(self._auto_tick)
        self._auto_timer.start(interval_ms)
        self._auto_running = True

        self._btn_auto.setEnabled(False)
        self._btn_stop_auto.setEnabled(True)

    def _on_stop_auto(self):
        """Stop automatic publishing."""
        if self._auto_timer:
            self._auto_timer.stop()
            self._auto_timer = None
        self._auto_running = False
        self._model.set_node_mode('manual_send')

        self._btn_auto.setEnabled(True)
        self._btn_stop_auto.setEnabled(False)

    def _on_reset(self):
        """Reset GUI to home position, clear errors."""
        self._model.load_home()
        self._model.clear_feedback()
        self._refresh_inputs()
        self._refresh_table()

        if self._show_raw_json and hasattr(self, '_txt_feedback'):
            self._txt_feedback.clear()
        if self._show_raw_json and hasattr(self, '_txt_command'):
            self._txt_command.clear()
        if self._show_raw_xml and hasattr(self, '_txt_cmd_xml'):
            self._txt_cmd_xml.clear()

    def _on_enable_move_changed(self, state):
        """Handle enable move checkbox change."""
        checked = (state == Qt.Checked)

        if checked and not self._enable_move_confirmed:
            reply = QMessageBox.warning(
                self,
                'Habilitar EnableMove',
                'Esto permite enviar EnableMove=true al bridge.\n\n'
                'El movimiento real solo será posible si:\n'
                '  • safe_mode = false\n'
                '  • allow_motion_commands = true\n'
                '  en el bridge.\n\n'
                '¿Deseas continuar?',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                self._chk_enable_move.blockSignals(True)
                self._chk_enable_move.setChecked(False)
                self._chk_enable_move.blockSignals(False)
                return
            self._enable_move_confirmed = True

        # If in auto mode but auto motion not allowed, override
        effective = checked
        if self._auto_running and not self._allow_auto_motion:
            effective = False

        self._model.set_enable_move(effective)
        status_text = 'enable_move = true' if checked else 'enable_move = false'
        clr = ACCENT2 if checked else TEXT_SEC
        self._lbl_enable_status.setText(status_text)
        self._lbl_enable_status.setStyleSheet(f'color: {clr}; font-size: 12px;')

    def _on_step(self, axis: str, direction: int):
        """Increment or decrement a joint target."""
        self._model.step_target(axis, direction)
        self._refresh_inputs()
        self._refresh_table()

    def _on_input_changed(self, axis: str):
        """Handle manual edit of a joint input field."""
        inp = self._joint_inputs.get(axis)
        if inp is None:
            return
        try:
            val = float(inp.text())
            self._model.set_target(axis, val)
        except ValueError:
            pass
        self._refresh_inputs()
        self._refresh_table()

    # ===================================================================
    # Auto tick
    # ===================================================================

    def _auto_tick(self):
        """Called by auto timer to publish the current target."""
        if not self._model.all_targets_in_limits():
            return

        self._validate_and_send()

    def _send_hold_tick(self):
        """Re-send the current command to keep it fresh in the bridge."""
        self._send_hold_count += 1
        if self._send_hold_count > 15:  # 15 * 200ms = 3 seconds max
            if self._send_hold_timer:
                self._send_hold_timer.stop()
                self._send_hold_timer = None
            return
        if not self._model.all_targets_in_limits():
            return
        self._validate_and_send()

    # ===================================================================
    # Bridge signal handlers
    # ===================================================================

    @pyqtSlot(str)
    def _on_feedback(self, data: str):
        """Handle feedback JSON from bridge."""
        try:
            fb = json.loads(data)
        except json.JSONDecodeError:
            return

        self._model.update_feedback(fb)

        # On first feedback, sync targets to robot's actual position
        if not self._synced_to_robot:
            axis_actual = fb.get('axis_actual', {})
            pos_actual = fb.get('position_actual', {})
            synced = False
            for a in AXES:
                val = axis_actual.get(a)
                if val is not None:
                    self._model.set_target(a, float(val))
                    synced = True
            for a in CARTESIAN_AXES:
                val = pos_actual.get(a)
                if val is not None:
                    self._model.set_target(a, float(val))
                    synced = True
            if synced:
                self._synced_to_robot = True
                self._refresh_inputs()

        self._refresh_table()

        # Update status indicators
        self._lbl_kuka[1].setText('Feedback activo')
        self._lbl_kuka[1].setStyleSheet(f'color: {ACCENT2}; font-weight: bold;')

        self._lbl_bridge[1].setText('Conectado')
        self._lbl_bridge[1].setStyleSheet(f'color: {ACCENT2}; font-weight: bold;')

        # Check KUKA-side status flags
        move_ready = fb.get('move_ready', False)
        limits_ok = fb.get('limits_ok', False)
        delta_ok = fb.get('delta_ok', False)
        move_executed = fb.get('move_executed', False)

        # Update safety status from bridge
        safe_mode = fb.get('bridge_safe_mode', True)
        allow_motion = fb.get('bridge_allow_motion', False)
        if safe_mode or not allow_motion:
            self._lbl_safety[1].setText('Bloqueado')
            self._lbl_safety[1].setStyleSheet(f'color: {WARN_CLR}; font-weight: bold;')
        else:
            self._lbl_safety[1].setText('Permitido')
            self._lbl_safety[1].setStyleSheet(f'color: {ACCENT2}; font-weight: bold;')

        # Update feedback display
        if self._show_raw_json and hasattr(self, '_txt_feedback'):
            try:
                pretty = json.dumps(fb, indent=2)
                self._txt_feedback.setPlainText(pretty)
            except Exception:
                self._txt_feedback.setPlainText(data)

    @pyqtSlot(str)
    def _on_raw_command_xml(self, data: str):
        """Handle raw command XML from bridge."""
        if self._show_raw_xml and hasattr(self, '_txt_cmd_xml'):
            self._txt_cmd_xml.setPlainText(data)

    @pyqtSlot(str)
    def _on_raw_robot_xml(self, data: str):
        """Handle raw robot XML from bridge (currently not displayed separately)."""
        pass

    @pyqtSlot(bool)
    def _on_ros_status(self, active: bool):
        """Handle ROS2 status change."""
        if active:
            self._lbl_ros[1].setText('Activo')
            self._lbl_ros[1].setStyleSheet(f'color: {ACCENT2}; font-weight: bold;')
        else:
            self._lbl_ros[1].setText('Inactivo')
            self._lbl_ros[1].setStyleSheet(f'color: {TEXT_SEC}; font-weight: bold;')

    # ===================================================================
    # Feedback timeout check
    # ===================================================================

    def _check_feedback_timeout(self):
        """Check if feedback is stale and update status."""
        if not self._model.has_recent_feedback(self._feedback_timeout):
            self._lbl_kuka[1].setText('Sin feedback')
            self._lbl_kuka[1].setStyleSheet(f'color: {TEXT_SEC}; font-weight: bold;')

    # ===================================================================
    # Close event
    # ===================================================================

    def closeEvent(self, event):
        """Clean shutdown on window close."""
        if self._auto_timer:
            self._auto_timer.stop()
        if self._send_hold_timer:
            self._send_hold_timer.stop()
        if self._feedback_timer:
            self._feedback_timer.stop()
        if getattr(self, '_trajectory_panel', None) is not None:
            self._trajectory_panel.shutdown()
        if getattr(self, '_trajectory_batch_panel', None) is not None:
            self._trajectory_batch_panel.shutdown()
        if self._bridge.is_running:
            self._bridge.stop()
        event.accept()
