"""
dual_kuka_rviz_window.py — PyQt5 main window for Dual KUKA + RViz mode.

Three-screen flow:
  Screen 0 (Welcome):   Title + START KUKA + MODO DUAL buttons.
  Screen 1 (Dual):      Unified A1-A6 controls, dual feedback table,
                         cartesian section for RViz.

All ROS2 communication is delegated to:
  - RosAxisMoveBridge   (KUKA TCP/IP — existing, unmodified)
  - RosMoveitMirrorBridge (RViz/MoveIt2 — new)

This module does NOT modify gui_axis_move_window.py.
"""

import json
import time
from typing import Dict, List, Optional

try:
    from PyQt5.QtCore import Qt, QTimer, pyqtSlot
    from PyQt5.QtGui import QColor, QFont, QPalette
    from PyQt5.QtWidgets import (
        QApplication, QCheckBox, QFrame, QGridLayout, QGroupBox,
        QHBoxLayout, QLabel, QLineEdit, QMainWindow,
        QMessageBox, QPushButton, QSizePolicy, QScrollArea,
        QStackedWidget, QTextEdit, QVBoxLayout, QWidget,
    )
except ImportError as e:
    raise ImportError(
        'PyQt5 is required. Install with:  sudo apt install python3-pyqt5'
    ) from e

from kuka_gui_control.dual_command_model import (
    DualCommandModel, AXES, CART_KEYS,
)

# ---------------------------------------------------------------------------
# Style constants (same palette as gui_axis_move_window.py)
# ---------------------------------------------------------------------------

DARK_BG = '#0d1117'
PANEL_BG = '#161b22'
BORDER_CLR = '#30363d'
ACCENT = '#58a6ff'
ACCENT2 = '#3fb950'
ACCENT3 = '#d2a8ff'       # Purple for RViz
WARN_CLR = '#f78166'
ERROR_CLR = '#da3633'
TEXT_PRI = '#e6edf3'
TEXT_SEC = '#8b949e'
AUTO_ON_CLR = '#388bfd'
STOP_CLR = '#da3633'
RVIZ_CLR = '#d2a8ff'

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
    border-radius: 6px;
    margin-top: 10px;
    padding-top: 14px;
    font-weight: bold;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 4px;
}}
QLineEdit {{
    background-color: {PANEL_BG};
    border: 1px solid {BORDER_CLR};
    border-radius: 4px;
    padding: 4px 6px;
    color: {TEXT_PRI};
}}
QLineEdit:focus {{
    border-color: {ACCENT};
}}
QPushButton {{
    background-color: {PANEL_BG};
    border: 1px solid {BORDER_CLR};
    border-radius: 4px;
    padding: 6px 14px;
    color: {TEXT_PRI};
}}
QPushButton:hover {{
    background-color: #21262d;
    border-color: {ACCENT};
}}
QTextEdit {{
    background-color: {PANEL_BG};
    border: 1px solid {BORDER_CLR};
    border-radius: 4px;
    color: {TEXT_SEC};
    font-family: 'Cascadia Code', 'Fira Code', monospace;
    font-size: 11px;
}}
QCheckBox {{
    color: {TEXT_PRI};
    spacing: 6px;
}}
QCheckBox::indicator {{
    width: 16px; height: 16px;
    border: 1px solid {BORDER_CLR};
    border-radius: 3px;
    background-color: {PANEL_BG};
}}
QCheckBox::indicator:checked {{
    background-color: {ACCENT2};
    border-color: {ACCENT2};
}}
"""

BTN_START = f"""
QPushButton {{
    background-color: {ACCENT};
    color: #ffffff;
    font-size: 16px;
    font-weight: bold;
    padding: 12px 40px;
    border-radius: 8px;
    border: none;
}}
QPushButton:hover {{
    background-color: #79c0ff;
}}
"""

BTN_DUAL = f"""
QPushButton {{
    background-color: {ACCENT3};
    color: #0d1117;
    font-size: 16px;
    font-weight: bold;
    padding: 12px 40px;
    border-radius: 8px;
    border: none;
}}
QPushButton:hover {{
    background-color: #e2bfff;
}}
"""

BTN_SEND = f"""
QPushButton {{
    background-color: {ACCENT2};
    color: #0d1117;
    font-weight: bold;
    padding: 8px 24px;
    border-radius: 6px;
    border: none;
}}
QPushButton:hover {{ background-color: #56d364; }}
"""

BTN_STOP = f"""
QPushButton {{
    background-color: {STOP_CLR};
    color: #ffffff;
    font-weight: bold;
    padding: 8px 24px;
    border-radius: 6px;
    border: none;
}}
QPushButton:hover {{ background-color: #f85149; }}
"""

BTN_BACK = f"""
QPushButton {{
    background-color: {PANEL_BG};
    border: 1px solid {BORDER_CLR};
    border-radius: 6px;
    padding: 6px 16px;
    color: {TEXT_SEC};
}}
QPushButton:hover {{
    border-color: {ACCENT};
    color: {TEXT_PRI};
}}
"""


# ---------------------------------------------------------------------------
# Main Window
# ---------------------------------------------------------------------------

class DualKukaRvizWindow(QMainWindow):
    """
    Main window for Dual KUKA + RViz mode.

    Screen 0: Welcome (START / MODO DUAL)
    Screen 1: Dual control (unified joints + cartesian RViz)
    """

    def __init__(
        self,
        model: DualCommandModel,
        kuka_bridge,          # RosAxisMoveBridge
        rviz_bridge,          # RosMoveitMirrorBridge
        config: dict,
        parent=None,
    ):
        super().__init__(parent)
        self._model = model
        self._kuka_bridge = kuka_bridge
        self._rviz_bridge = rviz_bridge
        self._config = config

        self._auto_timer: Optional[QTimer] = None
        self._auto_running = False
        self._feedback_timer: Optional[QTimer] = None
        self._send_hold_timer: Optional[QTimer] = None
        self._send_hold_count = 0
        self._first_send_confirmed = False
        self._synced_to_robot = False

        # Config values
        self._auto_hz = config.get('auto_publish_hz', 2.0)
        self._feedback_timeout = config.get('feedback_timeout_sec', 2.0)
        self._allow_auto_mode = config.get('allow_auto_mode', True)
        self._allow_auto_motion = config.get('allow_auto_motion', False)
        self._require_confirm = config.get('require_confirmation_for_first_send', True)
        self._show_raw_json = config.get('show_raw_json', True)
        self._show_raw_xml = config.get('show_raw_xml', True)

        self.setWindowTitle('KUKA Dual Control — KUKA Real + RViz/MoveIt2')
        self.setMinimumSize(1050, 780)
        self.setStyleSheet(BASE_STYLE)

        # ── Build UI ─────────────────────────────────────────────────
        self._stack = QStackedWidget()
        self.setCentralWidget(self._stack)

        self._build_welcome_screen()      # index 0
        self._build_dual_control_screen()  # index 1

        self._stack.setCurrentIndex(0)

        # ── Connect KUKA bridge signals ──────────────────────────────
        self._kuka_bridge.feedback_received.connect(self._on_kuka_feedback)
        self._kuka_bridge.raw_command_xml_received.connect(self._on_raw_cmd_xml)
        self._kuka_bridge.raw_robot_xml_received.connect(self._on_raw_robot_xml)
        self._kuka_bridge.ros_status_changed.connect(self._on_ros_status)

        # ── Connect RViz bridge signals ──────────────────────────────
        self._rviz_bridge.rviz_status_received.connect(self._on_rviz_status)
        self._rviz_bridge.rviz_joint_state_received.connect(self._on_rviz_joint_state)
        self._rviz_bridge.rviz_cartesian_state_received.connect(self._on_rviz_cart_state)

        # ── Feedback timeout timer ───────────────────────────────────
        self._feedback_timer = QTimer(self)
        self._feedback_timer.timeout.connect(self._check_feedback_timeout)
        self._feedback_timer.start(500)

    # ===================================================================
    # Screen 0: Welcome
    # ===================================================================

    def _build_welcome_screen(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setAlignment(Qt.AlignCenter)

        title = QLabel('KUKA Joint Control GUI')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            f'font-size: 28px; font-weight: bold; color: {TEXT_PRI};'
        )
        layout.addWidget(title)

        subtitle = QLabel('Modo Dual — KUKA Real + RViz/MoveIt2')
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setStyleSheet(f'font-size: 15px; color: {ACCENT3};')
        layout.addWidget(subtitle)

        layout.addSpacing(10)

        desc = QLabel(
            'Envía comandos simultáneamente al KUKA real (TCP/IP)\n'
            'y a RViz/MoveIt2 (ROS2 topics).\n\n'
            'Protocolo KUKA: TCP/XML via EthernetKRL — Puerto 59153\n'
            'Protocolo RViz: /kuka_bridge/joint_command_deg'
        )
        desc.setAlignment(Qt.AlignCenter)
        desc.setStyleSheet(f'font-size: 13px; color: {TEXT_SEC};')
        layout.addWidget(desc)

        layout.addSpacing(30)

        btn_dual = QPushButton('MODO DUAL: KUKA + RViz')
        btn_dual.setStyleSheet(BTN_DUAL)
        btn_dual.setCursor(Qt.PointingHandCursor)
        btn_dual.clicked.connect(lambda: self._stack.setCurrentIndex(1))
        layout.addWidget(btn_dual, alignment=Qt.AlignCenter)

        self._stack.addWidget(page)

    # ===================================================================
    # Screen 1: Dual Control
    # ===================================================================

    def _build_dual_control_screen(self):
        page = QWidget()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        scroll.setStyleSheet('QScrollArea { border: none; }')

        main_layout = QVBoxLayout(page)
        main_layout.setContentsMargins(12, 8, 12, 8)
        main_layout.setSpacing(8)

        # ── Header with BACK button ──────────────────────────────────
        header = QHBoxLayout()
        btn_back = QPushButton('← VOLVER')
        btn_back.setStyleSheet(BTN_BACK)
        btn_back.setCursor(Qt.PointingHandCursor)
        btn_back.clicked.connect(self._on_back)
        header.addWidget(btn_back)

        header.addStretch()

        lbl_title = QLabel('Modo Dual — KUKA Real + RViz/MoveIt2')
        lbl_title.setStyleSheet(
            f'font-size: 18px; font-weight: bold; color: {ACCENT3};'
        )
        header.addWidget(lbl_title)
        header.addStretch()
        main_layout.addLayout(header)

        # ── Status panel ─────────────────────────────────────────────
        self._build_status_panel(main_layout)

        # ── Joint table ──────────────────────────────────────────────
        self._build_joint_table(main_layout)

        # ── Joint controls ───────────────────────────────────────────
        self._build_joint_controls(main_layout)

        # ── Action buttons ───────────────────────────────────────────
        self._build_action_buttons(main_layout)

        # ── Cartesian section ────────────────────────────────────────
        self._build_cartesian_section(main_layout)

        # ── Raw data viewers ─────────────────────────────────────────
        self._build_raw_data_section(main_layout)

        main_layout.addStretch()

        self._stack.addWidget(scroll)

    # ── Status panel ─────────────────────────────────────────────────

    def _build_status_panel(self, parent_layout):
        grp = QGroupBox('Estado del Sistema')
        grid = QGridLayout(grp)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)

        # KUKA column
        lbl_kuka_title = QLabel('KUKA Real (TCP/IP)')
        lbl_kuka_title.setStyleSheet(f'font-weight: bold; color: {ACCENT};')
        grid.addWidget(lbl_kuka_title, 0, 0, 1, 2)

        grid.addWidget(QLabel('ROS2:'), 1, 0)
        self._lbl_ros = QLabel('Esperando...')
        self._lbl_ros.setStyleSheet(f'color: {TEXT_SEC};')
        grid.addWidget(self._lbl_ros, 1, 1)

        grid.addWidget(QLabel('KUKA:'), 2, 0)
        self._lbl_kuka_status = QLabel('Sin feedback')
        self._lbl_kuka_status.setStyleSheet(f'color: {TEXT_SEC};')
        grid.addWidget(self._lbl_kuka_status, 2, 1)

        grid.addWidget(QLabel('Safe Mode:'), 3, 0)
        self._lbl_safe = QLabel('—')
        grid.addWidget(self._lbl_safe, 3, 1)

        grid.addWidget(QLabel('Publicar a KUKA:'), 4, 0)
        self._chk_pub_kuka = QCheckBox()
        self._chk_pub_kuka.setChecked(self._model.publish_to_kuka)
        self._chk_pub_kuka.toggled.connect(
            lambda v: setattr(self._model, 'publish_to_kuka', v)
        )
        grid.addWidget(self._chk_pub_kuka, 4, 1)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setStyleSheet(f'color: {BORDER_CLR};')
        grid.addWidget(sep, 0, 2, 5, 1)

        # RViz column
        lbl_rviz_title = QLabel('RViz / MoveIt2')
        lbl_rviz_title.setStyleSheet(f'font-weight: bold; color: {RVIZ_CLR};')
        grid.addWidget(lbl_rviz_title, 0, 3, 1, 2)

        grid.addWidget(QLabel('MoveIt Status:'), 1, 3)
        self._lbl_moveit = QLabel('Sin estado')
        self._lbl_moveit.setStyleSheet(f'color: {TEXT_SEC};')
        grid.addWidget(self._lbl_moveit, 1, 4)

        grid.addWidget(QLabel('Joint State:'), 2, 3)
        self._lbl_rviz_joints = QLabel('—')
        self._lbl_rviz_joints.setStyleSheet(f'color: {TEXT_SEC};')
        grid.addWidget(self._lbl_rviz_joints, 2, 4)

        grid.addWidget(QLabel('Cartesiano:'), 3, 3)
        self._lbl_rviz_cart = QLabel('—')
        self._lbl_rviz_cart.setStyleSheet(f'color: {TEXT_SEC};')
        grid.addWidget(self._lbl_rviz_cart, 3, 4)

        grid.addWidget(QLabel('Publicar a RViz:'), 4, 3)
        self._chk_pub_rviz = QCheckBox()
        self._chk_pub_rviz.setChecked(self._model.publish_to_rviz)
        self._chk_pub_rviz.toggled.connect(
            lambda v: setattr(self._model, 'publish_to_rviz', v)
        )
        grid.addWidget(self._chk_pub_rviz, 4, 4)

        parent_layout.addWidget(grp)

    # ── Joint table ──────────────────────────────────────────────────

    def _build_joint_table(self, parent_layout):
        grp = QGroupBox('Tabla de Joints — KUKA vs RViz')
        grid = QGridLayout(grp)
        grid.setSpacing(4)

        headers = ['Joint', 'Target (°)', 'KUKA fb (°)', 'RViz fb (°)',
                    'Error KUKA', 'Error RViz']
        for col, h in enumerate(headers):
            lbl = QLabel(h)
            lbl.setStyleSheet(
                f'font-weight: bold; color: {TEXT_SEC}; font-size: 11px;'
            )
            lbl.setAlignment(Qt.AlignCenter)
            grid.addWidget(lbl, 0, col)

        self._table_labels: Dict[str, Dict[str, QLabel]] = {}
        for row, axis in enumerate(AXES, start=1):
            labels = {}

            # Joint name
            lbl_name = QLabel(axis)
            lbl_name.setStyleSheet(f'font-weight: bold; color: {ACCENT};')
            lbl_name.setAlignment(Qt.AlignCenter)
            grid.addWidget(lbl_name, row, 0)

            for col, key in enumerate(
                ['target', 'kuka_fb', 'rviz_fb', 'err_kuka', 'err_rviz'],
                start=1,
            ):
                lbl = QLabel('—')
                lbl.setAlignment(Qt.AlignCenter)
                lbl.setStyleSheet(f'color: {TEXT_PRI}; font-size: 12px;')
                grid.addWidget(lbl, row, col)
                labels[key] = lbl

            self._table_labels[axis] = labels

        parent_layout.addWidget(grp)

    # ── Joint controls ───────────────────────────────────────────────

    def _build_joint_controls(self, parent_layout):
        grp = QGroupBox('Controles de Joints (A1-A6)')
        grid = QGridLayout(grp)
        grid.setSpacing(6)

        self._joint_inputs: Dict[str, QLineEdit] = {}

        for i, axis in enumerate(AXES):
            row = i // 3
            col_base = (i % 3) * 4

            lbl = QLabel(axis)
            lbl.setStyleSheet(f'font-weight: bold; color: {ACCENT};')
            grid.addWidget(lbl, row, col_base)

            btn_minus = QPushButton('−')
            btn_minus.setFixedWidth(32)
            btn_minus.setCursor(Qt.PointingHandCursor)
            btn_minus.clicked.connect(
                lambda _, a=axis: self._on_step(a, -1)
            )
            grid.addWidget(btn_minus, row, col_base + 1)

            inp = QLineEdit(f'{self._model.get_target(axis):.2f}')
            inp.setFixedWidth(80)
            inp.setAlignment(Qt.AlignCenter)
            inp.returnPressed.connect(
                lambda a=axis: self._on_input_changed(a)
            )
            grid.addWidget(inp, row, col_base + 2)
            self._joint_inputs[axis] = inp

            btn_plus = QPushButton('+')
            btn_plus.setFixedWidth(32)
            btn_plus.setCursor(Qt.PointingHandCursor)
            btn_plus.clicked.connect(
                lambda _, a=axis: self._on_step(a, +1)
            )
            grid.addWidget(btn_plus, row, col_base + 3)

        # Enable move checkbox
        row_extra = 2
        self._chk_enable = QCheckBox('ENABLE MOVE (KUKA)')
        self._chk_enable.setChecked(self._model.get_enable_move())
        self._chk_enable.toggled.connect(self._on_enable_toggled)
        grid.addWidget(self._chk_enable, row_extra, 0, 1, 4)

        parent_layout.addWidget(grp)

    # ── Action buttons ───────────────────────────────────────────────

    def _build_action_buttons(self, parent_layout):
        layout = QHBoxLayout()

        btn_send = QPushButton('SEND DUAL')
        btn_send.setStyleSheet(BTN_SEND)
        btn_send.setCursor(Qt.PointingHandCursor)
        btn_send.clicked.connect(self._on_send)
        layout.addWidget(btn_send)

        btn_home = QPushButton('HOME')
        btn_home.setCursor(Qt.PointingHandCursor)
        btn_home.clicked.connect(self._on_home)
        layout.addWidget(btn_home)

        self._btn_auto = QPushButton('AUTO')
        self._btn_auto.setStyleSheet(
            f'QPushButton {{ background-color: {AUTO_ON_CLR}; color: white; '
            f'font-weight: bold; padding: 8px 24px; border-radius: 6px; border: none; }}'
            f'QPushButton:hover {{ background-color: #58a6ff; }}'
        )
        self._btn_auto.setCursor(Qt.PointingHandCursor)
        self._btn_auto.clicked.connect(self._on_auto)
        layout.addWidget(self._btn_auto)

        self._btn_stop = QPushButton('STOP AUTO')
        self._btn_stop.setStyleSheet(BTN_STOP)
        self._btn_stop.setCursor(Qt.PointingHandCursor)
        self._btn_stop.clicked.connect(self._on_stop_auto)
        self._btn_stop.setEnabled(False)
        layout.addWidget(self._btn_stop)

        parent_layout.addLayout(layout)

    # ── Cartesian section ────────────────────────────────────────────

    def _build_cartesian_section(self, parent_layout):
        grp = QGroupBox('Mundo / Cartesiano — Solo RViz')
        grp_layout = QVBoxLayout(grp)

        # Warning label
        lbl_warn = QLabel(
            '⚠ Cartesiano/Mundo disponible solo para RViz. '
            'No se envía al KUKA real por TCP/IP.'
        )
        lbl_warn.setStyleSheet(
            f'color: {WARN_CLR}; font-size: 11px; font-style: italic; '
            f'padding: 4px;'
        )
        lbl_warn.setWordWrap(True)
        grp_layout.addWidget(lbl_warn)

        # Cartesian inputs
        cart_grid = QGridLayout()
        self._cart_inputs: Dict[str, QLineEdit] = {}

        units = {'X': 'm', 'Y': 'm', 'Z': 'm', 'A': '°', 'B': '°', 'C': '°'}

        for i, key in enumerate(CART_KEYS):
            col_base = (i % 3) * 3
            row = i // 3

            lbl = QLabel(f'{key} ({units[key]})')
            lbl.setStyleSheet(f'font-weight: bold; color: {RVIZ_CLR};')
            cart_grid.addWidget(lbl, row, col_base)

            inp = QLineEdit(f'{self._model.get_cart_target(key):.4f}')
            inp.setFixedWidth(100)
            inp.setAlignment(Qt.AlignCenter)
            cart_grid.addWidget(inp, row, col_base + 1)
            self._cart_inputs[key] = inp

        grp_layout.addLayout(cart_grid)

        # Cartesian buttons
        cart_btns = QHBoxLayout()

        btn_send_cart = QPushButton('SEND CARTESIAN TO RViz')
        btn_send_cart.setStyleSheet(
            f'QPushButton {{ background-color: {RVIZ_CLR}; color: #0d1117; '
            f'font-weight: bold; padding: 8px 20px; border-radius: 6px; border: none; }}'
            f'QPushButton:hover {{ background-color: #e2bfff; }}'
        )
        btn_send_cart.setCursor(Qt.PointingHandCursor)
        btn_send_cart.clicked.connect(self._on_send_cartesian)
        cart_btns.addWidget(btn_send_cart)

        btn_reset_cart = QPushButton('RESET CARTESIAN')
        btn_reset_cart.setCursor(Qt.PointingHandCursor)
        btn_reset_cart.clicked.connect(self._on_reset_cartesian)
        cart_btns.addWidget(btn_reset_cart)

        grp_layout.addLayout(cart_btns)

        # Last cartesian state feedback
        self._lbl_cart_last = QLabel('Último estado cartesiano RViz: —')
        self._lbl_cart_last.setStyleSheet(f'color: {TEXT_SEC}; font-size: 11px;')
        grp_layout.addWidget(self._lbl_cart_last)

        parent_layout.addWidget(grp)

    # ── Raw data section ─────────────────────────────────────────────

    def _build_raw_data_section(self, parent_layout):
        grp = QGroupBox('Datos Raw — JSON / XML')
        layout = QHBoxLayout(grp)

        # Command JSON
        col1 = QVBoxLayout()
        col1.addWidget(QLabel('Último comando JSON:'))
        self._txt_command = QTextEdit()
        self._txt_command.setReadOnly(True)
        self._txt_command.setMaximumHeight(100)
        col1.addWidget(self._txt_command)
        layout.addLayout(col1)

        # Command XML
        col2 = QVBoxLayout()
        col2.addWidget(QLabel('Último XML enviado:'))
        self._txt_cmd_xml = QTextEdit()
        self._txt_cmd_xml.setReadOnly(True)
        self._txt_cmd_xml.setMaximumHeight(100)
        col2.addWidget(self._txt_cmd_xml)
        layout.addLayout(col2)

        # Robot XML
        col3 = QVBoxLayout()
        col3.addWidget(QLabel('Último XML recibido:'))
        self._txt_robot_xml = QTextEdit()
        self._txt_robot_xml.setReadOnly(True)
        self._txt_robot_xml.setMaximumHeight(100)
        col3.addWidget(self._txt_robot_xml)
        layout.addLayout(col3)

        parent_layout.addWidget(grp)

    # ===================================================================
    # Validate and send (dual)
    # ===================================================================

    def _validate_and_send(self):
        """
        Build JSON for KUKA and Float64MultiArray for RViz,
        validate, and publish atomically.
        """
        if not self._model.all_targets_in_limits():
            return

        # Build KUKA JSON (increments seq)
        json_str = self._model.build_target_json()

        # Publish to KUKA TCP/IP
        if self._model.publish_to_kuka:
            self._kuka_bridge.publish_command(json_str)

        # Publish to RViz/MoveIt2
        if self._model.publish_to_rviz:
            joint_array = self._model.build_rviz_joint_array()
            self._rviz_bridge.publish_joints(joint_array)

        # Update command display
        if hasattr(self, '_txt_command'):
            try:
                pretty = json.dumps(json.loads(json_str), indent=2)
                self._txt_command.setPlainText(pretty)
            except Exception:
                self._txt_command.setPlainText(json_str)

    # ===================================================================
    # Button handlers
    # ===================================================================

    def _on_back(self):
        """Return to welcome screen. Stop auto if running."""
        self._on_stop_auto()
        self._stack.setCurrentIndex(0)

    def _on_home(self):
        """Load home position and send."""
        self._model.load_home()
        self._refresh_inputs()
        self._refresh_table()
        self._validate_and_send()

    def _on_send(self):
        """Manual dual send."""
        if not self._model.all_targets_in_limits():
            return

        if self._require_confirm and not self._first_send_confirmed:
            reply = QMessageBox.question(
                self,
                'Confirmar primer envío',
                'Este es el primer SEND de esta sesión.\n'
                '¿Confirmas que deseas enviar el comando?\n\n'
                'El movimiento real depende de safe_mode y allow_motion_commands.',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                return
            self._first_send_confirmed = True

        self._model.set_mode('manual_send')
        self._validate_and_send()
        self._refresh_table()

        # Hold: keep re-sending for ~3 seconds
        self._send_hold_count = 0
        if self._send_hold_timer is not None:
            self._send_hold_timer.stop()
        self._send_hold_timer = QTimer(self)
        self._send_hold_timer.timeout.connect(self._send_hold_tick)
        self._send_hold_timer.start(200)

    def _on_auto(self):
        """Start automatic dual publishing."""
        if not self._allow_auto_mode:
            QMessageBox.warning(
                self, 'Modo Auto desactivado',
                'allow_auto_mode está desactivado en la configuración.'
            )
            return

        if self._auto_running:
            return

        self._model.set_mode('auto')
        interval = max(50, int(1000.0 / self._auto_hz))
        self._auto_timer = QTimer(self)
        self._auto_timer.timeout.connect(self._auto_tick)
        self._auto_timer.start(interval)
        self._auto_running = True
        self._btn_auto.setEnabled(False)
        self._btn_stop.setEnabled(True)

    def _on_stop_auto(self):
        """Stop automatic publishing."""
        if self._auto_timer:
            self._auto_timer.stop()
            self._auto_timer = None
        self._auto_running = False
        self._btn_auto.setEnabled(True)
        self._btn_stop.setEnabled(False)

    def _on_enable_toggled(self, checked):
        """Toggle ENABLE MOVE flag."""
        self._model.set_enable_move(checked)

    def _on_send_cartesian(self):
        """Send cartesian command to RViz only."""
        # Read values from inputs
        for key in CART_KEYS:
            try:
                val = float(self._cart_inputs[key].text())
                self._model.set_cart_target(key, val)
            except ValueError:
                QMessageBox.warning(
                    self, 'Valor inválido',
                    f'El valor de {key} no es un número válido.'
                )
                return

        if not self._model.cartesian_to_rviz:
            QMessageBox.information(
                self, 'Cartesiano desactivado',
                'cartesian_to_rviz está desactivado.'
            )
            return

        cart_array = self._model.build_cartesian_array()
        self._rviz_bridge.publish_cartesian(cart_array)

        # Update display
        self._lbl_cart_last.setText(
            f'Último envío: '
            + ' '.join(f'{k}={cart_array[i]:.4f}' for i, k in enumerate(CART_KEYS))
        )

    def _on_reset_cartesian(self):
        """Reset cartesian inputs to zero."""
        self._model.reset_cart_targets()
        for key in CART_KEYS:
            self._cart_inputs[key].setText('0.0000')

    # ===================================================================
    # Joint input handlers
    # ===================================================================

    def _on_step(self, axis: str, direction: int):
        """Step a joint value by step_deg."""
        new_val = self._model.step_target(axis, direction)
        self._joint_inputs[axis].setText(f'{new_val:.2f}')
        self._refresh_table()

    def _on_input_changed(self, axis: str):
        """Handle manual input change."""
        inp = self._joint_inputs.get(axis)
        if not inp:
            return
        try:
            val = float(inp.text())
            self._model.set_target(axis, val)
        except ValueError:
            pass
        self._refresh_inputs()
        self._refresh_table()

    # ===================================================================
    # Auto tick / send hold
    # ===================================================================

    def _auto_tick(self):
        """Called by auto timer."""
        if not self._model.all_targets_in_limits():
            return
        self._validate_and_send()
        self._refresh_table()

    def _send_hold_tick(self):
        """Re-send to keep command fresh in the bridge."""
        self._send_hold_count += 1
        if self._send_hold_count > 15:
            if self._send_hold_timer:
                self._send_hold_timer.stop()
                self._send_hold_timer = None
            return
        if not self._model.all_targets_in_limits():
            return
        self._validate_and_send()

    # ===================================================================
    # Refresh UI
    # ===================================================================

    def _refresh_inputs(self):
        """Sync input fields with model."""
        for axis in AXES:
            val = self._model.get_target(axis)
            self._joint_inputs[axis].setText(f'{val:.2f}')
            # Color by limits
            in_lim = self._model.is_in_limits(axis)
            color = TEXT_PRI if in_lim else ERROR_CLR
            self._joint_inputs[axis].setStyleSheet(
                f'color: {color}; background-color: {PANEL_BG}; '
                f'border: 1px solid {BORDER_CLR if in_lim else ERROR_CLR}; '
                f'border-radius: 4px; padding: 4px 6px;'
            )

    def _refresh_table(self):
        """Update the joint comparison table."""
        for axis in AXES:
            labels = self._table_labels.get(axis)
            if not labels:
                continue

            target = self._model.get_target(axis)
            labels['target'].setText(f'{target:.2f}')

            # KUKA feedback
            kuka_fb = self._model.get_feedback(axis)
            if kuka_fb is not None:
                labels['kuka_fb'].setText(f'{kuka_fb:.2f}')
                labels['kuka_fb'].setStyleSheet(f'color: {ACCENT};')
            else:
                labels['kuka_fb'].setText('N/A')
                labels['kuka_fb'].setStyleSheet(f'color: {TEXT_SEC};')

            # RViz feedback
            rviz_fb = self._model.get_rviz_feedback(axis)
            if rviz_fb is not None:
                labels['rviz_fb'].setText(f'{rviz_fb:.2f}')
                labels['rviz_fb'].setStyleSheet(f'color: {RVIZ_CLR};')
            else:
                labels['rviz_fb'].setText('N/A')
                labels['rviz_fb'].setStyleSheet(f'color: {TEXT_SEC};')

            # Error KUKA
            err_kuka = self._model.get_error(axis)
            if err_kuka is not None:
                color = ACCENT2 if abs(err_kuka) < 1.0 else WARN_CLR
                labels['err_kuka'].setText(f'{err_kuka:.2f}')
                labels['err_kuka'].setStyleSheet(f'color: {color};')
            else:
                labels['err_kuka'].setText('N/A')
                labels['err_kuka'].setStyleSheet(f'color: {TEXT_SEC};')

            # Error RViz
            err_rviz = self._model.get_rviz_error(axis)
            if err_rviz is not None:
                color = ACCENT2 if abs(err_rviz) < 1.0 else WARN_CLR
                labels['err_rviz'].setText(f'{err_rviz:.2f}')
                labels['err_rviz'].setStyleSheet(f'color: {color};')
            else:
                labels['err_rviz'].setText('N/A')
                labels['err_rviz'].setStyleSheet(f'color: {TEXT_SEC};')

    # ===================================================================
    # Bridge signal handlers
    # ===================================================================

    @pyqtSlot(str)
    def _on_kuka_feedback(self, data_str: str):
        """Handle KUKA feedback JSON."""
        try:
            data = json.loads(data_str)
        except json.JSONDecodeError:
            return

        self._model.update_feedback(data)

        # On first feedback, sync targets to robot's actual position
        if not self._synced_to_robot:
            axis_actual = data.get('axis_actual', {})
            synced = False
            for a in AXES:
                val = axis_actual.get(a)
                if val is not None:
                    self._model.set_target(a, float(val))
                    synced = True
            if synced:
                self._synced_to_robot = True
                self._refresh_inputs()

        # Update KUKA status
        move_ready = data.get('move_ready', False)
        limits_ok = data.get('limits_ok', False)
        safe_mode = data.get('bridge_safe_mode', True)
        allow_motion = data.get('bridge_allow_motion', False)

        self._lbl_kuka_status.setText(
            f'Feedback activo | MoveReady={move_ready}'
        )
        self._lbl_kuka_status.setStyleSheet(f'color: {ACCENT2};')

        safe_text = 'Bloqueado' if safe_mode else 'Permitido'
        safe_color = ERROR_CLR if safe_mode else ACCENT2
        self._lbl_safe.setText(f'{safe_text} (allow_motion={allow_motion})')
        self._lbl_safe.setStyleSheet(f'color: {safe_color};')

        self._refresh_table()

    @pyqtSlot(str)
    def _on_raw_cmd_xml(self, xml_str: str):
        """Handle raw command XML."""
        if hasattr(self, '_txt_cmd_xml'):
            self._txt_cmd_xml.setPlainText(xml_str)

    @pyqtSlot(str)
    def _on_raw_robot_xml(self, xml_str: str):
        """Handle raw robot XML."""
        if hasattr(self, '_txt_robot_xml'):
            self._txt_robot_xml.setPlainText(xml_str)

    @pyqtSlot(bool)
    def _on_ros_status(self, active: bool):
        """Handle ROS2 status change."""
        if active:
            self._lbl_ros.setText('Activo ✓')
            self._lbl_ros.setStyleSheet(f'color: {ACCENT2}; font-weight: bold;')
        else:
            self._lbl_ros.setText('Inactivo')
            self._lbl_ros.setStyleSheet(f'color: {ERROR_CLR};')

    @pyqtSlot(str)
    def _on_rviz_status(self, status: str):
        """Handle MoveIt status."""
        self._model.update_moveit_status(status)
        self._lbl_moveit.setText(status)
        self._lbl_moveit.setStyleSheet(f'color: {RVIZ_CLR};')

    @pyqtSlot(str)
    def _on_rviz_joint_state(self, json_str: str):
        """Handle RViz joint state."""
        try:
            data = json.loads(json_str)
        except json.JSONDecodeError:
            return

        self._model.update_rviz_feedback(data)

        # Summary label
        parts = [f'{a}={data.get(a, 0):.1f}' for a in AXES]
        self._lbl_rviz_joints.setText(' '.join(parts))
        self._lbl_rviz_joints.setStyleSheet(f'color: {RVIZ_CLR};')

        self._refresh_table()

    @pyqtSlot(str)
    def _on_rviz_cart_state(self, json_str: str):
        """Handle RViz cartesian state."""
        try:
            data = json.loads(json_str)
        except json.JSONDecodeError:
            return

        self._model.update_cart_feedback(data)

        parts = [f'{k}={data.get(k, 0):.3f}' for k in CART_KEYS]
        self._lbl_rviz_cart.setText(' '.join(parts))
        self._lbl_rviz_cart.setStyleSheet(f'color: {RVIZ_CLR};')

        self._lbl_cart_last.setText(f'Último estado cartesiano RViz: {" ".join(parts)}')

    # ===================================================================
    # Feedback timeout check
    # ===================================================================

    def _check_feedback_timeout(self):
        """Check if KUKA and RViz feedback are stale."""
        if not self._model.has_recent_feedback(self._feedback_timeout):
            self._lbl_kuka_status.setText('Sin feedback')
            self._lbl_kuka_status.setStyleSheet(f'color: {TEXT_SEC};')

        if not self._model.has_recent_rviz_feedback(self._feedback_timeout):
            self._lbl_rviz_joints.setText('Sin datos')
            self._lbl_rviz_joints.setStyleSheet(f'color: {TEXT_SEC};')

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
        if self._kuka_bridge.is_running:
            self._kuka_bridge.stop()
        event.accept()
