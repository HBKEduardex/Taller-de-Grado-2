"""
trajectory_panel.py — Panel compacto de secuencias de trayectorias.

Widget COMPARTIDO por las dos GUIs (original y dual). Toda la lógica de
secuencia vive aquí y en los módulos trajectory_*; las ventanas solo lo
instancian y lo añaden a su layout, sin cambiar nada de lo que ya hacían.

Controles:
    SET · Puntos: N · SET ABRIR GARRA · SET CERRAR GARRA · LIMPIAR
    ENVIAR PUNTOS (N) · PROBAR TRAYECTORIA · ENVIAR TRAYECTORIA
    (•) Manual   ( ) Automático
    estado + log temporal

Diseño deliberadamente estrecho: dos filas de botones pequeños y un log de
altura fija. No ensancha la ventana.

Reglas duras que este panel respeta:
  * SET lee AxisActual del feedback TCP/IP; nunca los spinboxes ni el target.
  * SET no escribe en disco.
  * SET ABRIR/CERRAR GARRA no mueve la garra: solo programa un evento.
  * PROBAR TRAYECTORIA solo publica en el tópico de previsualización.
  * ENVIAR TRAYECTORIA respeta safe_mode, allow_motion_commands y el
    checkbox ENABLE MOVE de la GUI, y no ofrece ningún bypass.
"""

import json
import time
import uuid
from pathlib import Path
from typing import Callable, Dict, Optional

try:
    from PyQt5.QtCore import Qt, QTimer, pyqtSlot
    from PyQt5.QtWidgets import (
        QButtonGroup, QFileDialog, QGroupBox, QHBoxLayout, QLabel,
        QMessageBox, QPushButton, QRadioButton, QSizePolicy, QTextEdit,
        QVBoxLayout,
    )
except ImportError as e:
    raise ImportError(
        'PyQt5 is required. Install with: sudo apt install python3-pyqt5'
    ) from e

from kuka_gui_control.trajectory_executor import TrajectoryExecutor
from kuka_gui_control.trajectory_ros_bridge import TrajectoryRosBridge
from kuka_gui_control.trajectory_sequence_model import (
    DEFAULT_KUKA_PTP_VELOCITY_NORMAL_PCT,
    DEFAULT_KUKA_PTP_VELOCITY_REDUCED_PCT,
    SCHEMA_VERSION,
    TrajectorySequenceModel,
    build_storage_document,
    is_finite_number,
    validate_result_payload,
)
from kuka_gui_control import trajectory_storage

# ---------------------------------------------------------------------------
# Estilo (mismos colores que ya usan las dos ventanas)
# ---------------------------------------------------------------------------

PANEL_BG = '#161b22'
BORDER_CLR = '#30363d'
ACCENT = '#58a6ff'
ACCENT2 = '#3fb950'
WARN_CLR = '#f78166'
ERROR_CLR = '#da3633'
TEXT_PRI = '#e6edf3'
TEXT_SEC = '#8b949e'

_BTN_COMPACT = f"""
QPushButton {{
    background-color: {PANEL_BG};
    color: {TEXT_PRI};
    border: 1px solid {BORDER_CLR};
    border-radius: 6px;
    padding: 5px 12px;
    font-weight: bold;
    font-size: 12px;
}}
QPushButton:hover {{ border-color: {ACCENT}; color: {ACCENT}; }}
QPushButton:disabled {{ background-color: {BORDER_CLR}; color: {TEXT_SEC}; }}
"""

_BTN_PRIMARY = f"""
QPushButton {{
    background-color: {ACCENT};
    color: #fff;
    border: none;
    border-radius: 6px;
    padding: 5px 12px;
    font-weight: bold;
    font-size: 12px;
}}
QPushButton:hover {{ background-color: #4090e0; }}
QPushButton:disabled {{ background-color: {BORDER_CLR}; color: {TEXT_SEC}; }}
"""

_BTN_PHYSICAL = f"""
QPushButton {{
    background-color: {ERROR_CLR};
    color: #fff;
    border: none;
    border-radius: 6px;
    padding: 5px 12px;
    font-weight: bold;
    font-size: 12px;
}}
QPushButton:hover {{ background-color: #b62324; }}
QPushButton:disabled {{ background-color: {BORDER_CLR}; color: {TEXT_SEC}; }}
"""

# Tiempo máximo de espera de la respuesta de MoveIt tras ENVIAR PUNTOS.
DEFAULT_GENERATION_TIMEOUT_SEC = 60.0


# ---------------------------------------------------------------------------
# Panel
# ---------------------------------------------------------------------------

class TrajectorySequencePanel(QGroupBox):
    """
    Panel de secuencias añadido al final del layout de cada GUI.

    Args:
        model:           JointCommandModel o DualCommandModel de la ventana.
        kuka_bridge:     RosAxisMoveBridge YA existente (aporta el nodo ROS2).
        config:          dict de configuración de la ventana.
        joint_send_fn:   función de la ventana que ya publica el target
                         articular (la misma que usa SEND).
        gripper_send_fn: función de la ventana que ya publica GripperCommand
                         (la misma que usan los botones de garra).
    """

    def __init__(
        self,
        model,
        kuka_bridge,
        config: dict,
        joint_send_fn: Callable[[], None],
        gripper_send_fn: Callable[[int], None],
        parent=None,
    ):
        super().__init__('Secuencias de Trayectorias', parent)

        self._model = model
        self._kuka_bridge = kuka_bridge
        self._config = config or {}

        self._sequence = TrajectorySequenceModel()
        self._traj_bridge = TrajectoryRosBridge(kuka_bridge)
        self._executor = TrajectoryExecutor(
            model=model,
            joint_send_fn=joint_send_fn,
            gripper_send_fn=gripper_send_fn,
            config=self._config,
        )

        self._trajectories_dir_cfg = self._config.get('trajectories_dir', '')
        self._feedback_timeout = float(
            self._config.get('feedback_timeout_sec', 2.0))
        self._generation_timeout = float(self._config.get(
            'trajectory_generation_timeout_sec',
            DEFAULT_GENERATION_TIMEOUT_SEC))
        self._ptp_velocity_normal_pct = self._configured_ptp_velocity(
            'trajectory_kuka_ptp_velocity_normal_pct',
            DEFAULT_KUKA_PTP_VELOCITY_NORMAL_PCT,
        )
        self._ptp_velocity_reduced_pct = self._configured_ptp_velocity(
            'trajectory_kuka_ptp_velocity_reduced_pct',
            DEFAULT_KUKA_PTP_VELOCITY_REDUCED_PCT,
        )

        # Último feedback REAL recibido por TCP/IP.
        self._last_axis_actual: Optional[Dict[str, float]] = None
        self._last_position_actual: Optional[Dict[str, float]] = None
        self._last_feedback_at: float = 0.0
        self._bridge_safe_mode: bool = True
        self._bridge_allow_motion: bool = False

        # Solicitud en curso.
        self._pending_request_id: Optional[str] = None
        self._request_sent_at: float = 0.0

        # Último archivo generado correctamente en esta sesión.
        self._last_saved_file: Optional[Path] = None
        self._preview_id: Optional[str] = None

        self._build_ui()
        self._connect_signals()

        # Timer de mantenimiento. Hace dos cosas:
        #   1. Engancha los tópicos al nodo rclpy en cuanto exista (en la GUI
        #      original el nodo nace DESPUÉS de construirse la ventana).
        #   2. Caduca una solicitud a MoveIt que nunca recibe respuesta, para
        #      que ENVIAR PUNTOS no quede deshabilitado para siempre.
        self._housekeeping_timer = QTimer(self)
        self._housekeeping_timer.timeout.connect(self._housekeeping)
        self._housekeeping_timer.start(500)
        self._housekeeping()

        self._refresh_controls()

    # ===================================================================
    # UI
    # ===================================================================

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(6)
        layout.setContentsMargins(8, 6, 8, 6)

        # ── Fila 1: captura ──────────────────────────────────────────
        row_capture = QHBoxLayout()
        row_capture.setSpacing(6)

        self._btn_set = QPushButton('SET')
        self._btn_set.setStyleSheet(_BTN_PRIMARY)
        self._btn_set.setCursor(Qt.PointingHandCursor)
        self._btn_set.setToolTip(
            'Captura la configuración articular REAL del KUKA (AxisActual '
            'A1..A6 recibido por TCP/IP).\n'
            'No lee los campos de la GUI ni el target. No guarda en disco.'
        )
        self._btn_set.clicked.connect(self._on_set_point)
        row_capture.addWidget(self._btn_set)

        reduced_label = f'{self._ptp_velocity_reduced_pct:g}'
        self._btn_set_reduced = QPushButton(f'SET VEL. {reduced_label}%')
        self._btn_set_reduced.setStyleSheet(_BTN_PRIMARY)
        self._btn_set_reduced.setCursor(Qt.PointingHandCursor)
        self._btn_set_reduced.setToolTip(
            'Captura el mismo AxisActual A1..A6 que SET y marca la velocidad '
            f'PTP del segmento entrante como {reduced_label} %.\n'
            'No consulta coordenadas cartesianas ni guarda en disco.'
        )
        self._btn_set_reduced.clicked.connect(self._on_set_reduced_point)
        row_capture.addWidget(self._btn_set_reduced)

        self._lbl_points = QLabel('Puntos: 0')
        self._lbl_points.setStyleSheet(
            f'color: {ACCENT}; font-weight: bold; font-size: 12px;')
        row_capture.addWidget(self._lbl_points)

        self._btn_set_open = QPushButton('SET ABRIR GARRA')
        self._btn_set_open.setStyleSheet(_BTN_COMPACT)
        self._btn_set_open.setCursor(Qt.PointingHandCursor)
        self._btn_set_open.setToolTip(
            'Programa un evento "abrir garra" en el último punto SET.\n'
            'NO mueve la garra ahora.'
        )
        self._btn_set_open.clicked.connect(
            lambda: self._on_set_gripper('open'))
        row_capture.addWidget(self._btn_set_open)

        self._btn_set_close = QPushButton('SET CERRAR GARRA')
        self._btn_set_close.setStyleSheet(_BTN_COMPACT)
        self._btn_set_close.setCursor(Qt.PointingHandCursor)
        self._btn_set_close.setToolTip(
            'Programa un evento "cerrar garra" en el último punto SET.\n'
            'NO mueve la garra ahora.'
        )
        self._btn_set_close.clicked.connect(
            lambda: self._on_set_gripper('close'))
        row_capture.addWidget(self._btn_set_close)

        self._btn_clear = QPushButton('LIMPIAR')
        self._btn_clear.setStyleSheet(_BTN_COMPACT)
        self._btn_clear.setCursor(Qt.PointingHandCursor)
        self._btn_clear.setToolTip(
            'Vacía el buffer temporal de puntos y eventos de garra.')
        self._btn_clear.clicked.connect(self._on_clear)
        row_capture.addWidget(self._btn_clear)

        row_capture.addStretch(1)
        layout.addLayout(row_capture)

        # ── Fila 2: generación y ejecución ───────────────────────────
        row_actions = QHBoxLayout()
        row_actions.setSpacing(6)

        self._btn_send_points = QPushButton('ENVIAR PUNTOS (0)')
        self._btn_send_points.setStyleSheet(_BTN_PRIMARY)
        self._btn_send_points.setCursor(Qt.PointingHandCursor)
        self._btn_send_points.setToolTip(
            'Publica P1..PN y los eventos de garra al contenedor MoveIt2 '
            'en una ÚNICA solicitud.\nRequiere al menos 2 puntos.'
        )
        self._btn_send_points.clicked.connect(self._on_send_points)
        row_actions.addWidget(self._btn_send_points)

        self._btn_preview = QPushButton('PROBAR TRAYECTORIA')
        self._btn_preview.setStyleSheet(_BTN_COMPACT)
        self._btn_preview.setCursor(Qt.PointingHandCursor)
        self._btn_preview.setToolTip(
            'Reproduce el archivo en RViz. SOLO RViz: nunca envía comandos '
            'al KUKA real ni acciona la garra.'
        )
        self._btn_preview.clicked.connect(self._on_preview)
        row_actions.addWidget(self._btn_preview)

        self._btn_execute = QPushButton('ENVIAR TRAYECTORIA')
        self._btn_execute.setStyleSheet(_BTN_PHYSICAL)
        self._btn_execute.setCursor(Qt.PointingHandCursor)
        self._btn_execute.setToolTip(
            'EJECUCIÓN FÍSICA del archivo seleccionado en el KUKA real.\n'
            'Respeta safe_mode, allow_motion_commands y ENABLE MOVE.'
        )
        self._btn_execute.clicked.connect(self._on_execute)
        row_actions.addWidget(self._btn_execute)

        self._btn_stop = QPushButton('DETENER')
        self._btn_stop.setStyleSheet(_BTN_COMPACT)
        self._btn_stop.setCursor(Qt.PointingHandCursor)
        self._btn_stop.setToolTip('Detiene la secuencia en curso.')
        self._btn_stop.clicked.connect(self._on_stop)
        self._btn_stop.setEnabled(False)
        row_actions.addWidget(self._btn_stop)

        # Selector mutuamente excluyente. Por seguridad, Manual por defecto.
        self._radio_manual = QRadioButton('Manual')
        self._radio_manual.setChecked(True)
        self._radio_manual.setToolTip(
            'Se detiene al terminar CADA SEGMENTO entre puntos SET y pide '
            'confirmación. No pregunta en cada punto intermedio.'
        )
        self._radio_auto = QRadioButton('Automático')
        self._radio_auto.setToolTip(
            'Encadena todos los segmentos sin pedir confirmación.')

        self._mode_group = QButtonGroup(self)
        self._mode_group.setExclusive(True)
        self._mode_group.addButton(self._radio_manual)
        self._mode_group.addButton(self._radio_auto)
        self._radio_manual.toggled.connect(self._on_mode_changed)

        for widget in (self._radio_manual, self._radio_auto):
            widget.setStyleSheet(
                f'QRadioButton {{ color: {TEXT_PRI}; font-size: 12px; }}')
            row_actions.addWidget(widget)

        row_actions.addStretch(1)
        layout.addLayout(row_actions)

        # ── Estado ───────────────────────────────────────────────────
        self._lbl_status = QLabel('Secuencia vacía. Pulsa SET para capturar P1.')
        self._lbl_status.setWordWrap(True)
        self._lbl_status.setStyleSheet(
            f'color: {TEXT_SEC}; font-size: 11px;')
        layout.addWidget(self._lbl_status)

        # ── Log temporal ─────────────────────────────────────────────
        self._txt_log = QTextEdit()
        self._txt_log.setReadOnly(True)
        # El log crece a lo vertical y se queda con el espacio sobrante,
        # de modo que la linea de estado queda pegada a los botones.
        self._txt_log.setMinimumHeight(96)
        self._txt_log.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._txt_log.setStyleSheet(
            f'background-color: {PANEL_BG}; border: 1px solid {BORDER_CLR}; '
            f'border-radius: 4px; color: {TEXT_PRI}; '
            f'font-family: monospace; font-size: 11px;'
        )
        layout.addWidget(self._txt_log, 1)

    def _connect_signals(self) -> None:
        # El feedback del KUKA se escucha directamente del bridge existente:
        # una conexión más a la misma señal, sin tocar los slots actuales.
        self._kuka_bridge.feedback_received.connect(self._on_feedback)

        self._traj_bridge.generation_result_received.connect(
            self._on_generation_result)
        self._traj_bridge.preview_status_received.connect(
            self._on_preview_status)

        self._executor.log_line.connect(self._log)
        self._executor.status_changed.connect(self._set_status)
        self._executor.segment_completed.connect(self._on_segment_completed)
        self._executor.finished.connect(self._on_execution_finished)

    # ===================================================================
    # Enganche ROS2
    # ===================================================================

    def _housekeeping(self) -> None:
        """Enganche diferido de los tópicos y caducidad de la solicitud."""
        self._traj_bridge.ensure_attached()
        self._check_generation_timeout()

    def _check_generation_timeout(self) -> None:
        """Soltar una solicitud a la que MoveIt nunca respondió."""
        if self._pending_request_id is None:
            return
        if self._generation_timeout <= 0.0:
            return
        waited = time.monotonic() - self._request_sent_at
        if waited < self._generation_timeout:
            return

        request_id = self._pending_request_id
        self._pending_request_id = None
        self._log(
            f'Sin respuesta de MoveIt tras {waited:.0f} s '
            f'(request_id {request_id}). Nada se ha guardado.')
        self._set_status(
            f'MoveIt no respondió en {self._generation_timeout:.0f} s. '
            f'Los puntos siguen en memoria: puedes reintentar.', WARN_CLR)
        self._refresh_controls()

    # ===================================================================
    # Feedback del KUKA
    # ===================================================================

    @pyqtSlot(str)
    def _on_feedback(self, data: str) -> None:
        """
        Guardar el AxisActual REAL que llega por TCP/IP.

        Esta es la única fuente que usa SET. No se toca el modelo de la GUI
        ni sus targets: solo se lee lo que el bridge publica.
        """
        try:
            feedback = json.loads(data)
        except (json.JSONDecodeError, TypeError):
            return

        axis_actual = feedback.get('axis_actual') or {}
        if axis_actual:
            self._last_axis_actual = dict(axis_actual)
            self._last_feedback_at = time.monotonic()

        position_actual = feedback.get('position_actual') or {}
        self._last_position_actual = (
            dict(position_actual) if position_actual else None)

        self._bridge_safe_mode = bool(feedback.get('bridge_safe_mode', True))
        self._bridge_allow_motion = bool(
            feedback.get('bridge_allow_motion', False))

        self._executor.update_feedback(feedback)

    def _feedback_is_fresh(self) -> bool:
        """
        Reutiliza el mecanismo de caducidad que ya existe en el modelo y
        además comprueba el sello propio del panel.
        """
        model_fresh = True
        try:
            model_fresh = self._model.has_recent_feedback(
                self._feedback_timeout)
        except AttributeError:
            pass
        if self._last_feedback_at <= 0.0:
            return False
        own_fresh = (
            time.monotonic() - self._last_feedback_at) < self._feedback_timeout
        return bool(model_fresh and own_fresh)

    # ===================================================================
    # SET
    # ===================================================================

    def _on_set_point(self) -> None:
        """Capturar P_n desde AxisActual. Nunca desde la GUI ni desde RViz."""
        self._capture_point(
            self._ptp_velocity_normal_pct,
            self._last_position_actual,
        )

    def _on_set_reduced_point(self) -> None:
        """Capturar P_n con el perfil PTP reducido del segmento entrante."""
        self._capture_point(self._ptp_velocity_reduced_pct, None)

    def _capture_point(
        self,
        incoming_velocity_pct: float,
        cartesian_diagnostic: Optional[Dict[str, float]],
    ) -> None:
        """Implementación única compartida por los dos botones SET."""
        if not self._feedback_is_fresh():
            self._warn(
                'Sin feedback válido del KUKA',
                'No hay feedback reciente del robot por TCP/IP.\n\n'
                'El punto NO se ha guardado: SET solo captura AxisActual '
                'real, nunca los valores de la GUI.'
            )
            self._set_status('SET rechazado: feedback del KUKA desactualizado.',
                             ERROR_CLR)
            return

        ok, message = self._sequence.add_point_from_axis_actual(
            self._last_axis_actual,
            cartesian_diagnostic,
            incoming_kuka_ptp_velocity_pct=incoming_velocity_pct,
        )

        if not ok:
            self._warn('No se pudo capturar el punto', message)
            self._set_status(f'SET rechazado: {message}', ERROR_CLR)
            return

        self._log(message)
        self._set_status(
            f'{self._sequence.last_point_id} capturado desde AxisActual '
            f'(solo en memoria).', ACCENT2)
        self._refresh_controls()

    def _on_set_gripper(self, action: str) -> None:
        """
        Programar un evento de garra. NO mueve la garra.

        No se publica nada por el bridge y no se toca
        request_gripper_command() del modelo: esto es programación pura.
        """
        ok, message = self._sequence.add_gripper_event(action)
        if not ok:
            self._warn('No se pudo programar el evento de garra', message)
            self._set_status(message, ERROR_CLR)
            return

        self._log(message)
        self._set_status(
            f'{message} (programado, la garra NO se ha movido).', ACCENT2)
        self._refresh_controls()

    def _on_clear(self) -> None:
        if self._executor.is_running:
            self._warn('Secuencia en ejecución',
                       'Detén la ejecución antes de limpiar el buffer.')
            return
        if self._sequence.point_count == 0 and \
                self._sequence.gripper_event_count == 0:
            return
        reply = QMessageBox.question(
            self, 'Limpiar secuencia',
            'Se descartarán los puntos y eventos de garra en memoria.\n'
            '¿Continuar?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        self._sequence.clear()
        self._pending_request_id = None
        self._txt_log.clear()
        self._set_status('Buffer temporal vacío.')
        self._refresh_controls()

    # ===================================================================
    # ENVIAR PUNTOS
    # ===================================================================

    def _on_send_points(self) -> None:
        request_id, json_str, message = self._sequence.build_request_json()
        if json_str is None:
            self._warn('Faltan puntos', message)
            self._set_status(message, ERROR_CLR)
            return

        if not self._traj_bridge.ensure_attached():
            self._warn(
                'ROS2 no disponible',
                'Todavía no hay nodo ROS2 activo para publicar la solicitud.')
            return

        if not self._traj_bridge.publish_generation_request(json_str):
            self._warn('Error de publicación',
                       'No se pudo publicar la solicitud de generación.')
            return

        self._pending_request_id = request_id
        self._request_sent_at = time.monotonic()
        self._log(
            f'ENVIAR PUNTOS: {self._sequence.point_count} puntos, '
            f'{self._sequence.gripper_event_count} eventos de garra.')
        self._log(f'request_id = {request_id}')
        self._set_status(
            'Solicitud enviada a MoveIt2. Esperando trayectoria…', ACCENT)
        self._refresh_controls()

    # ===================================================================
    # Resultado de MoveIt
    # ===================================================================

    @pyqtSlot(str)
    def _on_generation_result(self, data: str) -> None:
        try:
            payload = json.loads(data)
        except (json.JSONDecodeError, TypeError) as error:
            self._log(f'Resultado ilegible de MoveIt: {error}')
            return

        if not isinstance(payload, dict):
            self._log('Resultado de MoveIt con formato inesperado.')
            return

        # Correlación por request_id: una respuesta de otra solicitud se
        # ignora en silencio, nunca se da por válida.
        incoming_id = payload.get('request_id')
        if self._pending_request_id is None:
            return
        if incoming_id != self._pending_request_id:
            self._log(
                f'Resultado ignorado: request_id {incoming_id} no es el de '
                f'la solicitud en curso.')
            return

        status = str(payload.get('status', '')).lower()
        if status != 'ok':
            failed_segment = payload.get('failed_segment', 'desconocido')
            reason = payload.get('error') or payload.get(
                'message') or 'sin motivo indicado'
            self._pending_request_id = None
            self._log(f'MoveIt devolvió error en {failed_segment}: {reason}')
            self._set_status(
                f'MoveIt falló en {failed_segment}: {reason}', ERROR_CLR)
            self._warn(
                'MoveIt no pudo generar la trayectoria',
                f'Segmento fallido: {failed_segment}\n\n{reason}\n\n'
                f'La secuencia NO se ha guardado como ejecutable.'
            )
            self._refresh_controls()
            return

        ok, error = validate_result_payload(payload, self._pending_request_id)
        if not ok:
            self._pending_request_id = None
            self._log(f'Resultado rechazado: {error}')
            self._set_status(f'Resultado inválido: {error}', ERROR_CLR)
            self._warn('Resultado de MoveIt inválido',
                       f'{error}\n\nLa secuencia NO se ha guardado.')
            self._refresh_controls()
            return

        self._pending_request_id = None
        document = build_storage_document(self._sequence, payload)
        path, write_error = trajectory_storage.save_sequence_document(
            document, self._trajectories_dir_cfg)

        if path is None:
            self._log(f'ERROR al guardar: {write_error}')
            self._set_status(f'ERROR al guardar: {write_error}', ERROR_CLR)
            self._warn(
                'No se pudo guardar la trayectoria',
                f'{write_error}\n\n'
                f'La trayectoria NO fue guardada.'
            )
            self._refresh_controls()
            return

        self._last_saved_file = path
        summary = document.get('summary', {})
        segments = summary.get('num_segments', 0)
        points = summary.get('num_trajectory_points', 0)

        text = (
            f'Trayectorias generadas y guardadas en:\n{path}\n\n'
            f'Segmentos: {segments}\n'
            f'Puntos totales de trayectoria: {points}'
        )
        self._log(f'Guardado: {path.name} — {segments} segmentos, '
                  f'{points} puntos.')
        self._set_status(
            f'Guardado en {path} · Segmentos: {segments} · '
            f'Puntos totales de trayectoria: {points}', ACCENT2)
        QMessageBox.information(self, 'Trayectoria guardada', text)
        self._refresh_controls()

    # ===================================================================
    # PROBAR TRAYECTORIA (solo RViz)
    # ===================================================================

    def _on_preview(self) -> None:
        """
        Previsualizar en RViz.

        Este método NO tiene acceso al bridge de movimiento: su única salida
        es el tópico de previsualización. No puede activar EnableMove, ni
        llamar al sender TCP/IP, ni mover la garra real.
        """
        path = self._last_saved_file
        if path is None or not path.is_file():
            path = self._ask_for_file('Selecciona la trayectoria a previsualizar')
            if path is None:
                return

        document, error = trajectory_storage.load_sequence_file(path)
        if document is None:
            self._warn('Archivo no válido', error)
            self._set_status(error, ERROR_CLR)
            return

        if not self._traj_bridge.ensure_attached():
            self._warn('ROS2 no disponible',
                       'Todavía no hay nodo ROS2 activo para previsualizar.')
            return

        self._preview_id = str(uuid.uuid4())
        request = {
            'schema_version': SCHEMA_VERSION,
            'preview_id': self._preview_id,
            'request_id': document.get('request_id'),
            'source_file': str(path),
            'trajectory': document,
        }

        if not self._traj_bridge.publish_preview_request(json.dumps(request)):
            self._warn('Error de publicación',
                       'No se pudo publicar la solicitud de previsualización.')
            return

        self._log(f'PROBAR TRAYECTORIA (solo RViz): {path.name}')
        self._set_status('Previsualizando trayectoria en RViz...', ACCENT)

    @pyqtSlot(str)
    def _on_preview_status(self, data: str) -> None:
        try:
            payload = json.loads(data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(payload, dict):
            return

        incoming_id = payload.get('preview_id')
        if (self._preview_id is not None and incoming_id is not None
                and incoming_id != self._preview_id):
            return

        status = str(payload.get('status', '')).lower()
        message = payload.get('message', '')

        if status in ('started', 'running', 'playing'):
            self._set_status('Previsualizando trayectoria en RViz...', ACCENT)
        elif status in ('finished', 'done', 'ok', 'completed'):
            self._set_status('Previsualización finalizada.', ACCENT2)
            self._log('Previsualización finalizada.')
        elif status in ('error', 'failed', 'rejected'):
            self._set_status(
                f'Error en la previsualización: {message}', ERROR_CLR)
            self._log(f'Previsualización con error: {message}')

    # ===================================================================
    # ENVIAR TRAYECTORIA (ejecución física)
    # ===================================================================

    def _on_execute(self) -> None:
        if self._executor.is_running:
            self._warn('Secuencia en ejecución',
                       'Ya hay una trayectoria ejecutándose.')
            return

        path = self._ask_for_file('Selecciona la trayectoria a EJECUTAR')
        if path is None:
            return

        document, error = trajectory_storage.load_sequence_file(path)
        if document is None:
            self._warn('Archivo no válido', error)
            self._set_status(error, ERROR_CLR)
            return

        blocked = self._safety_blockers()
        if blocked:
            self._warn(
                'Ejecución bloqueada por las protecciones actuales',
                'No se ha enviado ningún comando.\n\n' + '\n'.join(
                    f'  • {reason}' for reason in blocked)
            )
            self._set_status(
                'ENVIAR TRAYECTORIA bloqueado: ' + '; '.join(blocked),
                ERROR_CLR)
            return

        ok, reason = self._executor.preflight(document)
        if not ok:
            self._warn('La trayectoria no puede ejecutarse tal cual', reason)
            self._set_status(reason, ERROR_CLR)
            return

        summary = document.get('summary', {})
        manual = self._radio_manual.isChecked()
        reply = QMessageBox.warning(
            self,
            'Confirmar ejecución física',
            f'Se va a EJECUTAR EN EL KUKA REAL:\n\n'
            f'  Archivo: {path.name}\n'
            f'  Segmentos: {summary.get("num_segments", "?")}\n'
            f'  Puntos de trayectoria: '
            f'{summary.get("num_trajectory_points", "?")}\n'
            f'  Eventos de garra: {summary.get("num_gripper_events", 0)}\n'
            f'  Modo: {"MANUAL" if manual else "AUTOMÁTICO"}\n\n'
            f'El robot se moverá. ¿Continuar?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        self._executor.set_manual_mode(manual)
        started, message = self._executor.start(document, manual_mode=manual)
        if not started:
            self._warn('No se pudo iniciar la ejecución', message)
            return

        self._log(f'ENVIAR TRAYECTORIA: {path.name} '
                  f'({"manual" if manual else "automático"}).')
        self._refresh_controls()

    def _safety_blockers(self) -> list:
        """
        Comprobar las protecciones EXISTENTES. No hay ningún bypass.

        safe_mode y allow_motion_commands viven en el bridge
        (eki_axis_move_node) y llegan en cada feedback. ENABLE MOVE es el
        checkbox que la GUI ya tenía.
        """
        blockers = []

        if not self._feedback_is_fresh():
            blockers.append(
                'No hay feedback reciente del KUKA por TCP/IP.')

        if self._bridge_safe_mode:
            blockers.append(
                'safe_mode = true en el bridge: EnableMove se fuerza a 0.')

        if not self._bridge_allow_motion:
            blockers.append(
                'allow_motion_commands = false en el bridge: '
                'EnableMove se fuerza a 0.')

        try:
            if not self._model.get_enable_move():
                blockers.append(
                    'El checkbox ENABLE MOVE de la GUI está desactivado.')
        except AttributeError:
            pass

        return blockers

    def _on_stop(self) -> None:
        self._executor.cancel('Detenido desde la GUI.')
        self._refresh_controls()

    def _on_mode_changed(self, _checked: bool) -> None:
        self._executor.set_manual_mode(self._radio_manual.isChecked())

    # ── Callbacks del executor ───────────────────────────────────────

    def _on_segment_completed(
        self,
        index: int,
        segment_id: str,
        to_point: str,
        has_more: bool,
    ) -> None:
        if not self._radio_manual.isChecked() or not has_more:
            return

        next_index = index + 1
        next_id = self._executor.segment_id(next_index)
        next_from, next_to = self._executor.segment_endpoints(next_index)

        box = QMessageBox(self)
        box.setIcon(QMessageBox.Question)
        box.setWindowTitle(f'{segment_id} completada')
        box.setText(
            f'Trayectoria {segment_id} completada.\n'
            f'Robot en {to_point}.\n\n'
            f'¿Continuar con la trayectoria {next_id}: '
            f'{next_from} -> {next_to}?'
        )
        btn_continue = box.addButton('CONTINUAR', QMessageBox.AcceptRole)
        box.addButton('CANCELAR', QMessageBox.RejectRole)
        box.setDefaultButton(btn_continue)
        box.exec_()

        if box.clickedButton() is btn_continue:
            self._log(f'CONTINUAR con {next_id}.')
            self._executor.continue_sequence()
        else:
            self._log('CANCELAR: no se envía nada más.')
            self._executor.cancel('Cancelado por el operador entre segmentos.')
            self._refresh_controls()

    def _on_execution_finished(self, ok: bool, message: str) -> None:
        self._refresh_controls()
        if ok:
            QMessageBox.information(self, 'Secuencia completada', message)
        else:
            self._set_status(message, WARN_CLR)

    # ===================================================================
    # Utilidades de UI
    # ===================================================================

    def _ask_for_file(self, title: str) -> Optional[Path]:
        directory, error = trajectory_storage.ensure_trajectories_dir(
            self._trajectories_dir_cfg)
        if directory is None:
            self._warn('Carpeta de trayectorias no disponible', error)
            return None

        selected, _ = QFileDialog.getOpenFileName(
            self, title, str(directory),
            'Trayectorias KUKA (*.json);;Todos los archivos (*)',
        )
        if not selected:
            return None
        return Path(selected)

    def _refresh_controls(self) -> None:
        count = self._sequence.point_count
        running = self._executor.is_running

        waiting = self._pending_request_id is not None

        self._lbl_points.setText(f'Puntos: {count}')
        self._btn_send_points.setText(f'ENVIAR PUNTOS ({count})')

        # Mientras hay una solicitud en vuelo la captura se congela: el
        # archivo se construye a partir de esta misma secuencia cuando llega
        # el resultado, y no debe contener puntos que nunca se enviaron.
        busy = running or waiting

        self._btn_set.setEnabled(not busy)
        self._btn_set_reduced.setEnabled(not busy)
        self._btn_set_open.setEnabled(not busy and count > 0)
        self._btn_set_close.setEnabled(not busy and count > 0)
        self._btn_clear.setEnabled(not busy)
        self._btn_send_points.setEnabled(
            not running and not waiting and count >= 2)
        self._btn_preview.setEnabled(not running)
        self._btn_execute.setEnabled(not running)
        self._btn_stop.setEnabled(running)
        self._radio_manual.setEnabled(not running)
        self._radio_auto.setEnabled(not running)

    def _set_status(self, text: str, color: str = TEXT_SEC) -> None:
        self._lbl_status.setText(text)
        self._lbl_status.setStyleSheet(f'color: {color}; font-size: 11px;')

    def _log(self, line: str) -> None:
        self._txt_log.append(line)
        scrollbar = self._txt_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _warn(self, title: str, text: str) -> None:
        QMessageBox.warning(self, title, text)

    def _configured_ptp_velocity(self, key: str, default: float) -> float:
        value = self._config.get(key, default)
        if not is_finite_number(value) or not 0.0 < float(value) <= 100.0:
            raise ValueError(
                f'{key} debe ser mayor que 0 y menor o igual que 100.'
            )
        return float(value)

    # ===================================================================
    # Cierre
    # ===================================================================

    def shutdown(self) -> None:
        """Parar timers y la ejecución al cerrar la ventana."""
        if self._housekeeping_timer.isActive():
            self._housekeeping_timer.stop()
        if self._executor.is_running:
            self._executor.cancel('La GUI se está cerrando.')
