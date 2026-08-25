"""
trajectory_batch_panel.py — panel del modo LOTE (batch).

Widget COMPARTIDO por las dos GUIs, igual que trajectory_panel.py, pero
completamente independiente de él: no lo importa, no hereda de él y no
comparte ninguna rama de código. El panel base y su botón
ENVIAR TRAYECTORIA quedan exactamente como están.

Interfaces donde se añade:
  - gui_axis_move_window.py   -> AxisMoveGuiWindow   (UI original)
  - dual_kuka_rviz_window.py  -> DualKukaRvizWindow  (UI dual)

Controles:
    ENVIAR TRAYECTORIA OPTIMIZADA · DETENER · (•) Manual ( ) Automático
    estado + log

Es un fork fiel del botón actual: mismo diálogo de confirmación, mismo
preflight (extendido para validar el lote completo), mismo modo
manual/automático por segmento, mismo DETENER. Lo único distinto es la ruta
de envío, que usa el pipeline de lotes.

REQUISITO: el controlador debe tener cargados los archivos _better
(XmlDualMove_better.xml, sps_submit_better.sub, config_submit_better.dat) y
XmlDualMove_better.src seleccionado, y en Ubuntu debe correr
eki_axis_move_better_node en lugar del nodo base. Si el KUKA no publica
Robot/BatchSeq, el preflight lo detecta y se niega a mover.
"""

import json
import time
from pathlib import Path
from typing import Callable, Dict, Optional

try:
    from PyQt5.QtCore import Qt, pyqtSlot
    from PyQt5.QtWidgets import (
        QButtonGroup, QFileDialog, QGroupBox, QHBoxLayout, QLabel,
        QMessageBox, QPushButton, QRadioButton, QTextEdit, QVBoxLayout,
    )
except ImportError as e:
    raise ImportError(
        'PyQt5 is required. Install with: sudo apt install python3-pyqt5'
    ) from e

from kuka_gui_control.trajectory_batch_executor import TrajectoryBatchExecutor
from kuka_gui_control.trajectory_batch_model import DEFAULT_MAX_BATCH_SIZE
from kuka_gui_control import trajectory_storage

# ---------------------------------------------------------------------------
# Estilo (mismos colores que el resto de la GUI)
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

_BTN_PHYSICAL = f"""
QPushButton {{
    background-color: #8957e5;
    color: #fff;
    border: none;
    border-radius: 6px;
    padding: 5px 12px;
    font-weight: bold;
    font-size: 12px;
}}
QPushButton:hover {{ background-color: #7048c8; }}
QPushButton:disabled {{ background-color: {BORDER_CLR}; color: {TEXT_SEC}; }}
"""


class TrajectoryBatchPanel(QGroupBox):
    """
    Panel del modo lote. Se añade DEBAJO del panel de secuencias existente.

    Args:
        model:            JointCommandModel/DualCommandModel de la ventana.
        kuka_bridge:      RosAxisMoveBridge YA existente. Se reutiliza su
                          publish_command() y su señal feedback_received.
        config:           dict de configuración de la ventana.
        gripper_send_fn:  la MISMA función de la ventana que usan los botones
                          de garra.
    """

    def __init__(
        self,
        model,
        kuka_bridge,
        config: dict,
        gripper_send_fn: Callable[[int], None],
        parent=None,
    ):
        super().__init__('Trayectorias — Modo LOTE (optimizado)', parent)

        self._model = model
        self._kuka_bridge = kuka_bridge
        self._config = config or {}

        self._executor = TrajectoryBatchExecutor(
            model=model,
            publish_json_fn=kuka_bridge.publish_command,
            gripper_send_fn=gripper_send_fn,
            config=self._config,
        )

        self._trajectories_dir_cfg = self._config.get('trajectories_dir', '')
        self._feedback_timeout = float(
            self._config.get('feedback_timeout_sec', 2.0))
        self._max_batch_size = int(self._config.get(
            'trajectory_batch_max_size', DEFAULT_MAX_BATCH_SIZE))

        self._last_feedback_at = 0.0
        self._bridge_safe_mode = True
        self._bridge_allow_motion = False
        self._batch_supported: Optional[bool] = None

        self._build_ui()
        self._connect_signals()
        self._refresh_controls()

    # ===================================================================
    # UI
    # ===================================================================

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(6)
        layout.setContentsMargins(8, 6, 8, 6)

        row = QHBoxLayout()
        row.setSpacing(6)

        self._btn_execute = QPushButton('ENVIAR TRAYECTORIA OPTIMIZADA')
        self._btn_execute.setStyleSheet(_BTN_PHYSICAL)
        self._btn_execute.setCursor(Qt.PointingHandCursor)
        self._btn_execute.setToolTip(
            'EJECUCIÓN FÍSICA por LOTES en el KUKA real.\n'
            f'Manda bloques de hasta {self._max_batch_size} puntos, que el '
            'KUKA ejecuta sin esperar red entre ellos.\n'
            'Cada punto sigue siendo un PTP de parada exacta.\n'
            'Respeta safe_mode, allow_motion_commands y ENABLE MOVE.\n'
            'Requiere los archivos _better cargados en el controlador.'
        )
        self._btn_execute.clicked.connect(self._on_execute)
        row.addWidget(self._btn_execute)

        self._btn_stop = QPushButton('DETENER')
        self._btn_stop.setStyleSheet(_BTN_COMPACT)
        self._btn_stop.setCursor(Qt.PointingHandCursor)
        self._btn_stop.setToolTip(
            'Levanta AbortBatch en el KUKA.\n'
            'El robot TERMINA el punto en curso y no arranca el siguiente.\n'
            'Nunca interrumpe un PTP a mitad de movimiento.'
        )
        self._btn_stop.clicked.connect(self._on_stop)
        self._btn_stop.setEnabled(False)
        row.addWidget(self._btn_stop)

        self._radio_manual = QRadioButton('Manual')
        self._radio_manual.setChecked(True)
        self._radio_manual.setToolTip(
            'Se detiene al terminar CADA SEGMENTO entre puntos SET, sin '
            'importar en cuántos lotes se troceó internamente.')
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
            row.addWidget(widget)

        row.addStretch(1)
        layout.addLayout(row)

        self._lbl_status = QLabel(
            'Modo lote inactivo. Requiere XmlDualMove_better.src en el KUKA.')
        self._lbl_status.setWordWrap(True)
        self._lbl_status.setStyleSheet(f'color: {TEXT_SEC}; font-size: 11px;')
        layout.addWidget(self._lbl_status)

        self._txt_log = QTextEdit()
        self._txt_log.setReadOnly(True)
        self._txt_log.setMaximumHeight(84)
        self._txt_log.setStyleSheet(
            f'background-color: {PANEL_BG}; border: 1px solid {BORDER_CLR}; '
            f'border-radius: 4px; color: {TEXT_PRI}; '
            f'font-family: monospace; font-size: 11px;')
        layout.addWidget(self._txt_log)

    def _connect_signals(self) -> None:
        # Una conexión más a la MISMA señal del bridge existente. Los slots
        # actuales no se tocan.
        self._kuka_bridge.feedback_received.connect(self._on_feedback)

        self._executor.log_line.connect(self._log)
        self._executor.status_changed.connect(self._set_status)
        self._executor.segment_completed.connect(self._on_segment_completed)
        self._executor.finished.connect(self._on_execution_finished)

    # ===================================================================
    # Feedback
    # ===================================================================

    @pyqtSlot(str)
    def _on_feedback(self, data: str) -> None:
        try:
            feedback = json.loads(data)
        except (json.JSONDecodeError, TypeError):
            return

        if feedback.get('axis_actual'):
            self._last_feedback_at = time.monotonic()

        self._bridge_safe_mode = bool(feedback.get('bridge_safe_mode', True))
        self._bridge_allow_motion = bool(
            feedback.get('bridge_allow_motion', False))

        if self._batch_supported is None:
            if feedback.get('batch_seq') is not None:
                self._batch_supported = True
                self._set_status(
                    'Modo lote disponible: el KUKA publica Robot/BatchSeq.',
                    ACCENT2)

        self._executor.update_feedback(feedback)

    def _feedback_is_fresh(self) -> bool:
        model_fresh = True
        try:
            model_fresh = self._model.has_recent_feedback(
                self._feedback_timeout)
        except AttributeError:
            pass
        if self._last_feedback_at <= 0.0:
            return False
        own = (time.monotonic() - self._last_feedback_at) < self._feedback_timeout
        return bool(model_fresh and own)

    # ===================================================================
    # ENVIAR TRAYECTORIA OPTIMIZADA
    # ===================================================================

    def _on_execute(self) -> None:
        if self._executor.is_running:
            self._warn('Secuencia en ejecución',
                       'Ya hay una trayectoria ejecutándose.')
            return

        path = self._ask_for_file(
            'Selecciona la trayectoria a EJECUTAR por lotes')
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
                    f'  • {reason}' for reason in blocked))
            self._set_status(
                'Modo lote bloqueado: ' + '; '.join(blocked), ERROR_CLR)
            return

        # Preflight del LOTE COMPLETO: valida todos los puntos de todos los
        # lotes antes de enviar el primero.
        ok, reason = self._executor.preflight(document)
        if not ok:
            self._warn('La trayectoria no puede ejecutarse por lotes', reason)
            self._set_status(reason, ERROR_CLR)
            return

        summary = document.get('summary', {})
        manual = self._radio_manual.isChecked()
        total_points = summary.get('num_trajectory_points', 0)
        try:
            approx_batches = -(-int(total_points) // self._max_batch_size)
        except (TypeError, ValueError):
            approx_batches = '?'

        reply = QMessageBox.warning(
            self,
            'Confirmar ejecución física por LOTES',
            f'Se va a EJECUTAR EN EL KUKA REAL en modo LOTE:\n\n'
            f'  Archivo: {path.name}\n'
            f'  Segmentos: {summary.get("num_segments", "?")}\n'
            f'  Puntos de trayectoria: {total_points}\n'
            f'  Tamaño de lote: {self._max_batch_size} '
            f'(~{approx_batches} lotes)\n'
            f'  Eventos de garra: {summary.get("num_gripper_events", 0)}\n'
            f'  Modo: {"MANUAL" if manual else "AUTOMÁTICO"}\n\n'
            f'El KUKA ejecutará cada lote sin esperar red entre puntos.\n'
            f'DETENER actúa al terminar el punto en curso.\n\n'
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

        self._log(f'ENVIAR TRAYECTORIA OPTIMIZADA: {path.name} '
                  f'({"manual" if manual else "automático"}).')
        self._refresh_controls()

    def _safety_blockers(self) -> list:
        """Las MISMAS protecciones que el modo base. Ningún bypass."""
        blockers = []

        if not self._feedback_is_fresh():
            blockers.append('No hay feedback reciente del KUKA por TCP/IP.')

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

        if self._batch_supported is not True:
            blockers.append(
                'El KUKA no publica Robot/BatchSeq: faltan los archivos '
                '_better en el controlador, o corre el nodo bridge base.')

        return blockers

    def _on_stop(self) -> None:
        self._executor.cancel('Detenido desde la GUI (modo lote).')
        self._refresh_controls()

    def _on_mode_changed(self, _checked: bool) -> None:
        self._executor.set_manual_mode(self._radio_manual.isChecked())

    # ===================================================================
    # Callbacks del executor
    # ===================================================================

    def _on_segment_completed(self, index: int, segment_id: str,
                              to_point: str, has_more: bool) -> None:
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
            f'{next_from} -> {next_to}?')
        btn_continue = box.addButton('CONTINUAR', QMessageBox.AcceptRole)
        box.addButton('CANCELAR', QMessageBox.RejectRole)
        box.setDefaultButton(btn_continue)
        box.exec_()

        if box.clickedButton() is btn_continue:
            self._log(f'CONTINUAR con {next_id}.')
            self._executor.continue_sequence()
        else:
            self._log('CANCELAR: no se envía nada más.')
            self._executor.cancel(
                'Cancelado por el operador entre segmentos.')
            self._refresh_controls()

    def _on_execution_finished(self, ok: bool, message: str) -> None:
        self._refresh_controls()
        if ok:
            QMessageBox.information(self, 'Secuencia completada', message)
        else:
            self._set_status(message, WARN_CLR)

    # ===================================================================
    # Utilidades
    # ===================================================================

    def _ask_for_file(self, title: str) -> Optional[Path]:
        directory, error = trajectory_storage.ensure_trajectories_dir(
            self._trajectories_dir_cfg)
        if directory is None:
            self._warn('Carpeta de trayectorias no disponible', error)
            return None
        selected, _ = QFileDialog.getOpenFileName(
            self, title, str(directory),
            'Trayectorias KUKA (*.json);;Todos los archivos (*)')
        if not selected:
            return None
        return Path(selected)

    def _refresh_controls(self) -> None:
        running = self._executor.is_running
        self._btn_execute.setEnabled(not running)
        self._btn_stop.setEnabled(running)
        self._radio_manual.setEnabled(not running)
        self._radio_auto.setEnabled(not running)

    def _set_status(self, text: str, color: str = TEXT_SEC) -> None:
        self._lbl_status.setText(text)
        self._lbl_status.setStyleSheet(f'color: {color}; font-size: 11px;')

    def _log(self, line: str) -> None:
        self._txt_log.append(line)
        bar = self._txt_log.verticalScrollBar()
        bar.setValue(bar.maximum())

    def _warn(self, title: str, text: str) -> None:
        QMessageBox.warning(self, title, text)

    def shutdown(self) -> None:
        """Parar la ejecución al cerrar la ventana."""
        if self._executor.is_running:
            self._executor.cancel('La GUI se está cerrando.')
