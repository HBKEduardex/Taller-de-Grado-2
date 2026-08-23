"""
trajectory_executor.py — Ejecución física de un archivo de trayectoria.

REUTILIZA por completo la infraestructura verificada que ya existe:

  * los objetivos articulares se escriben en el MISMO modelo
    (JointCommandModel / DualCommandModel) que usa el botón SEND;
  * el envío se hace con la MISMA función de la ventana que usa SEND, que
    a su vez llama a RosAxisMoveBridge.publish_command() y termina en
    build_axis_move_command_xml() dentro de kuka_eki_bridge;
  * la garra se acciona con la MISMA función de la ventana que usan los
    botones "Abrir garra"/"Cerrar garra" (GripperCommand 0/1).

No construye XML. No abre sockets. No crea nodos ROS2. No toca KRL, EKI,
SPS ni el protocolo.

Por qué punto a punto y no streaming
------------------------------------
XmlDualMove.src actúa sobre un Seq NUEVO exactamente una vez y ejecuta un
`PTP axisTarget` bloqueante, marcando después XD_MOVE_EXECUTED. El bucle del
intérprete de robot lleva un `WAIT SEC 0.1`. Además la memoria de recepción
de EthernetKRL cierra la conexión a los 16 elementos sin leer, y el bridge
ya frena mucho antes con su guard. En consecuencia, el único envío seguro es
UNO cada vez, esperando llegada real antes del siguiente. `time_from_start`
se conserva en el archivo pero no puede usarse para marcar el ritmo: el
controlador impone su propio tiempo de PTP.

La ejecución NO bloquea la GUI: es una máquina de estados dirigida por un
QTimer y por el feedback que ya llega del KUKA. No hay sleeps ni hilos
nuevos, igual que el resto del proyecto (AUTO y el hold de SEND ya usan
QTimer).
"""

import time
from typing import Callable, Dict, List, Optional, Tuple

try:
    from PyQt5.QtCore import QObject, QTimer, pyqtSignal
except ImportError as e:
    raise ImportError(
        'PyQt5 is required. Install with: sudo apt install python3-pyqt5'
    ) from e

from kuka_gui_control.trajectory_sequence_model import (
    AXES,
    GRIPPER_COMMAND_VALUE,
    is_finite_number,
)

# ---------------------------------------------------------------------------
# Valores por defecto de la ejecución
# ---------------------------------------------------------------------------

# Tolerancia de llegada por articulación. El PTP del KUKA es de parada
# exacta, así que el error residual es muy pequeño; el margen cubre el
# redondeo del XML (%.4f) y el muestreo de la telemetría (~6 Hz).
DEFAULT_ARRIVAL_TOLERANCE_DEG = 0.5

# Reenvío del MISMO objetivo mientras no se confirma llegada. Es el mismo
# mecanismo de "hold" que ya usa SEND para que el bridge no marque el
# comando como caducado (command_timeout_sec = 2.0 s en axis_move.yaml).
DEFAULT_RESEND_PERIOD_SEC = 0.5

# Tiempo máximo para alcanzar UN punto intermedio antes de abortar.
DEFAULT_POINT_TIMEOUT_SEC = 15.0

# Espera tras ordenar una acción de garra. No existe realimentación de
# estado de garra en el protocolo actual, así que se respeta un tiempo de
# asentamiento en lugar de inventar una confirmación que no llega.
DEFAULT_GRIPPER_SETTLE_SEC = 2.0

# Salto máximo permitido por ENVIAR TRAYECTORIA entre dos puntos
# consecutivos. Es independiente del max_delta_deg usado por SEND manual.
DEFAULT_MAX_DELTA_DEG = 10.0

# Archivos anteriores a execution_profile se ejecutan como trayectoria
# normal. Este valor no se aplica a SEND manual.
DEFAULT_KUKA_PTP_VELOCITY_PCT = 30.0

# Tiempo mínimo entre dos puntos consecutivos. Suelo de seguridad frente a
# trayectorias muy densas: la memoria de recepción de EthernetKRL cierra la
# conexión a los 16 elementos sin leer y el guard del bridge corta en 10, así
# que jamás se encadenan puntos más rápido de lo que el SPS puede drenar.
DEFAULT_MIN_POINT_PERIOD_SEC = 0.2

TICK_MS = 100


# ---------------------------------------------------------------------------
# Estados
# ---------------------------------------------------------------------------

STATE_IDLE = 'idle'
STATE_MOVING = 'moving'
STATE_GRIPPER = 'gripper'
STATE_WAIT_CONFIRM = 'wait_confirm'
STATE_DONE = 'done'
STATE_ABORTED = 'aborted'


class TrajectoryExecutor(QObject):
    """
    Ejecuta los segmentos de un archivo de trayectoria, en orden.

    Señales:
      log_line(str)              — línea para el log de la GUI
      status_changed(str)        — texto corto de estado
      segment_completed(int, str, str, bool)
            índice (0-based), id del segmento, id del punto final,
            y si queda algún segmento por ejecutar
      finished(bool, str)        — (ok, mensaje) al terminar o abortar
    """

    log_line = pyqtSignal(str)
    status_changed = pyqtSignal(str)
    segment_completed = pyqtSignal(int, str, str, bool)
    finished = pyqtSignal(bool, str)

    def __init__(
        self,
        model,
        joint_send_fn: Callable[[], None],
        gripper_send_fn: Callable[[int], None],
        config: Optional[dict] = None,
        parent=None,
    ):
        super().__init__(parent)
        self._model = model
        self._joint_send_fn = joint_send_fn
        self._gripper_send_fn = gripper_send_fn

        config = config or {}
        self._tolerance = float(config.get(
            'trajectory_arrival_tolerance_deg', DEFAULT_ARRIVAL_TOLERANCE_DEG))
        self._resend_period = float(config.get(
            'trajectory_resend_period_sec', DEFAULT_RESEND_PERIOD_SEC))
        self._point_timeout = float(config.get(
            'trajectory_point_timeout_sec', DEFAULT_POINT_TIMEOUT_SEC))
        self._gripper_settle = float(config.get(
            'trajectory_gripper_settle_sec', DEFAULT_GRIPPER_SETTLE_SEC))
        self._min_point_period = float(config.get(
            'trajectory_min_point_period_sec', DEFAULT_MIN_POINT_PERIOD_SEC))

        self._max_delta = float(config.get(
            'trajectory_max_delta_deg', DEFAULT_MAX_DELTA_DEG))
        self._default_ptp_velocity_pct = float(config.get(
            'trajectory_kuka_ptp_velocity_normal_pct',
            DEFAULT_KUKA_PTP_VELOCITY_PCT,
        ))
        if not 0.0 < self._default_ptp_velocity_pct <= 100.0:
            raise ValueError(
                'trajectory_kuka_ptp_velocity_normal_pct debe estar en '
                '(0, 100].')

        # ── Estado de ejecución ──────────────────────────────────────
        self._state = STATE_IDLE
        self._document: Optional[dict] = None
        self._segments: List[dict] = []
        self._gripper_events: Dict[str, List[str]] = {}
        self._manual_mode = True

        self._segment_index = 0
        self._point_index = 0
        self._current_target: Dict[str, float] = {}
        self._current_ptp_velocity_pct = self._default_ptp_velocity_pct
        self._point_started_at = 0.0
        self._last_send_at = 0.0
        self._gripper_queue: List[str] = []
        self._gripper_started_at = 0.0
        self._gripper_next_state = STATE_MOVING

        # Feedback más reciente del KUKA (AxisActual real).
        self._last_axis_actual: Dict[str, float] = {}
        self._last_axis_actual_at = 0.0
        self._move_executed_seen = 0

        # Robot/RxCounter: cuenta del SPS de comandos COMPLETOS sacados de la
        # memoria de recepción de EKI. Es el acuse de recibo explícito que el
        # bridge ya usa para su guard, así que aquí se reutiliza para no
        # adelantar un punto antes de que el KUKA haya leído el anterior.
        self._last_rx_counter: Optional[int] = None
        self._rx_counter_at_send: Optional[int] = None
        self._feedback_frames_at_send = 0
        self._feedback_frames = 0

        # Modo de objetivo original de la GUI, restaurado al terminar.
        self._saved_target_mode: Optional[str] = None

        self._timer = QTimer(self)
        self._timer.setInterval(TICK_MS)
        self._timer.timeout.connect(self._tick)

    # ── Estado público ───────────────────────────────────────────────

    @property
    def is_running(self) -> bool:
        return self._state in (STATE_MOVING, STATE_GRIPPER,
                               STATE_WAIT_CONFIRM)

    @property
    def state(self) -> str:
        return self._state

    @property
    def manual_mode(self) -> bool:
        return self._manual_mode

    @property
    def segment_count(self) -> int:
        return len(self._segments)

    def segment_endpoints(self, index: int) -> Tuple[str, str]:
        """(from_point, to_point) del segmento `index`, con respaldo."""
        return (
            self._segment_point_id(index, 'from_point', f'P{index + 1}'),
            self._segment_point_id(index, 'to_point', f'P{index + 2}'),
        )

    def segment_id(self, index: int) -> str:
        """Identificador del segmento `index` (T1, T2, ...)."""
        try:
            return self._segments[index].get('id') or f'T{index + 1}'
        except (AttributeError, IndexError, TypeError):
            return f'T{index + 1}'

    def set_manual_mode(self, manual: bool) -> None:
        """Manual = parar al final de cada SEGMENTO. Automático = seguir."""
        self._manual_mode = bool(manual)

    # ── Feedback del KUKA ────────────────────────────────────────────

    def update_feedback(self, feedback: dict) -> None:
        """
        Recibir el feedback JSON que ya publica el bridge TCP/IP.

        Solo se guarda AxisActual: es la única fuente autoritativa de dónde
        está realmente el robot.
        """
        axis_actual = feedback.get('axis_actual') or {}
        clean: Dict[str, float] = {}
        for axis in AXES:
            value = axis_actual.get(axis)
            if is_finite_number(value):
                clean[axis] = float(value)
        if len(clean) == len(AXES):
            self._last_axis_actual = clean
            self._last_axis_actual_at = time.monotonic()

        self._feedback_frames += 1

        rx_counter = feedback.get('rx_counter')
        if isinstance(rx_counter, int):
            if (self._last_rx_counter is not None
                    and rx_counter < self._last_rx_counter):
                # El SPS pone RxCounter a 0 al reabrir el canal: el acuse
                # anterior ya no significa nada.
                self._rx_counter_at_send = None
            self._last_rx_counter = rx_counter

        if feedback.get('move_executed'):
            self._move_executed_seen += 1

    # ── Validación previa ────────────────────────────────────────────

    def preflight(self, document: dict) -> Tuple[bool, str]:
        """
        Comprobar que el archivo se puede ejecutar TAL CUAL con el protocolo
        actual, antes de mover nada.

        Verifica:
          - hay feedback reciente del KUKA;
          - todos los puntos están dentro de los soft limits del modelo;
          - ningún salto entre puntos consecutivos supera el máximo
            configurado para trayectorias;
          - el robot está lo bastante cerca del primer punto del primer
            segmento como para que el primer PTP sea aceptado.
        """
        if not self._last_axis_actual:
            return False, (
                'No hay AxisActual del KUKA: no se puede validar la '
                'trayectoria contra la posición real.'
            )

        segments = document.get('segments', [])
        previous: Optional[List[float]] = [
            self._last_axis_actual[a] for a in AXES
        ]

        for seg_index, segment in enumerate(segments):
            label = segment.get('id') or f'T{seg_index + 1}'
            velocity_pct, velocity_error = self._segment_velocity_pct(segment)
            if velocity_error:
                return False, f'{label}: {velocity_error}'

            for pt_index, point in enumerate(
                    segment.get('trajectory_points', [])):
                positions = point.get('positions_deg')
                if not isinstance(positions, list) or len(positions) != len(AXES):
                    return False, (
                        f'{label}, punto {pt_index}: positions_deg inválido.'
                    )

                for axis, value in zip(AXES, positions):
                    if not self._model.is_in_limits(axis, float(value)):
                        low, high = self._model.get_limits(axis)
                        return False, (
                            f'{label}, punto {pt_index}: {axis}='
                            f'{float(value):.2f} fuera de los soft limits '
                            f'[{low:.1f}, {high:.1f}].'
                        )

                if previous is not None:
                    for axis, value, before in zip(AXES, positions, previous):
                        delta = abs(float(value) - float(before))
                        if delta > self._max_delta:
                            where = (
                                'respecto a la posición REAL del robot'
                                if seg_index == 0 and pt_index == 0
                                else 'respecto al punto anterior'
                            )
                            return False, (
                                f'{label}, punto {pt_index}: salto de '
                                f'{delta:.2f} deg en {axis} {where}, por '
                                f'encima del máximo de trayectoria '
                                f'configurado de {self._max_delta:.1f} deg.'
                                + (
                                    ' Acerca el robot al primer punto antes '
                                    'de ejecutar.'
                                    if seg_index == 0 and pt_index == 0 else ''
                                )
                            )
                previous = [float(v) for v in positions]

        return True, ''

    # ── Arranque ─────────────────────────────────────────────────────

    def start(
        self,
        document: dict,
        manual_mode: bool = True,
    ) -> Tuple[bool, str]:
        """
        Empezar la ejecución del documento ya validado.

        No genera nada nuevo ni vuelve a llamar a MoveIt: ejecuta exactamente
        lo que hay en el archivo.
        """
        if self.is_running:
            return False, 'Ya hay una trayectoria en ejecución.'

        segments = document.get('segments', [])
        if not segments:
            return False, 'El archivo no trae segmentos.'

        self._document = document
        self._segments = segments
        self._manual_mode = bool(manual_mode)
        self._gripper_events = self._index_gripper_events(document)

        self._segment_index = 0
        self._point_index = 0
        self._move_executed_seen = 0
        self._gripper_queue = []
        self._rx_counter_at_send = None

        # La ejecución es SIEMPRE articular. Se guarda el modo que tenía la
        # GUI para devolverlo intacto al terminar.
        self._saved_target_mode = self._model.get_target_mode()
        self._model.set_target_mode('AxisTarget')

        total = len(self._segments)
        initial_state = document.get('gripper', {}).get(
            'initial_state', 'open')
        self.log_line.emit(
            f'Ejecutando archivo: {total} segmentos, garra inicial = '
            f'{initial_state}.'
        )

        # Eventos de garra anclados al PRIMER punto SET: no hay ningún
        # segmento que termine en él, así que se ejecutan antes de T1.
        first_point = self._segment_point_id(0, 'from_point', 'P1')
        pending = self._gripper_events.get(first_point, [])
        if pending:
            self._gripper_queue = list(pending)
            self._begin_gripper_action(next_state_after=STATE_MOVING)
        else:
            self._begin_point()

        self._timer.start()
        return True, f'Ejecución iniciada ({total} segmentos).'

    def cancel(self, reason: str = 'Cancelado por el operador.') -> None:
        """Detener la secuencia. No se envía ningún comando más."""
        if not self.is_running:
            return
        self._stop_timer()
        self._restore_target_mode()
        self._state = STATE_ABORTED
        self.log_line.emit(f'Secuencia detenida: {reason}')
        self.status_changed.emit('Secuencia detenida.')
        self.finished.emit(False, reason)

    def continue_sequence(self) -> None:
        """Continuar con el siguiente segmento tras la pausa del modo manual."""
        if self._state != STATE_WAIT_CONFIRM:
            return
        self._segment_index += 1
        self._point_index = 0
        self._begin_point()

    # ── Máquina de estados ───────────────────────────────────────────

    def _tick(self) -> None:
        if self._state == STATE_MOVING:
            self._tick_moving()
        elif self._state == STATE_GRIPPER:
            self._tick_gripper()

    def _tick_moving(self) -> None:
        now = time.monotonic()

        if self._has_arrived():
            self._on_point_reached()
            return

        if now - self._point_started_at > self._point_timeout:
            segment = self._segments[self._segment_index]
            label = segment.get('id') or f'T{self._segment_index + 1}'
            self._fail(
                f'{label}: el punto {self._point_index} no se alcanzó en '
                f'{self._point_timeout:.0f} s. Revisa EnableMove, safe_mode, '
                f'allow_motion_commands y el estado del KUKA.'
            )
            return

        # Reenvío del mismo objetivo: el bridge caduca los comandos a los
        # 2 s (command_timeout_sec) y este es el mismo "hold" que usa SEND.
        if now - self._last_send_at >= self._resend_period:
            self._publish_current_target()

    def _tick_gripper(self) -> None:
        now = time.monotonic()

        if now - self._gripper_started_at >= self._gripper_settle:
            self._gripper_queue.pop(0)
            if self._gripper_queue:
                self._begin_gripper_action(self._gripper_next_state)
            else:
                self._after_gripper_queue()
            return

        if now - self._last_send_at >= self._resend_period:
            self._publish_current_gripper()

    # ── Movimiento ───────────────────────────────────────────────────

    def _begin_point(self) -> None:
        """Ordenar el punto intermedio actual del segmento actual."""
        if self._segment_index >= len(self._segments):
            self._complete()
            return

        segment = self._segments[self._segment_index]
        points = segment.get('trajectory_points', [])

        velocity_pct, velocity_error = self._segment_velocity_pct(segment)
        if velocity_error:
            label = segment.get('id') or f'T{self._segment_index + 1}'
            self._fail(f'{label}: {velocity_error}')
            return
        self._current_ptp_velocity_pct = velocity_pct

        if self._point_index >= len(points):
            self._on_segment_finished()
            return

        positions = points[self._point_index].get('positions_deg')
        if not isinstance(positions, list) or len(positions) != len(AXES):
            label = segment.get('id') or f'T{self._segment_index + 1}'
            self._fail(
                f'{label}, punto {self._point_index}: positions_deg inválido.')
            return
        self._current_target = {
            axis: float(value) for axis, value in zip(AXES, positions)
        }

        label = segment.get('id') or f'T{self._segment_index + 1}'
        if self._point_index == 0:
            self.status_changed.emit(
                f'Ejecutando {label} '
                f'({self._segment_index + 1}/{len(self._segments)}) — '
                f'{len(points)} puntos'
            )
            self.log_line.emit(
                f'{label}: {self._segment_index + 1}/{len(self._segments)} — '
                f'{len(points)} puntos intermedios — '
                f'PTP {self._current_ptp_velocity_pct:g} %.'
            )

        self._state = STATE_MOVING
        self._point_started_at = time.monotonic()
        self._publish_current_target()

    def _publish_current_target(self) -> None:
        """
        Escribir el objetivo en el modelo y enviarlo por el camino de SEND.

        Los objetivos se reescriben en CADA envío a propósito: la GUI dual
        sincroniza los targets con la posición real cuando está en reposo, y
        así ese seguimiento no puede pisar el punto que se está ordenando.
        """
        for axis, value in self._current_target.items():
            self._model.set_target(axis, value)
        self._model.set_target_mode('AxisTarget')
        self._model.request_trajectory_ptp_velocity_pct(
            self._current_ptp_velocity_pct)
        try:
            self._joint_send_fn()
        finally:
            self._model.clear_trajectory_ptp_velocity_pct()
        self._last_send_at = time.monotonic()
        self._rx_counter_at_send = self._last_rx_counter
        self._feedback_frames_at_send = self._feedback_frames

    def _has_arrived(self) -> bool:
        """
        True cuando el punto puede darse por hecho.

        Exige TRES cosas, no solo la posición:

          1. Un suelo de tiempo por punto. Impide encadenar cientos de
             puntos densos más rápido de lo que el SPS drena su memoria de
             recepción, que es exactamente lo que cierra la conexión EKI.
          2. Acuse de recibo del KUKA. Robot/RxCounter es la cuenta del
             propio SPS de comandos completos leídos; si no ha subido desde
             que se publicó, el controlador todavía no ha visto el punto.
             Si el controlador no publica RxCounter, se cae al respaldo de
             haber recibido al menos una trama de telemetría posterior.
          3. AxisActual dentro de tolerancia del objetivo.
        """
        if not self._last_axis_actual or not self._current_target:
            return False

        now = time.monotonic()
        if now - self._point_started_at < self._min_point_period:
            return False

        if not self._command_acknowledged():
            return False

        for axis, target in self._current_target.items():
            actual = self._last_axis_actual.get(axis)
            if actual is None:
                return False
            if abs(actual - target) > self._tolerance:
                return False
        return True

    def _command_acknowledged(self) -> bool:
        """¿Ha leído el KUKA algún comando desde el último envío?"""
        if self._last_rx_counter is not None \
                and self._rx_counter_at_send is not None:
            return self._last_rx_counter > self._rx_counter_at_send
        # Respaldo para un controlador que aún no publica RxCounter: al
        # menos una trama de telemetría posterior al envío.
        return self._feedback_frames > self._feedback_frames_at_send

    def _on_point_reached(self) -> None:
        self._point_index += 1
        segment = self._segments[self._segment_index]
        points = segment.get('trajectory_points', [])
        if self._point_index >= len(points):
            self._on_segment_finished()
        else:
            self._begin_point()

    # ── Fin de segmento ──────────────────────────────────────────────

    def _on_segment_finished(self) -> None:
        segment = self._segments[self._segment_index]
        label = segment.get('id') or f'T{self._segment_index + 1}'
        to_point = self._segment_point_id(
            self._segment_index, 'to_point', f'P{self._segment_index + 2}')

        self.log_line.emit(f'{label} completada. Robot en {to_point}.')

        pending = self._gripper_events.get(to_point, [])
        if pending:
            self._gripper_queue = list(pending)
            self._begin_gripper_action(next_state_after='segment_boundary')
            return

        self._segment_boundary()

    def _segment_boundary(self) -> None:
        """Decidir qué pasa al terminar un segmento y sus eventos de garra."""
        is_last = self._segment_index >= len(self._segments) - 1
        segment = self._segments[self._segment_index]
        label = segment.get('id') or f'T{self._segment_index + 1}'
        to_point = self._segment_point_id(
            self._segment_index, 'to_point', f'P{self._segment_index + 2}')

        if is_last:
            self._complete()
            return

        if self._manual_mode:
            self._state = STATE_WAIT_CONFIRM
            self.status_changed.emit(
                f'{label} completada. Esperando confirmación del operador.')
            self.segment_completed.emit(
                self._segment_index, label, to_point, True)
            return

        # Modo automático: encadenar sin preguntar.
        self.segment_completed.emit(
            self._segment_index, label, to_point, True)
        self._segment_index += 1
        self._point_index = 0
        self._begin_point()

    # ── Garra ────────────────────────────────────────────────────────

    def _begin_gripper_action(self, next_state_after) -> None:
        """
        Ordenar la acción de garra usando la implementación ACTUAL.

        No se reimplementa nada: se llama a la misma función de la ventana
        que usan los botones de garra, que publica GripperCommand 0/1 con
        EnableMove=false por el bridge TCP/IP de siempre.
        """
        self._gripper_next_state = next_state_after
        action = self._gripper_queue[0]
        verb = 'ABRIR' if action == 'open' else 'CERRAR'
        self.log_line.emit(f'Garra: {verb} (ejecución física).')
        self.status_changed.emit(f'Accionando garra: {verb}…')
        self._state = STATE_GRIPPER
        self._gripper_started_at = time.monotonic()
        self._publish_current_gripper()

    def _publish_current_gripper(self) -> None:
        action = self._gripper_queue[0]
        self._gripper_send_fn(GRIPPER_COMMAND_VALUE[action])
        self._last_send_at = time.monotonic()

    def _after_gripper_queue(self) -> None:
        if self._gripper_next_state == STATE_MOVING:
            self._begin_point()
        else:
            self._segment_boundary()

    # ── Terminación ──────────────────────────────────────────────────

    def _complete(self) -> None:
        self._stop_timer()
        self._restore_target_mode()
        self._state = STATE_DONE
        total = len(self._segments)
        self.log_line.emit(f'Secuencia completada: {total} segmentos.')
        self.status_changed.emit('Secuencia completada.')
        self.finished.emit(True, f'Secuencia completada ({total} segmentos).')

    def _fail(self, reason: str) -> None:
        self._stop_timer()
        self._restore_target_mode()
        self._state = STATE_ABORTED
        self.log_line.emit(f'ERROR: {reason}')
        self.status_changed.emit('Secuencia abortada.')
        self.finished.emit(False, reason)

    def _stop_timer(self) -> None:
        if self._timer.isActive():
            self._timer.stop()

    def _restore_target_mode(self) -> None:
        if self._saved_target_mode is not None:
            self._model.set_target_mode(self._saved_target_mode)
            self._saved_target_mode = None

    # ── Utilidades ───────────────────────────────────────────────────

    @staticmethod
    def _index_gripper_events(document: dict) -> Dict[str, List[str]]:
        """Agrupar los eventos de garra por punto, conservando el orden."""
        events: Dict[str, List[str]] = {}
        for event in document.get('gripper', {}).get('events', []):
            at_point = event.get('at_point')
            action = event.get('action')
            if not at_point or action not in ('open', 'close'):
                continue
            events.setdefault(at_point, []).append(action)
        return events

    def _segment_point_id(self, index: int, key: str, fallback: str) -> str:
        try:
            value = self._segments[index].get(key)
        except (IndexError, AttributeError):
            value = None
        return value or fallback

    def _segment_velocity_pct(
        self,
        segment: dict,
    ) -> Tuple[Optional[float], Optional[str]]:
        """Leer execution_profile con compatibilidad para archivos antiguos."""
        profile = segment.get('execution_profile')
        if profile is None:
            return self._default_ptp_velocity_pct, None
        if not isinstance(profile, dict):
            return None, 'execution_profile inválido.'

        value = profile.get('kuka_ptp_velocity_pct')
        if not is_finite_number(value):
            return None, 'kuka_ptp_velocity_pct no es un número finito.'

        velocity_pct = float(value)
        if not 0.0 < velocity_pct <= 100.0:
            return None, (
                'kuka_ptp_velocity_pct debe ser mayor que 0 y menor o igual '
                'que 100.'
            )
        return velocity_pct, None
