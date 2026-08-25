"""
trajectory_batch_executor.py — ejecución física por LOTES.

Ruta NUEVA y paralela a trajectory_executor.py. No lo importa, no hereda de
él y no comparte ninguna rama de código: un cambio futuro en el modo
punto-a-punto no puede alterar este, ni al revés. El modo base sigue siendo
el comportamiento por defecto y no se ha tocado.

Diferencia con el modo base
---------------------------
El modo base manda UN punto, espera confirmación de llegada por red y recién
entonces manda el siguiente. Sobre 100+ puntos ese ida y vuelta es lo que
produce el patrón avanza-frena-avanza-frena.

Aquí se manda un LOTE de hasta max_batch_size puntos. XmlDualMove_better.src
lo copia a arrays locales, libera el mailbox y los ejecuta uno tras otro sin
red entre ellos. Mientras el robot se mueve, este ejecutor recarga el mailbox
con el siguiente sub-lote, así que tampoco hay hueco ENTRE lotes.

Cada punto sigue siendo un PTP de parada exacta. No cambia el tipo de
movimiento: solo desaparece la espera de red.

Lo que NO cambia respecto al modo base
--------------------------------------
  * la garra sigue anclada a los puntos P1..Pn del usuario (fronteras de
    segmento), nunca a los puntos internos del lote;
  * la pausa manual sigue siendo por SEGMENTO completo, sin importar en
    cuántos lotes se troceó internamente;
  * ENABLE MOVE, safe_mode, allow_motion_commands, soft limits,
    MAX_DELTA_JOINT y el feedback siguen exactamente igual;
  * no bloquea la GUI: máquina de estados sobre QTimer, sin sleeps.
"""

import time
from typing import Callable, Dict, List, Optional, Tuple

try:
    from PyQt5.QtCore import QObject, QTimer, pyqtSignal
except ImportError as e:
    raise ImportError(
        'PyQt5 is required. Install with: sudo apt install python3-pyqt5'
    ) from e

from kuka_gui_control.trajectory_batch_model import (
    AXES,
    DEFAULT_MAX_BATCH_SIZE,
    DEFAULT_REFILL_THRESHOLD,
    build_abort_batch_json,
    build_batch_command_json,
    is_finite_number,
    pack_segment_into_batches,
)

# ---------------------------------------------------------------------------
# Valores por defecto
# ---------------------------------------------------------------------------

DEFAULT_MAX_DELTA_DEG = 10.0
DEFAULT_KUKA_PTP_VELOCITY_PCT = 30.0
DEFAULT_GRIPPER_SETTLE_SEC = 2.0

# Sin progreso del contador de puntos consumidos durante este tiempo -> abortar.
DEFAULT_BATCH_STALL_TIMEOUT_SEC = 20.0

# Reenvío del lote mientras el KUKA no acusa haberlo recibido. Por debajo del
# command_timeout_sec de 2.0 s del bridge, igual que el "hold" de SEND.
DEFAULT_BATCH_RESEND_PERIOD_SEC = 0.5

TICK_MS = 100

STATE_IDLE = 'idle'
STATE_RUNNING = 'running'
STATE_GRIPPER = 'gripper'
STATE_WAIT_CONFIRM = 'wait_confirm'
STATE_DONE = 'done'
STATE_ABORTED = 'aborted'


class TrajectoryBatchExecutor(QObject):
    """
    Ejecuta los segmentos de un archivo de trayectoria en modo LOTE.

    Señales (idénticas en forma a las del ejecutor base, para que el panel
    nuevo pueda ser un fork fiel del actual):
      log_line(str)
      status_changed(str)
      segment_completed(int, str, str, bool)
      finished(bool, str)
    """

    log_line = pyqtSignal(str)
    status_changed = pyqtSignal(str)
    segment_completed = pyqtSignal(int, str, str, bool)
    finished = pyqtSignal(bool, str)

    def __init__(
        self,
        model,
        publish_json_fn: Callable[[str], None],
        gripper_send_fn: Callable[[int], None],
        config: Optional[dict] = None,
        parent=None,
    ):
        """
        Args:
            model:            JointCommandModel/DualCommandModel de la ventana.
                              Se usa solo para next_seq(), los soft limits y
                              get_enable_move(). No se tocan sus targets.
            publish_json_fn:  publica un JSON en el tópico de comandos que YA
                              existe (RosAxisMoveBridge.publish_command).
            gripper_send_fn:  la MISMA función de la ventana que usan los
                              botones de garra.
        """
        super().__init__(parent)
        self._model = model
        self._publish_json = publish_json_fn
        self._gripper_send_fn = gripper_send_fn

        config = config or {}
        self._max_batch_size = int(config.get(
            'trajectory_batch_max_size', DEFAULT_MAX_BATCH_SIZE))
        self._refill_threshold = float(config.get(
            'trajectory_batch_refill_threshold', DEFAULT_REFILL_THRESHOLD))
        self._max_delta = float(config.get(
            'trajectory_max_delta_deg', DEFAULT_MAX_DELTA_DEG))
        self._gripper_settle = float(config.get(
            'trajectory_gripper_settle_sec', DEFAULT_GRIPPER_SETTLE_SEC))
        self._stall_timeout = float(config.get(
            'trajectory_batch_stall_timeout_sec',
            DEFAULT_BATCH_STALL_TIMEOUT_SEC))
        self._resend_period = float(config.get(
            'trajectory_batch_resend_period_sec',
            DEFAULT_BATCH_RESEND_PERIOD_SEC))
        self._default_velocity = float(config.get(
            'trajectory_kuka_ptp_velocity_normal_pct',
            DEFAULT_KUKA_PTP_VELOCITY_PCT))

        if self._max_batch_size < 1:
            self._max_batch_size = DEFAULT_MAX_BATCH_SIZE
        if not 0.0 < self._refill_threshold <= 1.0:
            self._refill_threshold = DEFAULT_REFILL_THRESHOLD

        # ── Estado ───────────────────────────────────────────────────
        self._state = STATE_IDLE
        self._document: Optional[dict] = None
        self._segments: List[dict] = []
        self._gripper_events: Dict[str, List[str]] = {}
        self._manual_mode = True

        self._segment_index = 0
        self._batches: List[List[Dict[str, float]]] = []
        self._batch_cursor = 0          # próximo lote a enviar
        self._sent_points = 0           # puntos ya entregados al KUKA
        self._segment_points = 0
        self._segment_velocity = self._default_velocity

        self._batch_seq_counter = 0
        self._last_sent_batch_seq = 0
        self._last_send_at = 0.0
        self._last_progress_at = 0.0
        self._last_consumed_total = 0

        self._gripper_queue: List[str] = []
        self._gripper_started_at = 0.0
        self._gripper_next_state = STATE_RUNNING

        # Feedback del KUKA
        self._last_axis_actual: Dict[str, float] = {}
        self._robot_batch_seq: Optional[int] = None
        self._robot_batch_consumed: Optional[int] = None
        self._robot_batch_active: Optional[bool] = None
        self._batch_supported: Optional[bool] = None

        self._timer = QTimer(self)
        self._timer.setInterval(TICK_MS)
        self._timer.timeout.connect(self._tick)

    # ── Estado público ───────────────────────────────────────────────

    @property
    def is_running(self) -> bool:
        return self._state in (STATE_RUNNING, STATE_GRIPPER,
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

    def set_manual_mode(self, manual: bool) -> None:
        self._manual_mode = bool(manual)

    def segment_id(self, index: int) -> str:
        try:
            return self._segments[index].get('id') or f'T{index + 1}'
        except (AttributeError, IndexError, TypeError):
            return f'T{index + 1}'

    def segment_endpoints(self, index: int) -> Tuple[str, str]:
        return (self._segment_point_id(index, 'from_point', f'P{index + 1}'),
                self._segment_point_id(index, 'to_point', f'P{index + 2}'))

    # ── Feedback ─────────────────────────────────────────────────────

    def update_feedback(self, feedback: dict) -> None:
        """Guardar AxisActual y la telemetría de lote del KUKA."""
        axis_actual = feedback.get('axis_actual') or {}
        clean = {a: float(axis_actual[a]) for a in AXES
                 if a in axis_actual and is_finite_number(axis_actual[a])}
        if len(clean) == len(AXES):
            self._last_axis_actual = clean

        batch_seq = feedback.get('batch_seq')
        consumed = feedback.get('batch_consumed')
        active = feedback.get('batch_active')

        # batch_seq ausente/None => el controlador no tiene los archivos
        # _better cargados. Se detecta aquí y se rechaza antes de mover.
        if self._batch_supported is None and batch_seq is not None:
            self._batch_supported = True

        self._robot_batch_seq = batch_seq
        self._robot_batch_active = active

        if isinstance(consumed, int):
            total = self._consumed_total(consumed)
            if total > self._last_consumed_total:
                self._last_consumed_total = total
                self._last_progress_at = time.monotonic()
            self._robot_batch_consumed = consumed

    def _consumed_total(self, consumed_in_batch: int) -> int:
        """
        Puntos consumidos del segmento en total.

        XD_BATCH_CONSUMED_COUNT se reinicia en cada lote, así que hay que
        sumarle los puntos de los lotes ya terminados.
        """
        finished_before = 0
        for index in range(self._batch_cursor - 1):
            if index < len(self._batches):
                finished_before += len(self._batches[index])
        return finished_before + max(0, int(consumed_in_batch))

    # ── Preflight ────────────────────────────────────────────────────

    def preflight(self, document: dict) -> Tuple[bool, str]:
        """
        Validar el archivo ENTERO antes de enviar el primer lote.

        Mismos criterios que el modo base, aplicados a todos los puntos de
        todos los lotes: soft limits, salto <= trajectory_max_delta_deg entre
        puntos consecutivos, y distancia del primer punto a la posición real.
        Además comprueba que el troceado en lotes es válido.
        """
        if not self._last_axis_actual:
            return False, ('No hay AxisActual del KUKA: no se puede validar '
                           'la trayectoria contra la posición real.')

        if self._batch_supported is not True:
            return False, (
                'El controlador no está publicando Robot/BatchSeq: parece que '
                'no tiene cargados XmlDualMove_better.xml / '
                'sps_submit_better.sub. El modo lote no puede ejecutarse.')

        segments = document.get('segments', [])
        if not segments:
            return False, 'El archivo no trae segmentos.'

        previous: List[float] = [self._last_axis_actual[a] for a in AXES]

        for seg_index, segment in enumerate(segments):
            label = segment.get('id') or f'T{seg_index + 1}'

            velocity, error = self._segment_velocity_pct(segment)
            if error:
                return False, f'{label}: {error}'

            batches, pack_error = pack_segment_into_batches(
                segment, self._max_batch_size)
            if batches is None:
                return False, f'{label}: {pack_error}'

            point_index = 0
            for batch in batches:
                for point in batch:
                    for axis in AXES:
                        value = point[axis]
                        if not self._model.is_in_limits(axis, value):
                            low, high = self._model.get_limits(axis)
                            return False, (
                                f'{label}, punto {point_index}: {axis}='
                                f'{value:.2f} fuera de los soft limits '
                                f'[{low:.1f}, {high:.1f}].')

                    for axis, before in zip(AXES, previous):
                        delta = abs(point[axis] - before)
                        if delta > self._max_delta:
                            where = ('respecto a la posición REAL del robot'
                                     if seg_index == 0 and point_index == 0
                                     else 'respecto al punto anterior')
                            extra = (' Acerca el robot al primer punto antes '
                                     'de ejecutar.'
                                     if seg_index == 0 and point_index == 0
                                     else '')
                            return False, (
                                f'{label}, punto {point_index}: salto de '
                                f'{delta:.2f} deg en {axis} {where}, por '
                                f'encima del máximo de {self._max_delta:.1f} '
                                f'deg que aceptan el bridge y '
                                f'XmlDualMove_better.src.{extra}')
                    previous = [point[a] for a in AXES]
                    point_index += 1

        return True, ''

    # ── Arranque ─────────────────────────────────────────────────────

    def start(self, document: dict,
              manual_mode: bool = True) -> Tuple[bool, str]:
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
        self._gripper_queue = []

        total = len(segments)
        initial_state = document.get('gripper', {}).get('initial_state', 'open')
        self.log_line.emit(
            f'Modo LOTE: {total} segmentos, lotes de hasta '
            f'{self._max_batch_size} puntos, garra inicial = {initial_state}.')

        first_point = self._segment_point_id(0, 'from_point', 'P1')
        pending = self._gripper_events.get(first_point, [])
        if pending:
            self._gripper_queue = list(pending)
            self._begin_gripper_action(STATE_RUNNING)
        else:
            self._begin_segment()

        self._timer.start()
        return True, f'Ejecución por lotes iniciada ({total} segmentos).'

    def cancel(self, reason: str = 'Cancelado por el operador.') -> None:
        """
        Detener la secuencia.

        Levanta XD_ABORT_BATCH en el KUKA: el robot TERMINA el PTP en vuelo y
        no arranca el siguiente. Nunca se interrumpe un PTP a mitad.
        """
        if not self.is_running:
            return
        self._send_abort()
        self._stop_timer()
        self._state = STATE_ABORTED
        self.log_line.emit(f'Secuencia detenida: {reason}')
        self.status_changed.emit('Secuencia detenida (abort enviado al KUKA).')
        self.finished.emit(False, reason)

    def continue_sequence(self) -> None:
        if self._state != STATE_WAIT_CONFIRM:
            return
        self._segment_index += 1
        self._begin_segment()

    # ── Máquina de estados ───────────────────────────────────────────

    def _tick(self) -> None:
        if self._state == STATE_RUNNING:
            self._tick_running()
        elif self._state == STATE_GRIPPER:
            self._tick_gripper()

    def _tick_running(self) -> None:
        now = time.monotonic()
        consumed = self._last_consumed_total

        # ---- Segmento terminado ----
        if (self._batch_cursor >= len(self._batches)
                and consumed >= self._segment_points
                and self._robot_batch_active is not True):
            self._on_segment_finished()
            return

        # ---- Reponer antes de que se agote ----
        # XmlDualMove_better.src copia el lote a arrays locales al empezar, de
        # modo que el mailbox queda libre y aceptar el siguiente sub-lote
        # mientras el robot se mueve no pisa nada.
        if self._batch_cursor < len(self._batches):
            pending_points = self._sent_points - consumed
            current_batch_size = len(self._batches[self._batch_cursor - 1]) \
                if self._batch_cursor > 0 else 0
            threshold = max(1, int(current_batch_size * self._refill_threshold))

            if self._batch_cursor == 0 or pending_points <= threshold:
                if self._last_sent_batch_seq == 0 or \
                        self._robot_batch_seq == self._last_sent_batch_seq:
                    self._send_next_batch()
                    return

        # ---- Reenvío mientras el KUKA no acuse el lote ----
        if (self._last_sent_batch_seq > 0
                and self._robot_batch_seq != self._last_sent_batch_seq
                and now - self._last_send_at >= self._resend_period):
            self._resend_current_batch()
            return

        # ---- Atasco ----
        if now - self._last_progress_at > self._stall_timeout:
            label = self.segment_id(self._segment_index)
            self._fail(
                f'{label}: sin progreso durante {self._stall_timeout:.0f} s '
                f'({consumed}/{self._segment_points} puntos). Revisa '
                f'EnableMove, safe_mode, allow_motion_commands y que '
                f'XmlDualMove_better.src esté seleccionado.')

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

    # ── Segmentos y lotes ────────────────────────────────────────────

    def _begin_segment(self) -> None:
        if self._segment_index >= len(self._segments):
            self._complete()
            return

        segment = self._segments[self._segment_index]
        label = self.segment_id(self._segment_index)

        batches, error = pack_segment_into_batches(
            segment, self._max_batch_size)
        if batches is None:
            self._fail(f'{label}: {error}')
            return

        velocity, velocity_error = self._segment_velocity_pct(segment)
        if velocity_error:
            self._fail(f'{label}: {velocity_error}')
            return

        self._batches = batches
        self._batch_cursor = 0
        self._sent_points = 0
        self._segment_points = sum(len(b) for b in batches)
        self._segment_velocity = velocity
        self._last_consumed_total = 0
        self._last_sent_batch_seq = 0
        self._last_progress_at = time.monotonic()

        self._state = STATE_RUNNING
        self.status_changed.emit(
            f'Ejecutando {label} ({self._segment_index + 1}/'
            f'{len(self._segments)}) — {self._segment_points} puntos en '
            f'{len(batches)} lotes, PTP {velocity:g} %')
        self.log_line.emit(
            f'{label}: {self._segment_points} puntos en {len(batches)} lotes '
            f'(máx {self._max_batch_size}), PTP {velocity:g} %.')
        self._send_next_batch()

    def _send_next_batch(self) -> None:
        if self._batch_cursor >= len(self._batches):
            return
        batch = self._batches[self._batch_cursor]
        self._batch_seq_counter += 1
        self._last_sent_batch_seq = self._batch_seq_counter

        json_str = build_batch_command_json(
            seq=self._model.next_seq(),
            batch_seq=self._batch_seq_counter,
            points=batch,
            ptp_velocity_pct=self._segment_velocity,
            enable_move=True,
        )
        self._publish_json(json_str)
        self._last_send_at = time.monotonic()
        self._batch_cursor += 1
        self._sent_points += len(batch)

        self.log_line.emit(
            f'  lote {self._batch_cursor}/{len(self._batches)} enviado '
            f'({len(batch)} puntos, batch_seq={self._batch_seq_counter}).')

    def _resend_current_batch(self) -> None:
        """Reenviar el lote en vuelo con el MISMO batch_seq."""
        if self._batch_cursor < 1:
            return
        batch = self._batches[self._batch_cursor - 1]
        json_str = build_batch_command_json(
            seq=self._model.next_seq(),
            batch_seq=self._last_sent_batch_seq,
            points=batch,
            ptp_velocity_pct=self._segment_velocity,
            enable_move=True,
        )
        self._publish_json(json_str)
        self._last_send_at = time.monotonic()

    def _send_abort(self) -> None:
        try:
            self._publish_json(build_abort_batch_json(self._model.next_seq()))
        except Exception:
            pass

    # ── Fin de segmento ──────────────────────────────────────────────

    def _on_segment_finished(self) -> None:
        label = self.segment_id(self._segment_index)
        to_point = self._segment_point_id(
            self._segment_index, 'to_point', f'P{self._segment_index + 2}')
        self.log_line.emit(f'{label} completada. Robot en {to_point}.')

        pending = self._gripper_events.get(to_point, [])
        if pending:
            self._gripper_queue = list(pending)
            self._begin_gripper_action('segment_boundary')
            return
        self._segment_boundary()

    def _segment_boundary(self) -> None:
        is_last = self._segment_index >= len(self._segments) - 1
        label = self.segment_id(self._segment_index)
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

        self.segment_completed.emit(
            self._segment_index, label, to_point, True)
        self._segment_index += 1
        self._begin_segment()

    # ── Garra (implementación ACTUAL, sin reimplementar) ─────────────

    def _begin_gripper_action(self, next_state_after) -> None:
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
        self._gripper_send_fn(0 if action == 'open' else 1)
        self._last_send_at = time.monotonic()

    def _after_gripper_queue(self) -> None:
        if self._gripper_next_state == STATE_RUNNING:
            self._begin_segment()
        else:
            self._segment_boundary()

    # ── Terminación ──────────────────────────────────────────────────

    def _complete(self) -> None:
        self._stop_timer()
        self._state = STATE_DONE
        total = len(self._segments)
        self.log_line.emit(f'Secuencia completada por lotes: {total} segmentos.')
        self.status_changed.emit('Secuencia completada.')
        self.finished.emit(True, f'Secuencia completada ({total} segmentos).')

    def _fail(self, reason: str) -> None:
        self._send_abort()
        self._stop_timer()
        self._state = STATE_ABORTED
        self.log_line.emit(f'ERROR: {reason}')
        self.status_changed.emit('Secuencia abortada.')
        self.finished.emit(False, reason)

    def _stop_timer(self) -> None:
        if self._timer.isActive():
            self._timer.stop()

    # ── Utilidades ───────────────────────────────────────────────────

    @staticmethod
    def _index_gripper_events(document: dict) -> Dict[str, List[str]]:
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
        except (IndexError, AttributeError, TypeError):
            value = None
        return value or fallback

    def _segment_velocity_pct(
        self,
        segment: dict,
    ) -> Tuple[Optional[float], Optional[str]]:
        """Perfil de velocidad del segmento, igual que en el modo base."""
        profile = segment.get('execution_profile')
        if profile is None:
            return self._default_velocity, None
        if not isinstance(profile, dict):
            return None, 'execution_profile inválido.'
        value = profile.get('kuka_ptp_velocity_pct')
        if not is_finite_number(value):
            return None, 'kuka_ptp_velocity_pct no es un número finito.'
        velocity = float(value)
        if not 0.0 < velocity <= 100.0:
            return None, ('kuka_ptp_velocity_pct debe ser mayor que 0 y menor '
                          'o igual que 100.')
        return velocity, None
