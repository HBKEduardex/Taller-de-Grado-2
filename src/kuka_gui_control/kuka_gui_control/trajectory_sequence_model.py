"""
trajectory_sequence_model.py — Buffer temporal de puntos SET y contrato JSON.

Capa NUEVA. No toca ni sustituye a joint_command_model.py ni a
dual_command_model.py: solo se apoya en los valores AxisActual que esos
modelos ya reciben del KUKA real por TCP/IP.

Responsabilidades:
  - Guardar EN MEMORIA los puntos P1...PN capturados con el botón SET.
  - Guardar EN MEMORIA los eventos de garra de la secuencia
    (SET ABRIR GARRA / SET CERRAR GARRA), asociados al último punto SET.
  - Construir el JSON de solicitud para el contenedor MoveIt2.
  - Validar el JSON de resultado que devuelve el contenedor.
  - Construir el documento JSON que se guarda en trajectories/.

Sin dependencias de ROS2 ni de PyQt5: se puede probar de forma aislada.

IMPORTANTE
  * Nada de lo que hay aquí escribe en disco. La persistencia vive en
    trajectory_storage.py y solo ocurre DESPUÉS de que MoveIt devuelva una
    trayectoria válida.
  * La fuente autoritativa de un punto es AxisActual (A1..A6) recibido del
    KUKA. XYZABC solo se guarda como diagnóstico opcional y jamás se usa
    para generar la trayectoria.
"""

import json
import math
import uuid
from datetime import datetime
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Constantes del contrato
# ---------------------------------------------------------------------------

SCHEMA_VERSION: int = 1

# Ejes tal y como los reporta el KUKA en <Robot><Data><AxisActual A1=.../>
AXES: List[str] = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']

# Nombres de articulación tal y como los espera el contenedor MoveIt2.
MOVEIT_JOINT_NAMES: List[str] = [
    'joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6',
]

CARTESIAN_KEYS: List[str] = ['X', 'Y', 'Z', 'A', 'B', 'C']

# Estado inicial de la garra en toda secuencia. Si el usuario no registra
# ningún evento, la garra permanece abierta.
GRIPPER_INITIAL_STATE: str = 'open'

GRIPPER_ACTIONS = ('open', 'close')

# Valor que el protocolo actual usa en <Command><GripperCommand>.
GRIPPER_COMMAND_VALUE = {'open': 0, 'close': 1}

DEFAULT_PLANNER_MODE: str = 'moveit_base'

DEFAULT_KUKA_PTP_VELOCITY_NORMAL_PCT: float = 30.0
DEFAULT_KUKA_PTP_VELOCITY_REDUCED_PCT: float = 5.0


# ---------------------------------------------------------------------------
# Utilidades numéricas
# ---------------------------------------------------------------------------

def is_finite_number(value) -> bool:
    """True solo si `value` es un número real finito (ni NaN ni inf)."""
    try:
        as_float = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(as_float)


def deg_list_from_rad(values: List[float]) -> List[float]:
    """Conversión de unidades rad -> deg. NO altera la trayectoria."""
    return [math.degrees(float(v)) for v in values]


def rad_list_from_deg(values: List[float]) -> List[float]:
    """Conversión de unidades deg -> rad. NO altera la trayectoria."""
    return [math.radians(float(v)) for v in values]


# ---------------------------------------------------------------------------
# Entradas del buffer temporal
# ---------------------------------------------------------------------------

class SequencePoint:
    """Un punto P_n capturado con SET desde AxisActual."""

    __slots__ = ('point_id', 'joints_deg', 'cartesian_diagnostic',
                 'captured_at', 'incoming_kuka_ptp_velocity_pct')

    def __init__(
        self,
        point_id: str,
        joints_deg: List[float],
        cartesian_diagnostic: Optional[Dict[str, float]] = None,
        captured_at: Optional[str] = None,
        incoming_kuka_ptp_velocity_pct: float =
            DEFAULT_KUKA_PTP_VELOCITY_NORMAL_PCT,
    ):
        self.point_id = point_id
        self.joints_deg = [float(v) for v in joints_deg]
        self.cartesian_diagnostic = cartesian_diagnostic
        self.captured_at = captured_at or datetime.now().isoformat(
            timespec='seconds')
        self.incoming_kuka_ptp_velocity_pct = float(
            incoming_kuka_ptp_velocity_pct)

    def to_request_dict(self) -> dict:
        """Forma exigida por el contrato de solicitud a MoveIt."""
        return {
            'id': self.point_id,
            'joints_deg': [round(v, 6) for v in self.joints_deg],
        }

    def to_storage_dict(self) -> dict:
        """Forma guardada en trajectories/ dentro de source_points."""
        data = {
            'id': self.point_id,
            'joints_deg': [round(v, 6) for v in self.joints_deg],
            'captured_at': self.captured_at,
            'incoming_kuka_ptp_velocity_pct':
                self.incoming_kuka_ptp_velocity_pct,
        }
        if self.cartesian_diagnostic:
            # Diagnóstico OPCIONAL. Nunca se usa para planificar ni ejecutar.
            data['cartesian_diagnostic'] = {
                k: round(float(v), 6)
                for k, v in self.cartesian_diagnostic.items()
            }
        return data

    def short_log(self) -> str:
        joints = ' '.join(
            f'{name}={value:.2f}'
            for name, value in zip(AXES, self.joints_deg)
        )
        return (
            f'{self.point_id} seteado: {joints} | '
            f'PTP entrante={self.incoming_kuka_ptp_velocity_pct:g}%'
        )


class GripperEvent:
    """Un evento de garra de la secuencia, anclado a un punto SET."""

    __slots__ = ('at_point', 'action', 'registered_at')

    def __init__(self, at_point: str, action: str,
                 registered_at: Optional[str] = None):
        self.at_point = at_point
        self.action = action
        self.registered_at = registered_at or datetime.now().isoformat(
            timespec='seconds')

    def to_dict(self) -> dict:
        return {'at_point': self.at_point, 'action': self.action}

    def short_log(self) -> str:
        verb = 'ABRIR' if self.action == 'open' else 'CERRAR'
        return f'Garra: {verb} en {self.at_point}'


# ---------------------------------------------------------------------------
# Modelo de secuencia (solo memoria)
# ---------------------------------------------------------------------------

class TrajectorySequenceModel:
    """
    Buffer TEMPORAL de la secuencia que se está programando.

    Vive solo mientras la aplicación está abierta. No hay persistencia,
    ni autoguardado, ni fichero de respaldo: SET no escribe en disco.
    """

    def __init__(self):
        self._points: List[SequencePoint] = []
        self._gripper_events: List[GripperEvent] = []
        self._log_lines: List[str] = []
        self._last_request_id: Optional[str] = None

    # ── Estado ───────────────────────────────────────────────────────

    @property
    def point_count(self) -> int:
        """Número de puntos P1...PN. Los eventos de garra NO cuentan."""
        return len(self._points)

    @property
    def gripper_event_count(self) -> int:
        return len(self._gripper_events)

    @property
    def points(self) -> List[SequencePoint]:
        return list(self._points)

    @property
    def gripper_events(self) -> List[GripperEvent]:
        return list(self._gripper_events)

    @property
    def last_point_id(self) -> Optional[str]:
        return self._points[-1].point_id if self._points else None

    @property
    def last_request_id(self) -> Optional[str]:
        return self._last_request_id

    def log_lines(self) -> List[str]:
        return list(self._log_lines)

    def append_log(self, line: str) -> None:
        """Añadir una línea al log temporal de la secuencia."""
        self._log_lines.append(line)

    def clear(self) -> None:
        """Vaciar el buffer temporal completo."""
        self._points.clear()
        self._gripper_events.clear()
        self._log_lines.clear()
        self._last_request_id = None

    # ── SET de punto ─────────────────────────────────────────────────

    def add_point_from_axis_actual(
        self,
        axis_actual: Optional[Dict[str, float]],
        position_actual: Optional[Dict[str, float]] = None,
        incoming_kuka_ptp_velocity_pct: float =
            DEFAULT_KUKA_PTP_VELOCITY_NORMAL_PCT,
    ) -> Tuple[bool, str]:
        """
        Capturar un punto desde el AxisActual REAL recibido del KUKA.

        Args:
            axis_actual:     dict {'A1': .., ..., 'A6': ..} tal y como llega
                             en el feedback JSON del bridge TCP/IP.
            position_actual: dict XYZABC opcional. Solo se guarda como
                             diagnóstico y solo si TODOS sus valores son
                             finitos. Nunca se usa para planificar.

        Returns:
            (ok, mensaje). Si ok es False no se guardó nada.
        """
        if not axis_actual:
            return False, (
                'No hay feedback válido del KUKA: no se capturó el punto. '
                'AxisActual no está disponible.'
            )

        if (not is_finite_number(incoming_kuka_ptp_velocity_pct)
                or not 0.0 < float(incoming_kuka_ptp_velocity_pct) <= 100.0):
            return False, (
                'La velocidad PTP entrante debe ser mayor que 0 y menor o '
                'igual que 100 %: no se capturó el punto.'
            )

        joints: List[float] = []
        for axis in AXES:
            value = axis_actual.get(axis)
            if value is None:
                return False, (
                    f'AxisActual no trae {axis}: no se capturó el punto.'
                )
            if not is_finite_number(value):
                return False, (
                    f'AxisActual.{axis} = {value!r} no es un número finito: '
                    f'no se capturó el punto.'
                )
            joints.append(float(value))

        point_id = f'P{len(self._points) + 1}'
        cartesian = self._sanitize_cartesian(position_actual)
        point = SequencePoint(
            point_id,
            joints,
            cartesian,
            incoming_kuka_ptp_velocity_pct=
                float(incoming_kuka_ptp_velocity_pct),
        )
        self._points.append(point)
        self._log_lines.append(point.short_log())
        return True, point.short_log()

    @staticmethod
    def _sanitize_cartesian(
        position_actual: Optional[Dict[str, float]],
    ) -> Optional[Dict[str, float]]:
        """
        Devolver XYZABC solo si TODO el conjunto es finito.

        Los problemas conocidos de recálculo/NaN en la pose cartesiana hacen
        que un valor sospechoso invalide el bloque entero como diagnóstico.
        """
        if not position_actual:
            return None
        clean: Dict[str, float] = {}
        for key in CARTESIAN_KEYS:
            value = position_actual.get(key)
            if not is_finite_number(value):
                return None
            clean[key] = float(value)
        return clean

    # ── SET de garra (solo programación, NO mueve la garra) ──────────

    def add_gripper_event(self, action: str) -> Tuple[bool, str]:
        """
        Registrar un evento de garra anclado al ÚLTIMO punto SET.

        No publica nada, no toca el bridge y no mueve la garra física: solo
        anota que al llegar a ese punto habrá que abrirla o cerrarla durante
        una futura ejecución de un archivo de trayectoria.
        """
        if action not in GRIPPER_ACTIONS:
            return False, f'Acción de garra desconocida: {action!r}.'

        if not self._points:
            return False, (
                'Todavía no hay ningún punto SET: un evento de garra debe '
                'ir asociado a un punto. Pulsa SET primero.'
            )

        event = GripperEvent(self._points[-1].point_id, action)
        self._gripper_events.append(event)
        self._log_lines.append(event.short_log())
        return True, event.short_log()

    # ── Solicitud a MoveIt ───────────────────────────────────────────

    def build_request(self, request_id: Optional[str] = None) -> Tuple[
            Optional[str], Optional[dict], str]:
        """
        Construir la solicitud de generación de trayectoria.

        Returns:
            (request_id, payload_dict, mensaje). request_id y payload son
            None cuando la secuencia no cumple el mínimo exigido.
        """
        if len(self._points) < 2:
            return None, None, (
                f'Se necesitan al menos 2 puntos para pedir una trayectoria '
                f'(hay {len(self._points)}).'
            )

        new_id = request_id or str(uuid.uuid4())
        payload = {
            'schema_version': SCHEMA_VERSION,
            'request_id': new_id,
            'joint_names': list(MOVEIT_JOINT_NAMES),
            'points': [p.to_request_dict() for p in self._points],
            'gripper': {
                'initial_state': GRIPPER_INITIAL_STATE,
                'events': [e.to_dict() for e in self._gripper_events],
            },
            'planner': {
                'mode': DEFAULT_PLANNER_MODE,
                # La GUI nunca pide ejecución al contenedor: la ejecución
                # física la hace ENVIAR TRAYECTORIA por el bridge TCP/IP.
                'execute': False,
            },
        }
        self._last_request_id = new_id
        return new_id, payload, (
            f'Solicitud preparada: {len(self._points)} puntos, '
            f'{len(self._gripper_events)} eventos de garra.'
        )

    def build_request_json(self, request_id: Optional[str] = None) -> Tuple[
            Optional[str], Optional[str], str]:
        """Igual que build_request() pero devolviendo el JSON serializado."""
        new_id, payload, message = self.build_request(request_id)
        if payload is None:
            return None, None, message
        return new_id, json.dumps(payload), message


# ---------------------------------------------------------------------------
# Validación del resultado devuelto por el contenedor MoveIt2
# ---------------------------------------------------------------------------

def validate_result_payload(
    payload: dict,
    expected_request_id: Optional[str],
) -> Tuple[bool, str]:
    """
    Validar el JSON de /kuka_moveit/trajectory_generation/result_json.

    Comprueba, en este orden:
      1. schema_version
      2. request_id (debe coincidir con la solicitud en curso)
      3. joint_names
      4. segmentos presentes y no vacíos
      5. cada punto con 6 articulaciones
      6. todos los números finitos
      7. time_from_start presente, finito, no negativo y no decreciente

    Returns:
        (ok, mensaje_de_error). Con ok=True el mensaje está vacío.
    """
    if not isinstance(payload, dict):
        return False, 'El resultado no es un objeto JSON.'

    # 1. schema_version
    schema = payload.get('schema_version')
    if schema != SCHEMA_VERSION:
        return False, (
            f'schema_version inesperado: {schema!r} '
            f'(se esperaba {SCHEMA_VERSION}).'
        )

    # 2. request_id
    request_id = payload.get('request_id')
    if not isinstance(request_id, str) or not request_id:
        return False, 'El resultado no trae request_id.'
    if expected_request_id is not None and request_id != expected_request_id:
        return False, (
            f'request_id {request_id} no corresponde a la solicitud en curso '
            f'({expected_request_id}).'
        )

    # 3. joint_names
    joint_names = payload.get('joint_names')
    if not isinstance(joint_names, list) or len(joint_names) != len(AXES):
        return False, (
            f'joint_names inválido: se esperaban {len(AXES)} nombres.'
        )
    if any(not isinstance(n, str) or not n for n in joint_names):
        return False, 'joint_names contiene entradas que no son cadenas.'

    # 4. segmentos
    segments = payload.get('segments')
    if not isinstance(segments, list) or not segments:
        return False, 'El resultado no trae segmentos.'

    for seg_index, segment in enumerate(segments):
        seg_id = None
        if isinstance(segment, dict):
            seg_id = segment.get('id')
        seg_label = seg_id or f'T{seg_index + 1}'

        if not isinstance(segment, dict):
            return False, f'Segmento {seg_label} no es un objeto JSON.'

        points = segment.get('trajectory_points')
        if not isinstance(points, list) or not points:
            return False, f'Segmento {seg_label} sin trajectory_points.'

        previous_time: Optional[float] = None
        for pt_index, point in enumerate(points):
            if not isinstance(point, dict):
                return False, (
                    f'Segmento {seg_label}, punto {pt_index}: no es un '
                    f'objeto JSON.'
                )

            # 5 y 6: posiciones — 6 articulaciones, todas finitas.
            ok, error = _validate_position_arrays(point, seg_label, pt_index)
            if not ok:
                return False, error

            # 6: velocidades y aceleraciones si vienen.
            for key in ('velocities_rad_s', 'accelerations_rad_s2'):
                values = point.get(key)
                if values is None:
                    continue
                if not isinstance(values, list) or len(values) != len(AXES):
                    return False, (
                        f'Segmento {seg_label}, punto {pt_index}: {key} debe '
                        f'tener {len(AXES)} valores.'
                    )
                if any(not is_finite_number(v) for v in values):
                    return False, (
                        f'Segmento {seg_label}, punto {pt_index}: {key} '
                        f'contiene valores no finitos.'
                    )

            # 7: time_from_start
            tfs = point.get('time_from_start_sec', point.get('time_from_start'))
            if not is_finite_number(tfs):
                return False, (
                    f'Segmento {seg_label}, punto {pt_index}: '
                    f'time_from_start_sec inválido.'
                )
            tfs = float(tfs)
            if tfs < 0.0:
                return False, (
                    f'Segmento {seg_label}, punto {pt_index}: '
                    f'time_from_start_sec negativo ({tfs}).'
                )
            if previous_time is not None and tfs < previous_time:
                return False, (
                    f'Segmento {seg_label}, punto {pt_index}: '
                    f'time_from_start_sec decrece ({previous_time} -> {tfs}).'
                )
            previous_time = tfs

    return True, ''


def _validate_position_arrays(
    point: dict,
    seg_label: str,
    pt_index: int,
) -> Tuple[bool, str]:
    """Validar positions_rad / positions_deg de un trajectory_point."""
    rad = point.get('positions_rad')
    deg = point.get('positions_deg')

    if rad is None and deg is None:
        return False, (
            f'Segmento {seg_label}, punto {pt_index}: sin positions_rad ni '
            f'positions_deg.'
        )

    for key, values in (('positions_rad', rad), ('positions_deg', deg)):
        if values is None:
            continue
        if not isinstance(values, list) or len(values) != len(AXES):
            return False, (
                f'Segmento {seg_label}, punto {pt_index}: {key} debe tener '
                f'{len(AXES)} articulaciones (trae '
                f'{len(values) if isinstance(values, list) else "?"}).'
            )
        if any(not is_finite_number(v) for v in values):
            return False, (
                f'Segmento {seg_label}, punto {pt_index}: {key} contiene '
                f'valores no finitos.'
            )

    return True, ''


# ---------------------------------------------------------------------------
# Documento que se guarda en trajectories/
# ---------------------------------------------------------------------------

def build_storage_document(
    sequence: TrajectorySequenceModel,
    result_payload: dict,
) -> dict:
    """
    Construir el documento JSON completo de una secuencia generada.

    Los datos de trayectoria se copian EXACTAMENTE como llegaron. Lo único
    que se añade es la pareja de unidades que falte (rad <-> deg), que es una
    conversión de unidades, no una transformación de la trayectoria: no se
    eliminan puntos, no se reinterpola, no se tocan los tiempos y no se
    suaviza nada.
    """
    generated_at = datetime.now()
    joint_names = list(result_payload.get('joint_names', MOVEIT_JOINT_NAMES))
    incoming_velocity_by_point = {
        point.point_id: point.incoming_kuka_ptp_velocity_pct
        for point in sequence.points
    }

    segments_out: List[dict] = []
    total_points = 0
    total_duration = 0.0

    for seg_index, segment in enumerate(result_payload.get('segments', [])):
        seg_id = segment.get('id') or f'T{seg_index + 1}'
        points_out: List[dict] = []
        last_time = 0.0

        for point in segment.get('trajectory_points', []):
            rad = point.get('positions_rad')
            deg = point.get('positions_deg')

            if rad is not None:
                positions_rad = [float(v) for v in rad]
            else:
                positions_rad = rad_list_from_deg(deg)

            if deg is not None:
                positions_deg = [float(v) for v in deg]
            else:
                positions_deg = deg_list_from_rad(rad)

            tfs = float(point.get(
                'time_from_start_sec', point.get('time_from_start', 0.0)))
            last_time = max(last_time, tfs)

            entry = {
                'positions_rad': positions_rad,
                'positions_deg': positions_deg,
                'positions_deg_source': (
                    'received' if deg is not None else 'converted_from_rad'),
                'velocities_rad_s': _copy_array(
                    point.get('velocities_rad_s')),
                'accelerations_rad_s2': _copy_array(
                    point.get('accelerations_rad_s2')),
                'time_from_start_sec': tfs,
            }
            points_out.append(entry)

        declared_duration = segment.get('duration_sec')
        duration = (
            float(declared_duration)
            if is_finite_number(declared_duration) else last_time
        )
        total_duration += duration
        total_points += len(points_out)

        to_point = segment.get('to_point')
        incoming_velocity = incoming_velocity_by_point.get(to_point)
        if incoming_velocity is None and seg_index + 1 < sequence.point_count:
            incoming_velocity = sequence.points[
                seg_index + 1].incoming_kuka_ptp_velocity_pct
        if incoming_velocity is None:
            incoming_velocity = DEFAULT_KUKA_PTP_VELOCITY_NORMAL_PCT

        segments_out.append({
            'id': seg_id,
            'from_point': segment.get('from_point'),
            'to_point': to_point,
            'duration_sec': duration,
            'execution_profile': {
                'kuka_ptp_velocity_pct': float(incoming_velocity),
            },
            'trajectory_points': points_out,
        })

    document = {
        'schema_version': SCHEMA_VERSION,
        'request_id': result_payload.get('request_id'),
        'generated_at': generated_at.isoformat(timespec='seconds'),
        'generated_at_date': generated_at.strftime('%Y-%m-%d'),
        'generated_at_time': generated_at.strftime('%H:%M:%S'),
        'source': 'kuka_gui_control',
        'source_points': [p.to_storage_dict() for p in sequence.points],
        'joint_names': joint_names,
        'gripper': {
            'initial_state': GRIPPER_INITIAL_STATE,
            'events': [e.to_dict() for e in sequence.gripper_events],
        },
        'planner_metadata': result_payload.get('planner_metadata', {}),
        'segments': segments_out,
        'summary': {
            'num_source_points': sequence.point_count,
            'num_gripper_events': sequence.gripper_event_count,
            'num_segments': len(segments_out),
            'num_trajectory_points': total_points,
            'total_duration_sec': round(total_duration, 6),
        },
    }
    return document


def _copy_array(values) -> Optional[List[float]]:
    """Copiar una lista de floats tal cual, o None si no vino."""
    if values is None:
        return None
    return [float(v) for v in values]
