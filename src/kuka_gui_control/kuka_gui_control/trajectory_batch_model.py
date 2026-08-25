"""
trajectory_batch_model.py — empaquetado de puntos en LOTES.

Ruta NUEVA y paralela. No importa, no hereda y no comparte ninguna rama de
código con trajectory_executor.py ni con trajectory_sequence_model.py, de
modo que un cambio futuro en el modo punto-a-punto no puede alterar el modo
lote por accidente, ni al revés.

Qué hace:
  - partir los trajectory_points de un segmento en lotes consecutivos de
    como mucho MAX_BATCH_SIZE puntos, EN ORDEN y SIN descartar ninguno;
  - construir el JSON que viaja por el tópico que YA existe
    (/kuka/axis_move/target_json) con los campos de lote añadidos.

No hay tópico nuevo, ni nodo nuevo, ni socket nuevo: los campos de lote son
campos opcionales del mismo mensaje de comando.

Sin dependencias de ROS2 ni de PyQt5: se puede probar de forma aislada.
"""

import json
import math
from typing import Dict, List, Optional, Tuple

AXES: List[str] = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']

# Tope duro del lado GUI. Debe ser <= XD_BATCH_MAX de config_submit_better.dat
# y <= max_batch_size del nodo bridge. El tamaño NO lo impone el tiempo de
# reacción a DETENER (eso lo acota $ADVANCE dentro de XmlDualMove_better.src),
# sino la memoria de recepción de EKI.
DEFAULT_MAX_BATCH_SIZE: int = 20

# Reponer cuando quedan por consumir menos de esta fracción del lote en curso.
# XmlDualMove_better.src copia el lote a arrays locales al empezar, así que el
# mailbox queda libre y se puede recargar mientras el robot sigue moviéndose.
DEFAULT_REFILL_THRESHOLD: float = 0.5


def is_finite_number(value) -> bool:
    """True solo si `value` es un número real finito."""
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def pack_segment_into_batches(
    segment: dict,
    max_batch_size: int = DEFAULT_MAX_BATCH_SIZE,
) -> Tuple[Optional[List[List[Dict[str, float]]]], str]:
    """
    Partir un segmento en lotes consecutivos.

    NO se elimina, reordena ni reinterpola ningún punto: es un troceado
    puro de la lista tal y como la generó MoveIt.

    Returns:
        (lotes, error). `lotes` es None si algo no valida.
    """
    if max_batch_size < 1:
        return None, f'max_batch_size={max_batch_size} inválido.'

    points = segment.get('trajectory_points')
    if not isinstance(points, list) or not points:
        return None, 'el segmento no trae trajectory_points.'

    flat: List[Dict[str, float]] = []
    for index, point in enumerate(points):
        positions = point.get('positions_deg')
        if not isinstance(positions, list) or len(positions) != len(AXES):
            return None, f'punto {index}: positions_deg debe traer 6 valores.'
        entry: Dict[str, float] = {}
        for axis, value in zip(AXES, positions):
            if not is_finite_number(value):
                return None, f'punto {index}: {axis} no es finito.'
            entry[axis] = float(value)
        flat.append(entry)

    batches = [
        flat[i:i + max_batch_size]
        for i in range(0, len(flat), max_batch_size)
    ]
    return batches, ''


def build_batch_command_json(
    seq: int,
    batch_seq: int,
    points: List[Dict[str, float]],
    ptp_velocity_pct: float,
    enable_move: bool,
    source: str = 'kuka_gui_control_batch',
) -> str:
    """
    Construir el comando de lote para /kuka/axis_move/target_json.

    Los campos batch_* son AÑADIDOS al mensaje que el bridge ya entiende.
    Un comando sin ellos —SEND manual, jog, HOME, garra— sigue produciendo
    exactamente el mismo XML que hoy.

    `axis_target` va a la posición del PRIMER punto del lote solo como valor
    inerte de relleno: XmlDualMove_better.src desactiva la ruta de punto
    suelto mientras hay un lote pendiente, y el nodo bridge lo sustituye por
    la posición REAL del robot al construir el XML.
    """
    payload = {
        'seq': int(seq),
        'source': source,
        'node_mode': 'trajectory_batch',
        'mode': 'AxisTarget',
        'enable_move': bool(enable_move),
        'gripper_command': -1,
        'axis_target': {axis: round(points[0][axis], 6) for axis in AXES},
        'cartesian_target': {},
        # ---- campos de lote ----
        'batch_seq': int(batch_seq),
        'batch_ptp_velocity_pct': round(float(ptp_velocity_pct), 6),
        'batch_points_deg': [
            {axis: round(float(point[axis]), 6) for axis in AXES}
            for point in points
        ],
    }
    for axis in AXES:
        payload[axis] = payload['axis_target'][axis]
    return json.dumps(payload)


def build_abort_batch_json(seq: int) -> str:
    """
    Construir la orden de aborto de lote.

    Lleva enable_move=False y ningún punto, así que por sí sola no puede
    mover nada: solo levanta XD_ABORT_BATCH en el KUKA. El robot termina el
    PTP en vuelo y no arranca el siguiente.
    """
    return json.dumps({
        'seq': int(seq),
        'source': 'kuka_gui_control_batch',
        'node_mode': 'trajectory_batch',
        'mode': 'AxisTarget',
        'enable_move': False,
        'gripper_command': -1,
        'abort_batch': True,
    })
