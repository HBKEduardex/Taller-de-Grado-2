"""
trajectory_storage.py — Persistencia de secuencias en trajectories/.

El contenedor MoveIt2 NO guarda el archivo final: lo guarda esta GUI.

Resolución de la carpeta trajectories/ (primer criterio que acierte):

  1. `trajectories_dir` del YAML de configuración de la GUI, si está puesto.
  2. Variable de entorno KUKA_TRAJECTORIES_DIR.
  3. Raíz del repositorio detectada en runtime -> <raíz>/trajectories.
     La detección salta deliberadamente cualquier ruta bajo install/: los
     archivos generados NUNCA se guardan dentro de install/.
  4. Último recurso: ~/kuka_trajectories.

La ruta finalmente elegida se devuelve siempre por escrito para poder
mostrarla en la GUI y dejarla documentada.

Formato: JSON, un archivo completo por secuencia. No YAML.
Nombre:  trajectory_sequence_YYYYMMDD_HHMMSS.json
"""

import json
import os
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

from kuka_gui_control.trajectory_sequence_model import (
    AXES,
    SCHEMA_VERSION,
    is_finite_number,
)

# ---------------------------------------------------------------------------
# Constantes
# ---------------------------------------------------------------------------

TRAJECTORIES_DIRNAME: str = 'trajectories'
ENV_OVERRIDE: str = 'KUKA_TRAJECTORIES_DIR'
FILENAME_PREFIX: str = 'trajectory_sequence_'
FILENAME_SUFFIX: str = '.json'

# Marcadores que identifican la raíz del repositorio de comunicación/GUI.
_ROOT_MARKERS = ('.git', 'src')


# ---------------------------------------------------------------------------
# Resolución de la carpeta
# ---------------------------------------------------------------------------

def _repo_root_from_here() -> Optional[Path]:
    """
    Buscar la raíz del repositorio subiendo desde este archivo.

    Cubre los dos casos reales:
      - ejecutado desde el código fuente
        (<raíz>/src/kuka_gui_control/kuka_gui_control/…)
      - ejecutado desde el paquete instalado
        (<raíz>/install/kuka_gui_control/lib/pythonX/site-packages/…)

    En el segundo caso la raíz es el padre de install/, nunca install/ mismo.
    """
    here = Path(__file__).resolve()

    for parent in here.parents:
        # Paquete instalado: <raíz>/install/... -> la raíz es el padre.
        if parent.name == 'install' and parent.parent is not None:
            return parent.parent

    for parent in here.parents:
        if any((parent / marker).exists() for marker in _ROOT_MARKERS):
            # No devolver nunca una raíz que caiga dentro de install/.
            if 'install' in parent.parts:
                continue
            return parent

    return None


def resolve_trajectories_dir(configured: Optional[str] = None) -> Path:
    """
    Devolver la carpeta trajectories/ que debe usarse (sin crearla).

    Args:
        configured: valor de `trajectories_dir` del YAML de la GUI. Vacío o
                    None significa "decidir automáticamente".
    """
    if configured:
        return Path(os.path.expanduser(str(configured))).resolve()

    env_value = os.environ.get(ENV_OVERRIDE, '').strip()
    if env_value:
        return Path(os.path.expanduser(env_value)).resolve()

    root = _repo_root_from_here()
    if root is not None:
        return (root / TRAJECTORIES_DIRNAME).resolve()

    return (Path.home() / 'kuka_trajectories').resolve()


def ensure_trajectories_dir(configured: Optional[str] = None) -> Tuple[
        Optional[Path], str]:
    """
    Resolver la carpeta y crearla si hace falta.

    Returns:
        (ruta, mensaje_de_error). La ruta es None si no se pudo crear.
    """
    directory = resolve_trajectories_dir(configured)
    try:
        directory.mkdir(parents=True, exist_ok=True)
    except OSError as error:
        return None, f'No se pudo crear la carpeta {directory}: {error}'
    if not os.access(str(directory), os.W_OK):
        return None, f'La carpeta {directory} no tiene permiso de escritura.'
    return directory, ''


# ---------------------------------------------------------------------------
# Escritura
# ---------------------------------------------------------------------------

def build_filename(when: Optional[datetime] = None) -> str:
    """trajectory_sequence_YYYYMMDD_HHMMSS.json"""
    stamp = (when or datetime.now()).strftime('%Y%m%d_%H%M%S')
    return f'{FILENAME_PREFIX}{stamp}{FILENAME_SUFFIX}'


def save_sequence_document(
    document: dict,
    configured_dir: Optional[str] = None,
) -> Tuple[Optional[Path], str]:
    """
    Guardar el documento de secuencia como JSON.

    Escritura atómica: primero a un archivo temporal en la misma carpeta y
    después os.replace(), para que un fallo a mitad de escritura no deje un
    archivo .json corrupto que luego parecería ejecutable.

    Returns:
        (ruta_guardada, mensaje_de_error). Si la ruta es None NO se guardó
        nada y el mensaje explica por qué.
    """
    directory, error = ensure_trajectories_dir(configured_dir)
    if directory is None:
        return None, error

    target = directory / build_filename()
    temporary = target.with_suffix('.json.tmp')

    try:
        with open(temporary, 'w', encoding='utf-8') as handle:
            json.dump(document, handle, indent=2, ensure_ascii=False)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(str(temporary), str(target))
    except (OSError, TypeError, ValueError) as write_error:
        try:
            if temporary.exists():
                temporary.unlink()
        except OSError:
            pass
        return None, f'No se pudo escribir {target}: {write_error}'

    return target, ''


# ---------------------------------------------------------------------------
# Lectura
# ---------------------------------------------------------------------------

def list_sequence_files(configured_dir: Optional[str] = None) -> List[Path]:
    """Listar los .json de secuencia disponibles, del más reciente al más antiguo."""
    directory = resolve_trajectories_dir(configured_dir)
    if not directory.is_dir():
        return []
    files = [
        entry for entry in directory.iterdir()
        if entry.is_file() and entry.suffix == '.json'
    ]
    files.sort(key=lambda p: p.name, reverse=True)
    return files


def load_sequence_file(path) -> Tuple[Optional[dict], str]:
    """
    Cargar y validar un archivo de secuencia.

    Returns:
        (documento, mensaje_de_error). El documento es None si no es válido.
    """
    file_path = Path(path)
    try:
        with open(file_path, 'r', encoding='utf-8') as handle:
            document = json.load(handle)
    except (OSError, json.JSONDecodeError) as error:
        return None, f'No se pudo leer {file_path}: {error}'

    ok, error = validate_sequence_document(document)
    if not ok:
        return None, f'{file_path.name}: {error}'
    return document, ''


def validate_sequence_document(document) -> Tuple[bool, str]:
    """
    Validar un documento de secuencia cargado desde disco.

    Es la misma exigencia que se aplicó al recibir el resultado de MoveIt,
    repetida aquí porque el archivo puede haberse editado a mano entre una
    cosa y la otra.
    """
    if not isinstance(document, dict):
        return False, 'El archivo no contiene un objeto JSON.'

    if document.get('schema_version') != SCHEMA_VERSION:
        return False, (
            f'schema_version {document.get("schema_version")!r} no soportado '
            f'(se espera {SCHEMA_VERSION}).'
        )

    joint_names = document.get('joint_names')
    if not isinstance(joint_names, list) or len(joint_names) != len(AXES):
        return False, 'joint_names inválido.'

    gripper = document.get('gripper')
    if not isinstance(gripper, dict):
        return False, 'Falta el bloque gripper.'
    if gripper.get('initial_state') not in ('open', 'close'):
        return False, 'gripper.initial_state inválido.'
    events = gripper.get('events')
    if not isinstance(events, list):
        return False, 'gripper.events debe ser una lista.'
    for event in events:
        if not isinstance(event, dict):
            return False, 'Un evento de garra no es un objeto JSON.'
        if not isinstance(event.get('at_point'), str):
            return False, 'Un evento de garra no trae at_point.'
        if event.get('action') not in ('open', 'close'):
            return False, (
                f'Acción de garra inválida: {event.get("action")!r}.'
            )

    segments = document.get('segments')
    if not isinstance(segments, list) or not segments:
        return False, 'El archivo no trae segmentos.'

    for index, segment in enumerate(segments):
        label = (segment.get('id') if isinstance(segment, dict)
                 else None) or f'T{index + 1}'
        if not isinstance(segment, dict):
            return False, f'Segmento {label} inválido.'

        execution_profile = segment.get('execution_profile')
        if execution_profile is not None:
            if not isinstance(execution_profile, dict):
                return False, (
                    f'Segmento {label}: execution_profile inválido.'
                )
            velocity_pct = execution_profile.get('kuka_ptp_velocity_pct')
            if (not is_finite_number(velocity_pct)
                    or not 0.0 < float(velocity_pct) <= 100.0):
                return False, (
                    f'Segmento {label}: kuka_ptp_velocity_pct debe ser mayor '
                    f'que 0 y menor o igual que 100.'
                )

        points = segment.get('trajectory_points')
        if not isinstance(points, list) or not points:
            return False, f'Segmento {label} sin trajectory_points.'
        for pt_index, point in enumerate(points):
            if not isinstance(point, dict):
                return False, f'Segmento {label}, punto {pt_index} inválido.'
            deg = point.get('positions_deg')
            if not isinstance(deg, list) or len(deg) != len(AXES):
                return False, (
                    f'Segmento {label}, punto {pt_index}: positions_deg debe '
                    f'tener {len(AXES)} articulaciones.'
                )
            if any(not is_finite_number(v) for v in deg):
                return False, (
                    f'Segmento {label}, punto {pt_index}: positions_deg '
                    f'contiene valores no finitos.'
                )
            tfs = point.get('time_from_start_sec')
            if not is_finite_number(tfs) or float(tfs) < 0.0:
                return False, (
                    f'Segmento {label}, punto {pt_index}: '
                    f'time_from_start_sec inválido.'
                )

    return True, ''
