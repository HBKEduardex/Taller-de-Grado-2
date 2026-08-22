"""
trajectory_ros_bridge.py — Publishers/subscribers de trayectorias sobre el
nodo ROS2 que YA existe.

NO llama a rclpy.init(). NO crea un nodo propio. NO crea un nodo por ventana.

Recibe el RosAxisMoveBridge que las dos GUIs ya usan y engancha sus
publishers y subscribers al mismo rclpy.Node que ese bridge tiene dentro de
su executor. Es exactamente el patrón de ros_moveit_mirror_bridge.py, con
una diferencia: el nodo de la GUI original nace DESPUÉS de construirse la
ventana, así que el enganche es diferido (`ensure_attached()`) en lugar de
hacerse en el constructor.

Contrato de tópicos (std_msgs/msg/String, JSON, QoS Reliable):

  GUI  -> MoveIt   /kuka_moveit/trajectory_generation/request_json
  MoveIt -> GUI    /kuka_moveit/trajectory_generation/result_json
  GUI  -> RViz     /kuka_moveit/trajectory_preview/request_json
  MoveIt -> GUI    /kuka_moveit/trajectory_preview/status_json
"""

from typing import Optional

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String

try:
    from PyQt5.QtCore import QObject, pyqtSignal
except ImportError as e:
    raise ImportError(
        'PyQt5 is required. Install with: sudo apt install python3-pyqt5'
    ) from e


# ---------------------------------------------------------------------------
# Contrato de tópicos — NO cambiar sin avisar
# ---------------------------------------------------------------------------

TOPIC_GENERATION_REQUEST = '/kuka_moveit/trajectory_generation/request_json'
TOPIC_GENERATION_RESULT = '/kuka_moveit/trajectory_generation/result_json'
TOPIC_PREVIEW_REQUEST = '/kuka_moveit/trajectory_preview/request_json'
TOPIC_PREVIEW_STATUS = '/kuka_moveit/trajectory_preview/status_json'


def reliable_qos(depth: int = 10) -> QoSProfile:
    """QoS fiable para JSON de trayectorias (no se puede perder ninguno)."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


# ---------------------------------------------------------------------------
# Bridge
# ---------------------------------------------------------------------------

class TrajectoryRosBridge(QObject):
    """
    Capa ROS2 de la funcionalidad de secuencias de trayectorias.

    Señales (emitidas desde el hilo ROS2, conectadas a slots Qt):
      generation_result_received(str) — JSON de result_json
      preview_status_received(str)    — JSON de status_json
      attached_changed(bool)          — True cuando ya hay nodo y endpoints

    Uso:
        traj_bridge = TrajectoryRosBridge(kuka_bridge)
        traj_bridge.generation_result_received.connect(slot)
        traj_bridge.ensure_attached()   # idempotente; se puede reintentar
    """

    generation_result_received = pyqtSignal(str)
    preview_status_received = pyqtSignal(str)
    attached_changed = pyqtSignal(bool)

    def __init__(
        self,
        kuka_bridge,
        generation_request_topic: str = TOPIC_GENERATION_REQUEST,
        generation_result_topic: str = TOPIC_GENERATION_RESULT,
        preview_request_topic: str = TOPIC_PREVIEW_REQUEST,
        preview_status_topic: str = TOPIC_PREVIEW_STATUS,
        parent=None,
    ):
        super().__init__(parent)
        self._kuka_bridge = kuka_bridge

        self._generation_request_topic = generation_request_topic
        self._generation_result_topic = generation_result_topic
        self._preview_request_topic = preview_request_topic
        self._preview_status_topic = preview_status_topic

        self._node = None
        self._pub_generation = None
        self._pub_preview = None
        self._attached = False

    # ── Enganche diferido al nodo existente ──────────────────────────

    @property
    def is_attached(self) -> bool:
        return self._attached

    def ensure_attached(self) -> bool:
        """
        Crear publishers/subscribers sobre el nodo del bridge existente.

        Idempotente y seguro de llamar repetidamente desde un QTimer: si el
        nodo todavía no existe (la GUI original arranca rclpy después de
        construir la ventana) simplemente devuelve False y no hace nada.
        """
        if self._attached:
            return True

        node = getattr(self._kuka_bridge, '_node', None)
        if node is None:
            return False

        qos = reliable_qos()

        self._pub_generation = node.create_publisher(
            String, self._generation_request_topic, qos)
        self._pub_preview = node.create_publisher(
            String, self._preview_request_topic, qos)

        node.create_subscription(
            String, self._generation_result_topic,
            self._on_generation_result, qos)
        node.create_subscription(
            String, self._preview_status_topic,
            self._on_preview_status, qos)

        self._node = node
        self._attached = True

        logger = node.get_logger()
        logger.info(
            f'[Trayectorias] Solicitud de generación -> '
            f'{self._generation_request_topic}')
        logger.info(
            f'[Trayectorias] Resultado de generación <- '
            f'{self._generation_result_topic}')
        logger.info(
            f'[Trayectorias] Solicitud de previsualización -> '
            f'{self._preview_request_topic}')
        logger.info(
            f'[Trayectorias] Estado de previsualización <- '
            f'{self._preview_status_topic}')

        self.attached_changed.emit(True)
        return True

    # ── Publicación ──────────────────────────────────────────────────

    def publish_generation_request(self, json_str: str) -> bool:
        """Publicar la solicitud de generación (ENVIAR PUNTOS)."""
        if not self.ensure_attached() or self._pub_generation is None:
            return False
        message = String()
        message.data = json_str
        try:
            self._pub_generation.publish(message)
        except Exception as error:
            self._log_error(f'No se pudo publicar la solicitud: {error}')
            return False
        return True

    def publish_preview_request(self, json_str: str) -> bool:
        """
        Publicar la solicitud de previsualización (PROBAR TRAYECTORIA).

        Este es el ÚNICO camino de salida de PROBAR TRAYECTORIA. Esta clase
        no conoce el tópico de comandos del KUKA ni el bridge TCP/IP de
        movimiento, así que la previsualización no puede alcanzar al robot.
        """
        if not self.ensure_attached() or self._pub_preview is None:
            return False
        message = String()
        message.data = json_str
        try:
            self._pub_preview.publish(message)
        except Exception as error:
            self._log_error(f'No se pudo publicar la previsualización: {error}')
            return False
        return True

    # ── Callbacks (hilo ROS2 -> señales Qt) ──────────────────────────

    def _on_generation_result(self, message: String) -> None:
        self.generation_result_received.emit(message.data)

    def _on_preview_status(self, message: String) -> None:
        self.preview_status_received.emit(message.data)

    # ── Utilidades ───────────────────────────────────────────────────

    def _log_error(self, text: str) -> None:
        if self._node is not None:
            self._node.get_logger().error(f'[Trayectorias] {text}')

    def topics(self) -> dict:
        """Los cuatro tópicos en uso, para mostrarlos en la GUI o el README."""
        return {
            'generation_request': self._generation_request_topic,
            'generation_result': self._generation_result_topic,
            'preview_request': self._preview_request_topic,
            'preview_status': self._preview_status_topic,
        }
