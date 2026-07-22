"""
eki_xml_server_node.py — ROS2 node for KUKA EthernetKRL XML communication.

This node starts a TCP server that:
  1. Listens for incoming connections from the KUKA robot.
  2. Receives XML data sent by the KUKA program XmlTransmit.src.
  3. Logs the raw and parsed XML content.
  4. Responds with a Sensor XML message.
  5. Supports multiple connection cycles (KUKA connect → send → disconnect).

Parameters (configured via eki_server.yaml):
  - bind_host:          IP to bind the TCP server (default: "0.0.0.0").
  - port:               TCP port to listen on (default: 59152).
  - response_xml_path:  Path to a custom sensor response XML file.
  - log_raw_xml:        Whether to log raw XML strings (default: true).
  - pretty_print_xml:   Whether to log pretty-printed XML (default: true).
  - receive_buffer_size: TCP receive buffer size in bytes (default: 8192).
  - keep_running:       Keep accepting connections after disconnect (default: true).

Usage:
  ros2 launch kuka_eki_bridge eki_server.launch.py
  ros2 launch kuka_eki_bridge eki_server.launch.py port:=59152 bind_host:=0.0.0.0
"""

import rclpy
from rclpy.node import Node

from kuka_eki_bridge.eki_protocol import EkiXmlServer
from kuka_eki_bridge.xml_utils import pretty_xml, extract_robot_fields


class EkiXmlServerNode(Node):
    """ROS2 node wrapping the EKI TCP/XML server."""

    def __init__(self):
        super().__init__('eki_xml_server')

        # ── Declare ROS2 parameters with defaults ────────────────────
        self.declare_parameter('bind_host', '0.0.0.0')
        self.declare_parameter('port', 59152)
        self.declare_parameter('response_xml_path', '')
        self.declare_parameter('log_raw_xml', True)
        self.declare_parameter('pretty_print_xml', True)
        self.declare_parameter('receive_buffer_size', 8192)
        self.declare_parameter('keep_running', True)

        # ── Read parameter values ────────────────────────────────────
        self._bind_host = (
            self.get_parameter('bind_host').get_parameter_value().string_value
        )
        self._port = (
            self.get_parameter('port').get_parameter_value().integer_value
        )
        self._response_xml_path = (
            self.get_parameter('response_xml_path')
            .get_parameter_value()
            .string_value
        )
        self._log_raw_xml = (
            self.get_parameter('log_raw_xml')
            .get_parameter_value()
            .bool_value
        )
        self._pretty_print_xml = (
            self.get_parameter('pretty_print_xml')
            .get_parameter_value()
            .bool_value
        )
        self._receive_buffer_size = (
            self.get_parameter('receive_buffer_size')
            .get_parameter_value()
            .integer_value
        )
        self._keep_running = (
            self.get_parameter('keep_running')
            .get_parameter_value()
            .bool_value
        )

        # ── Log configuration ───────────────────────────────────────
        self.get_logger().info('╔══════════════════════════════════════════╗')
        self.get_logger().info('║     KUKA EKI XML Server — ROS2 Node     ║')
        self.get_logger().info('╚══════════════════════════════════════════╝')
        self.get_logger().info(f'  Bind host:       {self._bind_host}')
        self.get_logger().info(f'  Port:            {self._port}')
        self.get_logger().info(f'  Response XML:    '
                               f'{self._response_xml_path or "(default)"}')
        self.get_logger().info(f'  Log raw XML:     {self._log_raw_xml}')
        self.get_logger().info(f'  Pretty print:    {self._pretty_print_xml}')
        self.get_logger().info(f'  Buffer size:     {self._receive_buffer_size}')
        self.get_logger().info(f'  Keep running:    {self._keep_running}')
        self.get_logger().info('──────────────────────────────────────────')

        # ── Create and start the TCP server ──────────────────────────
        self._server = EkiXmlServer(
            host=self._bind_host,
            port=self._port,
            logger=self.get_logger(),
            on_data_received=self._on_data_received,
            response_xml_path=self._response_xml_path,
            receive_buffer_size=self._receive_buffer_size,
            keep_running=self._keep_running,
        )

        try:
            self._server.start()
        except RuntimeError as e:
            self.get_logger().fatal(f'Server failed to start: {e}')
            raise SystemExit(1)

    def _on_data_received(self, client_address: tuple, raw_xml: str) -> None:
        """
        Callback invoked when XML data is received from the KUKA.

        Logs the client address, raw XML, pretty-printed XML,
        and extracted fields.

        Args:
            client_address: Tuple of (ip, port) of the connected KUKA.
            raw_xml: Decoded XML string received from the robot.
        """
        addr_str = f'{client_address[0]}:{client_address[1]}'

        self.get_logger().info(f'━━━ Data received from {addr_str} ━━━')

        # Log raw XML
        if self._log_raw_xml:
            self.get_logger().info(f'[RAW XML]\n{raw_xml}')

        # Log pretty-printed XML
        if self._pretty_print_xml:
            formatted = pretty_xml(raw_xml)
            if formatted:
                self.get_logger().info(f'[FORMATTED XML]\n{formatted}')
            else:
                self.get_logger().warn(
                    'Could not pretty-print XML (malformed?).'
                )

        # Extract and log important fields
        fields = extract_robot_fields(raw_xml)
        if fields:
            self.get_logger().info('[EXTRACTED FIELDS]')
            for key, value in fields.items():
                self.get_logger().info(f'  {key}: {value}')
        else:
            self.get_logger().info(
                'No standard robot fields found in received XML.'
            )

    def destroy_node(self):
        """Clean shutdown: stop TCP server before destroying the node."""
        self.get_logger().info('Shutting down EKI XML Server node...')
        if hasattr(self, '_server'):
            self._server.stop()
        super().destroy_node()


def main(args=None):
    """Entry point for the eki_xml_server_node."""
    rclpy.init(args=args)
    node = EkiXmlServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt — shutting down.')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
