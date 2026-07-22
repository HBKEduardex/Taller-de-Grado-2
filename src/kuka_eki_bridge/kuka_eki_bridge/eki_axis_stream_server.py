"""
eki_axis_stream_server.py — TCP server for continuous KUKA axis streaming.

Unlike the basic EKI server (eki_protocol.py) which handles single
request-response exchanges, this server:
  - Keeps the TCP connection open for continuous streaming.
  - Handles TCP stream fragmentation and concatenation.
  - Extracts individual <Robot>...</Robot> messages from the stream.
  - Optionally responds with a minimal <Sensor> keepalive.
  - Supports reconnection if the KUKA disconnects.

This module does NOT modify the existing eki_protocol.py.
"""

import socket
import threading
from typing import Callable, Optional

from kuka_eki_bridge.axis_xml_utils import TcpXmlBuffer


# Minimal Sensor response for keepalive (only sent if send_response=True)
KEEPALIVE_SENSOR_XML = '<Sensor><Status><IsActive>1</IsActive></Status></Sensor>'


class EkiAxisStreamServer:
    """
    TCP server optimized for continuous axis data streaming from KUKA.

    The KUKA sends a rapid stream of <Robot> XML messages containing
    $AXIS_ACT and $POS_ACT values. This server maintains a persistent
    TCP connection and processes each message as it arrives.

    Usage:
        server = EkiAxisStreamServer(
            host='0.0.0.0',
            port=59152,
            logger=ros_logger,
            on_message=callback,
        )
        server.start()
        # ... runs until stop() is called ...
        server.stop()
    """

    def __init__(
        self,
        host: str = '0.0.0.0',
        port: int = 59152,
        logger: Optional[object] = None,
        on_message: Optional[Callable] = None,
        receive_buffer_size: int = 8192,
        send_response: bool = False,
    ):
        """
        Initialize the axis stream server.

        Args:
            host: IP address to bind the server socket.
            port: TCP port to listen on.
            logger: ROS2 logger instance.
            on_message: Callback invoked with each complete XML message string.
            receive_buffer_size: TCP recv buffer size in bytes.
            send_response: If True, send a minimal Sensor XML after each message.
        """
        self._host = host
        self._port = port
        self._logger = logger
        self._on_message = on_message
        self._recv_size = receive_buffer_size
        self._send_response = send_response

        self._server_socket: Optional[socket.socket] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ── Logging helpers ──────────────────────────────────────────────

    def _log_info(self, msg: str) -> None:
        if self._logger:
            self._logger.info(msg)

    def _log_warn(self, msg: str) -> None:
        if self._logger:
            self._logger.warn(msg)

    def _log_error(self, msg: str) -> None:
        if self._logger:
            self._logger.error(msg)

    # ── Client handling ──────────────────────────────────────────────

    def _handle_client(
        self, client_socket: socket.socket, client_address: tuple
    ) -> None:
        """
        Handle a single streaming client connection.

        Uses TcpXmlBuffer to properly extract complete XML messages
        from the TCP byte stream, handling fragmentation and concatenation.
        """
        addr_str = f'{client_address[0]}:{client_address[1]}'
        self._log_info(f'KUKA connected: {addr_str}')

        xml_buffer = TcpXmlBuffer()

        try:
            while self._running:
                data = client_socket.recv(self._recv_size)

                if not data:
                    self._log_info(f'KUKA disconnected: {addr_str}')
                    break

                # Decode safely
                try:
                    decoded = data.decode('utf-8')
                except UnicodeDecodeError:
                    try:
                        decoded = data.decode('latin-1')
                    except Exception:
                        decoded = data.decode('utf-8', errors='replace')

                # Feed into buffer and extract complete messages
                messages = xml_buffer.feed(decoded)

                for xml_msg in messages:
                    # Invoke the node callback for each complete message
                    if self._on_message:
                        try:
                            self._on_message(xml_msg)
                        except Exception as cb_err:
                            self._log_error(f'Callback error: {cb_err}')

                    # Optionally send a keepalive response
                    if self._send_response:
                        try:
                            client_socket.sendall(
                                KEEPALIVE_SENSOR_XML.encode('utf-8')
                            )
                        except (BrokenPipeError, ConnectionResetError):
                            self._log_warn(
                                f'Failed to send response to {addr_str}'
                            )
                            return

        except ConnectionResetError:
            self._log_info(f'Connection reset by KUKA: {addr_str}')
        except socket.timeout:
            self._log_info(f'Connection timed out: {addr_str}')
        except Exception as e:
            self._log_error(f'Error with client {addr_str}: {e}')
        finally:
            try:
                client_socket.close()
            except Exception:
                pass
            xml_buffer.clear()

    # ── Server loop ──────────────────────────────────────────────────

    def _server_loop(self) -> None:
        """Main accept loop — runs in a background thread."""
        self._log_info(
            f'Axis stream server listening on {self._host}:{self._port}'
        )
        self._log_info('Waiting for KUKA connection...')

        while self._running:
            try:
                self._server_socket.settimeout(1.0)
                try:
                    client_sock, client_addr = self._server_socket.accept()
                except socket.timeout:
                    continue

                self._handle_client(client_sock, client_addr)

                if self._running:
                    self._log_info('Waiting for next KUKA connection...')

            except OSError as e:
                if self._running:
                    self._log_error(f'Accept error: {e}')
                break

        self._log_info('Axis stream server loop ended.')

    # ── Lifecycle ────────────────────────────────────────────────────

    def start(self) -> None:
        """Start the TCP server in a background thread."""
        try:
            self._server_socket = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM
            )
            self._server_socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
            )
            self._server_socket.bind((self._host, self._port))
            self._server_socket.listen(1)
        except OSError as e:
            self._log_error(
                f'Failed to bind on {self._host}:{self._port} — {e}'
            )
            if 'Address already in use' in str(e):
                self._log_error(
                    f'Port {self._port} is occupied. '
                    f'Check with: ss -tlnp | grep {self._port}'
                )
            raise RuntimeError(f'Cannot start axis stream server: {e}') from e

        self._running = True
        self._thread = threading.Thread(
            target=self._server_loop,
            daemon=True,
            name='eki_axis_stream_thread',
        )
        self._thread.start()
        self._log_info('Axis stream server started.')

    def stop(self) -> None:
        """Stop the server gracefully."""
        self._log_info('Stopping axis stream server...')
        self._running = False

        if self._server_socket:
            try:
                self._server_socket.close()
            except Exception:
                pass
            self._server_socket = None

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5.0)
            self._thread = None

        self._log_info('Axis stream server stopped.')
