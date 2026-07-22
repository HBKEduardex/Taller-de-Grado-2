"""
eki_protocol.py — TCP server for KUKA EthernetKRL (EKI) communication.

Implements the EkiXmlServer class that:
  - Opens a TCP server socket on a configurable host/port.
  - Accepts incoming connections from the KUKA controller.
  - Receives XML data sent by XmlTransmit.src.
  - Responds with a Sensor XML (loaded from file or using a default).
  - Handles connection close/reopen cycles from the KUKA.
  - Provides clean start/stop lifecycle methods.
"""

import socket
import threading
import os
from typing import Callable, Optional

from kuka_eki_bridge.xml_utils import safe_decode


# ─────────────────────────────────────────────────────────────────────
# Default Sensor XML response (used if no external file is provided)
# This matches the structure expected by KUKA EthernetKRL examples.
# ─────────────────────────────────────────────────────────────────────
DEFAULT_SENSOR_XML = """\
<Sensor>
  <Message>Example message</Message>
  <Positions>
    <Current X="4645.2"/>
    <Before>
      <X>123.4</X>
    </Before>
  </Positions>
  <Nmb>8</Nmb>
  <Status>
    <IsActive>1</IsActive>
  </Status>
  <Read>
    <xyzabc X="210.3" Y="825.3" Z="234.3" A="84.2" B="12.3" C="43.5"/>
  </Read>
  <Show error="0" temp="25">OK</Show>
  <Free>2912</Free>
</Sensor>"""


class EkiXmlServer:
    """
    TCP server that communicates with a KUKA robot via EthernetKRL XML protocol.

    Usage:
        server = EkiXmlServer(
            host='0.0.0.0',
            port=59152,
            logger=some_logger,
            on_data_received=callback_function,
        )
        server.start()
        # ... server runs until stop() is called ...
        server.stop()
    """

    def __init__(
        self,
        host: str = '0.0.0.0',
        port: int = 59152,
        logger: Optional[object] = None,
        on_data_received: Optional[Callable] = None,
        response_xml_path: str = '',
        receive_buffer_size: int = 8192,
        keep_running: bool = True,
    ):
        """
        Initialize the EKI XML server.

        Args:
            host: IP address to bind the server socket.
            port: TCP port number to listen on.
            logger: ROS2 logger instance (or any object with .info/.warn/.error).
            on_data_received: Optional callback invoked with (client_addr, raw_data).
            response_xml_path: Path to an XML file for the sensor response.
                               If empty or file not found, DEFAULT_SENSOR_XML is used.
            receive_buffer_size: Size of the TCP receive buffer in bytes.
            keep_running: If True, the server keeps accepting new connections
                          after a client disconnects.
        """
        self._host = host
        self._port = port
        self._logger = logger
        self._on_data_received = on_data_received
        self._response_xml_path = response_xml_path
        self._receive_buffer_size = receive_buffer_size
        self._keep_running = keep_running

        self._server_socket: Optional[socket.socket] = None
        self._running = False
        self._server_thread: Optional[threading.Thread] = None
        self._response_xml: str = ''

    # ── Logging helpers ──────────────────────────────────────────────

    def _log_info(self, msg: str) -> None:
        if self._logger:
            self._logger.info(msg)
        else:
            print(f'[INFO] {msg}')

    def _log_warn(self, msg: str) -> None:
        if self._logger:
            self._logger.warn(msg)
        else:
            print(f'[WARN] {msg}')

    def _log_error(self, msg: str) -> None:
        if self._logger:
            self._logger.error(msg)
        else:
            print(f'[ERROR] {msg}')

    # ── Response XML loading ─────────────────────────────────────────

    def load_response_xml(self) -> str:
        """
        Load the sensor response XML.

        Tries to read from the file specified by `response_xml_path`.
        Falls back to DEFAULT_SENSOR_XML if the file doesn't exist or is empty.

        Returns:
            The XML string that will be sent as a response to the KUKA.
        """
        if self._response_xml_path and os.path.isfile(self._response_xml_path):
            try:
                with open(self._response_xml_path, 'r', encoding='utf-8') as f:
                    content = f.read().strip()
                if content:
                    self._log_info(
                        f'Loaded response XML from: {self._response_xml_path}'
                    )
                    return content
                else:
                    self._log_warn(
                        f'Response XML file is empty: {self._response_xml_path}'
                        ' — using default.'
                    )
            except Exception as e:
                self._log_warn(
                    f'Could not read response XML file: {e} — using default.'
                )
        else:
            if self._response_xml_path:
                self._log_warn(
                    f'Response XML file not found: {self._response_xml_path}'
                    ' — using default.'
                )
            else:
                self._log_info('No response XML path configured — using default.')

        return DEFAULT_SENSOR_XML

    # ── Client handling ──────────────────────────────────────────────

    def handle_client(
        self, client_socket: socket.socket, client_address: tuple
    ) -> None:
        """
        Handle a single client connection from the KUKA robot.

        Receives XML data, invokes the callback, sends the response XML,
        and then allows the client to close the connection gracefully.

        Args:
            client_socket: The connected client socket.
            client_address: Tuple of (ip, port) of the connected client.
        """
        addr_str = f'{client_address[0]}:{client_address[1]}'
        self._log_info(f'Client connected: {addr_str}')

        try:
            while self._running:
                # Receive data from the KUKA
                data = client_socket.recv(self._receive_buffer_size)

                if not data:
                    # Client closed the connection
                    self._log_info(f'Client disconnected: {addr_str}')
                    break

                # Decode the received bytes
                raw_xml = safe_decode(data)

                # Invoke the data callback (the ROS2 node uses this to log)
                if self._on_data_received:
                    try:
                        self._on_data_received(client_address, raw_xml)
                    except Exception as cb_err:
                        self._log_error(f'Callback error: {cb_err}')

                # Send the sensor response XML back to the KUKA
                try:
                    response_bytes = self._response_xml.encode('utf-8')
                    client_socket.sendall(response_bytes)
                    self._log_info(
                        f'Sent response to {addr_str} '
                        f'({len(response_bytes)} bytes)'
                    )
                except (BrokenPipeError, ConnectionResetError) as send_err:
                    self._log_warn(f'Send failed to {addr_str}: {send_err}')
                    break

        except ConnectionResetError:
            self._log_info(f'Connection reset by client: {addr_str}')
        except socket.timeout:
            self._log_info(f'Connection timed out: {addr_str}')
        except Exception as e:
            self._log_error(f'Error handling client {addr_str}: {e}')
        finally:
            try:
                client_socket.close()
            except Exception:
                pass
            self._log_info(f'Connection closed: {addr_str}')

    # ── Server lifecycle ─────────────────────────────────────────────

    def _server_loop(self) -> None:
        """
        Main server loop: accept connections and handle them.

        Runs in a separate thread so the ROS2 spin can continue.
        """
        self._log_info(
            f'TCP server listening on {self._host}:{self._port}'
        )
        self._log_info('Waiting for KUKA connection...')

        while self._running:
            try:
                # Accept a new connection (blocks until one arrives)
                self._server_socket.settimeout(1.0)
                try:
                    client_sock, client_addr = self._server_socket.accept()
                except socket.timeout:
                    # No connection yet; loop back to check self._running
                    continue

                # Handle the client in the current thread (one client at a time)
                self.handle_client(client_sock, client_addr)

                if not self._keep_running:
                    self._log_info('keep_running=False — shutting down after '
                                   'first client.')
                    break

                self._log_info('Waiting for next KUKA connection...')

            except OSError as e:
                if self._running:
                    self._log_error(f'Socket error in accept loop: {e}')
                break

        self._log_info('Server loop ended.')

    def start(self) -> None:
        """
        Start the TCP server.

        Creates the socket, binds it, and begins listening in a background
        thread. Raises RuntimeError if the port is already in use.
        """
        # Load the response XML before starting
        self._response_xml = self.load_response_xml()

        try:
            self._server_socket = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM
            )
            # Allow immediate port reuse after restart (avoids "Address in use")
            self._server_socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
            )
            self._server_socket.bind((self._host, self._port))
            self._server_socket.listen(1)
        except OSError as e:
            self._log_error(
                f'Failed to bind TCP server on {self._host}:{self._port} — {e}'
            )
            if 'Address already in use' in str(e):
                self._log_error(
                    f'Port {self._port} is occupied. Check with: '
                    f'ss -tlnp | grep {self._port}'
                )
            raise RuntimeError(f'Cannot start EKI server: {e}') from e

        self._running = True
        self._server_thread = threading.Thread(
            target=self._server_loop,
            daemon=True,
            name='eki_xml_server_thread',
        )
        self._server_thread.start()
        self._log_info('EKI XML Server started.')

    def stop(self) -> None:
        """
        Stop the TCP server gracefully.

        Signals the server loop to exit and closes the listening socket.
        """
        self._log_info('Stopping EKI XML Server...')
        self._running = False

        # Close the server socket to unblock accept()
        if self._server_socket:
            try:
                self._server_socket.close()
            except Exception:
                pass
            self._server_socket = None

        # Wait for the server thread to finish
        if self._server_thread and self._server_thread.is_alive():
            self._server_thread.join(timeout=5.0)
            self._server_thread = None

        self._log_info('EKI XML Server stopped.')
