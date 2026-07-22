"""
eki_axis_command_loop_server.py — Bidirectional TCP server for KUKA command loop.

This server:
  - Listens for a KUKA TCP connection.
  - Receives continuous <Robot>...</Robot> XML feedback from KUKA.
  - Responds to each message with a <Command> XML containing the latest
    axis target received from the GUI via ROS2 topic.
  - Keeps the connection open for the entire session.
  - Supports reconnection after KUKA disconnects.
  - Handles TCP fragmentation and concatenation transparently.

This module does NOT modify any existing module.
"""

import socket
import threading
from typing import Callable, Dict, Optional

from kuka_eki_bridge.axis_command_loop_xml_utils import (
    TcpXmlCommandBuffer,
    parse_command_loop_xml,
    build_command_xml,
)


class EkiAxisCommandLoopServer:
    """
    Bidirectional TCP server for the KUKA axis command loop.

    The server receives <Robot> XML messages from the KUKA containing
    current axis/position feedback, and responds with <Command> XML
    messages containing the latest target from the GUI.

    Usage:
        server = EkiAxisCommandLoopServer(
            host='0.0.0.0',
            port=59153,
            logger=ros_logger,
            on_feedback=callback,      # called with parsed dict
            get_command_xml=provider,  # called to get the XML to send back
        )
        server.start()
        # ... runs until stop() is called ...
        server.stop()
    """

    def __init__(
        self,
        host: str = '0.0.0.0',
        port: int = 59153,
        logger: Optional[object] = None,
        on_feedback: Optional[Callable[[Dict, str], None]] = None,
        get_command_xml: Optional[Callable[[int], str]] = None,
        receive_buffer_size: int = 8192,
        log_raw_xml: bool = False,
        log_command_xml: bool = True,
    ):
        """
        Initialize the command loop server.

        Args:
            host:               IP address to bind the server socket.
            port:               TCP port to listen on.
            logger:             ROS2 logger instance.
            on_feedback:        Callback invoked with (parsed_dict, raw_xml_string)
                                for every complete <Robot> message received.
            get_command_xml:    Provider called with (seq: int) that returns
                                the <Command> XML string to send back to KUKA.
            receive_buffer_size: TCP recv buffer size in bytes.
            log_raw_xml:        If True, log raw incoming XML.
            log_command_xml:    If True, log the XML sent to the KUKA.
        """
        self._host = host
        self._port = port
        self._logger = logger
        self._on_feedback = on_feedback
        self._get_command_xml = get_command_xml
        self._recv_size = receive_buffer_size
        self._log_raw_xml = log_raw_xml
        self._log_command_xml = log_command_xml

        self._server_socket: Optional[socket.socket] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ── Logging helpers ──────────────────────────────────────────────

    def _info(self, msg: str) -> None:
        if self._logger:
            self._logger.info(msg)

    def _warn(self, msg: str) -> None:
        if self._logger:
            self._logger.warn(msg)

    def _error(self, msg: str) -> None:
        if self._logger:
            self._logger.error(msg)

    # ── Client handling ──────────────────────────────────────────────

    def _handle_client(
        self, client_socket: socket.socket, client_address: tuple
    ) -> None:
        """
        Handle a single KUKA connection.

        Maintains the connection open, reads continuous XML feedback,
        and replies with a <Command> XML for each received message.
        """
        addr_str = f'{client_address[0]}:{client_address[1]}'
        self._info(f'KUKA connected (command loop): {addr_str}')

        xml_buffer = TcpXmlCommandBuffer()

        try:
            while self._running:
                data = client_socket.recv(self._recv_size)

                if not data:
                    self._info(f'KUKA disconnected: {addr_str}')
                    break

                # Decode safely
                try:
                    decoded = data.decode('utf-8')
                except UnicodeDecodeError:
                    try:
                        decoded = data.decode('latin-1')
                    except Exception:
                        decoded = data.decode('utf-8', errors='replace')

                if self._log_raw_xml:
                    self._info(f'[RAW RX] {decoded}')

                messages = xml_buffer.feed(decoded)

                for xml_msg in messages:
                    # Parse the feedback
                    parsed = parse_command_loop_xml(xml_msg)

                    if parsed is None:
                        self._warn(
                            f'Malformed XML from KUKA — skipping. '
                            f'Content: {xml_msg[:120]}'
                        )
                        continue

                    # Deliver feedback to the ROS2 node
                    if self._on_feedback:
                        try:
                            self._on_feedback(parsed, xml_msg)
                        except Exception as cb_err:
                            self._error(f'Feedback callback error: {cb_err}')

                    # Build the command XML
                    seq = parsed.get('seq', 0)
                    command_xml: Optional[str] = None
                    if self._get_command_xml:
                        try:
                            command_xml = self._get_command_xml(seq)
                        except Exception as cmd_err:
                            self._error(f'Command provider error: {cmd_err}')

                    if command_xml is None:
                        continue

                    if self._log_command_xml:
                        self._info(f'[CMD TX] {command_xml}')

                    # Send the response
                    try:
                        client_socket.sendall(command_xml.encode('utf-8'))
                    except (BrokenPipeError, ConnectionResetError) as send_err:
                        self._warn(
                            f'Send failed to {addr_str}: {send_err}'
                        )
                        return

        except ConnectionResetError:
            self._info(f'Connection reset by KUKA: {addr_str}')
        except socket.timeout:
            self._info(f'Connection timed out: {addr_str}')
        except Exception as e:
            self._error(f'Unexpected error with client {addr_str}: {e}')
        finally:
            try:
                client_socket.close()
            except Exception:
                pass
            xml_buffer.clear()

    # ── Server accept loop ───────────────────────────────────────────

    def _server_loop(self) -> None:
        """Main accept loop — runs in a background thread."""
        self._info(
            f'Command loop server listening on {self._host}:{self._port}'
        )
        self._info('Waiting for KUKA connection...')

        while self._running:
            try:
                self._server_socket.settimeout(1.0)
                try:
                    client_sock, client_addr = self._server_socket.accept()
                except socket.timeout:
                    continue

                self._handle_client(client_sock, client_addr)

                if self._running:
                    self._info('Waiting for next KUKA connection...')

            except OSError as e:
                if self._running:
                    self._error(f'Accept error: {e}')
                break

        self._info('Command loop server loop ended.')

    # ── Lifecycle ────────────────────────────────────────────────────

    def start(self) -> None:
        """Bind the socket and start the accept loop in a background thread."""
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
            self._error(
                f'Failed to bind on {self._host}:{self._port} — {e}'
            )
            if 'Address already in use' in str(e):
                self._error(
                    f'Port {self._port} is already in use. '
                    f'Check with:  ss -tlnp | grep {self._port}'
                )
            raise RuntimeError(
                f'Cannot start command loop server: {e}'
            ) from e

        self._running = True
        self._thread = threading.Thread(
            target=self._server_loop,
            daemon=True,
            name='eki_command_loop_thread',
        )
        self._thread.start()
        self._info('Command loop server started.')

    def stop(self) -> None:
        """Stop the server gracefully."""
        self._info('Stopping command loop server...')
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

        self._info('Command loop server stopped.')
