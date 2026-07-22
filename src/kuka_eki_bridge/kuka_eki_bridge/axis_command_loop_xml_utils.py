"""
axis_command_loop_xml_utils.py — XML utilities for the axis command loop mode.

Provides:
  - TcpXmlCommandBuffer: extracts <Robot>...</Robot> frames from a TCP stream.
  - parse_command_loop_xml: parses incoming KUKA XML feedback.
  - build_command_xml: builds the <Command> XML response for the KUKA.

This module does NOT modify any existing module.
"""

import re
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional


# ---------------------------------------------------------------------------
# TCP stream buffer
# ---------------------------------------------------------------------------

_ROBOT_MSG_PATTERN = re.compile(
    r'(<Robot[\s>].*?</Robot>)',
    re.DOTALL,
)


class TcpXmlCommandBuffer:
    """
    Accumulates TCP stream data and extracts complete <Robot>...</Robot>
    XML messages.

    Handles both fragmented and concatenated TCP frames transparently.
    """

    def __init__(self):
        self._buffer: str = ''

    def feed(self, data: str) -> List[str]:
        """
        Feed a decoded string chunk and return any complete messages.

        Args:
            data: Decoded text received from the TCP socket.

        Returns:
            List of complete <Robot>...</Robot> strings.
        """
        self._buffer += data
        messages: List[str] = []

        while True:
            match = _ROBOT_MSG_PATTERN.search(self._buffer)
            if match is None:
                break
            messages.append(match.group(1))
            self._buffer = self._buffer[match.end():]

        # Guard against unbounded growth from garbage data.
        if len(self._buffer) > 65536:
            self._buffer = self._buffer[-16384:]

        return messages

    def clear(self):
        """Discard the internal buffer."""
        self._buffer = ''


# ---------------------------------------------------------------------------
# Incoming XML parser
# ---------------------------------------------------------------------------

def parse_command_loop_xml(xml_string: str) -> Optional[Dict]:
    """
    Parse a <Robot>...</Robot> message from the KUKA in CommandLoop mode.

    Expected structure:
        <Robot>
          <Seq>1</Seq>
          <Mode>CommandLoop</Mode>
          <Data>
            <AxisActual A1="0.0" A2="-90.0" A3="90.0"
                        A4="0.0"  A5="0.0"  A6="0.0"/>
            <PositionActual X="674.68" Y="-2.33" Z="885.20"
                            A="0.0" B="0.0" C="0.0"/>
          </Data>
          <Status>1</Status>
        </Robot>

    Args:
        xml_string: Complete XML string received from the KUKA.

    Returns:
        Dictionary with keys:
            'seq'              (int or str)
            'mode'             (str)
            'status'           (int or str)
            'axis_actual'      (dict A1-A6 float)
            'position_actual'  (dict X,Y,Z,A,B,C float)
        or None if parsing fails.
    """
    try:
        root = ET.fromstring(xml_string.strip())
    except ET.ParseError:
        return None

    if root.tag != 'Robot':
        return None

    result: Dict = {}

    # --- Seq ---
    seq_elem = root.find('Seq')
    if seq_elem is not None and seq_elem.text:
        try:
            result['seq'] = int(seq_elem.text.strip())
        except ValueError:
            result['seq'] = seq_elem.text.strip()
    else:
        result['seq'] = 0

    # --- Mode (child element preferred over attribute) ---
    mode_elem = root.find('Mode')
    if mode_elem is not None and mode_elem.text:
        result['mode'] = mode_elem.text.strip()
    else:
        result['mode'] = root.get('Mode', 'Unknown')

    # --- Status ---
    status_elem = root.find('Status')
    if status_elem is not None and status_elem.text:
        try:
            result['status'] = int(status_elem.text.strip())
        except ValueError:
            result['status'] = status_elem.text.strip()
    else:
        result['status'] = 0

    # --- AxisActual (A1-A6) ---
    data_elem = root.find('Data')
    axis_actual: Dict[str, float] = {}
    pos_actual: Dict[str, float] = {}

    if data_elem is not None:
        # AxisActual
        axis_elem = data_elem.find('AxisActual')
        if axis_elem is not None:
            for attr in ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']:
                val = axis_elem.get(attr)
                if val is not None:
                    try:
                        axis_actual[attr] = float(val)
                    except ValueError:
                        axis_actual[attr] = 0.0

        # PositionActual
        pos_elem = data_elem.find('PositionActual')
        if pos_elem is not None:
            for attr in ['X', 'Y', 'Z', 'A', 'B', 'C']:
                val = pos_elem.get(attr)
                if val is not None:
                    try:
                        pos_actual[attr] = float(val)
                    except ValueError:
                        pos_actual[attr] = 0.0

    result['axis_actual'] = axis_actual
    result['position_actual'] = pos_actual

    return result


# ---------------------------------------------------------------------------
# Outgoing XML builder
# ---------------------------------------------------------------------------

def build_command_xml(
    seq: int,
    target: Dict[str, float],
    enable_move: bool,
    safe_mode: bool,
) -> str:
    """
    Build the <Command> XML string to send back to the KUKA.

    Structure:
        <Command>
          <Seq>1</Seq>
          <Mode>AxisTarget</Mode>
          <EnableMove>0</EnableMove>
          <AxisTarget A1="0.0" A2="-90.0" A3="90.0"
                      A4="0.0"  A5="0.0"  A6="0.0"/>
        </Command>

    Args:
        seq:         Sequence number (mirrors the KUKA's Seq).
        target:      Dict with A1-A6 float values in degrees.
        enable_move: Requested enable-move flag from the GUI.
        safe_mode:   If True, EnableMove is always forced to 0.

    Returns:
        XML string ready to be sent over TCP.
    """
    # Safety: override enable_move when safe_mode is active
    effective_enable = 0 if safe_mode else (1 if enable_move else 0)

    a1 = target.get('A1', 0.0)
    a2 = target.get('A2', -90.0)
    a3 = target.get('A3', 90.0)
    a4 = target.get('A4', 0.0)
    a5 = target.get('A5', 0.0)
    a6 = target.get('A6', 0.0)

    xml = (
        f'<Command>'
        f'<Seq>{seq}</Seq>'
        f'<Mode>AxisTarget</Mode>'
        f'<EnableMove>{effective_enable}</EnableMove>'
        f'<AxisTarget'
        f' A1="{a1:.4f}"'
        f' A2="{a2:.4f}"'
        f' A3="{a3:.4f}"'
        f' A4="{a4:.4f}"'
        f' A5="{a5:.4f}"'
        f' A6="{a6:.4f}"'
        f'/>'
        f'</Command>'
    )
    return xml


# ---------------------------------------------------------------------------
# Compact log formatter
# ---------------------------------------------------------------------------

def format_command_loop_log(
    seq: int,
    mode: str,
    actual: Dict[str, float],
    target: Dict[str, float],
    enable_move: int,
) -> str:
    """
    Format a compact single-line log message for each command loop cycle.

    Example:
        Seq=15 | Mode=CommandLoop | Actual A1=0.00 A2=-90.00 ... | Target A1=... | EnableMove=0

    Args:
        seq:         Sequence number.
        mode:        Mode string from the KUKA.
        actual:      Dict A1-A6 actual values.
        target:      Dict A1-A6 target values.
        enable_move: Effective enable move flag (0 or 1).

    Returns:
        Formatted string.
    """
    def fmt_axis(d: Dict[str, float]) -> str:
        return ' '.join(
            f'{k}={d.get(k, 0.0):.2f}' for k in ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']
        )

    return (
        f'Seq={seq} | Mode={mode} | '
        f'Actual {fmt_axis(actual)} | '
        f'Target {fmt_axis(target)} | '
        f'EnableMove={enable_move}'
    )
