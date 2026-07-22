"""
axis_xml_utils.py — XML utilities for KUKA axis streaming.

Provides TCP stream buffering with proper XML message extraction,
and parsing of axis/position data from KUKA Robot XML messages.
"""

import math
import re
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
from typing import Dict, List, Optional, Tuple


# Regex to extract complete <Robot ...>...</Robot> messages from a TCP stream.
# Handles both <Robot> and <Robot attr="val"> opening tags.
_ROBOT_MSG_PATTERN = re.compile(
    r'(<Robot[\s>].*?</Robot>)',
    re.DOTALL,
)


class TcpXmlBuffer:
    """
    Buffer that accumulates TCP stream data and extracts complete
    <Robot>...</Robot> XML messages.

    TCP streams may deliver:
      - Fragmented messages (one message split across multiple recv calls).
      - Concatenated messages (multiple messages in a single recv call).

    This buffer handles both cases transparently.
    """

    def __init__(self):
        self._buffer: str = ''

    def feed(self, data: str) -> List[str]:
        """
        Feed decoded string data into the buffer and extract any
        complete <Robot>...</Robot> messages.

        Args:
            data: Decoded string chunk received from the TCP socket.

        Returns:
            List of complete XML message strings (may be empty if no
            complete message is available yet).
        """
        self._buffer += data
        messages: List[str] = []

        # Extract all complete <Robot>...</Robot> messages
        while True:
            match = _ROBOT_MSG_PATTERN.search(self._buffer)
            if match is None:
                break
            messages.append(match.group(1))
            # Remove everything up to and including the matched message
            self._buffer = self._buffer[match.end():]

        # Safety: prevent unbounded buffer growth from garbage data
        if len(self._buffer) > 65536:
            self._buffer = self._buffer[-16384:]

        return messages

    def clear(self):
        """Clear the internal buffer."""
        self._buffer = ''


def parse_axis_stream_xml(xml_string: str) -> Optional[Dict]:
    """
    Parse a KUKA axis stream XML message and extract relevant fields.

    Expected XML structure:
        <Robot Seq="N" Mode="AxisStream">
          <Data>
            <Axis A1="..." A2="..." A3="..." A4="..." A5="..." A6="..."/>
            <Position X="..." Y="..." Z="..." A="..." B="..." C="..."/>
          </Data>
          <Status>...</Status>
          <Mode>...</Mode>
        </Robot>

    Args:
        xml_string: Complete <Robot>...</Robot> XML string.

    Returns:
        Dictionary with parsed fields, or None if parsing fails.
        Keys: 'seq', 'mode', 'status', 'axis' (dict A1-A6),
              'position' (dict X,Y,Z,A,B,C), 'raw_xml'.
    """
    try:
        root = ET.fromstring(xml_string.strip())
    except ET.ParseError:
        return None

    if root.tag != 'Robot':
        return None

    result: Dict = {
        'raw_xml': xml_string.strip(),
    }

    # --- Seq attribute on <Robot> ---
    seq = root.get('Seq')
    if seq is not None:
        result['seq'] = seq

    # --- Mode: check attribute first, then child element ---
    mode_attr = root.get('Mode')
    if mode_attr:
        result['mode'] = mode_attr
    mode_elem = root.find('.//Mode')
    if mode_elem is not None and mode_elem.text:
        result['mode'] = mode_elem.text.strip()

    # --- Status ---
    status_elem = root.find('.//Status')
    if status_elem is not None and status_elem.text:
        result['status'] = status_elem.text.strip()

    # --- Axis values (A1-A6) ---
    # Try multiple element names: Axis, AxisAct, AXIS_ACT
    axis_elem = None
    for tag in ['Axis', 'AxisAct', 'AXIS_ACT']:
        axis_elem = root.find(f'.//{tag}')
        if axis_elem is not None:
            break

    if axis_elem is not None:
        axis: Dict[str, float] = {}
        for attr in ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']:
            val = axis_elem.get(attr)
            if val is not None:
                try:
                    axis[attr] = float(val)
                except ValueError:
                    axis[attr] = 0.0
        if axis:
            result['axis'] = axis

    # --- Position (X, Y, Z, A, B, C) ---
    # Try multiple element names: Position, ActPos, POS_ACT
    pos_elem = None
    for tag in ['Position', 'ActPos', 'POS_ACT']:
        pos_elem = root.find(f'.//{tag}')
        if pos_elem is not None:
            break

    if pos_elem is not None:
        position: Dict[str, float] = {}
        for attr in ['X', 'Y', 'Z', 'A', 'B', 'C']:
            val = pos_elem.get(attr)
            if val is not None:
                try:
                    position[attr] = float(val)
                except ValueError:
                    position[attr] = 0.0
        if position:
            result['position'] = position

    return result


def format_compact_line(parsed: Dict) -> str:
    """
    Format parsed axis stream data as a compact single-line log string.

    Output example:
        Seq=15 | Mode=AxisStream | A1=0.00 A2=-90.00 A3=90.00 A4=0.00
        A5=0.00 A6=0.00 | X=1000.12 Y=0.00 Z=500.00

    Args:
        parsed: Dictionary from parse_axis_stream_xml().

    Returns:
        Formatted string for console display.
    """
    parts: List[str] = []

    # Seq
    seq = parsed.get('seq', '?')
    parts.append(f'Seq={seq}')

    # Mode
    mode = parsed.get('mode', '?')
    parts.append(f'Mode={mode}')

    # Axis values
    axis = parsed.get('axis', {})
    if axis:
        axis_parts = []
        for a in ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']:
            val = axis.get(a, 0.0)
            axis_parts.append(f'{a}={val:.2f}')
        parts.append(' '.join(axis_parts))

    # Position values
    pos = parsed.get('position', {})
    if pos:
        pos_parts = []
        for p in ['X', 'Y', 'Z', 'A', 'B', 'C']:
            val = pos.get(p, 0.0)
            pos_parts.append(f'{p}={val:.2f}')
        parts.append(' '.join(pos_parts))

    return ' | '.join(parts)


def axis_degrees_to_radians(axis: Dict[str, float]) -> List[float]:
    """
    Convert axis values from degrees to radians.

    Args:
        axis: Dictionary with keys A1-A6 in degrees.

    Returns:
        List of 6 float values in radians [A1, A2, A3, A4, A5, A6].
    """
    result = []
    for a in ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']:
        deg = axis.get(a, 0.0)
        result.append(math.radians(deg))
    return result


def pretty_xml(xml_string: str) -> Optional[str]:
    """
    Format an XML string with indentation for human-readable output.

    Args:
        xml_string: Raw XML string.

    Returns:
        Formatted XML string, or None if parsing fails.
    """
    try:
        xml_string = xml_string.strip()
        if not xml_string:
            return None
        dom = minidom.parseString(xml_string)
        pretty = dom.toprettyxml(indent='  ')
        lines = pretty.split('\n')
        if lines and lines[0].startswith('<?xml'):
            lines = lines[1:]
        while lines and not lines[-1].strip():
            lines.pop()
        return '\n'.join(lines)
    except Exception:
        return None
