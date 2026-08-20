"""
axis_move_xml_utils.py — XML utilities for the XmlAxisMove mode.

Provides:
  - TcpXmlAxisMoveBuffer: extracts <Robot>...</Robot> frames from a TCP stream.
  - parse_axis_move_xml: parses incoming KUKA XML feedback (with MoveReady,
    LimitsOK, DeltaOK, MoveExecuted).
  - build_axis_move_command_xml: builds the <Command> XML response for the KUKA.
  - format_axis_move_log: compact single-line log for each cycle.

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


class TcpXmlAxisMoveBuffer:
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
# Boolean parser helper
# ---------------------------------------------------------------------------

def _parse_bool_text(text: Optional[str]) -> Optional[bool]:
    """Parse '0'/'1'/'true'/'false' to Python bool, or None."""
    if text is None:
        return None
    t = text.strip().lower()
    if t in ('1', 'true'):
        return True
    if t in ('0', 'false'):
        return False
    return None


# ---------------------------------------------------------------------------
# Incoming XML parser
# ---------------------------------------------------------------------------

def parse_axis_move_xml(xml_string: str) -> Optional[Dict]:
    """
    Parse a <Robot>...</Robot> message from the KUKA in AxisMove mode.

    Expected structure:
        <Robot>
          <Seq>1</Seq>
          <Mode>AxisMove</Mode>
          <Data>
            <AxisActual A1="0.0" A2="-90.0" A3="90.0"
                        A4="0.0"  A5="0.0"  A6="0.0"/>
            <PositionActual X="674.68" Y="-2.33" Z="885.20"
                            A="0.0" B="0.0" C="0.0"/>
          </Data>
          <Status>1</Status>
          <MoveReady>0</MoveReady>
          <LimitsOK>1</LimitsOK>
          <DeltaOK>1</DeltaOK>
          <MoveExecuted>0</MoveExecuted>
          <RxCounter>42</RxCounter>
        </Robot>

    Args:
        xml_string: Complete XML string received from the KUKA.

    Returns:
        Dictionary with keys:
            'seq'              (int)
            'mode'             (str)
            'status'           (int)
            'move_ready'       (bool)
            'limits_ok'        (bool)
            'delta_ok'         (bool)
            'move_executed'    (bool)
            'axis_actual'      (dict A1-A6 float)
            'position_actual'  (dict X,Y,Z,A,B,C float)
            'rx_counter'       (int or None)
        or None if parsing fails.

    rx_counter is the SPS's own count of COMPLETE commands taken out of the
    EthernetKRL receive buffer. It is None when the KUKA has not been updated
    with the Robot/RxCounter element yet, which lets the bridge fall back to
    its timing heuristic instead of assuming zero consumption.
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
            result['seq'] = 0
    else:
        result['seq'] = 0

    # --- Mode ---
    mode_elem = root.find('Mode')
    if mode_elem is not None and mode_elem.text:
        result['mode'] = mode_elem.text.strip()
    else:
        result['mode'] = root.get('Mode', 'Unknown')

    # --- RxCounter (explicit RX acknowledgement; absent on older configs) ---
    rx_elem = root.find('RxCounter')
    if rx_elem is not None and rx_elem.text:
        try:
            result['rx_counter'] = int(rx_elem.text.strip())
        except ValueError:
            result['rx_counter'] = None
    else:
        result['rx_counter'] = None

    # --- Status ---
    status_elem = root.find('Status')
    if status_elem is not None and status_elem.text:
        try:
            result['status'] = int(status_elem.text.strip())
        except ValueError:
            result['status'] = 0
    else:
        result['status'] = 0

    # --- Boolean fields ---
    for field, key in [
        ('MoveReady', 'move_ready'),
        ('LimitsOK', 'limits_ok'),
        ('DeltaOK', 'delta_ok'),
        ('MoveExecuted', 'move_executed'),
    ]:
        elem = root.find(field)
        if elem is not None and elem.text:
            val = _parse_bool_text(elem.text)
            result[key] = val if val is not None else False
        else:
            result[key] = False

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

def build_axis_move_command_xml(
    seq: int,
    target: Dict[str, float],
    enable_move: bool,
    safe_mode: bool,
    allow_motion: bool,
    cartesian_target: Optional[Dict[str, float]] = None,
    mode: str = 'AxisTarget',
    gripper_command: int = -1,
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
          <CartesianTarget X="0.0" Y="0.0" Z="0.0"
                           A="0.0" B="0.0" C="0.0"/>
          <GripperCommand>-1</GripperCommand>
        </Command>

    Args:
        seq:          Sequence number from the GUI command.
        target:       Dict with A1-A6 float values in degrees.
        enable_move:  Requested enable-move flag from the GUI.
        safe_mode:    If True, EnableMove is always forced to 0.
        allow_motion: If False, EnableMove is always forced to 0.
        cartesian_target: Optional dict with X, Y, Z, A, B, C values.
        mode:         'AxisTarget' or 'CartesianTarget'.
        gripper_command: -1 do nothing, 0 open, 1 close. Anything else is
                      coerced to -1, so a malformed value can never move the
                      gripper. Forced to -1 whenever motion is gated off,
                      because the gripper is a physical action too.

    Returns:
        XML string ready to be sent over TCP.
    """
    # Multi-layer safety: both gates must be open
    if safe_mode or not allow_motion:
        effective_enable = 0
    else:
        effective_enable = 1 if enable_move else 0

    # The gripper is a physical action, so it passes through exactly the same
    # gates as motion, and any unexpected value degrades to "do nothing".
    if safe_mode or not allow_motion:
        effective_gripper = -1
    elif gripper_command in (0, 1):
        effective_gripper = int(gripper_command)
    else:
        effective_gripper = -1

    a1 = target.get('A1', 0.0)
    a2 = target.get('A2', -90.0)
    a3 = target.get('A3', 90.0)
    a4 = target.get('A4', 0.0)
    a5 = target.get('A5', 0.0)
    a6 = target.get('A6', 0.0)
    
    if cartesian_target is None:
        cartesian_target = {}
        
    x = cartesian_target.get('X', 0.0)
    y = cartesian_target.get('Y', 0.0)
    z = cartesian_target.get('Z', 0.0)
    ca = cartesian_target.get('A', 0.0)
    cb = cartesian_target.get('B', 0.0)
    cc = cartesian_target.get('C', 0.0)

    xml = (
        f'<Command>'
        f'<Seq>{seq}</Seq>'
        f'<Mode>{mode}</Mode>'
        f'<EnableMove>{effective_enable}</EnableMove>'
        f'<AxisTarget'
        f' A1="{a1:.4f}"'
        f' A2="{a2:.4f}"'
        f' A3="{a3:.4f}"'
        f' A4="{a4:.4f}"'
        f' A5="{a5:.4f}"'
        f' A6="{a6:.4f}"'
        f'/>'
        f'<CartesianTarget'
        f' X="{x:.4f}"'
        f' Y="{y:.4f}"'
        f' Z="{z:.4f}"'
        f' A="{ca:.4f}"'
        f' B="{cb:.4f}"'
        f' C="{cc:.4f}"'
        f'/>'
        f'<GripperCommand>{effective_gripper}</GripperCommand>'
        f'</Command>'
    )
    return xml


# ---------------------------------------------------------------------------
# Compact log formatter
# ---------------------------------------------------------------------------

def format_axis_move_log(
    seq: int,
    mode: str,
    actual: Dict[str, float],
    target: Dict[str, float],
    limits_ok: bool,
    delta_ok: bool,
    enable_move: int,
    safe_mode: bool,
) -> str:
    """
    Format a compact single-line log message for each axis move cycle.

    Example:
        Seq=15 | Mode=AxisMove | Actual A1=0.00 ... | Target A1=1.00 ...
        | LimitsOK=True | DeltaOK=True | EnableMove=0 | SafeMode=True
    """
    def fmt_axis(d: Dict[str, float]) -> str:
        return ' '.join(
            f'{k}={d.get(k, 0.0):.2f}' for k in ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']
        )

    return (
        f'Seq={seq} | Mode={mode} | '
        f'Actual {fmt_axis(actual)} | '
        f'Target {fmt_axis(target)} | '
        f'LimitsOK={limits_ok} | DeltaOK={delta_ok} | '
        f'EnableMove={enable_move} | SafeMode={safe_mode}'
    )
