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
import math
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

    # --- BATCH telemetry (absent on a baseline controller -> None) ---
    # Rides the SAME periodic frame that already carries RxCounter.
    for tag, key in (('BatchSeq', 'batch_seq'),
                     ('BatchConsumed', 'batch_consumed')):
        elem = root.find(tag)
        if elem is not None and elem.text:
            try:
                result[key] = int(elem.text.strip())
            except ValueError:
                result[key] = None
        else:
            result[key] = None

    batch_active_elem = root.find('BatchActive')
    if batch_active_elem is not None:
        result['batch_active'] = _parse_bool_text(batch_active_elem.text)
    else:
        result['batch_active'] = None

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
    trajectory_ptp_velocity_pct: float = 0.0,
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
          <TrajectoryPtpVelocityPct>0.0</TrajectoryPtpVelocityPct>
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
        trajectory_ptp_velocity_pct: PTP articular programado para un comando
                      de ENVIAR TRAYECTORIA, dentro de (0, 100]. 0.0 indica
                      un comando anterior/manual y no modifica su velocidad.

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

    try:
        effective_trajectory_velocity = float(
            trajectory_ptp_velocity_pct)
    except (TypeError, ValueError):
        effective_trajectory_velocity = 0.0
    if (not math.isfinite(effective_trajectory_velocity)
            or not 0.0 < effective_trajectory_velocity <= 100.0):
        effective_trajectory_velocity = 0.0

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
        f'<TrajectoryPtpVelocityPct>'
        f'{effective_trajectory_velocity:.4f}'
        f'</TrajectoryPtpVelocityPct>'
        # The batch memories must never be left empty. sps_submit_better.sub
        # reads all four on every packet, and reading an empty EKI memory
        # raises EKI00015, which trips the fail-safe and closes the channel.
        # BatchCount = 0 is the "no batch" discriminator, so these zeros are
        # inert: the SPS skips its whole batch block and leaves XD_BATCH_*
        # untouched. AbortBatch = 0 cannot clear a latched abort either --
        # the SPS only ever latches it on TRUE.
        f'<BatchSeq>0</BatchSeq>'
        f'<BatchCount>0</BatchCount>'
        f'<AbortBatch>0</AbortBatch>'
        f'<BatchPtpVelocityPct>0.0000</BatchPtpVelocityPct>'
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


# ---------------------------------------------------------------------------
# Batch command builders (NEW — the baseline file has no equivalent)
# ---------------------------------------------------------------------------

MAX_BATCH_SIZE_HARD_LIMIT = 20

_BATCH_AXES = ('A1', 'A2', 'A3', 'A4', 'A5', 'A6')


def _hold_axis_values(hold_target: Optional[Dict[str, float]]) -> List[float]:
    """A1..A6 for the inert single-point slot of a batch command."""
    hold = hold_target or {}
    defaults = (0.0, -90.0, 90.0, 0.0, 0.0, 0.0)
    values = []
    for axis, fallback in zip(_BATCH_AXES, defaults):
        try:
            value = float(hold.get(axis, fallback))
        except (TypeError, ValueError):
            value = fallback
        if not math.isfinite(value):
            value = fallback
        values.append(value)
    return values


def build_axis_move_batch_command_xml(
    seq: int,
    batch_seq: int,
    points_deg: List[Dict[str, float]],
    enable_move: bool,
    safe_mode: bool,
    allow_motion: bool,
    batch_ptp_velocity_pct: float = 0.0,
    abort_batch: bool = False,
    hold_target: Optional[Dict[str, float]] = None,
    max_batch_size: int = MAX_BATCH_SIZE_HARD_LIMIT,
) -> Optional[str]:
    """
    Build a <Command> carrying a whole batch of joint points.

    Structure — the six Command/Batch/@Ax memories each receive ONE entry per
    <Batch/> element, which is what lets the SPS drain the lot with six
    EKI_GetRealArray calls instead of 6 x N EKI_GetReal calls:

        <Command>
          <Seq>0</Seq>                          <- SIEMPRE 0, ver abajo
          <Mode>AxisTarget</Mode>
          <EnableMove>1</EnableMove>
          <AxisTarget A1="..." ... />          <- inert hold target
          <CartesianTarget .../>
          <GripperCommand>-1</GripperCommand>
          <TrajectoryPtpVelocityPct>0.0</TrajectoryPtpVelocityPct>
          <BatchSeq>7</BatchSeq>
          <BatchCount>20</BatchCount>
          <AbortBatch>0</AbortBatch>
          <BatchPtpVelocityPct>30.0</BatchPtpVelocityPct>
          <Batch A1="..." A2="..." ... A6="..."/>
          ... x BatchCount
        </Command>

    <Mode> deliberately stays "AxisTarget".  The SPS decides the mode from the
    FIRST CHARACTER of that string, so a value like "AxisTargetBatch" would be
    indistinguishable from "AxisTarget"; BatchCount > 0 is the discriminator.

    hold_target should be the robot's CURRENT position, never the first batch
    point.  XmlDualMove_better.src stands the single-point path down while a
    batch is pending, and a current-position hold is inert for the baseline
    program too.

    Command/Seq VA SIEMPRE A 0 en un paquete de lote, y el argumento `seq` se
    ignora a proposito.  Un paquete de lote NO lleva ninguna orden de punto
    suelto: su AxisTarget es relleno y su GripperCommand es -1.  Con un Seq
    positivo, el bloque de punto suelto de XmlDualMove_better.src ve un Seq
    que nadie ha atendido en cuanto termina el lote (batchSeq ==
    handledBatchSeq y batchRunning == FALSE) y ejecuta ese relleno como un
    PTP: el robot vuelve al punto donde EMPEZO el lote.  En los tramos cortos
    (por debajo de MAX_DELTA_JOINT = 10 grados) la validacion de delta no lo
    frena, y se ve como un rebote que ademas se come la orden de garra.
    0 es el centinela DOCUMENTADO de "el buzon no lleva ninguna orden": el
    interprete ignora todo Seq <= 0 y ademas hace handledSeq = 0, que es
    justo lo que queremos.  El siguiente comando real (Seq > 0) se atiende
    con normalidad.

    Returns:
        XML string, or None if the batch is malformed. Never a partial batch.
    """
    if not isinstance(points_deg, list) or not points_deg:
        return None
    if len(points_deg) > max_batch_size:
        return None
    if int(batch_seq) <= 0:
        return None

    rows: List[str] = []
    for point in points_deg:
        values: List[float] = []
        for axis in _BATCH_AXES:
            try:
                value = float(point.get(axis))
            except (TypeError, ValueError):
                return None
            if not math.isfinite(value):
                return None
            values.append(value)
        rows.append(
            '<Pt'
            f' A1="{values[0]:.4f}"'
            f' A2="{values[1]:.4f}"'
            f' A3="{values[2]:.4f}"'
            f' A4="{values[3]:.4f}"'
            f' A5="{values[4]:.4f}"'
            f' A6="{values[5]:.4f}"'
            '/>'
        )

    # A batch is a physical action: exactly the same two gates as any move.
    if safe_mode or not allow_motion:
        effective_enable = 0
    else:
        effective_enable = 1 if enable_move else 0

    try:
        velocity = float(batch_ptp_velocity_pct)
    except (TypeError, ValueError):
        velocity = 0.0
    if not math.isfinite(velocity) or not 0.0 < velocity <= 100.0:
        velocity = 0.0

    h = _hold_axis_values(hold_target)

    return (
        '<Command>'
        # 0 = "este paquete no trae orden de punto suelto". Ver el docstring.
        '<Seq>0</Seq>'
        '<Mode>AxisTarget</Mode>'
        f'<EnableMove>{effective_enable}</EnableMove>'
        '<AxisTarget'
        f' A1="{h[0]:.4f}" A2="{h[1]:.4f}" A3="{h[2]:.4f}"'
        f' A4="{h[3]:.4f}" A5="{h[4]:.4f}" A6="{h[5]:.4f}"'
        '/>'
        '<CartesianTarget'
        ' X="0.0000" Y="0.0000" Z="0.0000"'
        ' A="0.0000" B="0.0000" C="0.0000"'
        '/>'
        '<GripperCommand>-1</GripperCommand>'
        '<TrajectoryPtpVelocityPct>0.0000</TrajectoryPtpVelocityPct>'
        f'<BatchSeq>{int(batch_seq)}</BatchSeq>'
        f'<BatchCount>{len(rows)}</BatchCount>'
        f'<AbortBatch>{1 if abort_batch else 0}</AbortBatch>'
        f'<BatchPtpVelocityPct>{velocity:.4f}</BatchPtpVelocityPct>'
        + ''.join(rows) +
        '</Command>'
    )


def build_abort_batch_command_xml(
    seq: int,
    hold_target: Optional[Dict[str, float]] = None,
) -> str:
    """
    Build a <Command> whose only job is to raise AbortBatch on the KUKA.

    BatchCount = 0 so it can never start a batch, EnableMove = 0 so it can
    never move anything by itself.  The SPS latches the abort; the robot
    interpreter finishes the PTP in flight and does not start the next one.
    """
    h = _hold_axis_values(hold_target)
    return (
        '<Command>'
        f'<Seq>{int(seq)}</Seq>'
        '<Mode>AxisTarget</Mode>'
        '<EnableMove>0</EnableMove>'
        '<AxisTarget'
        f' A1="{h[0]:.4f}" A2="{h[1]:.4f}" A3="{h[2]:.4f}"'
        f' A4="{h[3]:.4f}" A5="{h[4]:.4f}" A6="{h[5]:.4f}"'
        '/>'
        '<CartesianTarget'
        ' X="0.0000" Y="0.0000" Z="0.0000"'
        ' A="0.0000" B="0.0000" C="0.0000"'
        '/>'
        '<GripperCommand>-1</GripperCommand>'
        '<TrajectoryPtpVelocityPct>0.0000</TrajectoryPtpVelocityPct>'
        '<BatchSeq>0</BatchSeq>'
        '<BatchCount>0</BatchCount>'
        '<AbortBatch>1</AbortBatch>'
        '<BatchPtpVelocityPct>0.0000</BatchPtpVelocityPct>'
        '</Command>'
    )
