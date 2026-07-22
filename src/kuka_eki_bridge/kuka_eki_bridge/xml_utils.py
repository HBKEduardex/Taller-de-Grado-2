"""
xml_utils.py — Utility functions for XML parsing and formatting.

Provides safe decoding, pretty-printing, and field extraction for
XML messages exchanged with the KUKA robot via EthernetKRL.
"""

import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
from typing import Dict, Optional


def safe_decode(data: bytes, encoding: str = 'utf-8') -> str:
    """
    Safely decode raw bytes to a string.

    Handles common encoding issues that may occur when receiving
    data from the KUKA controller over TCP.

    Args:
        data: Raw bytes received from the socket.
        encoding: Character encoding to use (default: utf-8).

    Returns:
        Decoded string, or a replacement-character string if decoding fails.
    """
    try:
        return data.decode(encoding)
    except UnicodeDecodeError:
        # Fall back to latin-1 which accepts any byte value
        try:
            return data.decode('latin-1')
        except Exception:
            return data.decode(encoding, errors='replace')


def pretty_xml(xml_string: str) -> Optional[str]:
    """
    Format an XML string with indentation for human-readable output.

    Args:
        xml_string: Raw XML string.

    Returns:
        Formatted XML string with indentation, or None if parsing fails.
    """
    try:
        # Strip any leading/trailing whitespace
        xml_string = xml_string.strip()
        if not xml_string:
            return None

        dom = minidom.parseString(xml_string)
        # toprettyxml adds an XML declaration; remove it for cleaner output
        pretty = dom.toprettyxml(indent='  ')
        # Remove the XML declaration line
        lines = pretty.split('\n')
        if lines and lines[0].startswith('<?xml'):
            lines = lines[1:]
        # Remove trailing empty lines
        while lines and not lines[-1].strip():
            lines.pop()
        return '\n'.join(lines)
    except Exception:
        return None


def parse_robot_xml(xml_string: str) -> Optional[ET.Element]:
    """
    Parse an XML string into an ElementTree Element.

    Args:
        xml_string: Raw XML string from the KUKA robot.

    Returns:
        Root Element of the parsed XML, or None if parsing fails.
    """
    try:
        xml_string = xml_string.strip()
        if not xml_string:
            return None
        return ET.fromstring(xml_string)
    except ET.ParseError:
        return None


def extract_robot_fields(xml_string: str) -> Dict[str, str]:
    """
    Extract commonly used fields from a KUKA robot XML message.

    Looks for fields such as:
      - Robot/Mode (e.g., "T1", "T2", "AUT", "EXT")
      - Robot/Status
      - Robot/Data/ActPos (position attributes: X, Y, Z, A, B, C)
      - Any top-level element text values

    Args:
        xml_string: Raw XML string from the KUKA robot.

    Returns:
        Dictionary with extracted field names and their values.
        Returns an empty dict if the XML cannot be parsed.
    """
    fields: Dict[str, str] = {}

    root = parse_robot_xml(xml_string)
    if root is None:
        return fields

    # Store the root tag name
    fields['RootTag'] = root.tag

    # --- Extract Mode ---
    mode_elem = root.find('.//Mode')
    if mode_elem is not None and mode_elem.text:
        fields['Mode'] = mode_elem.text.strip()

    # --- Extract Status ---
    status_elem = root.find('.//Status')
    if status_elem is not None and status_elem.text:
        fields['Status'] = status_elem.text.strip()

    # --- Extract ActPos (actual position) ---
    actpos_elem = root.find('.//ActPos')
    if actpos_elem is not None:
        pos_attrs = {}
        for attr in ['X', 'Y', 'Z', 'A', 'B', 'C']:
            val = actpos_elem.get(attr)
            if val is not None:
                pos_attrs[attr] = val
        if pos_attrs:
            fields['ActPos'] = str(pos_attrs)

    # --- Extract axis values if present (for future /joint_states) ---
    axis_elem = root.find('.//AxisAct')
    if axis_elem is not None:
        axis_attrs = {}
        for attr in ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']:
            val = axis_elem.get(attr)
            if val is not None:
                axis_attrs[attr] = val
        if axis_attrs:
            fields['AxisAct'] = str(axis_attrs)

    # --- Extract any direct children with text content ---
    for child in root:
        if child.text and child.text.strip() and child.tag not in fields:
            fields[child.tag] = child.text.strip()

    return fields
