#!/usr/bin/env python3
"""
analyze_log.py — offline analysis of a kuka_telemetry_logger CSV.

Pure Python standard library. It does NOT import ROS2 and does not need a
sourced workspace. Run it anywhere, on any machine, at any time.

Usage:
    python3 analyze_log.py logs/kuka_telemetry_20260817_183521.csv
    python3 analyze_log.py logs/kuka_telemetry_20260817_183521.csv --no-histogram
    python3 analyze_log.py logs/kuka_telemetry_20260817_183521.csv --top-gaps 50

Reports:
    * total samples, total duration, first/last timestamps
    * average rate, plus the min/max instantaneous rate
    * inter-arrival delta statistics (mean / min / max / stdev / median)
    * sequence gap count and the individual gaps
    * every column found, with its NULL/empty count
    * min/max/mean for A1..A6 when joint columns are present
    * min/max/mean for X/Y/Z/A/B/C when Cartesian columns are present
    * an optional ASCII histogram of the inter-arrival deltas
"""

import argparse
import csv
import os
import re
import statistics
import sys
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

# Column names written by the logger.
TIME_NS_COLUMN = 'receive_ros_time_ns'
WALL_COLUMN = 'receive_wall_time_iso8601'
DELTA_COLUMN = 'delta_receive_ms'
SEQ_COLUMN = 'sequence'
DELTA_SEQ_COLUMN = 'delta_seq'

_JOINT_RE = re.compile(r'(?:^|[._])([Aa][1-6])$')
_CART_RE = re.compile(r'(?:^|[._])([XYZABCxyzabc])$')

JOINT_ORDER = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']
CART_ORDER = ['X', 'Y', 'Z', 'A', 'B', 'C']


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

def load_csv(path: str) -> Tuple[List[str], List[Dict[str, str]]]:
    """Read the CSV into a column list and a list of row dicts."""
    with open(path, 'r', newline='', encoding='utf-8') as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise SystemExit(f'ERROR: {path} has no header row.')
        columns = list(reader.fieldnames)
        rows = [row for row in reader]
    return columns, rows


def as_float(value: Optional[str]) -> Optional[float]:
    """Parse a CSV cell as float; empty/None/non-numeric becomes None."""
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def as_int(value: Optional[str]) -> Optional[int]:
    """Parse a CSV cell as int; empty/None/non-numeric becomes None."""
    number = as_float(value)
    return None if number is None else int(number)


def is_null_cell(value: Optional[str]) -> bool:
    """True when a cell carries no value."""
    return value is None or value.strip() == ''


# ---------------------------------------------------------------------------
# Column classification
# ---------------------------------------------------------------------------

def classify_columns(columns: List[str]) -> Tuple[Dict[str, str], Dict[str, str]]:
    """
    Find joint (A1..A6) and Cartesian (X,Y,Z,A,B,C) columns.

    Returns (joint_map, cartesian_map), each mapping the canonical name to the
    actual column name found in the file. Nothing is renamed in the output —
    the real column name is always shown.
    """
    joints: Dict[str, str] = {}
    cartesian: Dict[str, str] = {}

    for column in columns:
        lowered = column.lower()

        joint_match = _JOINT_RE.search(column)
        if joint_match and 'position' not in lowered:
            canonical = joint_match.group(1).upper()
            joints.setdefault(canonical, column)
            continue

        # Cartesian is only claimed when the column clearly belongs to a
        # position/pose group, so a bare "A" is never mistaken for axis A1.
        if any(tag in lowered for tag in ('position', 'cartesian', 'pose')):
            cart_match = _CART_RE.search(column)
            if cart_match:
                canonical = cart_match.group(1).upper()
                cartesian.setdefault(canonical, column)

    return joints, cartesian


# ---------------------------------------------------------------------------
# Statistics helpers
# ---------------------------------------------------------------------------

def column_stats(rows: List[Dict[str, str]], column: str) -> Optional[Dict[str, float]]:
    """min / max / mean / count for one numeric column."""
    values = [v for v in (as_float(row.get(column)) for row in rows) if v is not None]
    if not values:
        return None
    return {
        'min': min(values),
        'max': max(values),
        'mean': statistics.fmean(values),
        'count': len(values),
    }


def compute_deltas_ms(rows: List[Dict[str, str]], columns: List[str]) -> List[float]:
    """
    Inter-arrival deltas in milliseconds.

    Recomputed from receive_ros_time_ns when available (authoritative);
    otherwise the stored delta_receive_ms column is used; otherwise the
    ISO-8601 wall clock is parsed.
    """
    if TIME_NS_COLUMN in columns:
        times = [as_int(row.get(TIME_NS_COLUMN)) for row in rows]
        times = [t for t in times if t is not None]
        return [
            (times[i] - times[i - 1]) / 1_000_000.0
            for i in range(1, len(times))
        ]

    if DELTA_COLUMN in columns:
        return [
            d for d in (as_float(row.get(DELTA_COLUMN)) for row in rows)
            if d is not None
        ]

    if WALL_COLUMN in columns:
        stamps = []
        for row in rows:
            parsed = parse_iso(row.get(WALL_COLUMN))
            if parsed is not None:
                stamps.append(parsed)
        return [
            (stamps[i] - stamps[i - 1]).total_seconds() * 1000.0
            for i in range(1, len(stamps))
        ]

    return []


def parse_iso(value: Optional[str]) -> Optional[datetime]:
    """Parse an ISO-8601 timestamp, tolerating a trailing Z."""
    if is_null_cell(value):
        return None
    text = value.strip().replace('Z', '+00:00')
    try:
        return datetime.fromisoformat(text)
    except ValueError:
        return None


def find_sequence_gaps(rows: List[Dict[str, str]]) -> List[Dict[str, int]]:
    """
    List every place where the sequence counter jumped by more than 1.

    A gap is reported, not interpreted: a KUKA restart, a reconnect, and a
    dropped datagram all look identical in a CSV.
    """
    gaps: List[Dict[str, int]] = []
    previous: Optional[int] = None

    for index, row in enumerate(rows, start=1):
        current = as_int(row.get(SEQ_COLUMN))
        if current is None:
            continue
        if previous is not None:
            delta = current - previous
            if delta > 1:
                gaps.append({
                    'row': index,
                    'from_seq': previous,
                    'to_seq': current,
                    'delta_seq': delta,
                    'estimated_missing': delta - 1,
                })
            elif delta <= 0:
                gaps.append({
                    'row': index,
                    'from_seq': previous,
                    'to_seq': current,
                    'delta_seq': delta,
                    'estimated_missing': 0,
                })
        previous = current

    return gaps


def histogram(values: List[float], bins: int = 20, width: int = 46) -> List[str]:
    """Small ASCII histogram — no plotting library involved."""
    if not values:
        return ['  (no data)']

    low, high = min(values), max(values)
    if high - low < 1e-12:
        return [f'  all {len(values)} samples at {low:.3f} ms']

    step = (high - low) / bins
    counts = [0] * bins
    for value in values:
        index = min(bins - 1, int((value - low) / step))
        counts[index] += 1

    peak = max(counts) or 1
    lines = []
    for i, count in enumerate(counts):
        start = low + i * step
        end = start + step
        bar = '#' * int(round(width * count / peak))
        lines.append(f'  [{start:8.2f} .. {end:8.2f}] ms | {count:6d} | {bar}')
    return lines


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def section(title: str) -> None:
    print()
    print('=' * 72)
    print(title)
    print('=' * 72)


def fmt(value: Optional[float], digits: int = 3) -> str:
    return 'n/a' if value is None else f'{value:.{digits}f}'


def report(path: str, args: argparse.Namespace) -> int:
    columns, rows = load_csv(path)

    section('FILE')
    print(f'  Path:            {os.path.abspath(path)}')
    print(f'  Size:            {os.path.getsize(path):,} bytes')
    print(f'  Columns:         {len(columns)}')
    print(f'  Total samples:   {len(rows)}')

    if not rows:
        print('\n  The file has a header but no data rows. Nothing to analyse.')
        return 1

    # ── Time ────────────────────────────────────────────────────────
    section('TIME')
    first_wall = rows[0].get(WALL_COLUMN, '')
    last_wall = rows[-1].get(WALL_COLUMN, '')
    print(f'  First timestamp: {first_wall or "n/a"}')
    print(f'  Last timestamp:  {last_wall or "n/a"}')

    duration_s: Optional[float] = None
    if TIME_NS_COLUMN in columns:
        first_ns = as_int(rows[0].get(TIME_NS_COLUMN))
        last_ns = as_int(rows[-1].get(TIME_NS_COLUMN))
        if first_ns is not None and last_ns is not None:
            duration_s = (last_ns - first_ns) / 1e9
    if duration_s is None:
        first_dt, last_dt = parse_iso(first_wall), parse_iso(last_wall)
        if first_dt and last_dt:
            duration_s = (last_dt - first_dt).total_seconds()

    print(f'  Total duration:  {fmt(duration_s)} s')

    # ── Rate ────────────────────────────────────────────────────────
    section('RATE (measured — nothing assumed)')
    deltas = compute_deltas_ms(rows, columns)

    avg_rate = None
    if duration_s and duration_s > 0 and len(rows) > 1:
        avg_rate = (len(rows) - 1) / duration_s

    inst_rates = [1000.0 / d for d in deltas if d > 0]

    print(f'  Average rate:        {fmt(avg_rate)} Hz')
    if inst_rates:
        print(f'  Max instant rate:    {fmt(max(inst_rates))} Hz'
              f'   (shortest delta)')
        print(f'  Min instant rate:    {fmt(min(inst_rates))} Hz'
              f'   (longest delta)')
        print(f'  Median instant rate: {fmt(statistics.median(inst_rates))} Hz')
    else:
        print('  Instantaneous rate:  n/a (need at least 2 samples)')

    section('INTER-ARRIVAL DELTA (ms)')
    if deltas:
        print(f'  Samples:  {len(deltas)}')
        print(f'  Mean:     {fmt(statistics.fmean(deltas))} ms')
        print(f'  Median:   {fmt(statistics.median(deltas))} ms')
        print(f'  Min:      {fmt(min(deltas))} ms')
        print(f'  Max:      {fmt(max(deltas))} ms')
        if len(deltas) > 1:
            print(f'  Stdev:    {fmt(statistics.stdev(deltas))} ms')
        else:
            print('  Stdev:    n/a (need at least 2 deltas)')

        if not args.no_histogram:
            print()
            print('  Distribution:')
            for line in histogram(deltas, bins=args.bins):
                print(line)
    else:
        print('  No delta data available.')

    # ── Sequence ────────────────────────────────────────────────────
    section('SEQUENCE ANALYSIS')
    if SEQ_COLUMN not in columns:
        print(f'  Column "{SEQ_COLUMN}" not present — no sequence in this log.')
    else:
        seq_values = [
            v for v in (as_int(row.get(SEQ_COLUMN)) for row in rows)
            if v is not None
        ]
        if not seq_values:
            print('  Sequence column exists but every value is NULL.')
            print('  (The message carried no sequence field.)')
        else:
            gaps = find_sequence_gaps(rows)
            forward_gaps = [g for g in gaps if g['delta_seq'] > 1]
            resets = [g for g in gaps if g['delta_seq'] <= 0]
            missing = sum(g['estimated_missing'] for g in forward_gaps)

            print(f'  Sequence values:     {len(seq_values)}')
            print(f'  First sequence:      {seq_values[0]}')
            print(f'  Last sequence:       {seq_values[-1]}')
            print(f'  Span (last-first):   {seq_values[-1] - seq_values[0]}')
            print(f'  Sequence gaps:       {len(forward_gaps)}')
            print(f'  Estimated missing:   {missing}'
                  f'   (labelled only — NOT confirmed as network loss)')
            print(f'  Resets / rollbacks:  {len(resets)}')

            shown = forward_gaps[:args.top_gaps]
            if shown:
                print()
                print('  Gap detail:')
                for gap in shown:
                    print(f'    row {gap["row"]:>7}: '
                          f'{gap["from_seq"]} -> {gap["to_seq"]}  '
                          f'delta_seq={gap["delta_seq"]}  '
                          f'missing≈{gap["estimated_missing"]}')
                if len(forward_gaps) > len(shown):
                    print(f'    ... and {len(forward_gaps) - len(shown)} more '
                          f'(use --top-gaps N to see more)')

    # ── Joints ──────────────────────────────────────────────────────
    joints, cartesian = classify_columns(columns)

    section('JOINT AXES (A1..A6)')
    if not joints:
        print('  No joint columns detected in this file.')
    else:
        print(f'  {"axis":<6} {"column":<28} {"min":>12} {"max":>12} '
              f'{"mean":>12} {"n":>8}')
        print('  ' + '-' * 82)
        for canonical in JOINT_ORDER:
            column = joints.get(canonical)
            if column is None:
                continue
            stats = column_stats(rows, column)
            if stats is None:
                print(f'  {canonical:<6} {column:<28} {"(all NULL)":>12}')
                continue
            print(f'  {canonical:<6} {column:<28} '
                  f'{stats["min"]:>12.4f} {stats["max"]:>12.4f} '
                  f'{stats["mean"]:>12.4f} {stats["count"]:>8}')

    # ── Cartesian ───────────────────────────────────────────────────
    section('CARTESIAN POSE (X,Y,Z,A,B,C)')
    if not cartesian:
        print('  No Cartesian columns detected in this file.')
    else:
        print(f'  {"comp":<6} {"column":<28} {"min":>12} {"max":>12} '
              f'{"mean":>12} {"n":>8}')
        print('  ' + '-' * 82)
        for canonical in CART_ORDER:
            column = cartesian.get(canonical)
            if column is None:
                continue
            stats = column_stats(rows, column)
            if stats is None:
                print(f'  {canonical:<6} {column:<28} {"(all NULL)":>12}')
                continue
            print(f'  {canonical:<6} {column:<28} '
                  f'{stats["min"]:>12.4f} {stats["max"]:>12.4f} '
                  f'{stats["mean"]:>12.4f} {stats["count"]:>8}')

    # ── Field inventory ─────────────────────────────────────────────
    section('FIELDS FOUND / NULL COUNT')
    total = len(rows)
    print(f'  {"column":<44} {"nulls":>8} {"filled":>8} {"null%":>8}')
    print('  ' + '-' * 72)
    for column in columns:
        nulls = sum(1 for row in rows if is_null_cell(row.get(column)))
        filled = total - nulls
        pct = 100.0 * nulls / total if total else 0.0
        marker = '   <-- always NULL' if nulls == total else ''
        print(f'  {column:<44} {nulls:>8} {filled:>8} {pct:>7.1f}%{marker}')

    section('NOTES')
    print('  * "always NULL" on source_stamp_* is expected: the KUKA telemetry')
    print('    message carries no timestamp of its own, so only the logger\'s')
    print('    reception timestamps are real.')
    print('  * A sequence gap is reported, never interpreted. Investigate the')
    print('    KUKA side before calling it packet loss.')
    print()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        prog='analyze_log.py',
        description=(
            'Offline analysis of a kuka_telemetry_logger CSV. '
            'Standard library only — ROS2 is not required.'
        ),
    )
    parser.add_argument('csv_file', help='Path to the CSV produced by the logger.')
    parser.add_argument(
        '--no-histogram', action='store_true',
        help='Skip the ASCII delta histogram.')
    parser.add_argument(
        '--bins', type=int, default=20,
        help='Histogram bin count (default 20).')
    parser.add_argument(
        '--top-gaps', type=int, default=20,
        help='How many individual sequence gaps to list (default 20).')
    args = parser.parse_args()

    if not os.path.isfile(args.csv_file):
        print(f'ERROR: file not found: {args.csv_file}', file=sys.stderr)
        return 2

    return report(args.csv_file, args)


if __name__ == '__main__':
    raise SystemExit(main())
