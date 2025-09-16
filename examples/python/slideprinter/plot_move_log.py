"""Plot Klipper move logs emitted by the browser --log-move flag.

The log is produced by enabling the ``--log-move`` query parameter when
loading ``examples/js/slideprinter/index.html``. The worker will emit a JSON
line per aggregated move containing the command and absolute position for each
axis. Download the log via the browser console helper ``downloadKlipperMoveLog``
and feed the JSONL file to this script.

Usage::

    python -m examples.python.slideprinter.plot_move_log --input path/to/log.jsonl

Optionally specify ``--output`` to write a PNG (or any matplotlib-supported
format). Without ``--output`` the plot window is displayed interactively.
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, List

import matplotlib.pyplot as plt
import numpy as np


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot Klipper move JSONL log")
    parser.add_argument(
        "--input",
        required=True,
        help="Path to the JSON lines file produced by --log-move",
    )
    parser.add_argument(
        "--output",
        help="Optional file to write the plot to (e.g. output.png). If omitted, show interactively.",
    )
    parser.add_argument(
        "--time-unit",
        choices=["seconds", "milliseconds"],
        default="seconds",
        help="Unit to use on the X axis",
    )
    return parser.parse_args()


def _load_entries(path: Path) -> List[Dict]:
    entries: List[Dict] = []
    with path.open("r", encoding="utf-8") as fh:
        for line_no, raw in enumerate(fh, start=1):
            line = raw.strip()
            if not line:
                continue
            try:
                entry = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Failed to parse JSON on line {line_no}: {exc}") from exc
            if not isinstance(entry, dict):
                raise ValueError(f"Line {line_no} did not contain a JSON object")
            entries.append(entry)
    if not entries:
        raise ValueError("No log entries found in input file")
    return entries


def _extract_units(entries: Iterable[Dict]) -> Dict[str, str]:
    for entry in entries:
        units = entry.get("axis_units")
        if isinstance(units, dict):
            spool_unit = units.get("spool", "radians")
            extruder_unit = units.get("E", units.get("extruder", "millimeters"))
            # Build per-axis mapping lazily in caller
            return {
                "spool": spool_unit,
                "E": extruder_unit,
            }
    return {"spool": "radians", "E": "millimeters"}


def _build_series(entries: List[Dict]) -> Dict[str, List[float]]:
    series: Dict[str, List[float]] = defaultdict(list)
    for entry in entries:
        axes = entry.get("axes")
        if not isinstance(axes, dict):
            continue
        for axis_name, value in axes.items():
            try:
                series[axis_name].append(float(value))
            except (TypeError, ValueError):
                # Skip values that cannot be coerced to float.
                pass
    return series


def _build_time_vector(entries: List[Dict], unit: str) -> List[float]:
    multiplier = 1.0 / 1000.0 if unit == "seconds" else 1.0
    times: List[float] = []
    for entry in entries:
        at_ms = entry.get("at_ms")
        if at_ms is None:
            continue
        try:
            times.append(float(at_ms) * multiplier)
        except (TypeError, ValueError):
            times.append(float("nan"))
    return times


def plot_move_log(entries: List[Dict], output: Path | None, time_unit: str) -> None:
    axis_series = _build_series(entries)
    if not axis_series:
        raise ValueError("No axis data found in log entries")

    times = _build_time_vector(entries, time_unit)
    if len(times) != len(entries):
        raise ValueError("Mismatch between timestamps and entries count")

    unit_map = _extract_units(entries)
    axis_names = sorted(axis_series.keys())

    fig, axes_arr = plt.subplots(len(axis_names), 1, sharex=True, figsize=(10, 2.5 * len(axis_names)))
    if isinstance(axes_arr, np.ndarray):
        axes_iter = list(axes_arr.ravel())
    else:
        axes_iter = [axes_arr]

    for ax_obj, axis_name in zip(axes_iter, axis_names):
        values = axis_series[axis_name]
        if len(values) != len(times):
            # Pad with NaNs to align with timestamps.
            padded = list(values)
            if len(values) < len(times):
                padded.extend([float("nan")] * (len(times) - len(values)))
            values = padded
        ax_obj.plot(times, values, linewidth=0.9)
        unit_label = unit_map.get(axis_name, unit_map.get("spool", "radians"))
        ax_obj.set_ylabel(f"{axis_name} ({unit_label})")
        ax_obj.grid(True, which="both", linestyle="--", linewidth=0.4, alpha=0.5)

    axes_iter[-1].set_xlabel(f"Time ({time_unit})")
    fig.tight_layout()

    if output:
        fig.savefig(output, dpi=150)
    else:
        plt.show()


def main() -> None:
    args = _parse_args()
    input_path = Path(args.input)
    if not input_path.exists():
        raise FileNotFoundError(f"Input log file not found: {input_path}")
    entries = _load_entries(input_path)
    output_path = Path(args.output) if args.output else None
    plot_move_log(entries, output_path, args.time_unit)


if __name__ == "__main__":
    main()
