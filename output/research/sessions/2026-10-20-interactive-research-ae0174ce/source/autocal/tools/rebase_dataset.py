"""Offline reference-shift experiment for constant-radius encoder datasets.

Raw encoder angles stay absolute; only length deltas change. This is a research
transform, not a collection command. Four noisy readings need not define an
exact Cartesian reference: inspect restoration closure before using the result.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.theoretical_ellipse import canonicalize_anchor_gauge


def point_offsets(dataset: dict, sweep_index: int, point_index: int) -> np.ndarray:
    """Read a complete length vector using the solver's sweep-level fixed lengths."""
    normalized = copy.deepcopy(dataset)
    normalize_dataset_point_roles(normalized)
    sweep = normalized["sweeps"][sweep_index]
    point = sweep["data_points"][point_index]
    offsets = np.full(int(dataset["num_anchors"]), np.nan)
    for axis, value in zip(sweep["fixed_anchors"], sweep["fixed_lengths"]):
        offsets[axis] = value
    for role in ("drive", "sensor"):
        value = point.get(f"l_{role}_mu")
        offsets[sweep[f"{role}_anchor"]] = point[f"l_{role}"] if value is None else value
    if not np.all(np.isfinite(offsets)):
        raise ValueError("Reference point must contain a length for every anchor")
    return offsets


def rebase_dataset(dataset: dict, offsets_mm) -> dict:
    """Subtract one offset per cable, leaving absolute encoder/noise data intact."""
    offsets = np.asarray(offsets_mm, dtype=float)
    if offsets.shape != (int(dataset["num_anchors"]),) or not np.all(np.isfinite(offsets)):
        raise ValueError("Expected one finite offset per anchor")
    result = copy.deepcopy(dataset)
    normalize_dataset_point_roles(result)
    for sweep in result["sweeps"]:
        sweep["fixed_lengths"] = [float(v - offsets[a]) for a, v in
                                  zip(sweep["fixed_anchors"], sweep["fixed_lengths"])]
        for point in sweep["data_points"]:
            for role in ("drive", "sensor"):
                axis = int(sweep[f"{role}_anchor"])
                for key in (f"l_{role}", f"l_{role}_mu"):
                    if point.get(key) is not None:
                        point[key] = float(point[key] - offsets[axis])
            if point.get("drive_setpoint_mm") is not None:
                axis = int(point.get("source_drive_anchor", sweep["drive_anchor"]))
                point["drive_setpoint_mm"] = float(point["drive_setpoint_mm"] - offsets[axis])
        for key in ("start", "end"):
            if key in sweep.get("drive_range", {}):
                sweep["drive_range"][key] -= float(offsets[sweep["drive_anchor"]])
    result.setdefault("config", {}).setdefault("notes", {})["reference_shift"] = {
        "offsets_mm": offsets.tolist(), "raw_angles_remain_absolute": True,
        "scope": "offline constant-radius experiment; Cartesian closure is not guaranteed",
    }
    return result


def locate_reference(anchors, lengths_mm) -> tuple[np.ndarray, np.ndarray]:
    """Trilaterate a point and return signed per-cable distance closure (mm)."""
    anchors = np.asarray(anchors, dtype=float)
    lengths = np.asarray(lengths_mm, dtype=float)
    if anchors.ndim != 2 or lengths.shape != (len(anchors),) or not np.all(lengths > 0):
        raise ValueError("Expected positive distances and a matching anchor matrix")
    lhs = 2 * (anchors[1:] - anchors[0])
    rhs = np.sum(anchors[1:] ** 2, axis=1) - np.sum(anchors[0] ** 2) + lengths[0] ** 2 - lengths[1:] ** 2
    seed = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
    result = least_squares(lambda p: np.linalg.norm(anchors - p, axis=1) - lengths, seed,
                           xtol=1e-12, ftol=1e-12, gtol=1e-12)
    return result.x, result.fun


def restore_anchors(anchors, offsets_mm, machine_type="hangprinter_4") -> dict:
    """Recover the old origin using cable offsets, never simulator ground truth."""
    anchors = np.asarray(anchors, dtype=float)
    old_lengths = np.linalg.norm(anchors, axis=1) - np.asarray(offsets_mm, dtype=float)
    old_origin, closure = locate_reference(anchors, old_lengths)
    restored = canonicalize_anchor_gauge(machine_type, anchors - old_origin)
    return {"anchors": restored.tolist(), "old_origin_in_shifted_frame_mm": old_origin.tolist(),
            "closure_mm": closure.tolist(), "closure_rms_mm": float(np.sqrt(np.mean(closure ** 2)))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    shift = commands.add_parser("shift", help="Re-reference length deltas to a saved point")
    shift.add_argument("dataset", type=Path)
    shift.add_argument("--sweep-index", type=int, required=True, help="Zero-based sweep index")
    shift.add_argument("--point-index", type=int, required=True, help="Zero-based point index")
    shift.add_argument("--output", type=Path, required=True)
    restore = commands.add_parser("restore", help="Restore a fitted calibration's original reference")
    restore.add_argument("dataset", type=Path, help="Shifted dataset containing saved offsets")
    restore.add_argument("calibration", type=Path, help="Calibration JSON or autocal console log")
    restore.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output already exists; choose a new path to preserve prior evidence")
    data = json.loads(args.dataset.read_text())
    if args.command == "shift":
        offsets = point_offsets(data, args.sweep_index, args.point_index)
        result = rebase_dataset(data, offsets)
        result["config"]["notes"]["reference_shift"].update({
            "source_sha256": hashlib.sha256(args.dataset.read_bytes()).hexdigest(),
            "sweep_index": args.sweep_index, "point_index": args.point_index,
        })
    else:
        offsets = data["config"]["notes"]["reference_shift"]["offsets_mm"]
        text = args.calibration.read_text()
        try:
            anchors = json.loads(text)["anchors"]
        except json.JSONDecodeError:
            from autocal.tools.regress_calibration_logs import parse_summary

            summary = parse_summary(text, fixed_radii=[])
            if summary is None:
                parser.error("Calibration log has no final anchor summary")
            anchors = summary.anchors
        result = restore_anchors(anchors, offsets, data["machine_type"])
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"output": str(args.output), "offsets_mm": list(offsets)}))


if __name__ == "__main__":
    main()
