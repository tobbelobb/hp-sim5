#!/usr/bin/env python3
"""Rotate the slideprinter USD scene around Z by a given angle.

Usage:
    python scripts/rotate_slideprinter.py --angle 0.5

The angle is given in degrees. An angle of 0 aligns Spool A at (0, -0.2).
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable

import numpy as np

# Allow importing geometry helpers without requiring installation as a package.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.python.cable_joints.geometry import tangent_from_point_to_circle  # type: ignore

USD_PATH = REPO_ROOT / "examples/usd_scenes/slideprinter.usda"
RADIUS = 0.03

ANCHORS = {
    "A": np.array([0.0, -2.1, 0.0]),
    "B": np.array([1.818653347947321, 1.05, 0.0]),
    "C": np.array([-1.818653347947321, 1.05, 0.0]),
}

# Base triangle configuration before any global rotation.
BASE_CENTERS = {
    "A": np.array([0.0, -0.2, 0.0]),
    "B": np.array([0.17320508075688773, 0.1, 0.0]),
    "C": np.array([-0.17320508075688773, 0.1, 0.0]),
}


@dataclass(frozen=True)
class RotationResult:
    centers: Dict[str, np.ndarray]
    tangents: Dict[str, np.ndarray]
    rest_lengths: Dict[str, float]


def rotation_matrix(angle_rad: float) -> np.ndarray:
    """Return a 3x3 rotation matrix for a Z rotation."""
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return np.array([
        [cos_a, -sin_a, 0.0],
        [sin_a, cos_a, 0.0],
        [0.0, 0.0, 1.0],
    ])


def compute_positions(angle_deg: float, tightness: float) -> RotationResult:
    """Compute rotated spool centers and tangent points."""
    angle_rad = math.radians(angle_deg)
    rot = rotation_matrix(angle_rad)

    centers = {
        key: rot @ BASE_CENTERS[key]
        for key in BASE_CENTERS
    }
    tangents: Dict[str, np.ndarray] = {}
    rest_lengths: Dict[str, float] = {}
    for key in BASE_CENTERS:
        tangent = tangent_from_point_to_circle(ANCHORS[key], centers[key], RADIUS, cw=True)["a_circle"]
        tangents[key] = tangent
        rest_lengths[key] = float(np.linalg.norm(tangent - ANCHORS[key]) - tightness)

    return RotationResult(centers=centers, tangents=tangents, rest_lengths=rest_lengths)


def format_vec(vec: np.ndarray) -> str:
    """Format a 3D vector to match USD numeric style."""
    return "({0:.15f}, {1:.15f}, {2:.15f})".format(*vec.tolist())


def apply_updates(lines: Iterable[str], result: RotationResult) -> Iterable[str]:
    """Yield updated USD lines with rotated centers and tangents."""
    replacements_done = {
        "spools": set(),
        "local_pos0": set(),
        "tangents": set(),
        "rests": set(),
    }
    current_spool: str | None = None
    current_joint: str | None = None

    for raw_line in lines:
        line = raw_line
        stripped = raw_line.strip()

        if stripped.startswith('def Circle "Spool'):
            if '"SpoolA"' in stripped:
                current_spool = "A"
            elif '"SpoolB"' in stripped:
                current_spool = "B"
            elif '"SpoolC"' in stripped:
                current_spool = "C"
        elif stripped.startswith('def CableJoint "Joint'):
            if '"JointA"' in stripped:
                current_joint = "A"
            elif '"JointB"' in stripped:
                current_joint = "B"
            elif '"JointC"' in stripped:
                current_joint = "C"

        if current_spool and "double3 xformOp:translate" in stripped:
            vec = result.centers[current_spool]
            line = f"            double3 xformOp:translate = {format_vec(vec)}\n"
            replacements_done["spools"].add(current_spool)
            current_spool = None
        elif current_joint and "custom point3d localPos0" in stripped:
            zero_vec = np.zeros(3, dtype=float)
            line = f"            custom point3d localPos0 = {format_vec(zero_vec)}\n"
            replacements_done["local_pos0"].add(current_joint)
        elif current_joint and "custom point3d localPos1" in stripped:
            vec = result.tangents[current_joint] - result.centers[current_joint]
            line = f"            custom point3d localPos1 = {format_vec(vec)}\n"
            replacements_done["tangents"].add(current_joint)
        elif current_joint and "custom double restLength" in stripped:
            rest = result.rest_lengths[current_joint]
            line = f"            custom double restLength = {rest:.15f}\n"
            replacements_done["rests"].add(current_joint)
            current_joint = None

        yield line

    missing_spools = {"A", "B", "C"} - replacements_done["spools"]
    missing_local0 = {"A", "B", "C"} - replacements_done["local_pos0"]
    missing_tangents = {"A", "B", "C"} - replacements_done["tangents"]
    missing_rests = {"A", "B", "C"} - replacements_done["rests"]
    if missing_spools or missing_local0 or missing_tangents or missing_rests:
        raise RuntimeError(
            "Failed to update all targets in USD. "
            "Missing spools: {sp}, localPos0: {lp0}, tangents: {tan}, restLengths: {rest}".format(
                sp=sorted(missing_spools),
                lp0=sorted(missing_local0),
                tan=sorted(missing_tangents),
                rest=sorted(missing_rests),
            )
        )


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--angle", type=float, required=True, help="Rotation around +Z in degrees (0 => Spool A at (0,-0.2)).")
    parser.add_argument(
        "--line-tightness",
        type=float,
        default=0.0,
        help="Amount (meters) subtracted from each CableJoint restLength to preload the cables.",
    )
    parser.add_argument("--usd", type=Path, default=USD_PATH, help="Path to slideprinter.usda (defaults to repo copy).")
    parser.add_argument("--dry-run", action="store_true", help="Print the computed transforms without editing the file.")
    args = parser.parse_args(argv)

    usd_path = args.usd.resolve()
    if not usd_path.exists():
        raise FileNotFoundError(f"USD file not found: {usd_path}")

    result = compute_positions(args.angle, args.line_tightness)

    if args.dry_run:
        for key in sorted(result.centers):
            print(f"Spool {key} center: {format_vec(result.centers[key])}")
            print(f"Joint {key} tangent: {format_vec(result.tangents[key])}")
            print(f"Joint {key} restLength: {result.rest_lengths[key]:.15f}")
        return

    original = usd_path.read_text().splitlines(keepends=True)
    updated = list(apply_updates(original, result))
    usd_path.write_text("".join(updated))


if __name__ == "__main__":
    main()
