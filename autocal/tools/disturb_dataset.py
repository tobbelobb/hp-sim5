#!/usr/bin/env python3
"""Inject line disturbances into a recorded sweep dataset.

A hand or object pushing a line sideways lengthens its path by some detour.
On a position-controlled line (a fixed line or the physical drive line), the
encoder keeps its length, so the straight part shortens and the effector moves.
The tension-controlled lines (the physical sensor line and any extra sensor
ranges) then record that move. On a tension-controlled line, the spool pays out
the detour itself, so only that line's record changes.

Disturbance kinds:
  pull     a constant push that grows over three consecutive points and fades
           over the next three: detour * (1/3, 2/3, 1, 2/3, 1/3, 0)
  outlier  one point disturbed by the full detour
  noise    independent Gaussian length errors of sigma_mm on every measured
           line of every point. On a held line the encoder error moves the
           effector, exactly like a detour of that size.

The effector move is solved exactly from the true anchors. The records keep any
offsets and stretch that were already in them.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

PULL_PROFILE = (1 / 3, 2 / 3, 1.0, 2 / 3, 1 / 3, 0.0)


def _record_keys(sweep: dict, anchor: int):
    if anchor == int(sweep["drive_anchor"]):
        return ("l_drive", "l_drive_mu")
    if anchor == int(sweep["sensor_anchor"]):
        return ("l_sensor", "l_sensor_mu")
    return None


def _point_lines(sweep: dict, point: dict):
    """Position-controlled and tension-controlled anchors of one point."""
    pair = {int(sweep["drive_anchor"]), int(sweep["sensor_anchor"])}
    sensor = int(point.get("source_sensor_anchor", sweep["sensor_anchor"]))
    held = [int(anchor) for anchor in sweep["fixed_anchors"]] + sorted(pair - {sensor})
    pulled = [sensor] + [int(extra["anchor"]) for extra in point.get("extra_sensor_ranges", [])]
    return held, pulled


def _point_ranges(sweep: dict, point: dict, num_anchors: int) -> np.ndarray:
    ranges = np.full(num_anchors, np.nan)
    for anchor, value in zip(sweep["fixed_anchors"], sweep["fixed_lengths"]):
        ranges[int(anchor)] = float(value)
    ranges[int(sweep["drive_anchor"])] = float(point["l_drive"])
    ranges[int(sweep["sensor_anchor"])] = float(point["l_sensor"])
    for extra in point.get("extra_sensor_ranges", []):
        ranges[int(extra["anchor"])] = float(extra["length_mm"])
    return ranges


def _solve_position(anchors, lines, distances, start):
    position = np.asarray(start, dtype=float).copy()
    for _ in range(50):
        offsets = position - anchors[lines]
        current = np.linalg.norm(offsets, axis=1)
        jacobian = offsets / current[:, None]
        step = np.linalg.lstsq(jacobian, distances - current, rcond=None)[0]
        position += step
        if np.linalg.norm(step) < 1e-10:
            break
    return position


def disturb_point(sweep: dict, point: dict, anchors: np.ndarray, line: int, detour_mm: float) -> None:
    """Apply one detour on `line` to `point`, in place."""
    if detour_mm == 0.0:
        return
    held, pulled = _point_lines(sweep, point)
    changes = {}
    if line in pulled:
        changes[line] = detour_mm
    elif line in held:
        ranges = _point_ranges(sweep, point, len(anchors))
        measured = np.flatnonzero(np.isfinite(ranges))
        before = _solve_position(anchors, measured, np.linalg.norm(anchors[measured], axis=1) + ranges[measured],
                                 np.zeros(anchors.shape[1]))
        distances = np.linalg.norm(before - anchors[held], axis=1)
        distances[held.index(line)] -= detour_mm
        after = _solve_position(anchors, np.asarray(held), distances, before)
        for anchor in pulled:
            changes[anchor] = float(np.linalg.norm(after - anchors[anchor]) - np.linalg.norm(before - anchors[anchor]))
    else:
        raise ValueError(f"line {line} carries no measurement in this sweep")
    for anchor, change in changes.items():
        keys = _record_keys(sweep, anchor)
        if keys is None:
            extras = [extra for extra in point.get("extra_sensor_ranges", []) if int(extra["anchor"]) == anchor]
            keys = ("length_mm", "length_mu_mm")
            target = extras[0]
        else:
            target = point
        for key in keys:
            if target.get(key) is not None:
                target[key] = float(target[key]) + change


def apply_events(dataset: dict, anchors, events) -> dict:
    """Return a disturbed copy. Each event: kind, sweep, point, line, detour_mm."""
    anchors = np.asarray(anchors, dtype=float)
    result = copy.deepcopy(dataset)
    for event in events:
        if event["kind"] == "noise":
            rng = np.random.default_rng(int(event["seed"]))
            for sweep in result["sweeps"]:
                for point in sweep["data_points"]:
                    held, pulled = _point_lines(sweep, point)
                    for line in held + pulled:
                        disturb_point(sweep, point, anchors, line, float(rng.normal(0.0, event["sigma_mm"])))
            continue
        sweep = result["sweeps"][int(event["sweep"])]
        profile = PULL_PROFILE if event["kind"] == "pull" else (1.0,)
        for offset, fraction in enumerate(profile):
            index = int(event["point"]) + offset
            if index < len(sweep["data_points"]):
                disturb_point(sweep, sweep["data_points"][index], anchors, int(event["line"]),
                              fraction * float(event["detour_mm"]))
    result.setdefault("disturbances", []).extend(dict(event) for event in events)
    return result


def scenario_events(dataset: dict, scenario: str, *, seed: int = 0, pull_mm: float = 10.0,
                    outlier_mm: float = 20.0, outlier_fraction: float = 0.03, noise_mm: float = 0.0):
    """Deterministic event lists for the named scenario: pull, outliers, mixed or noise.

    pull      one push on a held line in the second sweep and one on a sensor
              line in the second-to-last sweep
    outliers  outlier_fraction of all points, each on a random measured line,
              with random sign and 0.5..1 x outlier_mm
    mixed     both
    noise     only Gaussian noise; noise_mm > 0 also adds it to the other scenarios
    """
    rng = np.random.default_rng(seed)
    sweeps = dataset["sweeps"]
    events = [dict(kind="noise", sigma_mm=float(noise_mm), seed=int(seed))] if noise_mm > 0 else []
    if scenario in ("pull", "mixed"):
        for sweep_index, held_line in ((1, True), (len(sweeps) - 2, False)):
            sweep = sweeps[sweep_index]
            start = int(rng.integers(0, max(1, len(sweep["data_points"]) - len(PULL_PROFILE))))
            held, pulled = _point_lines(sweep, sweep["data_points"][start])
            line = int(rng.choice(held if held_line else pulled[:1]))
            events.append(dict(kind="pull", sweep=sweep_index, point=start, line=line,
                               detour_mm=float(pull_mm)))
    if scenario in ("outliers", "mixed"):
        locations = [(s, p) for s, sweep in enumerate(sweeps) for p in range(len(sweep["data_points"]))]
        count = max(1, int(round(outlier_fraction * len(locations))))
        for index in rng.choice(len(locations), size=count, replace=False):
            sweep_index, point_index = locations[int(index)]
            sweep = sweeps[sweep_index]
            held, pulled = _point_lines(sweep, sweep["data_points"][point_index])
            line = int(rng.choice(held + pulled[:1]))
            detour = float(rng.uniform(0.5, 1.0) * outlier_mm)
            # A detour only lengthens a path, but an encoder glitch can go either way.
            if line in pulled:
                detour *= float(rng.choice((-1.0, 1.0)))
            events.append(dict(kind="outlier", sweep=sweep_index, point=point_index, line=line,
                               detour_mm=detour))
    if scenario not in ("pull", "outliers", "mixed", "noise") or not events:
        raise ValueError(f"unknown or empty scenario: {scenario}")
    return events


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--anchors", required=True, help="True anchors as JSON, e.g. [[0,-1900,-280],...]")
    parser.add_argument("--scenario", choices=("pull", "outliers", "mixed", "noise"), required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--pull-mm", type=float, default=10.0)
    parser.add_argument("--outlier-mm", type=float, default=20.0)
    parser.add_argument("--noise-mm", type=float, default=0.0, help="Gaussian length noise on every record")
    args = parser.parse_args(argv)
    dataset = json.loads(args.dataset.read_text())
    events = scenario_events(dataset, args.scenario, seed=args.seed, pull_mm=args.pull_mm,
                             outlier_mm=args.outlier_mm, noise_mm=args.noise_mm)
    args.output.write_text(json.dumps(apply_events(dataset, json.loads(args.anchors), events)))
    print(json.dumps(events))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
