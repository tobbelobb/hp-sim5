from __future__ import annotations

from itertools import combinations, permutations
from typing import Dict, List

from autocal.sweep_types import MachineConfig


def generate_sweep_configs(
    config: MachineConfig, forbidden_sensor_anchors: List[int] | None = None
) -> List[Dict[str, List[int] | int]]:
    """
    Generate all valid sweep configurations for a machine type.

    Returns list of dicts with keys:
      - fixed_anchors: List[int]
      - drive_anchor: int
      - sensor_anchor: int
    """
    n = config.num_anchors
    k = config.constraints_for_1dof
    if k < 0 or k >= n:
        return []

    sensor_block = set(forbidden_sensor_anchors or config.carrying_anchors or [])
    all_configs: List[Dict[str, List[int] | int]] = []

    for fixed in combinations(range(n), k):
        fixed_set = set(fixed)
        free_anchors = [i for i in range(n) if i not in fixed_set]
        for drive, sensor in permutations(free_anchors, 2):
            if sensor in sensor_block:
                continue
            all_configs.append(
                {
                    "fixed_anchors": list(fixed),
                    "drive_anchor": drive,
                    "sensor_anchor": sensor,
                }
            )

    return all_configs


def select_representative_configs(
    all_configs: List[Dict[str, List[int] | int]],
    config: MachineConfig,
    max_sweeps: int = 12,
) -> List[Dict[str, List[int] | int]]:
    """
    Select a representative subset of sweep configurations.

    Strategy:
    - Ensure each anchor appears as drive at least once
    - Ensure each non-carrying anchor appears as sensor at least once
    - Distribute fixed anchor combinations evenly
    """
    if len(all_configs) <= max_sweeps:
        return list(all_configs)

    selected: List[Dict[str, List[int] | int]] = []
    used_as_drive = set()
    used_as_sensor = set()
    used_fixed = set()
    sensor_block = set(config.carrying_anchors or [])

    for cfg in all_configs:
        drive = cfg["drive_anchor"]
        sensor = cfg["sensor_anchor"]
        fixed_combo = tuple(cfg["fixed_anchors"])

        if sensor in sensor_block:
            continue

        if (
            drive not in used_as_drive
            or sensor not in used_as_sensor
            or fixed_combo not in used_fixed
        ):
            selected.append(cfg)
            used_as_drive.add(drive)
            used_as_sensor.add(sensor)
            used_fixed.add(fixed_combo)
            if len(selected) >= max_sweeps:
                return selected

    if len(selected) < max_sweeps:
        remaining = [c for c in all_configs if c not in selected]
        for cfg in remaining:
            if len(selected) >= max_sweeps:
                break
            fixed_combo = tuple(cfg["fixed_anchors"])
            if fixed_combo in used_fixed:
                continue
            selected.append(cfg)
            used_fixed.add(fixed_combo)

    if len(selected) < max_sweeps:
        for cfg in all_configs:
            if len(selected) >= max_sweeps:
                break
            if cfg in selected:
                continue
            selected.append(cfg)

    return selected

