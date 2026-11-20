"""Ellipsoid autocal (`--collection-shape mesh`): blind sphere meshes and an ellipse fit.

A mesh holds one line (D, or the top line on HP5) at one length, so the effector
stays on a sphere around that anchor. There are len(SPHERES) meshes, and they
are backfilled: each visit adds one row per free line to one sphere. A row also
holds that free line, the level line, and is an ordinary force sweep: the drive
pulls out in force mode until it stalls and steps back to its start, with the
sensor in force mode. Each visit holds the level lines at a new length, so it
covers a new band of its sphere. Nothing is planned from a model: held and level
lengths are fixed fractions of the travel measured during the first visit's size
tuning, so the collector finds every reachable range itself, as the bootstrap
sweeps always have.

The anchors come from the ellipse fit (ellipse_fit.py): each sphere's points lie
on one ellipsoid in squared-length space, and the fit also estimates home
offsets and corrects line stretch at the recorded constant forces. With
`point_fit` the point fit (pose_fit.py) solves the same data instead. After each
visit the fit is repeated; the run stops when two consecutive visits moved no
anchor coordinate by more than the stable step (once every sphere has a visit),
when the stop file appears, after the last scheduled visit, or when a collection
fails once every sphere has a visit (the fit so far is kept).
The reported uncertainty is the spread of fits that leave out one row at a time.
"""
from __future__ import annotations

import json
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

from autocal.anchor_seed import build_anchor_initial_guess
from autocal.dataset_roles import mesh_metadata, normalize_dataset_point_roles
from autocal.ellipse_fit import fit_ellipses
from autocal.pose_fit import fit_pose_least_squares
from autocal.sweep_config_generator import generate_sweep_configs
from autocal.sweep_types import MachineConfig, MachineType

ROW_POINTS = 5            # per direction; each row runs both directions
STABLE_VISITS = 2
HOME_OFFSET_PRIOR_SIGMA_MM = 5.0
SEED_SCALES = (1.0, 2.0, 4.0)
# Held and level lengths as fractions of the measured travel. The held line only
# shortens. Level lines lengthen on HP3/HP4 and shorten on HP5, like the bootstrap.
# HP5's top line slips when held and level lines together shorten by more than
# about twice the travel (held -1.5 with level 0.75 failed), so the held line
# shortens at most by the travel.
SPHERES = (0.0, -1 / 3, -2 / 3, -1.0)
LEVELS = (1.0, 0.5, 0.0, 0.75, 0.25)
# Visit k adds to sphere k % 4, at a level that rotates through LEVELS, so every
# (sphere, level) pair comes once and early visits already differ in level.
# The first is (0, 1), which the collector's default targets reproduce.
SCHEDULE = [(SPHERES[k % len(SPHERES)], LEVELS[(k // len(SPHERES) + k % len(SPHERES)) % len(LEVELS)])
            for k in range(len(SPHERES) * len(LEVELS))]


def mesh_rows(machine_type: str, held_fraction: float, level_fraction: float, travel_mm: float) -> List[str]:
    """Sweep config lines for one mesh: one row per free line held as level line."""
    configs = generate_sweep_configs(MachineConfig.from_type(MachineType(machine_type)))
    held = max(int(anchor) for anchor in configs[0]["fixed_anchors"])
    level_sign = -1.0 if machine_type.startswith("hangprinter_5") else 1.0
    lines = []
    for config in configs:
        level = next(int(anchor) for anchor in config["fixed_anchors"] if int(anchor) != held)
        targets = {held: held_fraction * travel_mm, level: level_sign * level_fraction * travel_mm}
        fixed = sorted(targets)
        lines.append(f"[{','.join(map(str, fixed))}] {config['drive_anchor']} {config['sensor_anchor']} "
                     f"fixed={','.join(f'{targets[anchor]:.3f}' for anchor in fixed)}")
    return lines


def tag_mesh(records: Sequence[dict], mesh_id: str, held: int, visit: int) -> None:
    for row, record in enumerate(records):
        record.setdefault("metadata", {})["mesh"] = {
            "id": mesh_id, "visit": visit, "row": row, "held_anchor": held,
            "held_length_mm": float(record["fixed_lengths"][record["fixed_anchors"].index(held)]),
        }


def visits(dataset: dict) -> List[List[int]]:
    """Record indices of each collection visit, in order (a mesh id when no visit is recorded)."""
    order: Dict[str, List[int]] = {}
    for index, sweep in enumerate(dataset["sweeps"]):
        mesh = mesh_metadata(sweep)
        order.setdefault(str(mesh.get("visit", mesh["id"])), []).append(index)
    return list(order.values())


def fit(dataset: dict, starts: Sequence[np.ndarray], low_anchor_z: Optional[float], use_noise_mean: bool,
        point_fit: bool = False) -> dict:
    """Best fit, with home offsets, over the given starts (lowest rms)."""
    options = dict(low_anchor_z=low_anchor_z, use_noise_mean=use_noise_mean, fit_home_offsets=True,
                   home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    if point_fit:
        return fit_pose_least_squares(dataset, starts[0], extra_initial_anchors=starts[1:], **options)
    return min((fit_ellipses(dataset, start, **options) for start in starts), key=lambda result: result["rms_mm"])


def row_jackknife_mm(dataset: dict, anchors: np.ndarray, low_anchor_z, use_noise_mean, point_fit=False) -> Optional[float]:
    """Largest anchor coordinate shift when one row is left out."""
    if len(dataset["sweeps"]) < 3:
        return None
    shifts = []
    for index in range(len(dataset["sweeps"])):
        subset = dict(dataset, sweeps=[s for i, s in enumerate(dataset["sweeps"]) if i != index])
        refit = fit(subset, [anchors], low_anchor_z, use_noise_mean, point_fit)["anchors"]
        shifts.append(np.max(np.abs(refit - anchors)))
    return float(np.max(shifts))


def run_ellipsoid_autocal(
    dataset_path: Path,
    *,
    machine_type: str,
    collector_args: Sequence[str],
    run_collection: Callable[[List[str]], None],
    log_line: Callable[[str], None],
    log_console: Callable[[str], None],
    append_log: Callable[[dict], None],
    no_collect: bool,
    low_anchor_z: Optional[float],
    use_noise_mean: bool,
    stable_step_mm: float,
    stop_file: Path,
    point_fit: bool = False,
) -> Optional[dict]:
    """Collect and fit sphere visits until the anchors are stable. Returns the accepted plan and summary."""
    from autocal._autocal_common import _inject_force_args, _merge_sweep_datasets

    def collect(index: int, dataset: Optional[dict]) -> dict:
        held_fraction, level_fraction = SCHEDULE[index]
        travel = float(dataset["config"]["max_travel_mm"]) if dataset else 0.0
        cfg = dataset_path.with_suffix(f".visit_{index + 1:03d}.cfg.txt")
        out = dataset_path.with_suffix(f".visit_{index + 1:03d}.json")
        # The first visit measures the travel; its rows use the collector's default targets.
        rows = mesh_rows(machine_type, held_fraction, level_fraction, travel)
        cfg.write_text("\n".join(row if dataset else row.split(" fixed=")[0] for row in rows) + "\n")
        args = list(collector_args)
        if dataset:
            args, _, _ = _inject_force_args(dataset, [a for a in args if a != "--hp-sim-reset"])
            args += ["--completed-sweeps", str(len(dataset["sweeps"]))]
        cmd = ["node", "autocal/control/cli/collect_sweep_data.mjs", "--machineType", machine_type,
               "--sweep-config-file", str(cfg), "--output-file", str(out), "--sweepPoints", str(ROW_POINTS),
               "--return-to-origin", *args]
        sphere = SPHERES.index(held_fraction) + 1
        log_line(f"; visit {index + 1}: sphere {sphere} (held {held_fraction:+.2f}), level {level_fraction:+.2f} x travel")
        log_line(";   " + " ".join(cmd))
        run_collection(cmd)
        new = json.loads(out.read_text())
        normalize_dataset_point_roles(new)
        held = max(int(a) for a in new["sweeps"][0]["fixed_anchors"]) if new["sweeps"] else 0
        tag_mesh(new["sweeps"], f"sphere_{sphere}", held, index + 1)
        merged = new if dataset is None else _merge_sweep_datasets(dataset, new)
        dataset_path.write_text(json.dumps(merged, indent=2))
        return merged

    recorded = json.loads(dataset_path.read_text()) if dataset_path.exists() else None
    if recorded is not None:
        normalize_dataset_point_roles(recorded)
        # Older sweep datasets: sweeps that hold the same line at the same length share a sphere.
        for sweep in recorded["sweeps"]:
            if mesh_metadata(sweep) is None:
                held = max(int(a) for a in sweep["fixed_anchors"])
                length = sweep["fixed_lengths"][sweep["fixed_anchors"].index(held)]
                tag_mesh([sweep], f"sphere_{held}_{length:.0f}", held, 1)
    replay = visits(recorded) if recorded is not None else []
    if no_collect and not replay:
        log_console("; --no-collect needs a mesh dataset")
        return None
    if recorded is not None and not no_collect:
        log_console("; ellipsoid autocal continues an existing dataset by replaying it")
        no_collect = True

    model = "point" if point_fit else "ellipsoid"
    seed = None
    history: List[np.ndarray] = []
    dataset: Optional[dict] = None
    result = None
    reason = "schedule-complete"
    total = len(replay) if no_collect else len(SCHEDULE)
    for index in range(total):
        if no_collect:
            dataset = dict(recorded, sweeps=[recorded["sweeps"][i] for group in replay[:index + 1] for i in group])
        else:
            try:
                dataset = collect(index, dataset)
            except subprocess.CalledProcessError as error:
                # A collection that fails once every sphere has a visit keeps the fit so far.
                if len(history) < len(SPHERES):
                    raise
                log_console(f"; visit {index + 1} collection failed ({error}); keeping the fit so far")
                reason = "collection-failed"
                break
        if not dataset["sweeps"]:
            continue
        seed = build_anchor_initial_guess(dataset, machine_type=machine_type) if seed is None else seed
        # The blind seed underestimates the anchor distances; on HP5 its own scale can
        # settle in a far minimum, so larger copies start too and the lowest rms wins.
        result = fit(dataset, [seed * scale for scale in SEED_SCALES] + history[-1:], low_anchor_z, use_noise_mean,
                     point_fit)
        history.append(result["anchors"])
        uncertainty = row_jackknife_mm(dataset, result["anchors"], low_anchor_z, use_noise_mean, point_fit)
        shape = "points" if point_fit else f"{result['num_ellipsoids']} ellipsoids"
        log_console(f"; visit {index + 1}: {len(dataset['sweeps'])} rows, {len({mesh_metadata(s)['id'] for s in dataset['sweeps']})} "
                    f"spheres, rms {result['rms_mm']:.3f} mm, {shape}, row jackknife "
                    f"{'n/a' if uncertainty is None else f'{uncertainty:.2f} mm'}, offsets "
                    f"{np.round(result['home_offsets_mm'], 2).tolist()}")
        append_log({"timestamp": datetime.now().isoformat(), "dataset": str(dataset_path), "event": "ellipsoid_fit",
                    "iteration": index + 1, "num_sweeps": index + 1, "selected_run": model,
                    "runs": [{"id": model, "metrics": {"pose_fit_anchors": result["anchors"].tolist(),
                                                              "anchor_uncertainty_mm": uncertainty}}]})
        steps = [np.max(np.abs(a - b)) for a, b in zip(history[1:], history[:-1])]
        if (len(history) >= len(SPHERES) and len(steps) >= STABLE_VISITS
                and max(steps[-STABLE_VISITS:]) <= stable_step_mm):
            reason = "stable-estimate"
            break
        if stop_file.exists():
            reason = "stop-file"
            break
    if result is None:
        return None
    log_console(f"; ellipsoid autocal stopped: {reason}")
    return {"plan": {"anchors": result["anchors"], "machine_type": machine_type},
            "meta": {"anchor_uncertainty_mm": uncertainty, "fit_model": model},
            "reason": reason}
