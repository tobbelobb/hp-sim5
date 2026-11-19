"""Ellipsoid autocal (`--collection-shape mesh`): blind sphere meshes and an ellipsoid-only fit.

A mesh holds one line (D, or the top line on HP5) at one length, so the effector
stays on a sphere around that anchor. Each row also holds one more line, the
level line, and is an ordinary force sweep: the drive pulls out in force mode
until it stalls and steps back to its start, with the sensor in force mode. The
rows of a mesh hold each free line as level line in turn. Nothing is planned
from a model: held and level lengths are fixed fractions of the travel measured
during the first mesh's size tuning, so the collector finds every reachable
range itself, as the bootstrap sweeps always have.

The anchors come from the ellipse fit alone (ellipse_fit.py): each mesh's points
lie on one ellipsoid in squared-length space, and the fit also estimates home
offsets and corrects line stretch at the recorded constant forces. The point fit is never used. After each mesh the fit is repeated; the
run stops when two consecutive meshes moved no anchor coordinate by more than
the stable step, when the stop file appears, or after the last scheduled mesh.
The reported uncertainty is the spread of fits that leave out one row at a time.
"""
from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

from autocal.anchor_seed import build_anchor_initial_guess
from autocal.dataset_roles import mesh_metadata, normalize_dataset_point_roles
from autocal.ellipse_fit import fit_ellipses
from autocal.sweep_config_generator import generate_sweep_configs
from autocal.sweep_types import MachineConfig, MachineType

ROW_POINTS = 5            # per direction; each row runs both directions
MIN_MESHES = 3
STABLE_MESHES = 2
HOME_OFFSET_PRIOR_SIGMA_MM = 5.0
SEED_SCALES = (1.0, 2.0, 4.0)
# (held, level) lengths as fractions of the measured travel. The held line only
# shortens. Level lines lengthen on HP3/HP4 and shorten on HP5, like the bootstrap.
SCHEDULE = [(0.0, 1.0), (-1.0, 0.5), (-0.5, 1.0), (-1.5, 0.0), (0.0, 0.5), (-0.5, 0.0), (-1.0, 1.0), (-2.0, 0.5)]


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


def tag_mesh(records: Sequence[dict], mesh_id: str, held: int) -> None:
    for row, record in enumerate(records):
        record.setdefault("metadata", {})["mesh"] = {
            "id": mesh_id, "row": row, "held_anchor": held,
            "held_length_mm": float(record["fixed_lengths"][record["fixed_anchors"].index(held)]),
        }


def meshes(dataset: dict) -> List[List[int]]:
    """Record indices of each mesh, in collection order."""
    order: Dict[str, List[int]] = {}
    for index, sweep in enumerate(dataset["sweeps"]):
        order.setdefault(str(mesh_metadata(sweep)["id"]), []).append(index)
    return list(order.values())


def fit(dataset: dict, starts: Sequence[np.ndarray], low_anchor_z: Optional[float], use_noise_mean: bool) -> dict:
    """Best ellipsoid fit, with home offsets, over the given starts (lowest robust rms)."""
    fits = [fit_ellipses(dataset, start, low_anchor_z=low_anchor_z, use_noise_mean=use_noise_mean,
                         fit_home_offsets=True, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM,
                         correct_constant_force=True)
            for start in starts]
    return min(fits, key=lambda result: result["rms_mm"])


def row_jackknife_mm(dataset: dict, anchors: np.ndarray, low_anchor_z, use_noise_mean) -> Optional[float]:
    """Largest anchor coordinate shift when one row is left out."""
    if len(dataset["sweeps"]) < 3:
        return None
    shifts = []
    for index in range(len(dataset["sweeps"])):
        subset = dict(dataset, sweeps=[s for i, s in enumerate(dataset["sweeps"]) if i != index])
        shifts.append(np.max(np.abs(fit(subset, [anchors], low_anchor_z, use_noise_mean)["anchors"] - anchors)))
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
) -> Optional[dict]:
    """Collect and fit meshes until the anchors are stable. Returns the accepted plan and summary."""
    from autocal._autocal_common import _inject_force_args, _merge_sweep_datasets

    def collect(index: int, dataset: Optional[dict]) -> dict:
        held_fraction, level_fraction = SCHEDULE[index]
        travel = float(dataset["config"]["max_travel_mm"]) if dataset else 0.0
        cfg = dataset_path.with_suffix(f".mesh_{index + 1:03d}.cfg.txt")
        out = dataset_path.with_suffix(f".mesh_{index + 1:03d}.json")
        # The first mesh measures the travel; its rows use the collector's default targets.
        rows = mesh_rows(machine_type, held_fraction, level_fraction, travel)
        cfg.write_text("\n".join(row if dataset else row.split(" fixed=")[0] for row in rows) + "\n")
        args = list(collector_args)
        if dataset:
            args, _, _ = _inject_force_args(dataset, [a for a in args if a != "--hp-sim-reset"])
            args += ["--completed-sweeps", str(len(dataset["sweeps"]))]
        cmd = ["node", "autocal/control/cli/collect_sweep_data.mjs", "--machineType", machine_type,
               "--sweep-config-file", str(cfg), "--output-file", str(out), "--sweepPoints", str(ROW_POINTS),
               "--return-to-origin", *args]
        log_line(f"; mesh {index + 1}: held {held_fraction:+.1f}, level {level_fraction:+.1f} x travel")
        log_line(";   " + " ".join(cmd))
        run_collection(cmd)
        new = json.loads(out.read_text())
        normalize_dataset_point_roles(new)
        held = max(int(a) for a in new["sweeps"][0]["fixed_anchors"]) if new["sweeps"] else 0
        tag_mesh(new["sweeps"], f"mesh_{index + 1:03d}", held)
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
                tag_mesh([sweep], f"sphere_{held}_{length:.0f}", held)
    replay = meshes(recorded) if recorded is not None else []
    if no_collect and not replay:
        log_console("; --no-collect needs a mesh dataset")
        return None
    if recorded is not None and not no_collect:
        log_console("; ellipsoid autocal continues an existing dataset by replaying it")
        no_collect = True

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
            dataset = collect(index, dataset)
        if not dataset["sweeps"]:
            continue
        seed = build_anchor_initial_guess(dataset, machine_type=machine_type) if seed is None else seed
        # The blind seed underestimates the anchor distances; on HP5 its own scale can
        # settle in a far minimum, so larger copies start too and the lowest rms wins.
        result = fit(dataset, [seed * scale for scale in SEED_SCALES] + history[-1:], low_anchor_z, use_noise_mean)
        history.append(result["anchors"])
        uncertainty = row_jackknife_mm(dataset, result["anchors"], low_anchor_z, use_noise_mean)
        log_console(f"; mesh {index + 1}: {len(dataset['sweeps'])} rows, rms {result['rms_mm']:.3f} mm, "
                    f"{result['num_ellipsoids']} ellipsoids, row jackknife "
                    f"{'n/a' if uncertainty is None else f'{uncertainty:.2f} mm'}, offsets "
                    f"{np.round(result['home_offsets_mm'], 2).tolist()}")
        append_log({"timestamp": datetime.now().isoformat(), "dataset": str(dataset_path), "event": "ellipsoid_fit",
                    "iteration": index + 1, "num_sweeps": index + 1, "selected_run": "ellipsoid",
                    "runs": [{"id": "ellipsoid", "metrics": {"pose_fit_anchors": result["anchors"].tolist(),
                                                              "anchor_uncertainty_mm": uncertainty}}]})
        steps = [np.max(np.abs(a - b)) for a, b in zip(history[1:], history[:-1])]
        if (len(history) >= MIN_MESHES and len(steps) >= STABLE_MESHES
                and max(steps[-STABLE_MESHES:]) <= stable_step_mm):
            reason = "stable-estimate"
            break
        if stop_file.exists():
            reason = "stop-file"
            break
    if result is None:
        return None
    log_console(f"; ellipsoid autocal stopped: {reason}")
    return {"plan": {"anchors": result["anchors"], "machine_type": machine_type},
            "meta": {"anchor_uncertainty_mm": uncertainty, "fit_model": "ellipsoid"},
            "reason": reason}
