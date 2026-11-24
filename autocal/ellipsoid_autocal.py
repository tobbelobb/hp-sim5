"""Ellipsoid autocal (`--ellipsoid-shape`): blind sphere meshes, fitted as ellipsoids.

An ellipsoid sweep holds one line (D, or the top line on HP5) at one length, so
the effector stays on a sphere around that anchor. It has one row per free line.
A row also holds that free line, the level line, and is an ordinary force sweep:
the drive pulls out in force mode until it stalls and steps back to its start,
with the sensor in force mode. The collector finds every reachable range itself.

The heights follow the reachable volume rather than a fixed schedule:
1. the bootstrap rows, with the held line at home (the lowest sphere);
2. half way up from there to the highest the held anchor's clearance allows,
   by the fit of the first;
3. that highest sphere, by the fit of the first two.
Later ellipsoids, when the fit has not settled, split the largest gap between
held lengths so far. From the second on, each level line settles once the held
line is in place, with every free line idling, and is then shortened from there
('n-<mm>' in the sweep config) half way to the shortest it can be on that
sphere, by the latest fit. Idling, it would be nearly slack; shortened all the
way, its row's circle would shrink to a point. Half way it stays taut, pulls the
effector out towards its anchor and leaves a wide circle. A held line that cannot reach its length holds where it can
(the collector backs it off its saturation), and the recorded lengths are used.

The anchors come from the ellipse fit (ellipse_fit.py): each sphere's points lie
on one ellipsoid in squared-length space, and the fit also estimates home
offsets and corrects line stretch at the recorded constant forces. The point fit
(pose_fit.py) can solve the same data instead. After each ellipsoid the fit is
repeated; from the third on, the run stops when the last ellipsoid moved no
anchor coordinate by more than the stable step and leaving out any one row moves
none by more than twice that. It also stops when the stop file appears, after
MAX_ELLIPSOIDS, or when a collection fails after the third (the fit so far is kept).
The reported uncertainty is the spread of fits that leave out one row at a time.
"""
from __future__ import annotations

import json
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

from autocal.active_learning import SweepConfig
from autocal.anchor_seed import build_anchor_initial_guess
from autocal.dataset_roles import mesh_metadata, normalize_dataset_point_roles
from autocal.ellipse_fit import fit_ellipses
from autocal.pose_fit import fit_pose_least_squares
from autocal.pose_planning import ANCHOR_CLEARANCE_SHARE, SWEEP_PRETENSION_SHARE, predict_sweep_positions
from autocal.sweep_config_generator import generate_sweep_configs
from autocal.sweep_types import MachineConfig, MachineType

ROW_POINTS = 5            # per direction; each row runs both directions
MIN_ELLIPSOIDS = 3
MAX_ELLIPSOIDS = 6
HOME_OFFSET_PRIOR_SIGMA_MM = 5.0
SEED_SCALES = (1.0, 2.0, 4.0)
FITS = ("ellipsoid", "point")
# Every row of a planned sphere needs enough coupled travel for both sweep
# directions (roughly one spool turn on a 30 mm spool), backed off slightly
# from the exact feasibility cliff so early anchor error does not push rows over it.
MIN_ROW_SPAN_MM = 200.0
FEASIBLE_HEIGHT_MARGIN = 0.90


def mesh_rows(machine_type: str, held_mm: Optional[float], level_mm: Optional[Dict[int, float]] = None) -> List[str]:
    """Sweep config lines for one ellipsoid: one row per free line held as level line.

    None gives the bootstrap rows (the collector's default targets). Otherwise the
    held line holds `held_mm` and each level line `level_mm[line]` from where it settles.
    """
    configs = generate_sweep_configs(MachineConfig.from_type(MachineType(machine_type)))
    held = held_anchor(machine_type)
    lines = []
    for config in configs:
        fixed = sorted(int(anchor) for anchor in config["fixed_anchors"])
        row = f"[{','.join(map(str, fixed))}] {config['drive_anchor']} {config['sensor_anchor']}"
        if held_mm is not None:
            row += " fixed=" + ",".join(f"{held_mm:.3f}" if anchor == held
                                        else f"n{(level_mm or {}).get(anchor, 0.0):+.3f}" for anchor in fixed)
        lines.append(row)
    return lines


def held_anchor(machine_type: str) -> int:
    configs = generate_sweep_configs(MachineConfig.from_type(MachineType(machine_type)))
    return max(int(anchor) for anchor in configs[0]["fixed_anchors"])


def _infer_machine_type(anchors: np.ndarray, machine_type: Optional[str]) -> str:
    if machine_type is not None:
        return str(machine_type)
    return "hangprinter_5" if len(anchors) >= 5 else "hangprinter_4"


def _row_min_span_mm(anchors: np.ndarray, held: int, held_mm: float,
                     machine_type: Optional[str] = None) -> float:
    """Smallest predicted drive/sensor travel (mm) across all rows on the sphere at `held_mm`."""
    anchors = np.asarray(anchors, dtype=float)
    configs = generate_sweep_configs(MachineConfig.from_type(MachineType(_infer_machine_type(anchors, machine_type))))
    top = anchors[held]
    radius = float(np.linalg.norm(top)) + float(held_mm)
    if radius <= 0.0:
        return 0.0
    hanging = top - np.array([0.0, 0.0, radius])
    offsets = level_offsets_mm(anchors, held, held_mm)
    norms = np.linalg.norm(anchors, axis=1)
    min_span = float("inf")
    for config in configs:
        fixed = sorted(int(anchor) for anchor in config["fixed_anchors"])
        deltas = [
            float(held_mm) if anchor == held
            else (float(np.linalg.norm(anchors[anchor] - hanging)) - float(norms[anchor])) + offsets[anchor]
            for anchor in fixed
        ]
        sweep = SweepConfig(
            fixed_anchors=fixed,
            fixed_deltas_mm=deltas,
            drive_anchor=int(config["drive_anchor"]),
            sensor_anchor=int(config["sensor_anchor"]),
        )
        points = predict_sweep_positions(anchors, sweep, ROW_POINTS * 2, SWEEP_PRETENSION_SHARE)
        if len(points) < ROW_POINTS * 2:
            return 0.0
        drive_span = float(np.ptp(np.linalg.norm(points - anchors[sweep.drive_anchor], axis=1)))
        sensor_span = float(np.ptp(np.linalg.norm(points - anchors[sweep.sensor_anchor], axis=1)))
        min_span = min(min_span, drive_span, sensor_span)
    return 0.0 if not np.isfinite(min_span) else min_span


def highest_held_mm(
    anchors: np.ndarray,
    held: int,
    *,
    machine_type: Optional[str] = None,
    min_span_mm: float = MIN_ROW_SPAN_MM,
    grid_steps: int = 16,
) -> float:
    """Highest held-line length (most negative) whose rows all keep a usable coupled arc.

    Scans a coarse grid from home (0) toward the anchor-clearance ceiling to find the
    contiguous feasible interval starting at home (avoiding assumptions of global
    monotonicity), bisects that bracket, backs off by FEASIBLE_HEIGHT_MARGIN, and
    verifies that the returned height is itself feasible.
    """
    anchors = np.asarray(anchors, dtype=float)
    ceiling = -(1.0 - ANCHOR_CLEARANCE_SHARE) * float(np.linalg.norm(anchors[held]))
    home_span = _row_min_span_mm(anchors, held, 0.0, machine_type)
    if home_span <= 0.0:
        raise ValueError("No kinematically feasible ellipsoid sweep span at home for the current anchor estimate")
    required_span = min(float(min_span_mm), 0.4 * home_span)

    grid = [float(value) for value in np.linspace(0.0, ceiling, max(2, int(grid_steps)) + 1)]
    feasible_prefix = [0.0]
    infeasible_bound: Optional[float] = None
    for height in grid[1:]:
        if _row_min_span_mm(anchors, held, height, machine_type) >= required_span:
            feasible_prefix.append(height)
        else:
            infeasible_bound = height
            break

    lo = feasible_prefix[-1]
    if infeasible_bound is not None:
        hi = infeasible_bound
        for _ in range(10):
            mid = 0.5 * (lo + hi)
            if _row_min_span_mm(anchors, held, mid, machine_type) >= required_span:
                lo = mid
            else:
                hi = mid

    candidate = lo * FEASIBLE_HEIGHT_MARGIN
    if _row_min_span_mm(anchors, held, candidate, machine_type) >= required_span:
        return candidate
    for height in reversed(feasible_prefix):
        if abs(height) <= abs(candidate) + 1e-9:
            return height
    return 0.0


def level_offsets_mm(anchors: np.ndarray, held: int, held_mm: float) -> Dict[int, float]:
    """Per level line: half the way from its idle length to its shortest on the held sphere.

    Idling, the effector hangs straight below the held anchor. The shortest a
    level line can be is where the sphere comes closest to its anchor.
    """
    top = anchors[held]
    radius = float(np.linalg.norm(top)) + held_mm
    hanging = top - np.array([0.0, 0.0, radius])
    offsets = {}
    for line in range(len(anchors)):
        if line != held:
            idle = float(np.linalg.norm(anchors[line] - hanging))
            shortest = float(np.linalg.norm(anchors[line] - top)) - radius
            offsets[line] = -0.5 * (idle - shortest)
    return offsets


def next_held_mm(
    index: int,
    anchors: Optional[np.ndarray],
    held: int,
    used: Sequence[float],
    *,
    machine_type: Optional[str] = None,
) -> Optional[float]:
    """Held length of ellipsoid `index` (0-based); None for the bootstrap rows.

    `used` holds every held length recorded or tried so far, so a height whose
    rows were all rejected is not tried again.
    """
    if index == 0:
        return None
    if index <= 2:
        return highest_held_mm(anchors, held, machine_type=machine_type) * (0.5 if index == 1 else 1.0)
    heights = sorted(set(round(length, 1) for length in used) | {0.0})
    gaps = [(upper - lower, 0.5 * (lower + upper)) for lower, upper in zip(heights[:-1], heights[1:])]
    return max(gaps)[1]


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


def fit(dataset: dict, starts: Sequence[np.ndarray], use_noise_mean: bool,
        model: str = "ellipsoid", *, fit_stretch_varpro: bool = True,
        varpro_allow_ripple: bool = True, varpro_fixed_ripple_period: Optional[int] = None) -> dict:
    """Best fit, with home offsets, over the given starts (lowest rms)."""
    options = dict(use_noise_mean=use_noise_mean, fit_home_offsets=True,
                   home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    if model == "point":
        return fit_pose_least_squares(
            dataset, starts[0], extra_initial_anchors=starts[1:],
            fit_stretch_varpro=fit_stretch_varpro,
            varpro_allow_ripple=varpro_allow_ripple,
            varpro_fixed_ripple_period=varpro_fixed_ripple_period,
            **options,
        )
    return min((fit_ellipses(dataset, start, **options) for start in starts), key=lambda result: result["rms_mm"])


def row_jackknife_mm(dataset: dict, anchors: np.ndarray, use_noise_mean, model="ellipsoid",
                     *, fit_stretch_varpro: bool = False, fixed_ripple_period: Optional[int] = None) -> Optional[float]:
    """Largest anchor coordinate shift when one row is left out."""
    if len(dataset["sweeps"]) < 3:
        return None
    shifts = []
    for index in range(len(dataset["sweeps"])):
        subset = dict(dataset, sweeps=[s for i, s in enumerate(dataset["sweeps"]) if i != index])
        refit = fit(
            subset, [anchors], use_noise_mean, model,
            fit_stretch_varpro=fit_stretch_varpro,
            varpro_allow_ripple=False,
            varpro_fixed_ripple_period=fixed_ripple_period,
        )["anchors"]
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
    use_noise_mean: bool,
    stable_step_mm: float,
    stop_file: Path,
    model: str = "ellipsoid",
    fit_stretch_varpro: bool = True,
) -> Optional[dict]:
    """Collect and fit ellipsoids until the anchors are stable. Returns the accepted plan and summary."""
    from autocal._autocal_common import _inject_force_args, _merge_sweep_datasets
    if model not in FITS:
        raise ValueError(f"The ellipsoid autocal fits with one of {FITS}, not {model!r}")
    held = held_anchor(machine_type)

    def collect(index: int, dataset: Optional[dict], held_mm: Optional[float]) -> dict:
        name = f"ellipsoid_{index + 1:03d}"
        cfg = dataset_path.with_suffix(f".{name}.cfg.txt")
        out = dataset_path.with_suffix(f".{name}.json")
        levels = None if held_mm is None else level_offsets_mm(history[-1], held, held_mm)
        cfg.write_text("\n".join(mesh_rows(machine_type, held_mm, levels)) + "\n")
        args = list(collector_args)
        if dataset:
            args, _, _ = _inject_force_args(dataset, [a for a in args if a != "--hp-sim-reset"])
            args += ["--completed-sweeps", str(len(dataset["sweeps"]))]
        cmd = ["node", "autocal/control/cli/collect_sweep_data.mjs", "--machineType", machine_type,
               "--sweep-config-file", str(cfg), "--output-file", str(out), "--sweepPoints", str(ROW_POINTS),
               "--return-to-origin", *args]
        log_line(f"; ellipsoid {index + 1}: held line {'at the bootstrap targets' if held_mm is None else f'{held_mm:+.1f} mm'}")
        log_line(";   " + " ".join(cmd))
        run_collection(cmd)
        new = json.loads(out.read_text())
        normalize_dataset_point_roles(new)
        tag_mesh(new["sweeps"], name, held, index + 1)
        merged = new if dataset is None else _merge_sweep_datasets(dataset, new)
        dataset_path.write_text(json.dumps(merged, indent=2))
        return merged

    recorded = json.loads(dataset_path.read_text()) if dataset_path.exists() else None
    if recorded is not None:
        normalize_dataset_point_roles(recorded)
        # Older sweep datasets: sweeps that hold the same line at the same length share a sphere.
        for sweep in recorded["sweeps"]:
            if mesh_metadata(sweep) is None:
                top = max(int(a) for a in sweep["fixed_anchors"])
                length = sweep["fixed_lengths"][sweep["fixed_anchors"].index(top)]
                tag_mesh([sweep], f"sphere_{top}_{length:.0f}", top, 1)
    replay = visits(recorded) if recorded is not None else []
    if no_collect and not replay:
        log_console("; --no-collect needs a mesh dataset")
        return None
    if recorded is not None and not no_collect:
        log_console("; ellipsoid autocal continues an existing dataset by replaying it")
        no_collect = True

    seed = None
    tried: List[float] = []
    history: List[np.ndarray] = []
    dataset: Optional[dict] = None
    result = None
    uncertainty = None
    reason = "max-ellipsoids"
    total = len(replay) if no_collect else MAX_ELLIPSOIDS
    for index in range(total):
        if no_collect:
            dataset = dict(recorded, sweeps=[recorded["sweeps"][i] for group in replay[:index + 1] for i in group])
        else:
            held_lengths = [mesh_metadata(s)["held_length_mm"] for s in dataset["sweeps"]] if dataset else []
            held_mm = next_held_mm(index, history[-1] if history else None, held, held_lengths + tried,
                                   machine_type=machine_type)
            tried += [] if held_mm is None else [held_mm]
            try:
                dataset = collect(index, dataset, held_mm)
            except subprocess.CalledProcessError as error:
                # A collection that fails after the first three ellipsoids keeps the fit so far.
                if len(history) < MIN_ELLIPSOIDS:
                    raise
                log_console(f"; ellipsoid {index + 1} collection failed ({error}); keeping the fit so far")
                reason = "collection-failed"
                break
        if not dataset["sweeps"]:
            continue
        seed = build_anchor_initial_guess(dataset, machine_type=machine_type) if seed is None else seed
        # The blind seed underestimates the anchor distances; on HP5 its own scale can
        # settle in a far minimum, so larger copies start too and the lowest rms wins.
        result = fit(dataset, [seed * scale for scale in SEED_SCALES] + history[-1:], use_noise_mean,
                     model, fit_stretch_varpro=fit_stretch_varpro)
        step = float(np.max(np.abs(result["anchors"] - history[-1]))) if history else None
        history.append(result["anchors"])
        varpro = result.get("varpro") if isinstance(result, dict) else None
        uncertainty = row_jackknife_mm(
            dataset, result["anchors"], use_noise_mean, model,
            fit_stretch_varpro=bool(varpro is not None),
            fixed_ripple_period=(varpro.get("ripple_period") if isinstance(varpro, dict) else None),
        )
        lengths = sorted({round(mesh_metadata(s)["held_length_mm"], 1) for s in dataset["sweeps"]})
        log_console(f"; ellipsoid {index + 1}: {len(dataset['sweeps'])} rows, held lengths {lengths} mm, "
                    f"{model} fit rms {result['rms_mm']:.3f} mm, step "
                    f"{'n/a' if step is None else f'{step:.2f} mm'}, row jackknife "
                    f"{'n/a' if uncertainty is None else f'{uncertainty:.2f} mm'}, offsets "
                    f"{np.round(result['home_offsets_mm'], 2).tolist()}")
        append_log({"timestamp": datetime.now().isoformat(), "dataset": str(dataset_path), "event": "ellipsoid_fit",
                    "iteration": index + 1, "num_sweeps": index + 1, "selected_run": model,
                    "runs": [{"id": model, "metrics": {"pose_fit_anchors": result["anchors"].tolist(),
                                                       "anchor_uncertainty_mm": uncertainty, "step_mm": step}}]})
        if (len(history) >= MIN_ELLIPSOIDS and step <= stable_step_mm
                and uncertainty is not None and uncertainty <= 2.0 * stable_step_mm):
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
