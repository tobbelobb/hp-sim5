"""Recovery of constrained fits from the current dataset, without collection."""
from __future__ import annotations

import copy
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from autocal._autocal_common import _load_json, _write_json
from autocal.theoretical_ellipse import get_anchor_bounds


def sparse_recovery_dataset(dataset: Dict[str, object], fraction: float = 0.5) -> Dict[str, object]:
    """Return a small, endpoint-preserving training subset for recovery seeding."""
    result = copy.deepcopy(dataset)
    for sweep in result.get("sweeps", []):
        if not isinstance(sweep, dict) or not isinstance(sweep.get("data_points"), list):
            continue
        points = sweep["data_points"]
        groups: Dict[object, List[int]] = {}
        for index, point in enumerate(points):
            if not isinstance(point, dict):
                continue
            direction = point.get("source_drive_anchor", sweep.get("drive_anchor"))
            groups.setdefault(direction, []).append(index)
        keep = set()
        for indices in groups.values():
            if not indices:
                continue
            target = min(len(indices), max(5, int(np.ceil(float(fraction) * len(indices)))))
            positions = {
                i: float(points[i].get("drive_setpoint_mm", points[i].get("l_drive", i)))
                for i in indices
            }
            selected = {min(indices, key=positions.get), max(indices, key=positions.get)}
            while len(selected) < target:
                selected.add(
                    max(
                        (i for i in indices if i not in selected),
                        key=lambda i: (
                            min(abs(positions[i] - positions[j]) for j in selected),
                            -i,
                        ),
                    )
                )
            keep.update(selected)
        sweep["data_points"] = [point for i, point in enumerate(points) if i in keep]
    return result



def recovery_geometry_valid(plan: Dict[str, object]) -> bool:
    """Reject non-finite, out-of-bounds, or degenerate anchor layouts."""
    try:
        anchors = np.asarray(plan.get("anchors"), dtype=float)
        machine_type = str(plan.get("machine_type", ""))
        if anchors.ndim != 2 or anchors.size == 0 or not np.all(np.isfinite(anchors)):
            return False
        lower, upper = get_anchor_bounds(machine_type)
        lower = np.asarray(lower, dtype=float).reshape(anchors.shape)
        upper = np.asarray(upper, dtype=float).reshape(anchors.shape)
        margin = np.maximum(1e-5, 1e-5 * np.maximum(np.abs(lower), np.abs(upper)))
        if np.any(anchors <= lower + margin) or np.any(anchors >= upper - margin):
            return False
        centered = anchors - np.mean(anchors, axis=0, keepdims=True)
        if np.linalg.matrix_rank(centered) < min(anchors.shape[1], anchors.shape[0] - 1):
            return False
        for i in range(anchors.shape[0]):
            for j in range(i):
                if float(np.linalg.norm(anchors[i] - anchors[j])) <= 1e-3:
                    return False
        return True
    except (TypeError, ValueError, np.linalg.LinAlgError):
        return False



def warm_start_seeds_from_plan(plan: Dict[str, object]) -> Dict[str, np.ndarray]:
    def _coerce_matrix(value: object) -> Optional[np.ndarray]:
        try:
            arr = np.asarray(value, dtype=float)
        except (TypeError, ValueError):
            return None
        if arr.ndim != 2 or arr.size <= 0 or not np.all(np.isfinite(arr)):
            return None
        return arr

    def _coerce_vector(
        value: object,
        *,
        size: Optional[int] = None,
        positive: bool = False,
    ) -> Optional[np.ndarray]:
        try:
            arr = np.asarray(value, dtype=float).reshape(-1)
        except (TypeError, ValueError):
            return None
        if arr.size <= 0 or not np.all(np.isfinite(arr)):
            return None
        if arr.size == 1 and size is not None and size > 1:
            arr = np.full(size, float(arr[0]), dtype=float)
        if size is not None and arr.size != size:
            return None
        if positive and np.any(arr <= 0.0):
            return None
        return arr

    seeds: Dict[str, np.ndarray] = {}
    anchors = _coerce_matrix(plan.get("anchors"))
    if anchors is not None:
        seeds["initial_guess"] = anchors
    anchor_count = int(anchors.shape[0]) if anchors is not None else None

    length_model = plan.get("length_model")
    if not isinstance(length_model, dict):
        return seeds

    radii_seed = _coerce_vector(
        length_model.get("effective_radii_mm", length_model.get("modeled_radii_mm")),
        size=anchor_count,
        positive=True,
    )
    if radii_seed is not None:
        seeds["initial_radii_mm"] = radii_seed

    buildup_seed = _coerce_vector(
        length_model.get("modeled_buildup_factor"),
        size=anchor_count,
    )
    if buildup_seed is None and length_model.get("buildup_factor_k") is not None:
        buildup_seed = _coerce_vector(
            length_model.get("buildup_factor_k"),
            size=anchor_count,
        )
    if buildup_seed is not None:
        seeds["initial_buildup_factor"] = buildup_seed
    return seeds



def _single_objective_schedule(value: object) -> List[int]:
    if isinstance(value, (list, tuple)) and value:
        try:
            return [int(value[-1])]
        except (TypeError, ValueError):
            pass
    return [1]



def build_recovery_attempts(
    settings: Dict[str, object]
) -> List[Tuple[str, Dict[str, object]]]:
    attempts: List[Tuple[str, Dict[str, object]]] = []
    current = dict(settings)
    if bool(current.get("sweep_wise_filtering")):
        current = dict(current)
        current["sweep_wise_filtering"] = False
        attempts.append(("restore_sweeps", dict(current)))
    if bool(current.get("pointwise_filtering")):
        current = dict(current)
        current["pointwise_filtering"] = False
        current["pointwise_global_mad"] = False
        attempts.append(("restore_points", dict(current)))

    out: List[Tuple[str, Dict[str, object]]] = []
    for label, attempt_settings in attempts:
        settings_eff = dict(attempt_settings)
        if (
            not bool(settings_eff.get("pointwise_filtering"))
            and not bool(settings_eff.get("sweep_wise_filtering"))
        ):
            settings_eff["filter_schedule"] = ["warmup"]
            settings_eff["objective_schedule"] = _single_objective_schedule(
                settings_eff.get("objective_schedule")
            )
        out.append((label, settings_eff))
    return out



def attempt_underconstrained_recovery(
    selected: Dict[str, object],
    *,
    collector_output: Path,
    work_path: Path,
    sparse_recovery: bool,
    execute_plan_run: Callable[..., Dict[str, object]],
    log_console: Callable[[str], None],
    log_line: Callable[[str], None],
) -> Tuple[Optional[Dict[str, object]], Optional[Dict[str, object]]]:
    settings = dict(selected.get("settings") or {})
    attempts = build_recovery_attempts(settings)
    if not attempts:
        return None, None

    run_id = str(selected.get("id", "run"))
    run_flags = str(selected.get("flags", ""))
    overrides = dict(selected.get("overrides") or {})
    log_console(
        "; selected run underconstrained; trying to restore sweeps/points from current data."
    )

    warm_start = warm_start_seeds_from_plan(selected["plan"])
    started = time.perf_counter()
    valid_results: List[Dict[str, object]] = []
    recovery_labels = [label for label, _settings in attempts]
    if sparse_recovery:
        recovery_labels.extend(("sparse_seed", "sparse_refine"))
    recovery_info: Dict[str, object] = {
        "attempts_total": int(len(recovery_labels)),
        "attempt_labels": recovery_labels,
        "sparse_enabled": bool(sparse_recovery),
        "success": False,
    }
    for idx, (label, settings_try) in enumerate(attempts, start=1):
        log_line(
            f"; underconstrained_recovery: run={run_id} "
            f"attempt={idx}/{len(attempts)} mode={label}"
        )
        result = execute_plan_run(
            run_id=run_id,
            run_flags=run_flags,
            overrides=overrides,
            settings=settings_try,
            collector_output=collector_output,
            log_prefix=f"; full-auto recovery {run_id} [{label}]",
            path_tag=f"{run_id}.{label}",
            initial_guess=warm_start.get("initial_guess"),
            initial_radii_mm=warm_start.get("initial_radii_mm"),
            initial_buildup_factor=warm_start.get("initial_buildup_factor"),
        )
        recovery_info.update(
            {
                "attempt": str(label),
                "attempt_index": int(idx),
            }
        )
        result["recovery"] = dict(recovery_info)
        result_valid = (
            bool(result["metrics"].get("valid"))
            and not bool(result["metrics"].get("underconstrained_penalty", False))
            and recovery_geometry_valid(result["plan"])
        )
        if result_valid:
            valid_results.append(result)
            if not sparse_recovery:
                recovery_info["success"] = True
                result["recovery"] = dict(recovery_info)
                log_console(
                    "; underconstrained recovery succeeded; continuing with the repaired constrained estimate."
                )
                return result, dict(recovery_info)
        if recovery_geometry_valid(result["plan"]):
            warm_start = warm_start_seeds_from_plan(result["plan"])

    if sparse_recovery:
        sparse_settings = dict(attempts[-1][1])
        sparse_path = work_path.with_name(
            f".{work_path.stem}.{run_id}.sparse_recovery.json"
        )
        sparse_dataset = sparse_recovery_dataset(_load_json(work_path))
        _write_json(sparse_path, sparse_dataset)
        try:
            log_line(
                f"; underconstrained_recovery: run={run_id} sparse seed points="
                f"{sum(len(s.get('data_points', [])) for s in sparse_dataset.get('sweeps', []))}"
            )
            sparse_result = execute_plan_run(
                run_id=run_id,
                run_flags=run_flags,
                overrides=overrides,
                settings=sparse_settings,
                collector_output=collector_output,
                log_prefix=f"; full-auto recovery {run_id} [sparse_seed]",
                path_tag=f"{run_id}.sparse_seed",
                dataset_path_override=sparse_path,
                initial_guess=warm_start.get("initial_guess"),
                initial_radii_mm=warm_start.get("initial_radii_mm"),
                initial_buildup_factor=warm_start.get("initial_buildup_factor"),
            )
            recovery_info["sparse_seed_valid"] = recovery_geometry_valid(sparse_result["plan"])
            if recovery_info["sparse_seed_valid"]:
                sparse_warm_start = warm_start_seeds_from_plan(sparse_result["plan"])
                refined = execute_plan_run(
                    run_id=run_id,
                    run_flags=run_flags,
                    overrides=overrides,
                    settings=sparse_settings,
                    collector_output=collector_output,
                    log_prefix=f"; full-auto recovery {run_id} [sparse_refine]",
                    path_tag=f"{run_id}.sparse_refine",
                    initial_guess=sparse_warm_start.get("initial_guess"),
                    initial_radii_mm=sparse_warm_start.get("initial_radii_mm"),
                    initial_buildup_factor=sparse_warm_start.get("initial_buildup_factor"),
                )
                refined_valid = (
                    bool(refined["metrics"].get("valid"))
                    and not bool(refined["metrics"].get("underconstrained_penalty", False))
                    and recovery_geometry_valid(refined["plan"])
                )
                recovery_info["sparse_refine_valid"] = refined_valid
                if refined_valid:
                    valid_results.append(refined)
        finally:
            try:
                sparse_path.unlink()
            except OSError:
                pass

    if valid_results:
        best = min(
            valid_results,
            key=lambda item: (
                float(item["metrics"].get("rank_score", float("inf"))),
                float(item["metrics"].get("primary_cost", float("inf"))),
            ),
        )
        recovery_info["success"] = True
        recovery_info["valid_candidates"] = int(len(valid_results))
        recovery_info["elapsed_seconds"] = float(time.perf_counter() - started)
        best["recovery"] = dict(recovery_info)
        log_console(
            "; underconstrained recovery succeeded; continuing with the best valid repaired estimate."
        )
        return best, dict(recovery_info)

    log_console(
        "; underconstrained recovery exhausted current data without re-establishing constraints."
    )
    recovery_info["elapsed_seconds"] = float(time.perf_counter() - started)
    return None, dict(recovery_info)

