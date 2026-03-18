from __future__ import annotations

from typing import Any, Callable, Dict, Optional, Tuple

import numpy as np
from scipy.optimize import minimize_scalar


def run_uniform_scale_polish(
    *,
    search_r: bool,
    num_anchors: int,
    radii_mm: np.ndarray,
    buildup_factor: np.ndarray,
    anchors_eval: np.ndarray,
    prior_cost: Callable[[np.ndarray, np.ndarray], float],
    data_cost: Callable[[dict, np.ndarray], float],
    build_dataset_and_params: Callable[[np.ndarray, np.ndarray], Tuple[Any, dict]],
    radii_respect_bounds: Callable[[np.ndarray], bool],
    uniform_radius_scale: Callable[[np.ndarray, np.ndarray], Optional[float]],
    s_lo: float = 0.8,
    s_hi: float = 1.2,
    coarse_points: int = 21,
) -> Tuple[np.ndarray, np.ndarray, Any, dict, float, Dict[str, object]]:
    radii_arr = np.asarray(radii_mm, dtype=float).reshape(-1)
    buildup_arr = np.asarray(buildup_factor, dtype=float).reshape(-1)
    anchors_arr = np.asarray(anchors_eval, dtype=float)
    spool_start, transformed_start = build_dataset_and_params(radii_arr, buildup_arr)
    start_data_cost = float(data_cost(transformed_start, anchors_arr))
    start_prior_cost = float(prior_cost(radii_arr, buildup_arr))
    start_total_cost = (
        float(start_data_cost + start_prior_cost)
        if np.isfinite(start_data_cost) and np.isfinite(start_prior_cost)
        else float("inf")
    )
    data_improve_tol = (
        max(1e-9, 1e-4 * max(1.0, abs(float(start_data_cost))))
        if np.isfinite(start_data_cost)
        else 1e-9
    )
    info: Dict[str, object] = {
        "attempted": True,
        "success": False,
        "accepted": False,
        "accepted_data_cost": False,
        "accepted_total_objective": False,
        "start_scale": 1.0,
        "start_data_cost": (float(start_data_cost) if np.isfinite(start_data_cost) else None),
        "start_prior_cost": (float(start_prior_cost) if np.isfinite(start_prior_cost) else None),
        "start_total_cost": (float(start_total_cost) if np.isfinite(start_total_cost) else None),
        "best_scale": 1.0,
        "best_data_cost": (float(start_data_cost) if np.isfinite(start_data_cost) else None),
        "best_prior_cost": (float(start_prior_cost) if np.isfinite(start_prior_cost) else None),
        "best_total_cost": (float(start_total_cost) if np.isfinite(start_total_cost) else None),
        "coarse_points": int(max(3, int(coarse_points))),
        "refined": False,
        "message": "scale_polish_not_run",
    }
    if not search_r:
        info["message"] = "scale polish skipped (find_radii=off)"
        return radii_arr, anchors_arr, spool_start, transformed_start, float(start_data_cost), info
    if not np.isfinite(s_lo) or not np.isfinite(s_hi) or s_hi <= s_lo:
        info["message"] = "scale polish skipped (invalid bounds)"
        return radii_arr, anchors_arr, spool_start, transformed_start, float(start_data_cost), info
    if not np.all(np.isfinite(radii_arr)) or np.any(radii_arr <= 0.0):
        info["message"] = "scale polish skipped (invalid radii)"
        return radii_arr, anchors_arr, spool_start, transformed_start, float(start_data_cost), info
    if anchors_arr.ndim != 2 or anchors_arr.shape[0] != num_anchors or not np.all(np.isfinite(anchors_arr)):
        info["message"] = "scale polish skipped (invalid anchors)"
        return radii_arr, anchors_arr, spool_start, transformed_start, float(start_data_cost), info

    eval_cache: Dict[
        float,
        Tuple[float, float, float, np.ndarray, np.ndarray, Any, dict],
    ] = {}

    def _eval_scale(
        scale: float,
    ) -> Tuple[float, float, float, np.ndarray, np.ndarray, Any, dict]:
        key = float(np.round(float(scale), decimals=9))
        cached = eval_cache.get(key)
        if cached is not None:
            return cached
        if not np.isfinite(scale) or scale <= 0.0:
            out = (
                float("inf"),
                float("inf"),
                float("inf"),
                radii_arr,
                anchors_arr,
                spool_start,
                transformed_start,
            )
            eval_cache[key] = out
            return out
        radii_try = np.asarray(radii_arr * float(scale), dtype=float)
        if not radii_respect_bounds(radii_try):
            out = (
                float("inf"),
                float("inf"),
                float("inf"),
                radii_arr,
                anchors_arr,
                spool_start,
                transformed_start,
            )
            eval_cache[key] = out
            return out
        anchors_try = np.asarray(anchors_arr * float(scale), dtype=float)
        spool_try, transformed_try = build_dataset_and_params(radii_try, buildup_arr)
        data_cost_try = float(data_cost(transformed_try, anchors_try))
        prior_cost_try = float(prior_cost(radii_try, buildup_arr))
        total_cost_try = (
            float(data_cost_try + prior_cost_try)
            if np.isfinite(data_cost_try) and np.isfinite(prior_cost_try)
            else float("inf")
        )
        out = (
            float(total_cost_try),
            float(data_cost_try),
            float(prior_cost_try),
            radii_try,
            anchors_try,
            spool_try,
            transformed_try,
        )
        eval_cache[key] = out
        return out

    grid_count = max(3, int(coarse_points))
    grid = np.linspace(float(s_lo), float(s_hi), grid_count)
    best_idx = -1
    best_eval = (
        float(start_total_cost),
        float(start_data_cost),
        float(start_prior_cost),
        radii_arr,
        anchors_arr,
        spool_start,
        transformed_start,
    )
    for idx, s in enumerate(grid):
        eval_row = _eval_scale(float(s))
        if np.isfinite(eval_row[0]) and (not np.isfinite(best_eval[0]) or eval_row[0] + 1e-12 < best_eval[0]):
            best_eval = eval_row
            best_idx = int(idx)

    if best_idx >= 0:
        lo_ref = float(grid[max(0, best_idx - 1)])
        hi_ref = float(grid[min(grid_count - 1, best_idx + 1)])
        if np.isfinite(lo_ref) and np.isfinite(hi_ref) and hi_ref - lo_ref > 1e-6:
            try:
                ref = minimize_scalar(
                    lambda s: float(_eval_scale(float(s))[0]),
                    bounds=(lo_ref, hi_ref),
                    method="bounded",
                    options={"xatol": 1e-4, "maxiter": 40},
                )
                if bool(getattr(ref, "success", False)):
                    info["refined"] = True
                    eval_ref = _eval_scale(float(ref.x))
                    if np.isfinite(eval_ref[0]) and eval_ref[0] + 1e-12 < best_eval[0]:
                        best_eval = eval_ref
            except Exception:
                pass

    best_total_cost = float(best_eval[0])
    best_data_cost = float(best_eval[1])
    best_prior_cost = float(best_eval[2])
    improve_tol = (
        max(1e-9, 1e-4 * max(1.0, abs(float(start_total_cost))))
        if np.isfinite(start_total_cost)
        else 1e-9
    )
    if np.isfinite(best_total_cost):
        best_scale = uniform_radius_scale(best_eval[3], radii_arr)
        if best_scale is not None and np.isfinite(best_scale):
            info["best_scale"] = float(best_scale)
    if np.isfinite(best_data_cost):
        info["best_data_cost"] = float(best_data_cost)
    if np.isfinite(best_prior_cost):
        info["best_prior_cost"] = float(best_prior_cost)
    if np.isfinite(best_total_cost):
        info["best_total_cost"] = float(best_total_cost)
    info["accepted_data_cost"] = bool(
        np.isfinite(best_data_cost)
        and np.isfinite(start_data_cost)
        and best_data_cost + data_improve_tol < float(start_data_cost)
    )
    if (
        np.isfinite(best_total_cost)
        and np.isfinite(start_total_cost)
        and best_total_cost + improve_tol < float(start_total_cost)
    ):
        info["success"] = True
        info["accepted"] = True
        info["accepted_total_objective"] = True
        info["message"] = "scale polish improved total objective"
        return (
            np.asarray(best_eval[3], dtype=float),
            np.asarray(best_eval[4], dtype=float),
            best_eval[5],
            best_eval[6],
            float(best_data_cost),
            info,
        )
    info["success"] = bool(np.isfinite(best_total_cost))
    info["accepted"] = False
    info["accepted_total_objective"] = False
    if bool(info.get("accepted_data_cost", False)):
        info["message"] = "scale polish rejected (total objective did not improve)"
    else:
        info["message"] = "scale polish did not improve total objective"
    return radii_arr, anchors_arr, spool_start, transformed_start, float(start_data_cost), info


def apply_final_scale_polish(
    *,
    best: Dict[str, object],
    use_scale_fix_2: bool,
    use_scale_fix_3: bool,
    prior_cost: Callable[[np.ndarray, np.ndarray], float],
    spool_rank_score: Callable[..., Tuple[float, Optional[float]]],
    rank_better: Callable[[float, float, float, float], bool],
    run_uniform_scale_polish_fn: Callable[..., Tuple[np.ndarray, np.ndarray, Any, dict, float, Dict[str, object]]],
) -> Tuple[Dict[str, object], Dict[str, object]]:
    final_scale_polish_info: Dict[str, object] = {
        "attempted": False,
        "success": False,
        "accepted": False,
        "message": "final scale polish disabled",
    }
    if not (use_scale_fix_2 or use_scale_fix_3):
        return best, final_scale_polish_info
    if best["dataset"] is None or best["spool_params"] is None:
        return best, final_scale_polish_info

    prior_before = float(
        prior_cost(
            np.asarray(best["radii_mm"], dtype=float),
            np.asarray(best["buildup_factor"], dtype=float),
        )
    )
    (
        radii_polished,
        anchors_polished,
        spool_params_polished,
        transformed_polished,
        polished_cost,
        polish_info,
    ) = run_uniform_scale_polish_fn(
        radii_mm=np.asarray(best["radii_mm"], dtype=float),
        buildup_factor=np.asarray(best["buildup_factor"], dtype=float),
        anchors_eval=np.asarray(best["anchors"], dtype=float),
    )
    final_scale_polish_info = dict(polish_info)
    prior_after = float(
        prior_cost(
            np.asarray(radii_polished, dtype=float),
            np.asarray(best["buildup_factor"], dtype=float),
        )
    )
    if np.isfinite(prior_before):
        final_scale_polish_info["start_prior_cost"] = float(prior_before)
    if np.isfinite(prior_after):
        final_scale_polish_info["best_prior_cost"] = float(prior_after)
    start_data = final_scale_polish_info.get("start_data_cost")
    best_data = final_scale_polish_info.get("best_data_cost")
    if isinstance(start_data, (int, float)) and np.isfinite(float(start_data)) and np.isfinite(prior_before):
        final_scale_polish_info["start_total_cost"] = float(float(start_data) + prior_before)
    if isinstance(best_data, (int, float)) and np.isfinite(float(best_data)) and np.isfinite(prior_after):
        final_scale_polish_info["best_total_cost"] = float(float(best_data) + prior_after)
    if bool(polish_info.get("accepted", False)) and np.isfinite(polished_cost):
        best_rank_ref = float(best.get("rank_score", float("inf")))
        best_total_ref = float(best.get("total_cost", float("inf")))
        polished_total_cost = (
            float(polished_cost + prior_after)
            if np.isfinite(polished_cost) and np.isfinite(prior_after)
            else float("inf")
        )
        polished_rank_score, polished_rank_internal = spool_rank_score(
            transformed_polished,
            anchors_polished,
        )
        polished_rank_score_f = (
            float(polished_rank_score) if np.isfinite(polished_rank_score) else float("inf")
        )
        rank_accept = rank_better(
            polished_rank_score_f,
            float(polished_total_cost),
            best_rank_ref,
            best_total_ref,
        )
        final_scale_polish_info["accepted_total_objective"] = True
        final_scale_polish_info["candidate_rank_score"] = (
            float(polished_rank_score_f) if np.isfinite(polished_rank_score_f) else None
        )
        final_scale_polish_info["candidate_rank_internal"] = (
            None if polished_rank_internal is None else float(polished_rank_internal)
        )
        final_scale_polish_info["start_rank_score"] = (
            float(best_rank_ref) if np.isfinite(best_rank_ref) else None
        )
        final_scale_polish_info["rank_better"] = bool(rank_accept)
        if bool(rank_accept):
            best = {
                "cost": float(polished_cost),
                "total_cost": float(polished_total_cost),
                "rank_score": float(polished_rank_score_f),
                "rank_internal": (
                    None if polished_rank_internal is None else float(polished_rank_internal)
                ),
                "radii_mm": np.asarray(radii_polished, dtype=float),
                "buildup_factor": np.asarray(best["buildup_factor"], dtype=float),
                "anchors": np.asarray(anchors_polished, dtype=float),
                "spool_params": spool_params_polished,
                "dataset": transformed_polished,
            }
        else:
            final_scale_polish_info["accepted"] = False
            final_scale_polish_info["message"] = "scale polish rejected by rank objective"
    return best, final_scale_polish_info
