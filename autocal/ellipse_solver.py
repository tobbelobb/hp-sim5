from __future__ import annotations

"""Optimization helpers for ellipse-based calibration."""

import concurrent.futures
import csv
from pathlib import Path
from typing import Dict, Optional, Tuple, Union

import numpy as np
from scipy.optimize import OptimizeResult, differential_evolution, minimize

from autocal.ellipse_cost import CostResult, EllipseCostFunction
from autocal.sweep_types import MachineConfig, MachineType
from autocal.theoretical_ellipse import anchors_vec_to_matrix, get_anchor_bounds

_BOUND_PENALTY_WEIGHT = 1e4


def _optimize_restart_worker(payload: dict) -> dict:
    """Run a single restart in a separate process; returns an OptimizeResult-compatible dict."""
    dataset = payload["dataset"]
    x0_clipped = np.asarray(payload["x0"], dtype=float)
    method_raw = str(payload["method"])
    max_iterations = int(payload["max_iterations"])
    residual_threshold = float(payload["residual_threshold"])
    pointwise_residual_mode = str(payload.get("pointwise_residual_mode", "sampson"))
    invalid_sweep_penalty = float(payload["invalid_sweep_penalty"])
    spring_k_multiplier = float(payload["spring_k_multiplier"])
    use_flex = bool(payload["use_flex"])
    robust_loss = bool(payload.get("robust_loss", False))
    huber_delta = float(payload.get("huber_delta", 1.0))
    pointwise_filtering = bool(payload.get("pointwise_filtering", True))
    pointwise_global_mad = bool(payload.get("pointwise_global_mad", True))
    sweep_wise_filtering = bool(payload.get("sweep_wise_filtering", True))
    sweep_metric = str(payload.get("sweep_metric", "outlier_ratio"))
    pointwise_filter_stage = payload.get("pointwise_filter_stage", 0)

    machine_type_raw = dataset.get("machine_type", "hangprinter_4")
    machine_type = machine_type_raw.value if isinstance(machine_type_raw, MachineType) else str(machine_type_raw)
    num_anchors = int(dataset.get("num_anchors", 4))
    dimensions = int(dataset.get("dimensions", 3))
    lb, ub = get_anchor_bounds(machine_type)
    bounds = list(zip(lb, ub))

    cost_fn = EllipseCostFunction(
        dataset,
        residual_threshold=residual_threshold,
        pointwise_residual_mode=str(pointwise_residual_mode),
        invalid_sweep_penalty=invalid_sweep_penalty,
        spring_k_multiplier=float(spring_k_multiplier),
        use_flex=bool(use_flex),
        robust_loss=bool(robust_loss),
        huber_delta=float(huber_delta),
        pointwise_filtering=bool(pointwise_filtering),
        pointwise_global_mad=bool(pointwise_global_mad),
        sweep_wise_filtering=bool(sweep_wise_filtering),
        sweep_metric=str(sweep_metric),
        pointwise_filter_stage=int(pointwise_filter_stage) if pointwise_filter_stage is not None else 0,
    )

    method_norm = method_raw.strip().replace("_", "-").lower()
    if method_norm in ("slsqp", "sqp"):
        method_norm = "l-bfgs-b"

    def _bounded_objective(x: np.ndarray) -> float:
        x = np.asarray(x, dtype=float).reshape(-1)
        x_clipped = np.clip(x, lb, ub)
        penalty = float(_BOUND_PENALTY_WEIGHT * np.dot(x - x_clipped, x - x_clipped))
        return float(cost_fn.evaluate(x_clipped) + penalty)

    if method_norm in ("nelder-mead", "neldermead"):
        result = minimize(
            _bounded_objective,
            x0_clipped,
            method="Nelder-Mead",
            options={
                "maxiter": max_iterations,
                "xatol": 1e-4,
                "fatol": 1e-8,
                "adaptive": True,
                "disp": False,
            },
        )
        result.x = np.clip(np.asarray(result.x, dtype=float), lb, ub)
        result.fun = float(cost_fn.evaluate(result.x))
    elif method_norm in ("hybrid", "nm+lbfgsb", "nelder-mead+lbfgsb"):
        nm = minimize(
            _bounded_objective,
            x0_clipped,
            method="Nelder-Mead",
            options={
                "maxiter": max(200, int(max_iterations // 2)),
                "xatol": 1e-3,
                "fatol": 1e-6,
                "adaptive": True,
                "disp": False,
            },
        )
        x1 = np.clip(np.asarray(nm.x, dtype=float), lb, ub)
        result = minimize(
            cost_fn.evaluate,
            x1,
            method="L-BFGS-B",
            bounds=bounds,
            options={"maxiter": max_iterations, "ftol": 1e-12, "disp": False},
        )
    else:
        scipy_method = method_raw
        if method_norm in ("l-bfgs-b", "lbfgsb"):
            scipy_method = "L-BFGS-B"
        elif method_norm == "powell":
            scipy_method = "Powell"

        options: Dict[str, object] = {"maxiter": max_iterations, "disp": False}
        if scipy_method == "L-BFGS-B":
            options["ftol"] = 1e-12
            options["maxls"] = 40
        elif scipy_method == "Powell":
            options["xtol"] = 1e-4
            options["ftol"] = 1e-8

        result = minimize(
            cost_fn.evaluate,
            x0_clipped,
            method=scipy_method,
            bounds=bounds,
            options=options,
        )

    best = OptimizeResult(
        x=np.asarray(result.x, dtype=float),
        fun=float(result.fun),
        success=bool(getattr(result, "success", True)),
        message=str(getattr(result, "message", "")),
        nit=int(getattr(result, "nit", 0) or 0),
        nfev=int(getattr(result, "nfev", 0) or 0),
    )
    return dict(best)


def _dataset_metadata(dataset: Union[dict, "SweepDataset"]) -> Tuple[str, int, int]:
    """Extract (machine_type, num_anchors, dimensions) from datasets or dicts."""
    if hasattr(dataset, "machine_config"):
        cfg: MachineConfig = dataset.machine_config  # type: ignore[assignment]
        return cfg.machine_type.value, cfg.num_anchors, cfg.dimensions

    machine_type_raw = dataset.get("machine_type", "hangprinter_4")  # type: ignore[index]
    machine_type = (
        machine_type_raw.value if isinstance(machine_type_raw, MachineType) else str(machine_type_raw)
    )
    num_anchors = int(dataset.get("num_anchors", 4))  # type: ignore[index]
    dimensions = int(dataset.get("dimensions", 3))  # type: ignore[index]
    return machine_type, num_anchors, dimensions


def solve_anchors(
    dataset: Union[dict, "SweepDataset"],
    initial_guess: Optional[np.ndarray] = None,
    method: str = "L-BFGS-B",
    max_iterations: int = 1000,
    num_restarts: int = 4,
    use_parallel: bool = True,
    max_workers: Optional[int] = None,
    progress_every: int = 10,
    residual_threshold: float = 0.01,
    pointwise_residual_mode: str = "sampson",
    invalid_sweep_penalty: float = 1e6,
    spring_k_multiplier: float = 1.0,
    use_flex: bool = True,
    robust_loss: bool = False,
    huber_delta: float = 1.0,
    pointwise_filtering: bool = True,
    pointwise_global_mad: bool = True,
    sweep_wise_filtering: bool = True,
    sweep_metric: str = "outlier_ratio",
    pointwise_filter_stage: Optional[int] = None,
    robust_debug: bool = False,
    residuals_csv: Optional[Union[str, Path]] = None,
    use_noise_mean: bool = True,
    noise_normalized: bool = True,
    sigma_source: str = "auto",
    cost_callback: Optional[callable] = None,
    verbose: bool = False,
) -> Dict[str, object]:
    """
    Optimize anchor positions to match observed ellipses.

    Returns a dict with anchors (matrix), cost, success flag, detailed cost info,
    and the raw scipy result from the best restart.
    """
    machine_type, num_anchors, dimensions = _dataset_metadata(dataset)
    lb, ub = get_anchor_bounds(machine_type)

    expected_len = num_anchors * dimensions
    if lb.size != expected_len or ub.size != expected_len:
        raise ValueError(
            f"Bounds size mismatch for {machine_type}: expected {expected_len}, got {lb.size}"
        )

    bounds = list(zip(lb, ub))
    rng = np.random.default_rng(0)

    if pointwise_filter_stage is None and bool(pointwise_filtering):
        stages = [
            {"stage": 0, "restarts": int(max(num_restarts, 1)), "iter_factor": 1.0, "label": "wide-huber"},
            {"stage": 1, "restarts": 1, "iter_factor": 0.6, "label": "tight-huber"},
            {"stage": 2, "restarts": 1, "iter_factor": 0.4, "label": "trim"},
        ]
        current_guess = initial_guess
        final_result: Optional[Dict[str, object]] = None
        for idx, stage in enumerate(stages):
            stage_iters = int(max(1, round(max_iterations * stage["iter_factor"])))
            stage_restarts = int(stage["restarts"])
            if verbose or robust_debug:
                print(f"[gnc] stage {idx + 1}/{len(stages)}: {stage['label']} maxiter={stage_iters}")
            final_result = solve_anchors(
                dataset,
                initial_guess=current_guess,
                method=method,
                max_iterations=stage_iters,
                num_restarts=stage_restarts,
                use_parallel=bool(use_parallel) and stage_restarts > 1,
                max_workers=max_workers,
                progress_every=progress_every,
                residual_threshold=residual_threshold,
                pointwise_residual_mode=pointwise_residual_mode,
                invalid_sweep_penalty=invalid_sweep_penalty,
                spring_k_multiplier=spring_k_multiplier,
                use_flex=use_flex,
                robust_loss=robust_loss,
                huber_delta=huber_delta,
                pointwise_filtering=pointwise_filtering,
                pointwise_global_mad=pointwise_global_mad,
                sweep_wise_filtering=sweep_wise_filtering,
                sweep_metric=sweep_metric,
                pointwise_filter_stage=int(stage["stage"]),
                robust_debug=bool(robust_debug) if idx == len(stages) - 1 else False,
                residuals_csv=residuals_csv if idx == len(stages) - 1 else None,
                use_noise_mean=use_noise_mean,
                noise_normalized=noise_normalized,
                sigma_source=sigma_source,
                cost_callback=cost_callback if idx == len(stages) - 1 else None,
                verbose=verbose,
            )
            if final_result is None:
                break
            anchors_out = final_result.get("anchors")
            if anchors_out is not None:
                current_guess = np.asarray(anchors_out, dtype=float).ravel()
        if final_result is not None:
            return final_result

    cost_fn = EllipseCostFunction(
        dataset,
        residual_threshold=residual_threshold,
        pointwise_residual_mode=str(pointwise_residual_mode),
        invalid_sweep_penalty=invalid_sweep_penalty,
        spring_k_multiplier=float(spring_k_multiplier),
        use_flex=bool(use_flex),
        robust_loss=bool(robust_loss),
        huber_delta=float(huber_delta),
        pointwise_filtering=bool(pointwise_filtering),
        pointwise_global_mad=bool(pointwise_global_mad),
        sweep_wise_filtering=bool(sweep_wise_filtering),
        sweep_metric=str(sweep_metric),
        pointwise_filter_stage=int(pointwise_filter_stage) if pointwise_filter_stage is not None else 0,
        use_noise_mean=bool(use_noise_mean),
        noise_normalized=bool(noise_normalized),
        sigma_source=str(sigma_source),
    )

    method_raw = str(method or "L-BFGS-B")
    method_norm = method_raw.strip().replace("_", "-").lower()
    if method_norm in ("slsqp", "sqp"):
        # SLSQP has repeatedly shown immediate termination for this objective (often status=4).
        # Keep the CLI/API stable by treating it as an alias for a bounded quasi-Newton method.
        if verbose:
            print("[solver] Mapping method SLSQP -> L-BFGS-B for robustness.")
        method_norm = "l-bfgs-b"

    def _summarize_cost(tag: str, x: np.ndarray, *, include_details: bool = True) -> None:
        detailed: CostResult = cost_fn.evaluate_detailed(np.asarray(x, dtype=float))
        costs = detailed.per_sweep_costs
        worst = sorted(costs.items(), key=lambda kv: kv[1], reverse=True)[:3]
        worst_str = ", ".join(f"{sid}={c:.3g}" for sid, c in worst)
        print(
            f"[{tag}] cost={detailed.total_cost:.6g} valid={detailed.num_valid_sweeps} invalid={detailed.num_invalid_sweeps}"
            + (f" worst: {worst_str}" if worst_str else "")
        )
        if not include_details or not worst:
            return

        try:
            anchors = anchors_vec_to_matrix(np.asarray(x, dtype=float), num_anchors, dimensions)
            norms = np.linalg.norm(anchors, axis=1)
            norms_str = ", ".join(f"{v:.3f}" for v in norms.tolist())
            print(f"  anchor norms: [{norms_str}]")
            if anchors.shape == (3, 2):
                d01 = float(np.linalg.norm(anchors[0] - anchors[1]))
                d02 = float(np.linalg.norm(anchors[0] - anchors[2]))
                d12 = float(np.linalg.norm(anchors[1] - anchors[2]))
                print(f"  pairwise dists: d01={d01:.3f} d02={d02:.3f} d12={d12:.3f}")
            for sid, _ in worst[:2]:
                sweep = next((s for s in cost_fn.sweeps if (s.get("id", "") if isinstance(s, dict) else s.id) == sid), None)
                if sweep is None:
                    continue
                try:
                    p_cost, p_rms, p_max, _p_viol, _p_sid = cost_fn._pointwise_predicted_cost(sweep, anchors)  # type: ignore[attr-defined]
                    if np.isfinite(p_cost):
                        resid_kind = "Sampson"
                        if str(pointwise_residual_mode or "").strip().lower() in ("euclidean", "exact", "distance"):
                            resid_kind = "Euclidean"
                        print(f"  [{sid}] pred {resid_kind} rms={p_rms:.3g} max={p_max:.3g}")
                except Exception:
                    pass
        except Exception:
            # Debug printing must never derail the solver.
            return

    def _print_robust_diagnostics() -> None:
        diag = cost_fn.robustness_diagnostics(best_result.x, top_n=5)
        pw = diag.get("pointwise_filtering") if isinstance(diag, dict) else None
        sw = diag.get("sweep_wise_filtering") if isinstance(diag, dict) else None
        if isinstance(pw, dict):
            def _fmt_float(value: object) -> str:
                if value is None:
                    return "n/a"
                try:
                    val = float(value)
                except (TypeError, ValueError):
                    return "n/a"
                return f"{val:.3g}" if np.isfinite(val) else "n/a"

            sigma_source_mode = str(pw.get("sigma_source") or "auto")
            sigma_counts = pw.get("sigma_source_counts")
            if isinstance(sigma_counts, dict):
                total = sigma_counts.get("total", 0)
                counts_str = " ".join(
                    f"{k}={int(sigma_counts.get(k, 0))}"
                    for k in ("point", "origin", "min", "mixed")
                    if k in sigma_counts
                )
                total_str = f"total={int(total)}" if isinstance(total, (int, float)) else "total=n/a"
                print(f"[robust] pointwise sigma_source: mode={sigma_source_mode} {counts_str} {total_str}")
            else:
                print(f"[robust] pointwise sigma_source: mode={sigma_source_mode}")

            print(
                "[robust] pointwise:"
                f" enabled={bool(pw.get('enabled'))}"
                f" stage={pw.get('stage')} ({pw.get('stage_name')})"
                f" hard_cut={bool(pw.get('hard_cut'))}"
                f" huber_mult={float(pw.get('huber_multiplier', 0.0)):.3g}"
            )
            scale_source = pw.get("scale_source")
            if scale_source:
                scale_global = pw.get("scale_global")
                scale_floor = pw.get("scale_floor")
                scale_floor_mm = pw.get("scale_floor_mm")
                global_str = _fmt_float(scale_global)
                floor_str = _fmt_float(scale_floor)
                floor_mm_str = _fmt_float(scale_floor_mm)
                print(
                    "[robust] pointwise scale:"
                    f" source={scale_source} global={global_str}"
                    f" floor={floor_str} floor_mm={floor_mm_str}"
                )
            sigma_noise = pw.get("sigma_noise_mm")
            sigma_mult = pw.get("sigma_mult")
            sigma_scaled = pw.get("sigma_scaled_mm")
            sigma_min = pw.get("sigma_min_mm")
            sigma_floor = pw.get("sigma_floor_mm")
            sigma_source = pw.get("sigma_floor_source")
            sigma_floor_deg = pw.get("sigma_floor_deg")
            if (
                sigma_noise is not None
                or sigma_scaled is not None
                or sigma_min is not None
                or sigma_floor is not None
                or sigma_floor_deg is not None
            ):
                noise_str = (
                    f"{float(sigma_noise):.6g}"
                    if sigma_noise is not None and np.isfinite(sigma_noise)
                    else "n/a"
                )
                mult_str = (
                    f"{float(sigma_mult):.3g}"
                    if sigma_mult is not None and np.isfinite(sigma_mult)
                    else "n/a"
                )
                scaled_str = (
                    f"{float(sigma_scaled):.6g}"
                    if sigma_scaled is not None and np.isfinite(sigma_scaled)
                    else "n/a"
                )
                min_str = (
                    f"{float(sigma_min):.6g}"
                    if sigma_min is not None and np.isfinite(sigma_min)
                    else "n/a"
                )
                used_str = (
                    f"{float(sigma_floor):.6g}"
                    if sigma_floor is not None and np.isfinite(sigma_floor)
                    else "n/a"
                )
                floor_deg_str = (
                    f"{float(sigma_floor_deg):.6g}"
                    if sigma_floor_deg is not None and np.isfinite(sigma_floor_deg)
                    else "n/a"
                )
                source_str = str(sigma_source or "n/a")
                print(
                    "[robust] pointwise sigma_floor_mm:"
                    f" origin={noise_str}"
                    f" mult={mult_str}"
                    f" scaled={scaled_str}"
                    f" min={min_str}"
                    f" used={used_str}"
                    f" floor_deg={floor_deg_str}"
                    f" source={source_str}"
                )
            stats = pw.get("inlier_ratio_stats")
            if isinstance(stats, dict):
                stat_str = (
                    f"min={float(stats.get('min', 0.0)):.3g}"
                    f" med={float(stats.get('median', 0.0)):.3g}"
                    f" max={float(stats.get('max', 0.0)):.3g}"
                )
                print(f"[robust] pointwise inlier_ratio: {stat_str}")
            worst = pw.get("worst_inliers")
            if isinstance(worst, list) and worst:
                entries = []
                for item in worst:
                    if not isinstance(item, dict):
                        continue
                    sid = str(item.get("sweep_id", ""))
                    nin = item.get("num_inliers")
                    npt = item.get("num_points")
                    ratio = item.get("inlier_ratio")
                    if ratio is not None and np.isfinite(ratio):
                        ratio_str = f"{float(ratio):.3g}"
                    else:
                        ratio_str = "n/a"
                    entries.append(f"{sid} {nin}/{npt} ({ratio_str})")
                if entries:
                    print("[robust] pointwise worst: " + ", ".join(entries))

        if isinstance(sw, dict):
            status = str(sw.get("status", ""))
            if status and status != "disabled":
                threshold = sw.get("threshold")
                if threshold is None or not np.isfinite(threshold):
                    thresh_str = "n/a"
                else:
                    thresh_str = f"{float(threshold):.3g}"
                metric_mode = sw.get("metric_mode")
                metric_str = str(metric_mode) if metric_mode else "n/a"
                print(
                    "[robust] sweep-wise:"
                    f" enabled={bool(sw.get('enabled'))}"
                    f" status={status}"
                    f" metric={metric_str}"
                    f" threshold={thresh_str}"
                    f" rejected={int(sw.get('rejected', 0))}"
                )
                worst = sw.get("worst_sweeps")
                if isinstance(worst, list) and worst:
                    entries = []
                    for item in worst:
                        if not isinstance(item, dict):
                            continue
                        sid = str(item.get("sweep_id", ""))
                        dist = float(item.get("metric", float("nan")))
                        keep = bool(item.get("keep", True))
                        entries.append(f"{sid} m={dist:.3g} {'keep' if keep else 'drop'}")
                    if entries:
                        print("[robust] sweep-wise worst: " + ", ".join(entries))

    initial_guesses = []
    if initial_guess is not None:
        guess = np.asarray(initial_guess, dtype=float).reshape(expected_len)
        initial_guesses.append(np.clip(guess, lb, ub))

    while len(initial_guesses) < max(num_restarts, 1):
        guess = rng.uniform(lb, ub)
        initial_guesses.append(guess)

    best_result = None
    best_cost = np.inf

    def _bounded_objective(x: np.ndarray) -> float:
        """Objective wrapper that penalizes leaving bounds (for methods without bounds support)."""
        x = np.asarray(x, dtype=float).reshape(-1)
        x_clipped = np.clip(x, lb, ub)
        penalty = float(_BOUND_PENALTY_WEIGHT * np.dot(x - x_clipped, x - x_clipped))
        return float(cost_fn.evaluate(x_clipped) + penalty)

    run_parallel = (
        bool(use_parallel)
        and not bool(verbose)
        and cost_callback is None
        and method_norm != "differential-evolution"
        and isinstance(dataset, dict)
        and len(initial_guesses) > 1
    )

    if run_parallel:
        payloads = [
            {
                "dataset": dataset,
                "x0": np.clip(np.asarray(x0, dtype=float), lb, ub),
                "method": method_raw,
                "max_iterations": int(max_iterations),
                "residual_threshold": float(residual_threshold),
                "pointwise_residual_mode": str(pointwise_residual_mode),
                "invalid_sweep_penalty": float(invalid_sweep_penalty),
                "spring_k_multiplier": float(spring_k_multiplier),
                "use_flex": bool(use_flex),
                "robust_loss": bool(robust_loss),
                "huber_delta": float(huber_delta),
                "pointwise_filtering": bool(pointwise_filtering),
                "pointwise_global_mad": bool(pointwise_global_mad),
                "sweep_wise_filtering": bool(sweep_wise_filtering),
                "sweep_metric": str(sweep_metric),
                "pointwise_filter_stage": (
                    int(pointwise_filter_stage) if pointwise_filter_stage is not None else 0
                ),
            }
            for x0 in initial_guesses
        ]
        parallel_exc: Optional[BaseException] = None
        try:
            with concurrent.futures.ProcessPoolExecutor(max_workers=max_workers) as executor:
                for idx, out in enumerate(executor.map(_optimize_restart_worker, payloads), start=1):
                    result = OptimizeResult(out)
                    if verbose:
                        print(
                            f"[restart {idx} done] fun={float(result.fun):.6g} success={bool(getattr(result,'success',True))} "
                            f"nit={getattr(result,'nit',None)} nfev={getattr(result,'nfev',None)} msg={getattr(result,'message','')}"
                        )
                    if float(result.fun) < best_cost:
                        best_cost = float(result.fun)
                        best_result = result
        except BaseException as exc:
            parallel_exc = exc

        if parallel_exc is not None:
            # Some sandboxed environments disallow creating multiprocessing primitives; fall back to threads.
            if verbose:
                print(f"[solver] Parallel restarts unavailable ({parallel_exc}); falling back to threads.")
            with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as executor:
                for idx, out in enumerate(executor.map(_optimize_restart_worker, payloads), start=1):
                    result = OptimizeResult(out)
                    if float(result.fun) < best_cost:
                        best_cost = float(result.fun)
                        best_result = result
    else:
        for idx, x0 in enumerate(initial_guesses):
            x0_clipped = np.clip(x0, lb, ub)
            if verbose:
                print(f"Starting optimization {idx + 1}/{len(initial_guesses)}...")
                _summarize_cost(f"restart {idx + 1} init", x0_clipped, include_details=True)

            progress_stride = int(progress_every) if verbose else 0
            if progress_stride <= 0:
                progress_stride = 0
            details_stride = 10 if verbose else 0
            progress_state = {"iter": 0}

            def _progress_callback(xk: np.ndarray, _convergence: Optional[float] = None) -> None:
                if cost_callback is not None:
                    try:
                        cost_callback(np.asarray(xk, dtype=float), _convergence)
                    except TypeError:
                        cost_callback(np.asarray(xk, dtype=float))

                if progress_stride <= 0:
                    return
                progress_state["iter"] += 1
                if progress_state["iter"] % progress_stride != 0:
                    return
                include_details = bool(details_stride and (progress_state["iter"] % details_stride == 0))
                _summarize_cost(
                    f"restart {idx + 1} iter {progress_state['iter']}",
                    np.asarray(xk, dtype=float),
                    include_details=include_details,
                )

            if method_norm == "differential-evolution":
                result = differential_evolution(
                    cost_fn.evaluate,
                    bounds=bounds,
                    maxiter=max_iterations,
                    polish=True,
                    updating="deferred",
                    callback=(
                        (
                            lambda xk, convergence: _progress_callback(
                                np.asarray(xk, dtype=float), float(convergence)
                            )
                        )
                    ),
                )
            else:
                if method_norm in ("nelder-mead", "neldermead"):
                    result = minimize(
                        _bounded_objective,
                        x0_clipped,
                        method="Nelder-Mead",
                        callback=(lambda xk: _progress_callback(np.asarray(xk, dtype=float), None)),
                        options={
                            "maxiter": max_iterations,
                            "xatol": 1e-4,
                            "fatol": 1e-8,
                            "adaptive": True,
                            "disp": False,
                        },
                    )
                    # Project to bounds for downstream consumption.
                    result.x = np.clip(np.asarray(result.x, dtype=float), lb, ub)
                    result.fun = float(cost_fn.evaluate(result.x))
                elif method_norm in ("hybrid", "nm+lbfgsb", "nelder-mead+lbfgsb"):
                    nm = minimize(
                        _bounded_objective,
                        x0_clipped,
                        method="Nelder-Mead",
                        options={
                            "maxiter": max(200, int(max_iterations // 2)),
                            "xatol": 1e-3,
                            "fatol": 1e-6,
                            "adaptive": True,
                            "disp": False,
                        },
                    )
                    x1 = np.clip(np.asarray(nm.x, dtype=float), lb, ub)
                    result = minimize(
                        cost_fn.evaluate,
                        x1,
                        method="L-BFGS-B",
                        bounds=bounds,
                        callback=(lambda xk: _progress_callback(np.asarray(xk, dtype=float), None)),
                        options={"maxiter": max_iterations, "ftol": 1e-12, "disp": False},
                    )
                else:
                    scipy_method = method_raw
                    if method_norm in ("l-bfgs-b", "lbfgsb"):
                        scipy_method = "L-BFGS-B"
                    elif method_norm == "powell":
                        scipy_method = "Powell"

                    options: Dict[str, object] = {"maxiter": max_iterations, "disp": False}
                    if scipy_method == "L-BFGS-B":
                        options["ftol"] = 1e-12
                        options["maxls"] = 40
                    elif scipy_method == "Powell":
                        options["xtol"] = 1e-4
                        options["ftol"] = 1e-8

                    result = minimize(
                        cost_fn.evaluate,
                        x0_clipped,
                        method=scipy_method,
                        bounds=bounds,
                        callback=(lambda xk: _progress_callback(np.asarray(xk, dtype=float), None)),
                        options=options,
                    )
            if verbose:
                try:
                    print(
                        f"[restart {idx + 1} done] fun={float(result.fun):.6g} success={bool(getattr(result,'success',True))} "
                        f"nit={getattr(result,'nit',None)} nfev={getattr(result,'nfev',None)} msg={getattr(result,'message','')}"
                    )
                except Exception:
                    pass

            if result.fun < best_cost:
                best_cost = float(result.fun)
                best_result = result

    if best_result is None:
        return {
            "anchors": None,
            "cost": np.inf,
            "success": False,
            "details": None,
            "raw_result": None,
        }

    anchors_matrix = anchors_vec_to_matrix(best_result.x, num_anchors, dimensions)
    detailed: CostResult = cost_fn.evaluate_detailed(best_result.x)

    if verbose:
        print("Optimization complete.")
        print(f"  Best cost: {best_cost:.6e}")
        print(f"  Valid sweeps: {detailed.num_valid_sweeps}")
        print(f"  Invalid sweeps: {detailed.num_invalid_sweeps}")
    if robust_debug:
        try:
            _print_robust_diagnostics()
        except Exception as exc:
            print(f"[robust] diagnostics failed: {exc}")

    if residuals_csv is not None:
        try:
            path = Path(residuals_csv)
            rows = cost_fn.pointwise_residual_rows(best_result.x)
            if not rows:
                print(f"[residuals] no pointwise residuals to write for {path}")
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open("w", encoding="utf-8", newline="") as f:
                    writer = csv.DictWriter(
                        f,
                        fieldnames=[
                            "sweep_id",
                            "point_idx",
                            "drive_anchor",
                            "sensor_anchor",
                            "l_drive_mm",
                            "l_sensor_mm",
                            "residual_l2",
                            "residual_mm",
                            "cutoff_mm",
                            "sigma_noise_mm",
                            "sigma_mult",
                            "sigma_scaled_mm",
                            "sigma_min_mm",
                            "sigma_floor_mm",
                            "sigma_floor_source",
                        ],
                    )
                    writer.writeheader()
                    writer.writerows(rows)
                print(f"[residuals] wrote {len(rows)} points to {path}")
        except Exception as exc:
            print(f"[residuals] failed to write residuals: {exc}")

    return {
        "anchors": anchors_matrix,
        "cost": best_cost,
        "success": bool(getattr(best_result, "success", True)),
        "details": detailed,
        "raw_result": best_result,
    }


def format_anchors_gcode(anchors: np.ndarray, machine_type: Union[str, MachineType]) -> str:
    """Render anchors as a G-code M669 string."""
    machine_type_str = machine_type.value if isinstance(machine_type, MachineType) else str(machine_type)

    if machine_type_str in ("hangprinter_4", "hangprinter_5"):
        labels = ["A", "B", "C", "D", "I"]
        parts = []
        for idx, anchor in enumerate(anchors):
            label = labels[idx] if idx < len(labels) else f"P{idx}"
            if anchor.shape[0] < 3:
                raise ValueError("Hangprinter anchors must be 3D for G-code formatting")
            parts.append(f"{label}{anchor[0]:.2f}:{anchor[1]:.2f}:{anchor[2]:.2f}")
        return "M669 " + " ".join(parts)

    if machine_type_str == "slideprinter":
        return f"; Anchors: {anchors.tolist()}"

    return f"; Anchors ({machine_type_str}): {anchors.tolist()}"
