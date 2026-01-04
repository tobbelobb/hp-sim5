from __future__ import annotations

"""Optimization helpers for ellipse-based calibration."""

import concurrent.futures
from typing import Dict, Optional, Tuple, Union

import numpy as np
from scipy.optimize import OptimizeResult, differential_evolution, minimize

from autocal.ellipse_cost import (
    CostResult,
    EllipseCostFunction,
    geometry_distance_squared_normalized,
)
from autocal.sweep_types import MachineConfig, MachineType
from autocal.theoretical_ellipse import (
    anchors_vec_to_matrix,
    get_anchor_bounds,
    predict_ellipse_geometry,
)

_BOUND_PENALTY_WEIGHT = 1e4


def _optimize_restart_worker(payload: dict) -> dict:
    """Run a single restart in a separate process; returns an OptimizeResult-compatible dict."""
    dataset = payload["dataset"]
    x0_clipped = np.asarray(payload["x0"], dtype=float)
    method_raw = str(payload["method"])
    max_iterations = int(payload["max_iterations"])
    geometry_weights = tuple(payload["geometry_weights"])
    residual_threshold = float(payload["residual_threshold"])
    use_weights = bool(payload["use_weights"])
    cost_mode = str(payload["cost_mode"])
    pointwise_residual_mode = str(payload.get("pointwise_residual_mode", "sampson"))
    invalid_sweep_penalty = float(payload["invalid_sweep_penalty"])
    spring_k_multiplier = float(payload["spring_k_multiplier"])
    use_flex = bool(payload["use_flex"])
    robust_loss = bool(payload.get("robust_loss", False))
    huber_delta = float(payload.get("huber_delta", 1.0))

    machine_type_raw = dataset.get("machine_type", "hangprinter_4")
    machine_type = machine_type_raw.value if isinstance(machine_type_raw, MachineType) else str(machine_type_raw)
    num_anchors = int(dataset.get("num_anchors", 4))
    dimensions = int(dataset.get("dimensions", 3))
    lb, ub = get_anchor_bounds(machine_type)
    bounds = list(zip(lb, ub))

    cost_fn = EllipseCostFunction(
        dataset,
        geometry_weights=geometry_weights,
        residual_threshold=residual_threshold,
        use_weights=use_weights,
        cost_mode=str(cost_mode),
        pointwise_residual_mode=str(pointwise_residual_mode),
        invalid_sweep_penalty=invalid_sweep_penalty,
        spring_k_multiplier=float(spring_k_multiplier),
        use_flex=bool(use_flex),
        robust_loss=bool(robust_loss),
        huber_delta=float(huber_delta),
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
    geometry_weights: Tuple[float, float, float] = (1.0, 1.0, 0.2),
    residual_threshold: float = 0.01,
    use_weights: bool = True,
    cost_mode: str = "geometry",
    pointwise_residual_mode: str = "sampson",
    invalid_sweep_penalty: float = 1e6,
    spring_k_multiplier: float = 1.0,
    use_flex: bool = True,
    robust_loss: bool = False,
    huber_delta: float = 1.0,
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

    cost_fn = EllipseCostFunction(
        dataset,
        geometry_weights=geometry_weights,
        residual_threshold=residual_threshold,
        use_weights=use_weights,
        cost_mode=str(cost_mode),
        pointwise_residual_mode=str(pointwise_residual_mode),
        invalid_sweep_penalty=invalid_sweep_penalty,
        spring_k_multiplier=float(spring_k_multiplier),
        use_flex=bool(use_flex),
        robust_loss=bool(robust_loss),
        huber_delta=float(huber_delta),
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
                (
                    obs_geom,
                    weight,
                    fixed_lengths_abs,
                    _sid,
                    residual_ratio,
                    violation_penalty,
                ) = cost_fn._fit_observed_geometry(sweep, anchors)  # type: ignore[attr-defined]
                fixed_indices, _, drive_idx, sensor_idx, *_rest = cost_fn._extract_sweep_arrays(sweep)  # type: ignore[attr-defined]
                pred_geom = predict_ellipse_geometry(
                    anchors,
                    fixed_indices,
                    fixed_lengths_abs,
                    drive_idx,
                    sensor_idx,
                    dimensions,
                )
                rr = residual_ratio if np.isfinite(residual_ratio) else float("inf")
                print(
                    f"  [{sid}] w={weight:.3g} rr={rr:.3g} viol={violation_penalty:.3g} pred={'ok' if pred_geom else 'none'} obs={'ok' if obs_geom else 'none'}"
                )
                try:
                    p_cost, p_rms, p_max, _p_viol, _p_sid = cost_fn._pointwise_predicted_cost(sweep, anchors)  # type: ignore[attr-defined]
                    if np.isfinite(p_cost):
                        resid_kind = "Sampson"
                        if str(pointwise_residual_mode or "").strip().lower() in ("euclidean", "exact", "distance"):
                            resid_kind = "Euclidean"
                        print(f"    pred {resid_kind} rms={p_rms:.3g} max={p_max:.3g}")
                except Exception:
                    pass

                # Print a compact geometric mismatch breakdown when available.
                if obs_geom is None or pred_geom is None:
                    continue
                from autocal.ellipse_cost import canonicalize_geometry

                pred_center, pred_axes, pred_theta = canonicalize_geometry(*pred_geom)
                obs_center, obs_axes, obs_theta = obs_geom
                l2_scale = float(getattr(cost_fn, "_l2_scale", 1.0))
                geom_cost = geometry_distance_squared_normalized(
                    obs_center,
                    obs_axes,
                    obs_theta,
                    pred_center,
                    pred_axes,
                    pred_theta,
                    cost_fn.geometry_weights,
                    center_scale=l2_scale,
                    axes_scale=l2_scale,
                )
                print(
                    f"    fixed={fixed_indices}@{[round(float(v),3) for v in fixed_lengths_abs]} drive={drive_idx} sensor={sensor_idx} geom_cost={geom_cost:.3g}"
                )
                print(
                    f"    obs ctr=({obs_center[0]:.3g},{obs_center[1]:.3g}) axes=({obs_axes[0]:.3g},{obs_axes[1]:.3g}) th={obs_theta:.3g}"
                )
                print(
                    f"    pre ctr=({pred_center[0]:.3g},{pred_center[1]:.3g}) axes=({pred_axes[0]:.3g},{pred_axes[1]:.3g}) th={pred_theta:.3g}"
                )
        except Exception:
            # Debug printing must never derail the solver.
            return

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
                "geometry_weights": geometry_weights,
                "residual_threshold": float(residual_threshold),
                "use_weights": bool(use_weights),
                "cost_mode": str(cost_mode),
                "pointwise_residual_mode": str(pointwise_residual_mode),
                "invalid_sweep_penalty": float(invalid_sweep_penalty),
                "spring_k_multiplier": float(spring_k_multiplier),
                "use_flex": bool(use_flex),
                "robust_loss": bool(robust_loss),
                "huber_delta": float(huber_delta),
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
