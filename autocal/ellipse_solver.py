from __future__ import annotations

"""Optimization helpers for ellipse-based calibration."""

from typing import Dict, Optional, Tuple, Union

import numpy as np
from scipy.optimize import differential_evolution, minimize

from autocal.ellipse_cost import CostResult, EllipseCostFunction
from autocal.sweep_types import MachineConfig, MachineType
from autocal.theoretical_ellipse import (
    anchors_vec_to_matrix,
    get_anchor_bounds,
)

_BOUND_PENALTY_WEIGHT = 1e4

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
    geometry_weights: Tuple[float, float, float] = (1.0, 1.0, 0.2),
    residual_threshold: float = 0.01,
    use_weights: bool = True,
    invalid_sweep_penalty: float = 1e6,
    spring_k_multiplier: float = 1.0,
    use_flex: bool = True,
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
        invalid_sweep_penalty=invalid_sweep_penalty,
        spring_k_multiplier=float(spring_k_multiplier),
        use_flex=bool(use_flex),
    )

    method_raw = str(method or "L-BFGS-B")
    method_norm = method_raw.strip().replace("_", "-").lower()
    if method_norm in ("slsqp", "sqp"):
        # SLSQP has repeatedly shown immediate termination for this objective (often status=4).
        # Keep the CLI/API stable by treating it as an alias for a bounded quasi-Newton method.
        if verbose:
            print("[solver] Mapping method SLSQP -> L-BFGS-B for robustness.")
        method_norm = "l-bfgs-b"

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

    for idx, x0 in enumerate(initial_guesses):
        if verbose:
            print(f"Starting optimization {idx + 1}/{len(initial_guesses)}...")

        x0_clipped = np.clip(x0, lb, ub)

        if method_norm == "differential-evolution":
            result = differential_evolution(
                cost_fn.evaluate,
                bounds=bounds,
                maxiter=max_iterations,
                polish=True,
                updating="deferred",
                callback=(
                    (lambda xk, convergence: cost_callback(np.asarray(xk, dtype=float), float(convergence)))
                    if cost_callback
                    else None
                ),
            )
        else:
            if method_norm in ("nelder-mead", "neldermead"):
                result = minimize(
                    _bounded_objective,
                    x0_clipped,
                    method="Nelder-Mead",
                    callback=((lambda xk: cost_callback(np.asarray(xk, dtype=float), None)) if cost_callback else None),
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
                    callback=((lambda xk: cost_callback(np.asarray(xk, dtype=float), None)) if cost_callback else None),
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
                    callback=((lambda xk: cost_callback(np.asarray(xk, dtype=float), None)) if cost_callback else None),
                    options=options,
                )

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
