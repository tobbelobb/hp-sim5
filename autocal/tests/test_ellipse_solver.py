import numpy as np
from scipy.optimize import OptimizeResult
from typing import Optional

from autocal import ellipse_solver


class _QuadraticCost:
    def __init__(self) -> None:
        self.last_f0 = None
        self.evaluate_calls = 0
        self.gradient_calls = 0

    def evaluate(self, x: np.ndarray) -> float:
        self.evaluate_calls += 1
        vec = np.asarray(x, dtype=float).reshape(-1)
        return float(np.dot(vec, vec))

    def gradient_numerical(
        self,
        x: np.ndarray,
        epsilon: float = 1e-6,
        f0: Optional[float] = None,
    ) -> np.ndarray:
        _ = epsilon
        self.gradient_calls += 1
        self.last_f0 = f0
        vec = np.asarray(x, dtype=float).reshape(-1)
        return 2.0 * vec


def test_run_lbfgsb_minimize_supplies_explicit_jac(monkeypatch):
    seen = {"jac": None, "value": None, "grad": None}

    def fake_minimize(fun, x0, **kwargs):
        seen["jac"] = kwargs.get("jac")
        value, grad = fun(np.asarray(x0, dtype=float))
        seen["value"] = float(value)
        seen["grad"] = np.asarray(grad, dtype=float)
        return OptimizeResult(
            x=np.asarray(x0, dtype=float),
            fun=float(value),
            success=True,
            message="ok",
            nit=1,
            nfev=1,
        )

    monkeypatch.setattr(ellipse_solver, "build_compiled_value_and_grad", lambda _c, _lb, _ub: None)
    monkeypatch.setattr(ellipse_solver, "minimize", fake_minimize)

    cost = _QuadraticCost()
    x0 = np.asarray([0.2, -0.3, 0.4], dtype=float)
    result = ellipse_solver._run_lbfgsb_minimize(
        cost,
        x0,
        lb=np.full(x0.shape, -10.0, dtype=float),
        ub=np.full(x0.shape, 10.0, dtype=float),
        bounds=[(-10.0, 10.0)] * x0.size,
        max_iterations=5,
    )

    assert seen["jac"] is True
    assert np.allclose(seen["grad"], 2.0 * x0)
    assert np.isclose(cost.last_f0, seen["value"])
    assert bool(result.success) is True


def test_lbfgsb_objective_falls_back_when_jax_grad_errors(monkeypatch):
    def _broken_jax(_cost_fn, _lb, _ub):
        def _broken(_x):
            raise RuntimeError("jax trace failed")

        return _broken

    monkeypatch.setattr(ellipse_solver, "build_compiled_value_and_grad", _broken_jax)

    cost = _QuadraticCost()
    objective = ellipse_solver._build_lbfgsb_objective_with_jac(
        cost,
        lb=np.asarray([-10.0, -10.0], dtype=float),
        ub=np.asarray([10.0, 10.0], dtype=float),
    )
    value, grad = objective(np.asarray([0.5, -0.5], dtype=float))

    assert np.isfinite(float(value))
    assert np.allclose(np.asarray(grad, dtype=float), np.asarray([1.0, -1.0], dtype=float))
    assert cost.last_f0 is not None


def test_lbfgsb_objective_falls_back_when_jax_grad_nonfinite(monkeypatch):
    def _bad_grad(_cost_fn, _lb, _ub):
        def _return_nonfinite(_x):
            return 1.0, np.asarray([np.nan, np.nan], dtype=float)

        return _return_nonfinite

    monkeypatch.setattr(ellipse_solver, "build_compiled_value_and_grad", _bad_grad)

    cost = _QuadraticCost()
    objective = ellipse_solver._build_lbfgsb_objective_with_jac(
        cost,
        lb=np.asarray([-10.0, -10.0], dtype=float),
        ub=np.asarray([10.0, 10.0], dtype=float),
    )
    value, grad = objective(np.asarray([0.5, -0.5], dtype=float))

    assert np.isfinite(float(value))
    assert np.allclose(np.asarray(grad, dtype=float), np.asarray([1.0, -1.0], dtype=float))
    assert cost.last_f0 is not None


def test_run_lbfgsb_minimize_value_only_mode_uses_scipy_fd(monkeypatch):
    seen = {"jac": "<missing>", "value": None}

    def fake_minimize(fun, x0, **kwargs):
        seen["jac"] = kwargs.get("jac", "<missing>")
        seen["value"] = float(fun(np.asarray(x0, dtype=float)))
        return OptimizeResult(
            x=np.asarray(x0, dtype=float),
            fun=float(seen["value"]),
            success=True,
            message="ok",
            nit=1,
            nfev=1,
        )

    def _should_not_call(_cost_fn, _lb, _ub):
        raise AssertionError("jac path should not be used in fun mode")

    monkeypatch.setenv("AUTOCAL_JAX_LBFGSB_MODE", "fun")
    monkeypatch.setattr(
        ellipse_solver,
        "build_compiled_objective",
        lambda _c, _lb, _ub: (lambda x: float(np.dot(np.asarray(x, dtype=float), np.asarray(x, dtype=float)))),
    )
    monkeypatch.setattr(ellipse_solver, "build_compiled_value_and_grad", _should_not_call)
    monkeypatch.setattr(ellipse_solver, "minimize", fake_minimize)

    cost = _QuadraticCost()
    x0 = np.asarray([0.3, -0.2], dtype=float)
    result = ellipse_solver._run_lbfgsb_minimize(
        cost,
        x0,
        lb=np.full(x0.shape, -10.0, dtype=float),
        ub=np.full(x0.shape, 10.0, dtype=float),
        bounds=[(-10.0, 10.0)] * x0.size,
        max_iterations=5,
    )

    assert seen["jac"] == "<missing>"
    assert np.isclose(float(seen["value"]), float(np.dot(x0, x0)))
    assert cost.gradient_calls == 0
    assert bool(result.success) is True


def test_run_lbfgsb_minimize_value_only_mode_falls_back_when_jax_fun_errors(monkeypatch):
    seen = {"jac": "<missing>", "value": None}

    def fake_minimize(fun, x0, **kwargs):
        seen["jac"] = kwargs.get("jac", "<missing>")
        seen["value"] = float(fun(np.asarray(x0, dtype=float)))
        return OptimizeResult(
            x=np.asarray(x0, dtype=float),
            fun=float(seen["value"]),
            success=True,
            message="ok",
            nit=1,
            nfev=1,
        )

    def _broken_fun(_cost_fn, _lb, _ub):
        def _raise(_x):
            raise RuntimeError("broken jax objective")

        return _raise

    monkeypatch.setenv("AUTOCAL_JAX_LBFGSB_MODE", "fun")
    monkeypatch.setattr(ellipse_solver, "build_compiled_objective", _broken_fun)
    monkeypatch.setattr(ellipse_solver, "minimize", fake_minimize)

    cost = _QuadraticCost()
    x0 = np.asarray([0.4, -0.1], dtype=float)
    result = ellipse_solver._run_lbfgsb_minimize(
        cost,
        x0,
        lb=np.full(x0.shape, -10.0, dtype=float),
        ub=np.full(x0.shape, 10.0, dtype=float),
        bounds=[(-10.0, 10.0)] * x0.size,
        max_iterations=5,
    )

    assert seen["jac"] == "<missing>"
    assert np.isclose(float(seen["value"]), float(np.dot(x0, x0)))
    assert cost.evaluate_calls > 0
    assert cost.gradient_calls == 0
    assert bool(result.success) is True


def test_prepare_frozen_dataset_and_flags_filters_points_and_sweeps(monkeypatch):
    dataset = {
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "sweeps": [
            {"id": "s0", "data_points": [{"i": 0}, {"i": 1}, {"i": 2}]},
            {"id": "s1", "data_points": [{"i": 0}, {"i": 1}, {"i": 2}]},
            {"id": "s2", "data_points": [{"i": 0}, {"i": 1}, {"i": 2}]},
        ],
    }

    class _FakeCostFn:
        def _pointwise_entries(self, _anchors):
            return [
                {"sweep_metric": 0.1, "_inlier_mask": np.asarray([True, False, True]), "num_inliers": 3},
                {"sweep_metric": 0.2, "_inlier_mask": np.asarray([True, False, False]), "num_inliers": 2},
                {"sweep_metric": 9.0, "_inlier_mask": np.asarray([True, True, True]), "num_inliers": 3},
            ], None

        def _sweep_wise_keep_mask(self, _metrics):
            return np.asarray([True, True, False], dtype=bool), 1.0, "ok"

    monkeypatch.setattr(ellipse_solver, "_build_cost_fn", lambda *args, **kwargs: _FakeCostFn())
    monkeypatch.setattr(
        ellipse_solver,
        "get_anchor_opt_bounds",
        lambda _m, _n, _d: (np.full(5, -1000.0, dtype=float), np.full(5, 1000.0, dtype=float)),
    )

    frozen_dataset, flags = ellipse_solver._prepare_frozen_dataset_and_flags(
        dataset,
        np.zeros(6, dtype=float),
        residual_threshold=0.01,
        pointwise_residual_mode="sampson",
        invalid_sweep_penalty=1e6,
        spring_k_multiplier=1.0,
        use_flex=False,
        robust_loss=False,
        huber_delta=1.0,
        pointwise_filtering=True,
        pointwise_global_mad=True,
        sweep_wise_filtering=True,
        sweep_metric="outlier_ratio",
        pointwise_filter_stage=2,
        use_noise_mean=True,
        noise_normalized=True,
        sigma_source="auto",
    )

    assert isinstance(frozen_dataset, dict)
    sweeps = frozen_dataset.get("sweeps", [])
    assert isinstance(sweeps, list)
    assert [s.get("id") for s in sweeps] == ["s0", "s1"]
    assert len(sweeps[0].get("data_points", [])) == 2
    assert len(sweeps[1].get("data_points", [])) == 3
    assert flags["sweep_wise_filtering"] is False
    assert flags["pointwise_filtering"] is False
    assert flags["pointwise_filter_stage"] == 1


def test_prepare_frozen_dataset_and_flags_respects_disable_env(monkeypatch):
    dataset = {
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "sweeps": [{"id": "s0", "data_points": [{"i": 0}]}],
    }
    monkeypatch.setenv("AUTOCAL_FREEZE_FILTER_MASKS", "0")

    frozen_dataset, flags = ellipse_solver._prepare_frozen_dataset_and_flags(
        dataset,
        np.zeros(6, dtype=float),
        residual_threshold=0.01,
        pointwise_residual_mode="sampson",
        invalid_sweep_penalty=1e6,
        spring_k_multiplier=1.0,
        use_flex=False,
        robust_loss=False,
        huber_delta=1.0,
        pointwise_filtering=True,
        pointwise_global_mad=True,
        sweep_wise_filtering=True,
        sweep_metric="outlier_ratio",
        pointwise_filter_stage=2,
        use_noise_mean=True,
        noise_normalized=True,
        sigma_source="auto",
    )

    assert frozen_dataset is dataset
    assert flags["pointwise_filtering"] is True
    assert flags["sweep_wise_filtering"] is True


def test_solve_anchors_uses_reduced_slideprinter_optimizer_vector(monkeypatch):
    dataset = {
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "sweeps": [],
    }
    initial_guess = np.asarray(
        [
            [-400.0, 0.0],
            [400.0, 0.0],
            [0.0, 500.0],
        ],
        dtype=float,
    )
    seen: dict = {}

    class _FakeCostFn:
        sweeps = []

        def evaluate(self, x: np.ndarray) -> float:
            vec = np.asarray(x, dtype=float).reshape(-1)
            return float(np.dot(vec, vec))

        def evaluate_detailed(self, x: np.ndarray):
            anchors = ellipse_solver.anchor_opt_vec_to_matrix(np.asarray(x, dtype=float), "slideprinter", 3, 2)
            return ellipse_solver.CostResult(
                total_cost=float(self.evaluate(x)),
                per_sweep_costs={},
                num_valid_sweeps=0,
                num_invalid_sweeps=0,
                anchor_estimate=anchors,
                noise_metrics=None,
            )

        def pointwise_residual_rows(self, _x: np.ndarray):
            return []

        def robustness_diagnostics(self, _x: np.ndarray, *, top_n: int = 5):
            _ = top_n
            return {}

    def fake_build_restart_cost_fn(*args, **kwargs):
        _ = args, kwargs
        return _FakeCostFn(), dataset, {}

    def fake_run_lbfgsb_minimize(cost_fn, x0, **kwargs):
        _ = kwargs
        seen["x0"] = np.asarray(x0, dtype=float).copy()
        return OptimizeResult(
            x=np.asarray(x0, dtype=float),
            fun=float(cost_fn.evaluate(x0)),
            success=True,
            message="ok",
            nit=1,
            nfev=1,
        )

    monkeypatch.setattr(ellipse_solver, "_build_cost_fn", lambda *args, **kwargs: _FakeCostFn())
    monkeypatch.setattr(ellipse_solver, "_build_restart_cost_fn", fake_build_restart_cost_fn)
    monkeypatch.setattr(ellipse_solver, "_run_lbfgsb_minimize", fake_run_lbfgsb_minimize)

    result = ellipse_solver.solve_anchors(
        dataset,
        initial_guess=initial_guess,
        method="L-BFGS-B",
        max_iterations=4,
        num_restarts=1,
        use_parallel=False,
        pointwise_filtering=False,
        pointwise_global_mad=False,
        sweep_wise_filtering=False,
    )

    x0_seen = np.asarray(seen["x0"], dtype=float)
    anchors = np.asarray(result["anchors"], dtype=float)
    assert x0_seen.shape == (5,)
    assert anchors.shape == (3, 2)
    assert np.isclose(float(anchors[0, 0]), 0.0)
    assert np.allclose(
        anchors,
        ellipse_solver.anchor_opt_vec_to_matrix(x0_seen, "slideprinter", 3, 2),
    )
