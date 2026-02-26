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
