import numpy as np
import pytest

from autocal import active_calibrate as ac


@pytest.fixture(autouse=True)
def _clear_cost_fn_cache():
    ac._ELLIPSE_COST_FN_CACHE.clear()
    yield
    ac._ELLIPSE_COST_FN_CACHE.clear()


def _evaluate(dataset, anchors, **overrides):
    kwargs = {
        "residual_threshold": 1.0,
        "spring_k_multiplier": 1.0,
        "use_flex": False,
        "pointwise_residual_mode": "sampson",
        "pointwise_filtering": True,
        "pointwise_global_mad": True,
        "sweep_wise_filtering": True,
        "sweep_metric": "outlier_ratio",
        "use_noise_mean": True,
        "noise_normalized": True,
        "sigma_source": "auto",
    }
    kwargs.update(overrides)
    return ac._evaluate_cost_at_anchors(dataset, np.asarray(anchors, dtype=float), **kwargs)


def test_evaluate_cost_at_anchors_reuses_cached_cost_function(monkeypatch):
    build_count = {"n": 0}

    class FakeCostFn:
        def __init__(self, offset: float):
            self.offset = float(offset)

        def evaluate(self, anchor_vec):
            return float(np.sum(np.asarray(anchor_vec, dtype=float)) + self.offset)

    def fake_build(dataset, **kwargs):
        _ = (dataset, kwargs)
        build_count["n"] += 1
        return FakeCostFn(build_count["n"])

    monkeypatch.setattr(ac, "_build_ellipse_cost_function", fake_build)

    dataset = {"machine_type": "slideprinter", "num_anchors": 3, "dimensions": 2, "sweeps": []}
    cost1 = _evaluate(dataset, np.ones((3, 2), dtype=float))
    cost2 = _evaluate(dataset, np.full((3, 2), 2.0, dtype=float))

    assert build_count["n"] == 1
    assert np.isclose(cost1, 7.0)
    assert np.isclose(cost2, 13.0)


def test_evaluate_cost_at_anchors_cache_key_includes_config(monkeypatch):
    build_count = {"n": 0}

    class FakeCostFn:
        def evaluate(self, anchor_vec):
            return float(np.sum(np.asarray(anchor_vec, dtype=float)))

    def fake_build(dataset, **kwargs):
        _ = (dataset, kwargs)
        build_count["n"] += 1
        return FakeCostFn()

    monkeypatch.setattr(ac, "_build_ellipse_cost_function", fake_build)

    dataset = {"machine_type": "slideprinter", "num_anchors": 3, "dimensions": 2, "sweeps": []}
    _evaluate(dataset, np.ones((3, 2), dtype=float), noise_normalized=True)
    _evaluate(dataset, np.ones((3, 2), dtype=float), noise_normalized=False)

    assert build_count["n"] == 2
