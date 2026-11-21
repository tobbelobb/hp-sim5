import numpy as np
import pytest

from autocal.force_ripple import _PERIODS, _force_period


@pytest.mark.parametrize("ripple", [True, False])
def test_force_period_requires_a_signal_across_sweeps(ripple):
    rng = np.random.default_rng(14)
    groups = np.repeat(np.arange(6), 24)
    theta = rng.uniform(0, 2 * np.pi, len(groups))
    motor = np.tile(np.arange(3), len(groups) // 3)
    geometry = np.column_stack([np.ones(len(groups)), rng.normal(size=(len(groups), 3))])
    sensitivity = rng.uniform(.03, .10, size=len(groups))
    phase = theta[:, None] * _PERIODS
    trig = np.stack([np.sin(phase), np.cos(phase)], axis=-1)
    harmonics = (sensitivity[:, None, None, None] * np.eye(3)[motor, None, :, None]
                 * trig[:, :, None, :]).reshape(len(groups), len(_PERIODS), 6)
    coefficients = np.array([[.6, 0.], [-.29, .23], [.3, -.11]])
    residuals = geometry @ np.array([.02, -.04, .08, .017]) + rng.normal(0, .001, len(groups))
    if ripple:
        residuals -= harmonics[:, 136] @ coefficients.ravel()
    fit = _force_period(residuals, geometry, harmonics, groups)
    if not ripple:
        assert fit is None
        return
    period, fitted, gain = fit
    assert period == 137
    assert fitted == pytest.approx(coefficients, abs=.007)
    assert gain > .99
