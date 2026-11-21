"""Five-anchor geometry, gauge, acquisition and fit regressions."""
import numpy as np
import pytest

from autocal.active_learning import generate_candidate_sweeps
from autocal.pose_fit import dataset_line_lengths, fit_pose_least_squares, shift_dataset_home_offsets
from autocal.pose_planning import ANCHOR_CLEARANCE_SHARE, predict_sweep_positions, SWEEP_PRETENSION_SHARE
from autocal.sweep_config_generator import generate_sweep_configs, select_representative_configs
from autocal.sweep_types import MachineConfig, MachineType
from autocal.theoretical_ellipse import (
    anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec, get_anchor_opt_bounds,
)

ANCHORS = np.array([[0., -1900., -280.], [1900., 0., -280.], [0., 1900., -280.],
                    [-1900., 0., -280.], [0., 0., 1900.]])


def test_five_anchor_plans_avoid_the_forward_solvers_collision_regime():
    candidates = generate_candidate_sweeps(num_anchors=5, dimensions=3, machine_type='hangprinter_5',
        fixed_delta_values_mm=[-1600., -1200., -800., -400., 0.])
    positions = [predict_sweep_positions(ANCHORS, cfg, 20, SWEEP_PRETENSION_SHARE) for cfg in candidates]
    positions = np.concatenate([points for points in positions if len(points)])
    assert len(positions) > 0
    distances = np.linalg.norm(ANCHORS[None,:,:] - positions[:,None,:], axis=2)
    assert np.all(distances >= ANCHOR_CLEARANCE_SHARE*np.linalg.norm(ANCHORS, axis=1))


@pytest.mark.parametrize('machine', ['hangprinter_5', 'hangprinter_5_pulley_reduced'])
def test_bootstrap_and_planner_keep_the_top_fixed_and_opposing_held_low_compliant(machine):
    config = MachineConfig.from_type(MachineType(machine))
    sweeps = generate_sweep_configs(config)
    selected = select_representative_configs(sweeps, config, max_sweeps=3)
    assert len({tuple(s['fixed_anchors']) for s in selected}) == 3
    for sweep in sweeps:
        assert 4 in sweep['fixed_anchors']
        low = next(axis for axis in sweep['fixed_anchors'] if axis != 4)
        assert {sweep['drive_anchor'], sweep['sensor_anchor']} == {(low+1) % 4, (low+3) % 4}
    candidates = generate_candidate_sweeps(num_anchors=5, dimensions=3, machine_type=machine,
                                           fixed_delta_values_mm=[-200., 0., 200.])
    assert candidates
    assert all(cfg.fixed_deltas_mm[cfg.fixed_anchors.index(4)] <= 0 for cfg in candidates)
    assert all(config.sweep_pair_allowed(cfg.fixed_anchors, cfg.drive_anchor, cfg.sensor_anchor) for cfg in candidates)


@pytest.mark.parametrize('machine', ['hangprinter_5', 'hangprinter_5_pulley_reduced'])
def test_numpy_and_jax_reduced_anchor_layouts_agree(machine):
    from autocal.slideprinter_2d.ellipse_objective_jax import _anchors_from_opt_vec
    from autocal.theoretical_ellipse import anchor_opt_constraint_mode
    import jax.numpy as jnp
    uneven = ANCHORS + [[0., 0., 3.], [0., 0., -2.], [0., 0., 5.], [0., 0., 0.], [0., 0., 0.]]
    vector = anchors_matrix_to_opt_vec(uneven, machine)
    assert len(vector) == 12
    lb, ub = get_anchor_opt_bounds(machine, 5, 3)
    restored = anchor_opt_vec_to_matrix(vector, machine, 5, 3)
    np.testing.assert_allclose(restored, uneven, atol=1e-10)
    compiled = _anchors_from_opt_vec(jnp.asarray(vector), jnp.asarray(lb), jnp.asarray(ub), num_anchors=5,
        dimensions=3, constraint_mode=anchor_opt_constraint_mode(machine, 5, 3))
    np.testing.assert_allclose(compiled, restored, atol=1e-8)


@pytest.mark.parametrize('machine', ['hangprinter_5', 'hangprinter_5_pulley_reduced'])
def test_measured_lengths_recover_all_five_anchors(machine):
    rng = np.random.default_rng(5)
    sweeps = []
    for low in range(4):
        for position in rng.uniform([-500., -500., 0.], [500., 500., 800.], (15, 3)):
            length = np.linalg.norm(ANCHORS-position, axis=1) - np.linalg.norm(ANCHORS, axis=1)
            drive, sensor = (low+1) % 4, (low+3) % 4
            sweeps.append(dict(fixed_anchors=[low, 4], fixed_lengths=length[[low, 4]].tolist(),
                drive_anchor=drive, sensor_anchor=sensor,
                data_points=[dict(l_drive=length[drive], l_sensor=length[sensor], raw_angles_deg=length.tolist())]))
    dataset = dict(machine_type=machine, num_anchors=5, dimensions=3,
                   config={'mm_per_degree': [1.]*5}, sweeps=sweeps)
    measured = dataset_line_lengths(dataset)['lengths_mm']
    shifted = dataset_line_lengths(shift_dataset_home_offsets(dataset, [1., 2., 3., 4., 5.]))['lengths_mm']
    mask = np.isfinite(measured)
    assert np.isnan(measured).sum() == 60
    np.testing.assert_allclose((shifted-measured)[mask], np.tile([1., 2., 3., 4., 5.], (60, 1))[mask])
    seed = ANCHORS * 1.08 + [0., 0., 40.]
    fit = fit_pose_least_squares(dataset, seed, fit_home_offsets=False, home_offset_prior_sigma_mm=5.)
    assert fit['success']
    assert np.max(np.linalg.norm(fit['anchors']-ANCHORS, axis=1)) < .001
