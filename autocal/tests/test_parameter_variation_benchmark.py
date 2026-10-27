"""Validate the analytical data contract without spending numerical fit budgets."""
import copy
import json

import numpy as np
import pytest

from autocal.tools import benchmark_parameter_variation as pv


def test_exact_world_satisfies_unseen_encoder_geometry_without_truth_in_observations():
    spec = dict(id='exact', split='development', seed=3, scale=1.1, noise_mm=0.)
    data, truth = pv.generate_world(spec)
    assert set(data) == {'version', 'machine_type', 'num_anchors', 'dimensions', 'sweeps', 'config'}
    assert 'anchors_mm' not in json.dumps(data)
    assert 'axial_stiffness_n' not in json.dumps(data)
    assert 'seed' not in json.dumps(data)
    result = pv.prediction(data, np.asarray(truth['anchors_mm']))
    assert result['valid_points'] == result['expected_points'] == 240
    assert result['rms_mm'] < 1e-7
    for sweep in data['sweeps']:
        for point in sweep['data_points']:
            angles = np.asarray(point['raw_angles_deg'])
            assert angles[sweep['drive_anchor']] * (30*np.pi/180) == pytest.approx(point['l_drive'])
            assert angles[sweep['sensor_anchor']] * (30*np.pi/180) == pytest.approx(point['l_sensor'])
            np.testing.assert_allclose(angles[sweep['fixed_anchors']]*(30*np.pi/180), sweep['fixed_lengths'])


def test_worlds_are_reproducible_and_development_holdout_seeds_do_not_overlap():
    worlds = pv.default_worlds()
    development = {w['seed'] for w in worlds if w['split']=='development'}
    holdout = {w['seed'] for w in worlds if w['split']=='holdout'}
    assert development.isdisjoint(holdout)
    assert len(worlds) == len({w['id'] for w in worlds}) == 8
    for spec in worlds:
        first = pv.generate_world(spec)
        assert pv.digest(first) == pv.digest(pv.generate_world(spec))
        np.testing.assert_array_equal(pv.BASE_ANCHORS[0], [0., -1900., -280.])


def test_thinning_keeps_exact_samples_and_both_direction_endpoints():
    data, _ = pv.generate_world(pv.default_worlds()[2])
    before = copy.deepcopy(data)
    half = pv.thin_dataset(data)
    assert data == before
    for full, sparse in zip(data['sweeps'], half['sweeps']):
        assert len(sparse['data_points']) == 10
        assert sparse['data_points'][0] == full['data_points'][0]
        assert sparse['data_points'][4] == full['data_points'][9]
        assert sparse['data_points'][5] == full['data_points'][10]
        assert sparse['data_points'][9] == full['data_points'][19]
        assert all(point in full['data_points'] for point in sparse['data_points'])


def test_model_error_is_visible_at_truth_and_not_native_friction():
    clean = dict(id='clean',split='holdout',seed=0,noise_mm=0.)
    data, truth = pv.generate_world(clean)
    assert pv.prediction(data,np.asarray(truth['anchors_mm']))['rms_mm'] < 1e-7
    for changes in (dict(reversal_offset_mm=.6), dict(axial_stiffness_n=8000.,tension_span_n=8.)):
        altered, altered_truth = pv.generate_world(dict(clean,**changes))
        np.testing.assert_array_equal(truth['anchors_mm'],altered_truth['anchors_mm'])
        assert pv.prediction(altered,np.asarray(truth['anchors_mm']))['rms_mm'] > .01
    assert 'No force equilibrium' in pv.LIMITATION


def test_reject_impossible_stiffness_or_too_few_points():
    with pytest.raises(ValueError,match='stiffness'):
        pv.generate_world(dict(seed=0,axial_stiffness_n=10.))
    with pytest.raises(ValueError,match='five points'):
        pv.generate_world(dict(seed=0),points_per_direction=3)


def test_explicit_fit_cap_prevents_solver_calls(tmp_path, monkeypatch):
    def forbidden(*args,**kwargs):
        raise AssertionError('No solver may run beyond the fit cap')
    monkeypatch.setattr(pv,'solve_anchors',forbidden)
    with pytest.raises(ValueError,match='exceed'):
        pv.run(tmp_path/'screen',max_fits=31)


def test_failed_fits_are_retained_and_truth_never_enters_solver_inputs(tmp_path, monkeypatch):
    seen = []
    seed = np.ones((4,3))*123.
    monkeypatch.setattr(pv,'build_anchor_initial_guess',lambda data,**kwargs:seed.copy())
    def fail(data, *, initial_guess, **kwargs):
        assert 'anchors_mm' not in json.dumps(data)
        np.testing.assert_array_equal(initial_guess,seed)
        assert len(data['sweeps']) == 9
        seen.append(kwargs['method'])
        raise ValueError('deliberate failed fit')
    monkeypatch.setattr(pv,'solve_anchors',fail)
    result = pv.run(tmp_path/'failed',worlds=[pv.default_worlds()[0]],max_fits=4)
    assert seen == ['lbfgsb','trf','lbfgsb','trf']
    assert len(result['rows']) == 4
    assert all(row['error']=='ValueError: deliberate failed fit' for row in result['rows'])
    assert all(not row['complete_coverage'] for row in result['rows'])
    assert not any(comparison['accepted'] for comparison in result['summary']['comparisons'])


def test_nonfinite_fit_is_retained_as_failure(tmp_path, monkeypatch):
    monkeypatch.setattr(pv,'solve_anchors',lambda *args,**kwargs:dict(anchors=np.full((4,3),np.nan),cost=0.))
    result = pv.run(tmp_path/'nan',worlds=[pv.default_worlds()[0]],max_fits=4)
    assert len(result['rows']) == 4
    assert all('Nonfinite' in row['error'] for row in result['rows'])
    json.dumps(result,allow_nan=False)


def test_soft_wall_budget_stops_before_next_fit_and_preserves_partial(tmp_path,monkeypatch):
    calls=[]
    def slow(*args,**kwargs):
        calls.append(True)
        raise ValueError('first failure')
    monkeypatch.setattr(pv,'solve_anchors',slow)
    result=pv.run(tmp_path/'budget',worlds=[pv.default_worlds()[0]],max_fits=4,max_wall_seconds=1e-12)
    assert calls == []
    assert result['complete'] is False
    assert result['stop_reason']=='wall_budget_before_next_fit'
    assert (tmp_path/'budget/results.json').exists()


def test_coverage_requires_status_and_all_sweeps_not_only_zero_invalid():
    row=dict(cost=.1,anchor_rmse_mm=2.,optimizer_success=True,anchors_mm=np.ones((4,3)).tolist(),
             invalid_sweeps=0,valid_sweeps=9,
             unseen_prediction=dict(rms_mm=.2,expected_points=60,valid_points=60))
    assert pv.row_is_complete(row)
    for changed in (dict(valid_sweeps=8),dict(optimizer_success=False),dict(cost=float('inf')),
                    dict(unseen_prediction=dict(rms_mm=.2,expected_points=60,valid_points=59))):
        assert not pv.row_is_complete(dict(row,**changed))


@pytest.mark.parametrize('worlds',[ [dict(id='../escape',split='holdout')],
                                  [dict(id='same',split='holdout'),dict(id='same',split='development')] ])
def test_invalid_world_paths_and_duplicate_ids_do_not_create_output(tmp_path,worlds):
    with pytest.raises(ValueError,match='World IDs'):
        pv.run(tmp_path/'unsafe',worlds=worlds)
    assert not (tmp_path/'unsafe').exists()


def test_cli_distinguishes_complete_and_soft_wall_stop(tmp_path,monkeypatch):
    for complete, expected in [(True,0),(False,1)]:
        monkeypatch.setattr(pv,'run',lambda *args,**kwargs:dict(complete=complete))
        assert pv.main(['--output',str(tmp_path/'screen')]) == expected
