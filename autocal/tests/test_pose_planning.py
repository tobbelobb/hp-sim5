import numpy as np
import pytest
import autocal.planning_pass as planning

from autocal.active_learning import SweepConfig, generate_candidate_sweeps
from autocal.pose_planning import (ANCHOR_CLEARANCE_SHARE, SWEEP_PRETENSION_SHARE, predict_sweep_positions,
                                   rank_candidates_pose_information, pose_information)

ANCHORS = np.array([[0.0, -1900.0, -280.0], [1645.4, 950.0, -280.0], [-1645.4, 950.0, -280.0], [0.0, 0.0, 1900.0]])
HOME = np.linalg.norm(ANCHORS, axis=1)


def _lengths(configs):
    positions = np.concatenate([predict_sweep_positions(ANCHORS, cfg, 20, SWEEP_PRETENSION_SHARE) for cfg in configs])
    return np.linalg.norm(ANCHORS[None, :, :] - positions[:, None, :], axis=2) - HOME


def test_predicted_positions_keep_the_fixed_lengths_and_stay_between_the_anchors():
    cfg = SweepConfig((0, 3), (390.0, -150.0), 1, 2)
    positions = predict_sweep_positions(ANCHORS, cfg, 20, SWEEP_PRETENSION_SHARE)
    assert positions.shape == (20, 3)
    lengths = np.linalg.norm(ANCHORS[None, :, :] - positions[:, None, :], axis=2) - HOME
    assert np.allclose(lengths[:, 0], 390.0) and np.allclose(lengths[:, 3], -150.0)
    assert np.all(np.abs(positions[:, 0]) < 900.0) and np.all(positions[:, 2] > -280.0)
    # The stretch is symmetric about the plane through A and D, like the recorded sweeps.
    assert np.allclose(positions[0] * [-1.0, 1.0, 1.0], positions[-1], atol=30.0)


@pytest.mark.parametrize('count', [4, 5])
def test_analytic_candidate_information_matches_range_residual_differences(count):
    anchors = (ANCHORS if count == 4 else np.array([[0.,-1900.,-280.], [1900.,0.,-280.],
        [0.,1900.,-280.], [-1900.,0.,-280.], [0.,0.,1900.]]))
    cfg = SweepConfig((0, count-1), (100. if count == 4 else -200., -100.), 1, 2 if count == 4 else 3)
    positions = predict_sweep_positions(anchors, cfg, 20, SWEEP_PRETENSION_SHARE)
    assert len(positions) == 20
    lengths = np.linalg.norm(anchors[None,:,:] - positions[:,None,:], axis=2) - np.linalg.norm(anchors, axis=1)
    if count == 5:
        lengths[:,2] = np.nan
    options = dict(machine_type=f'hangprinter_{count}', low_anchor_z=None, sigma_mm=.02)
    finite_difference = pose_information(anchors, lengths, **options)
    analytic = pose_information(anchors, lengths, positions=positions, **options)
    np.testing.assert_allclose(analytic, finite_difference, rtol=1e-3, atol=1e-3)


def test_unreachable_fixed_lengths_give_no_positions():
    assert predict_sweep_positions(ANCHORS, SweepConfig((0, 3), (-1500.0, -1500.0), 1, 2), 20, SWEEP_PRETENSION_SHARE).shape[0] == 0


def test_sweeps_are_not_planned_within_a_tenth_of_an_anchors_home_distance(monkeypatch):
    # D held 150 mm from its anchor (limit: 190 mm) is otherwise a reachable sweep.
    near_d = SweepConfig((0, 3), (850.0, -1750.0), 1, 2)
    assert predict_sweep_positions(ANCHORS, near_d, 20, SWEEP_PRETENSION_SHARE).shape[0] == 0
    monkeypatch.setattr("autocal.pose_planning.ANCHOR_CLEARANCE_SHARE", 0.0)
    assert predict_sweep_positions(ANCHORS, near_d, 20, SWEEP_PRETENSION_SHARE).shape[0] == 20
    monkeypatch.undo()
    allowed = predict_sweep_positions(ANCHORS, SweepConfig((0, 3), (781.9, -1623.5), 1, 2), 20, SWEEP_PRETENSION_SHARE)
    distances = np.linalg.norm(ANCHORS[None, :, :] - allowed[:, None, :], axis=2)
    assert allowed.shape[0] == 20 and np.all(distances >= ANCHOR_CLEARANCE_SHARE * HOME)


def test_data_on_one_d_sphere_makes_a_shortened_d_line_the_best_next_sweep():
    observed = [SweepConfig((low, 3), (390.0, 0.0), *[a for a in range(3) if a != low]) for low in range(3)]
    candidates = generate_candidate_sweeps(
        num_anchors=4, dimensions=3, fixed_delta_values_mm=[-390.0, -195.0, 0.0, 195.0, 390.0],
        machine_type="hangprinter_3",
    )
    ranked = rank_candidates_pose_information(
        ANCHORS, machine_type="hangprinter_3", low_anchor_z=None, observed_lengths_mm=_lengths(observed),
        candidates=candidates, points_per_sweep=20, sigma_mm=0.05, regularization=1e-6, top_k=3,
    )
    assert len(ranked) == 3
    assert ranked[0][0] >= ranked[1][0] >= ranked[2][0]
    assert all(cfg.fixed_deltas_mm[cfg.fixed_anchors.index(3)] < 0.0 for _, cfg in ranked)


@pytest.mark.parametrize("equal_strain", [False, True])
def test_profiled_data_cannot_silently_collect_a_constant_force_sweep(tmp_path, monkeypatch, equal_strain):
    cfg = SweepConfig((0, 3), (390.0, -150.0), 1, 2)
    dataset = {"machine_type": "hangprinter_4", "num_anchors": 4, "dimensions": 3,
               "config": {}, "sweeps": [{"fixed_anchors": [0, 3], "fixed_lengths": [390., 0.],
                   "drive_anchor": 1, "sensor_anchor": 2, "data_points": [{"l_drive": 0., "l_sensor": 0.}],
                   "metadata": {"sensor_force_profile": True}}]}
    fit = {"dataset": dataset, "dataset_for_estimation": dataset, "anchors": ANCHORS,
           "machine_type": "hangprinter_4", "num_anchors": 4, "dimensions": 3,
           "spool_model_params": None, "sweep_configs_for_info": [], "info": np.eye(9),
           "pose_fit": {"robust_scale_mm": .02},
           "information_settings": {"fd_eps_mm": 1., "regularization": 1e-6},
           "collection_settings": {"find_radii_mode": "off", "find_buildup_mode": "off",
                                   "base_radii": None, "buildup_factor": None}}
    monkeypatch.setattr(planning, "generate_candidate_sweeps", lambda **kw: [cfg])
    monkeypatch.setattr(planning, "rank_candidates_pose_information", lambda *a, **kw: [(1., cfg)])
    plan = planning.plan_ellipse_sweep(fit, tmp_path / "sweeps.json", equal_strain=equal_strain)
    assert plan["collect_command"] is None
    assert plan["collection_error"]


def test_no_candidate_position_violates_any_anchors_clearance():
    candidates = generate_candidate_sweeps(
        num_anchors=4, dimensions=3, machine_type="hangprinter_3",
        fixed_delta_values_mm=np.arange(-1850.0, 1601.0, 50.0).tolist(),
    )
    planned = [predict_sweep_positions(ANCHORS, cfg, 20, SWEEP_PRETENSION_SHARE) for cfg in candidates]
    positions = np.concatenate([p for p in planned if p.shape[0]])
    distances = np.linalg.norm(ANCHORS[None, :, :] - positions[:, None, :], axis=2)
    assert positions.shape[0] > 1000
    assert np.all(distances >= ANCHOR_CLEARANCE_SHARE * HOME - 1e-9)


def test_force_infeasible_best_candidate_uses_the_next_ranked_sweep(tmp_path, monkeypatch):
    from autocal.equal_strain import InfeasibleForceProfileError
    first, second = [SweepConfig((0, 3), (390., z), 1, 2) for z in (-150., -300.)]
    dataset = {'machine_type':'hangprinter_4', 'num_anchors':4, 'dimensions':3,
        'config':{}, 'sweeps':[{'fixed_anchors':[0,3], 'fixed_lengths':[390.,0.],
        'drive_anchor':1, 'sensor_anchor':2, 'data_points':[{'l_drive':0.,'l_sensor':0.}],
        'metadata':{'sensor_force_profile':True}}]}
    fit = {'dataset':dataset, 'dataset_for_estimation':dataset, 'anchors':ANCHORS,
        'machine_type':'hangprinter_4', 'num_anchors':4, 'dimensions':3,
        'spool_model_params':None, 'sweep_configs_for_info':[], 'info':np.eye(9),
        'pose_fit':{'robust_scale_mm':.02},
        'information_settings':{'fd_eps_mm':1.,'regularization':1e-6},
        'collection_settings':{'find_radii_mode':'off','find_buildup_mode':'off','base_radii':None,'buildup_factor':None}}
    monkeypatch.setattr(planning, 'generate_candidate_sweeps', lambda **kw:[first,second])
    def rank(*args, **kwargs):
        assert kwargs['top_k'] == 2
        return [(2.,first),(1.,second)]
    monkeypatch.setattr(planning, 'rank_candidates_pose_information', rank)
    monkeypatch.setattr(planning, 'profile_policy_error', lambda dataset:None)
    calls=[]
    def write(path, dataset, anchors, cfgs):
        calls.append(cfgs[0])
        if cfgs[0] == first:raise InfeasibleForceProfileError('no positive tensions')
        return True
    monkeypatch.setattr(planning, 'write_sensor_force_profile', write)
    plan=planning.plan_ellipse_sweep(fit,tmp_path/'sweeps.json',top_k=1)
    assert plan['best_cfg'] == second and calls == [first,second]
    assert plan['collect_command'] and plan['collection_error'] is None
    assert '-300' in ' '.join(plan['collect_command'])
    def bad_configuration(*args):raise ValueError('invalid mechanical advantage')
    monkeypatch.setattr(planning, 'write_sensor_force_profile', bad_configuration)
    with pytest.raises(ValueError,match='mechanical advantage'):
        planning.plan_ellipse_sweep(fit,tmp_path/'sweeps.json')


def test_forecast_counts_the_information_in_the_fifth_range():
    anchors=np.array([[0.,-1900.,-280.],[1900.,0.,-280.],[0.,1900.,-280.],[-1900.,0.,-280.],[0.,0.,1900.]])
    cfg=SweepConfig((0,4),(-340.,0.),1,3)
    positions=predict_sweep_positions(anchors,cfg,12,SWEEP_PRETENSION_SHARE)
    lengths=np.linalg.norm(anchors[None,:,:]-positions[:,None,:],axis=2)-np.linalg.norm(anchors,axis=1)
    options=dict(machine_type='hangprinter_5',low_anchor_z=None,observed_lengths_mm=lengths,
        candidates=[cfg],points_per_sweep=12,sigma_mm=.02,regularization=1e-6,top_k=1)
    four=rank_candidates_pose_information(anchors,**options)
    five=rank_candidates_pose_information(anchors,measure_extra_ranges=True,**options)
    assert five[0][0]>four[0][0]
