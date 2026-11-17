from pathlib import Path

import numpy as np

import autocal.stopping as stopping


def _candidate(iteration, score):
    return {"iteration": iteration, "rank_score": score, "cost": score, "rel_std": 1.0}


def test_patience_only_advances_for_admissible_history_and_resets_on_improvement(tmp_path):
    policy = stopping.StoppingPolicy(patience=2, stop_file=tmp_path / "stop")
    history = [_candidate(1, 1.0)]
    first = policy.update(history, iteration=1, admissible=True,
                          underconstrained=False, log_line=lambda _: None)
    assert first.history_improved and first.rank == 1
    assert policy.remaining == 2

    history.append(_candidate(2, 100.0))
    second = policy.update(history, iteration=2, admissible=True,
                           underconstrained=False, log_line=lambda _: None)
    assert second.decision == "collect" and policy.remaining == 1
    invalid = policy.update(history, iteration=3, admissible=False,
                            underconstrained=True, log_line=lambda _: None)
    assert invalid.rank is None and invalid.reason is None
    assert policy.remaining == 1

    history.append(_candidate(4, 200.0))
    exhausted = policy.update(history, iteration=4, admissible=True,
                              underconstrained=False, log_line=lambda _: None)
    assert exhausted.decision == "accept" and exhausted.reason == "patience"
    assert policy.remaining == 0

    history.append(_candidate(5, -100.0))
    improved = policy.update(history, iteration=5, admissible=True,
                             underconstrained=False, log_line=lambda _: None)
    assert improved.history_improved and improved.decision == "collect"
    assert policy.remaining == 2


def _fit(iteration, offset_mm):
    anchors = np.array([[0.0, -1900.0, -280.0], [1645.4, 950.0, -280.0],
                        [-1645.4, 950.0, -280.0], [0.0, 0.0, 1900.0]]) + offset_mm
    return {**_candidate(iteration, 1.0 + iteration), "plan": {"anchors": anchors}}


def test_two_consecutive_small_steps_stop_and_a_large_step_resets(tmp_path):
    # Learning curve of a recorded HP3 run: 2.16 mm, then sub-0.3 mm steps.
    policy = stopping.StoppingPolicy(patience=20, stop_file=tmp_path / "stop")
    history, decisions = [], []
    for iteration, offset in enumerate([0.0, 2.16, 2.33, 2.05, 2.29], start=1):
        history.append(_fit(iteration, offset))
        decisions.append(policy.update(history, iteration=iteration, admissible=True,
                                       underconstrained=False, log_line=lambda _: None))
    assert [d.decision for d in decisions] == ["collect", "collect", "collect", "accept", "accept"]
    assert decisions[3].reason == "stable-estimate" and np.isclose(decisions[3].step_mm, 0.28)
    history.append(_fit(6, 5.0))
    reset = policy.update(history, iteration=6, admissible=True, underconstrained=False, log_line=lambda _: None)
    assert reset.decision == "collect" and policy.stable_streak == 0


def test_stability_ignores_inadmissible_fits_and_underconstraint_and_can_be_disabled(tmp_path):
    policy = stopping.StoppingPolicy(patience=20, stop_file=tmp_path / "stop")
    history = [_fit(1, 0.0), _fit(2, 0.1), _fit(3, 0.2)]
    for iteration in (1, 2):
        policy.update(history, iteration=iteration, admissible=True, underconstrained=False, log_line=lambda _: None)
    skipped = policy.update(history, iteration=9, admissible=False, underconstrained=False, log_line=lambda _: None)
    assert skipped.step_mm is None and policy.stable_streak == 1
    held = policy.update(history, iteration=3, admissible=True, underconstrained=True, log_line=lambda _: None)
    assert held.stable_hit and held.decision == "collect"
    disabled = stopping.StoppingPolicy(patience=20, stop_file=tmp_path / "stop", stable_step_mm=0.0)
    for iteration in (1, 2, 3):
        result = disabled.update(history, iteration=iteration, admissible=True,
                                 underconstrained=False, log_line=lambda _: None)
    assert result.decision == "collect" and not result.stable_hit


def test_stop_file_requests_acceptance_and_io_errors_do_not(tmp_path, monkeypatch):
    path = tmp_path / "stop"
    policy = stopping.StoppingPolicy(patience=0, stop_file=path)
    assert policy.patience == 1
    assert not policy.stop_requested()
    path.touch()
    assert policy.stop_requested()

    def denied(_):
        raise OSError("unavailable")

    monkeypatch.setattr(Path, "exists", denied)
    assert not policy.stop_requested()


def test_one_coordinate_beyond_the_limit_resets_stability(tmp_path):
    # Uniform shifts would hide a mean- or norm-based step; the rule is per coordinate.
    policy = stopping.StoppingPolicy(patience=20, stop_file=tmp_path / "stop")
    base = _fit(1, 0.0)["plan"]["anchors"]
    history = []

    def fit_with_d_z(iteration, d_z):
        anchors = base.copy()
        anchors[3, 2] += d_z
        history.append({**_candidate(iteration, 1.0 + iteration), "plan": {"anchors": anchors}})
        return policy.update(history, iteration=iteration, admissible=True,
                             underconstrained=False, log_line=lambda _: None)

    fit_with_d_z(1, 0.0)
    assert fit_with_d_z(2, 0.5).decision == "collect" and policy.stable_streak == 1  # exactly at the limit
    assert fit_with_d_z(3, 1.01).decision == "collect" and policy.stable_streak == 0  # 0.51 mm in one coordinate
    assert fit_with_d_z(4, 1.3).decision == "collect" and policy.stable_streak == 1
    assert fit_with_d_z(5, 1.1).decision == "accept"


def _hp5_fit(iteration, offset, *, std=.4, rms=.03, sweeps=None):
    candidate = _fit(iteration, offset)
    anchors = np.vstack([candidate['plan']['anchors'], [0.,0.,1900.]])
    count = sweeps if sweeps is not None else iteration+3
    candidate.update(max_std_mm=std,plan={'anchors':anchors,'machine_type':'hangprinter_5',
        'info_rank_deficient':False,'pose_fit':{'rms_mm':rms,'stretch_corrected':True},
        'dataset':{'sweeps':[{'data_points':[{'extra_sensor_ranges':[{'accepted':True}]}]} for _ in range(count)]}})
    return candidate


def test_hp5_two_good_fits_stop_without_waiting_for_three_half_mm_steps(tmp_path):
    policy=stopping.StoppingPolicy(patience=20,stop_file=tmp_path/'stop')
    history=[]
    for iteration,offset in enumerate([0.,1.2],1):
        history.append(_hp5_fit(iteration,offset))
        result=policy.update(history,iteration=iteration,admissible=True,underconstrained=False,log_line=lambda _:None)
    assert result.decision=='accept' and result.reason=='good-fit'
    assert np.isclose(result.step_mm,1.2) and not result.stable_hit


def test_hp5_quality_checks_reset_and_repeated_data_do_not_count(tmp_path):
    policy=stopping.StoppingPolicy(patience=20,stop_file=tmp_path/'stop')
    history=[]
    for iteration,options in enumerate([{}, {'std':5.}, {'rms':.2}, {}, {'sweeps':7}, {}],1):
        history.append(_hp5_fit(iteration,.8*iteration,**options))
        result=policy.update(history,iteration=iteration,admissible=True,underconstrained=False,log_line=lambda _:None)
        if iteration<6:assert result.decision=='collect'
    assert result.reason=='good-fit'


def test_hp5_small_steps_with_weak_geometry_do_not_trigger_stable_acceptance(tmp_path):
    policy=stopping.StoppingPolicy(patience=20,stop_file=tmp_path/'stop')
    history=[]
    for iteration in range(1,4):
        history.append(_hp5_fit(iteration,.1*iteration,std=5.))
        result=policy.update(history,iteration=iteration,admissible=True,underconstrained=False,log_line=lambda _:None)
    assert result.decision=='collect'


def test_hp5_weak_flat_fits_do_not_count_towards_two_good_fits(tmp_path):
    policy=stopping.StoppingPolicy(patience=20,stop_file=tmp_path/'stop')
    history=[]
    for iteration,std in enumerate([5.,5.,.4,.4],1):
        history.append(_hp5_fit(iteration,0.,std=std))
        result=policy.update(history,iteration=iteration,admissible=True,underconstrained=False,log_line=lambda _:None)
        if iteration<4:assert result.decision=='collect'
    assert result.reason=='good-fit'
