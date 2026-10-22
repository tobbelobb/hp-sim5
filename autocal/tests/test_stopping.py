from pathlib import Path

import autocal.stopping as stopping


def _candidate(iteration, score):
    return {"iteration": iteration, "rank_score": score, "cost": score, "rel_std": 1.0}


def test_patience_only_advances_for_admissible_history_and_resets_on_improvement(tmp_path):
    policy = stopping.StoppingPolicy(patience=2, stop_file=tmp_path / "stop", spread_fraction=0)
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


def test_spread_requires_consecutive_hits_and_underconstraint_overrides_acceptance(tmp_path, monkeypatch):
    policy = stopping.StoppingPolicy(patience=20, stop_file=tmp_path / "stop", spread_fraction=.0015)
    monkeypatch.setattr(stopping, "consensus_anchor_estimate", lambda _: ([], (1, 2), 1.0))
    hits = iter([True, False, True, True, True])
    monkeypatch.setattr(stopping, "consensus_spread_hit", lambda *_, **__: (next(hits), 1.0, 5.0))
    history = [_candidate(i, i) for i in range(1, 5)]
    decisions = [policy.update(history, iteration=4, admissible=False,
                               underconstrained=False, log_line=lambda _: None)
                 for _ in range(4)]
    assert [d.decision for d in decisions] == ["collect", "collect", "collect", "accept"]
    assert decisions[-1].reason == "spread-consensus"
    invalid = policy.update(history, iteration=5, admissible=False,
                            underconstrained=True, log_line=lambda _: None)
    assert invalid.spread_hit and invalid.decision == "collect" and invalid.reason is None


def test_stop_file_requests_acceptance_and_io_errors_do_not(tmp_path, monkeypatch):
    path = tmp_path / "stop"
    policy = stopping.StoppingPolicy(patience=0, stop_file=path, spread_fraction=0)
    assert policy.patience == 1
    assert not policy.stop_requested()
    path.touch()
    assert policy.stop_requested()

    def denied(_):
        raise OSError("unavailable")

    monkeypatch.setattr(Path, "exists", denied)
    assert not policy.stop_requested()
