"""Predict the applied-anchor error of the shipped gate using the shipped code."""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
ROOT = Path("/home/torbjorn/repos/hp-sim5-worktrees/improve-encoder-based-cdpr-autocalibrati-af9af5fc")
sys.path.insert(0, str(ROOT))
from autocal.stage_artifacts import read_stage_artifact
from autocal.history_selection import (
    evaluate_history_candidates, rank_history_candidates,
    consensus_anchor_estimate, HISTORY_CONSENSUS_MAX_SPREAD_MM,
)
from autocal.tools.regress_calibration_logs import DATASETS, _anchor_distance_common_dims
_SESS = ROOT / ("output/research/sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-"
                "the-curren-35233092/artifacts")
SWEEPS = [_SESS / "stage-sweep2", _SESS / "stage-sweep-slide"]


def err(a, ta):
    return None if a is None else _anchor_distance_common_dims([tuple(map(float, x)) for x in a], ta)


def applied(scored, ranked, gate):
    a, m, s = consensus_anchor_estimate(scored, max_spread_mm=gate)
    if a is not None:
        return err(a, None) if False else a, m, s, "consensus"
    return ranked[0][1]["plan"]["anchors"], (), s, "best-ranked"


tot_gate = tot_old = tot_best = 0.0
print(f"{'dataset':52s} {'best1':>8s} {'HEAD':>8s} {'gated':>8s} {'gain':>7s} mode")
for spec in DATASETS:
    hr = [h for root in SWEEPS for h in sorted((root / spec.name).glob("stages*/history-*.json"))]
    if not hr:
        print(f"# MISSING {spec.name}")
        continue
    payload = read_stage_artifact(hr[-1], expected_kind="history")["payload"]
    scored = evaluate_history_candidates(payload["candidates"], log_line=lambda _l: None)
    ranked = rank_history_candidates(scored)
    ta = [tuple(a) for a in spec.true_anchors]
    anchor_gate, mem, spread, mode = applied(scored, ranked, HISTORY_CONSENSUS_MAX_SPREAD_MM)
    anchor_old, _, _, _ = applied(scored, ranked, float("inf"))
    e_best = err(ranked[0][1]["plan"]["anchors"], ta)
    e_old = err(anchor_old, ta)
    e_gate = err(anchor_gate, ta)
    tot_gate += e_gate; tot_old += e_old; tot_best += e_best
    print(f"{spec.name:52s} {e_best:8.2f} {e_old:8.2f} {e_gate:8.2f} {e_old-e_gate:7.2f} {mode} spread={spread:.3f}")
print(f"\nSUM best-ranked-only={tot_best:.3f}  HEAD(consensus always)={tot_old:.3f}  gated={tot_gate:.3f}")
print(f"gate={HISTORY_CONSENSUS_MAX_SPREAD_MM:g} mm")
