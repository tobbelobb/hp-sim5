"""One frozen final-selection comparison, with actual applied-model diagnostics."""
from __future__ import annotations

import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[4]
SESSION = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

import numpy as np

from autocal._autocal_common import _arg_has_flag
from autocal.history_selection import (
    apply_consensus_plan, consensus_anchor_estimate, select_history_plan,
)
from autocal.predictive_validation import score_future_sweeps
from autocal.stage_artifacts import _encode, read_stage_artifact


def sha(data):
    return hashlib.sha256(data).hexdigest()


def encoded(value):
    return json.dumps(_encode(value), sort_keys=True, allow_nan=False)


def error(anchors, truth):
    anchors = np.asarray(anchors, dtype=float)
    return float(np.linalg.norm(anchors - np.asarray(truth)[:, :anchors.shape[1]], axis=1).sum())


prediction_calls = 0


def predict(dataset, anchors, training, spool, settings, options):
    global prediction_calls
    prediction_calls += 1
    result = score_future_sweeps(
        dataset, anchors, training, spool_params=spool,
        prefer_zero_tension_angles=_arg_has_flag(options.get("collector_args", ()), "--project-zero-tension"),
        use_noise_mean=settings.get("use_noise_mean", options.get("use_noise_mean", True)),
        pointwise_residual_mode=settings.get("pointwise_residual_mode", options.get("pointwise_residual_mode", "sampson")),
        sigma_source=settings.get("sigma_source", options.get("sigma_source", "auto")),
    )
    if result is None:
        return {"status": "no_unseen_sweeps", "score": None, "sweeps": 0}
    finite = bool(np.isfinite(result[0]))
    return {"status": "finite" if finite else "failed", "score": float(result[0]) if finite else None,
            "sweeps": result[1]}


started = time.perf_counter()
manifest_path = SESSION / "inputs.json"
manifest = json.loads(manifest_path.read_text())
assert len(manifest) <= 11
rows = []
with tempfile.TemporaryDirectory(prefix="consensus-membership-") as temporary:
    for entry in manifest:
        compressed = (SESSION / entry["input"]).read_bytes()
        assert sha(compressed) == entry["gzip_sha256"]
        raw = gzip.decompress(compressed)
        assert sha(raw) == entry["sha256"]
        decoded_path = Path(temporary) / "history.json"
        decoded_path.write_bytes(raw)
        artifact = read_stage_artifact(decoded_path, expected_kind="history")
        assert artifact["source_revision"] == entry["producer_source_revision"]
        payload = artifact["payload"]
        scored = payload["evaluated"]
        before = encoded(payload)
        candidates = {candidate["iteration"]: candidate for candidate in payload["candidates"]}
        for _key, candidate, _info in scored:
            assert encoded(candidate) == encoded(candidates[candidate["iteration"]])

        baseline, meta, ranked = select_history_plan(scored)
        assert baseline is not None and ranked
        chosen = ranked[0][1]
        eligible = [item for item in scored if item[2].get("prediction_score") is not None
                    and np.isfinite(item[2]["prediction_score"])]
        # Preserve the existing no-heldout fallback, although every input here
        # has finite predictions. All-failed histories are vetoed above.
        anchors, members, spread = consensus_anchor_estimate(eligible or scored)
        variant = apply_consensus_plan(chosen["plan"], anchors)
        assert variant["spool_model_params"] == baseline["spool_model_params"] == chosen["plan"]["spool_model_params"]

        dataset = next(item[1]["plan"]["dataset"] for item in reversed(scored)
                       if isinstance(item[1]["plan"].get("dataset"), dict))
        options = payload["validation_options"]
        settings = chosen.get("settings") or {}
        # Old artifacts include now-removed evaluator options. They were off in
        # this corpus; do not silently drop an active flexibility correction.
        assert options.get("use_flex", False) is False
        assert options.get("spring_k_multiplier", 1.0) == 1.0
        chosen_training = set(map(str, chosen["training_sweep_ids"]))
        all_ids = {str(sweep["id"]) for sweep in dataset["sweeps"]}
        chosen_fold = all_ids - chosen_training
        single_prediction = predict(dataset, chosen["plan"]["anchors"], chosen_training,
                                    chosen["plan"]["spool_model_params"], settings, options)
        saved_prediction = ranked[0][2]["prediction_score"]
        score_matches = (single_prediction["status"] == "finite"
                         and bool(np.isclose(single_prediction["score"], saved_prediction, atol=1e-8, rtol=1e-10)))

        arms = {}
        training_by_arm = {}
        for name, plan, member_ids, distance in (
            ("all_candidates", baseline, meta["history_consensus_members"], meta["history_consensus_spread_mm"]),
            ("finite_prediction_only", variant, members, spread),
        ):
            training = set(chosen_training)  # the applied spool fit also matters
            for iteration in member_ids:
                training.update(map(str, candidates[iteration]["training_sweep_ids"]))
            training_by_arm[name] = training
            prediction = predict(dataset, plan["anchors"], chosen_training,
                                 plan["spool_model_params"], settings, options)
            arms[name] = {
                "members": list(member_ids), "spread_mm": distance,
                "sum_anchor_error_mm": error(plan["anchors"], entry["truth_anchors_mm"]),
                "anchors_mm": np.asarray(plan["anchors"]).tolist(),
                "applied_spool_model": _encode(plan["spool_model_params"]),
                "chosen_fit_future_fold": prediction,
                "future_fold_training_overlap_ids": sorted(chosen_fold & training),
            }

        common_training = set.union(*training_by_arm.values())
        common_ids = all_ids - common_training
        for name, plan in (("all_candidates", baseline), ("finite_prediction_only", variant)):
            arms[name]["common_unseen_fold"] = (
                predict(dataset, plan["anchors"], common_training, plan["spool_model_params"], settings, options)
                if common_ids else {"status": "no_unseen_sweeps", "score": None, "sweeps": 0}
            )
        assert encoded(payload) == before, "Probe mutated a frozen candidate"
        current_sources = {name: sha((ROOT / name).read_bytes()) for name in artifact["source_sha256"]
                           if (ROOT / name).is_file()}
        row = {
            "fixture": entry["fixture"], "cohort": entry["cohort"], "input": entry["input"],
            "input_sha256": entry["sha256"], "producer_source_revision": artifact["source_revision"],
            "producer_source_sha256": artifact["source_sha256"],
            "changed_source_files": [name for name, value in current_sources.items()
                                     if artifact["source_sha256"][name] != value],
            "single_fit_iteration": chosen["iteration"],
            "single_fit_sum_anchor_error_mm": error(chosen["plan"]["anchors"], entry["truth_anchors_mm"]),
            "single_fit_saved_prediction": saved_prediction,
            "single_fit_recomputed_prediction": single_prediction,
            "single_fit_prediction_matches_saved": score_matches,
            "validation_options": options, "chosen_settings": settings,
            "candidate_iterations": [item[1]["iteration"] for item in scored],
            "eligible_iterations": [item[1]["iteration"] for item in eligible],
            "failed_prediction_iterations": [item[1]["iteration"] for item in scored
                                              if item[2].get("prediction_score") is not None
                                              and not np.isfinite(item[2]["prediction_score"])],
            "chosen_fit_future_fold_ids": sorted(chosen_fold), "common_unseen_fold_ids": sorted(common_ids),
            **arms,
            "delta_sum_anchor_error_mm": arms["finite_prediction_only"]["sum_anchor_error_mm"]
                                         - arms["all_candidates"]["sum_anchor_error_mm"],
        }
        rows.append(row)
        print(json.dumps({"fixture": row["fixture"], "delta_mm": row["delta_sum_anchor_error_mm"],
                          "members": [row[name]["members"] for name in arms],
                          "score_compatible": score_matches,
                          "prediction": {name: arms[name]["chosen_fit_future_fold"] for name in arms}}), flush=True)

assert prediction_calls <= 55
summary = {}
for cohort in ("additional_dataset", "motivating_dataset_overlap"):
    subset = [row for row in rows if row["cohort"] == cohort]
    summary[cohort] = {
        "datasets": len(subset),
        "improved": sum(row["delta_sum_anchor_error_mm"] < -1e-8 for row in subset),
        "worsened": sum(row["delta_sum_anchor_error_mm"] > 1e-8 for row in subset),
        "unchanged": sum(abs(row["delta_sum_anchor_error_mm"]) <= 1e-8 for row in subset),
        "baseline_sum_anchor_error_mm": sum(row["all_candidates"]["sum_anchor_error_mm"] for row in subset),
        "variant_sum_anchor_error_mm": sum(row["finite_prediction_only"]["sum_anchor_error_mm"] for row in subset),
        "delta_sum_anchor_error_mm": sum(row["delta_sum_anchor_error_mm"] for row in subset),
    }
result = {
    "source_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
    "command": ".venv/bin/python output/research/sessions/2026-10-23-consensus-membership/probe.py",
    "script_sha256": sha(Path(__file__).read_bytes()), "inputs_manifest_sha256": sha(manifest_path.read_bytes()),
    "current_source_sha256": current_sources,
    "budget_used": {"histories": len(rows), "selection_evaluations": 2 * len(rows),
                    "prediction_evaluations": prediction_calls, "fresh_fits": 0, "physics_steps": 0, "collections": 0},
    "metric": "sum of per-anchor Euclidean distances to fixture simulation truth, mm",
    "scope": "Frozen final selection from saved simulated fits, no stopping/fitting/collection or hardware claim",
    "summary": summary, "rows": rows, "wall_s": time.perf_counter() - started,
}
(SESSION / "results.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
print(json.dumps({"summary": summary, "budget_used": result["budget_used"], "wall_s": result["wall_s"]}, indent=2))
