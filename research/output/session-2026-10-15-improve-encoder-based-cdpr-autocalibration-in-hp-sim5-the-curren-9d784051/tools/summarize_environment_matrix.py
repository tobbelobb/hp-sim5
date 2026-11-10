#!/usr/bin/env python3
"""Rebuild candidate-iteration metrics from saved typed stage artifacts."""
import gzip
import json
import math
from pathlib import Path
import re
import statistics
import sys


SESSION = Path(__file__).resolve().parents[1]
ARTIFACTS = SESSION / "environment-matrix-20261015-retry1"
RAW = Path("/tmp/hp-sim5-autocal-env-matrix-20261015-retry1")
TRUTH = [(0.0, -1900.0, -280.0), (1645.44826719, 950.0, -280.0),
         (-1645.44826719, 950.0, -280.0), (0.0, 0.0, 1900.0)]


def rmse(anchors):
    if isinstance(anchors, dict) and anchors.get("$type") == "array":
        anchors = anchors.get("value")
    if not isinstance(anchors, list) or len(anchors) != 4:
        return None
    return math.sqrt(sum(math.dist([float(x) for x in got[:3]], truth) ** 2
                         for got, truth in zip(anchors, TRUTH)) / 4)


def summarize(world_id, solver):
    stage_dir = RAW / world_id / f"fit_{solver}" / "stages"
    candidates = []
    for path in sorted(stage_dir.glob("history-*.json")):
        document = json.loads(path.read_text())
        for candidate in document.get("payload", {}).get("candidates", []):
            error = rmse((candidate.get("plan") or {}).get("anchors"))
            if error is not None:
                candidates.append({"iteration": candidate.get("iteration"),
                                   "run_id": candidate.get("run_id"),
                                   "anchor_rmse_mm": error,
                                   "score_ui": candidate.get("score_ui"),
                                   "history_rank_score": candidate.get("history_rank_score")})
    if not candidates:
        raise RuntimeError(f"No anchor candidates decoded for {world_id} {solver}")
    errors = [row["anchor_rmse_mm"] for row in candidates]
    compressed_path = ARTIFACTS / world_id / f"{solver}_summary.json.gz"
    with gzip.open(compressed_path, "rt", encoding="utf8") as stream:
        summary = json.load(stream)
    summary.update(candidate_count=len(candidates),
                   candidate_mean_rmse_mm=statistics.mean(errors),
                   candidate_median_rmse_mm=statistics.median(errors),
                   candidate_max_rmse_mm=max(errors),
                   candidate_over_20mm=sum(e > 20 for e in errors),
                   candidate_over_100mm=sum(e > 100 for e in errors),
                   candidate_over_500mm=sum(e > 500 for e in errors),
                   iteration_max_rmse_mm={
                       str(i): max(row["anchor_rmse_mm"] for row in candidates if row["iteration"] == i)
                       for i in sorted({row["iteration"] for row in candidates})},
                   candidates=candidates)
    with gzip.open(compressed_path, "wt", encoding="utf8", compresslevel=9) as stream:
        json.dump(summary, stream, separators=(",", ":"))
    return summary


def main():
    all_summaries = {world: {solver: summarize(world, solver)
                             for solver in ("lbfgsb", "trf")}
                     for world in [f"W{i:02d}-" + name for i, name in enumerate((
                         "nominal", "light", "heavy", "low-gravity", "high-gravity",
                         "light-low-g", "light-high-g", "heavy-low-g", "heavy-high-g",
                         "heavy-high-load"))]}
    previous = json.loads((ARTIFACTS / "matrix_summary.json").read_text())
    for row in previous["worlds"]:
        for solver in ("lbfgsb", "trf"):
            summary = all_summaries[row["world_id"]][solver]
            row["solvers"][solver].update({k: summary[k] for k in (
                "candidate_count", "candidate_mean_rmse_mm", "candidate_median_rmse_mm",
                "candidate_max_rmse_mm", "candidate_over_20mm", "candidate_over_100mm",
                "candidate_over_500mm", "iteration_max_rmse_mm")})
    aggregate = {}
    for solver in ("lbfgsb", "trf"):
        fits = [all_summaries[row["world_id"]][solver] for row in previous["worlds"]]
        candidate_errors = [candidate["anchor_rmse_mm"] for fit in fits for candidate in fit["candidates"]]
        applied_errors = [fit["applied_anchor_rmse_mm"] for fit in fits if fit["applied_anchor_rmse_mm"] is not None]
        aggregate[solver] = {
            "worlds": len(fits),
            "candidate_count": len(candidate_errors),
            "candidate_mean_rmse_mm": statistics.mean(candidate_errors),
            "candidate_median_rmse_mm": statistics.median(candidate_errors),
            "candidate_max_rmse_mm": max(candidate_errors),
            "candidate_over_20mm": sum(e > 20 for e in candidate_errors),
            "candidate_over_100mm": sum(e > 100 for e in candidate_errors),
            "candidate_over_500mm": sum(e > 500 for e in candidate_errors),
            "applied_mean_rmse_mm": statistics.mean(applied_errors),
            "applied_max_rmse_mm": max(applied_errors),
        }
    paired = {
        "applied_trf_wins": sum(r["solvers"]["trf"]["applied_anchor_rmse_mm"] < r["solvers"]["lbfgsb"]["applied_anchor_rmse_mm"] for r in previous["worlds"]),
        "candidate_mean_trf_wins": sum(r["solvers"]["trf"]["candidate_mean_rmse_mm"] < r["solvers"]["lbfgsb"]["candidate_mean_rmse_mm"] for r in previous["worlds"]),
        "world_mean_candidate_delta_trf_minus_lbfgsb_mm": statistics.mean(
            r["solvers"]["trf"]["candidate_mean_rmse_mm"] - r["solvers"]["lbfgsb"]["candidate_mean_rmse_mm"] for r in previous["worlds"]),
        "world_applied_delta_trf_minus_lbfgsb_mm": statistics.mean(
            r["solvers"]["trf"]["applied_anchor_rmse_mm"] - r["solvers"]["lbfgsb"]["applied_anchor_rmse_mm"] for r in previous["worlds"]),
    }
    final = {"design": previous["design"], "git_revision": previous["git_revision"],
             "aggregate": aggregate, "paired": paired, "worlds": previous["worlds"]}
    (ARTIFACTS / "matrix_summary.json").write_text(json.dumps(final, indent=2) + "\n")
    print(json.dumps({"aggregate": aggregate, "paired": paired}, indent=2))


if __name__ == "__main__":
    main()
