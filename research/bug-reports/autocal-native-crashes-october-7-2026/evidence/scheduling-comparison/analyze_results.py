"""Compare full-battery scheduling runs and their numerical outputs."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import runpy

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
namespace = runpy.run_path(str(ROOT / "autocal/tools/regress_calibration_logs.py"), run_name="benchmark_analysis")
parse_log = namespace["parse_log_file"]


def numerical_output(row):
    path = Path(row["scratch_dir"]) / (row["dataset"] + ".full_auto.log")
    parsed = parse_log(path)
    if parsed.summary is None or not parsed.iterations:
        raise ValueError(f"Missing complete numerical output: {path}")
    return {"summary": asdict(parsed.summary), "iterations": [
        {k: getattr(item, k) for k in ("anchors", "radii", "fit_score_ui", "rank_score", "history_rank_score")}
        for item in parsed.iterations]}


runs = json.loads((HERE / "summary.json").read_text())
outputs = {}
for run in runs:
    path = Path(run["output_dir"])
    recorded = json.loads((path / "datasets.json").read_text())
    rows = recorded["datasets"]
    run["verified_peak_active"] = recorded["peak_active"]
    run["normal_gc"] = recorded["gc_enabled"]
    run["fixture_copies_match_provenance"] = all(
        row["fixture_sha256"] == json.loads((HERE / "provenance.json").read_text())["fixtures"][row["dataset"] + ".json"]
        for row in rows)
    outputs[run["run"]] = {r["dataset"]: numerical_output(r) for r in rows if r["returncode"] == 0}

comparisons = []
for full, bounded in ((1, 2), (4, 3)):
    by_id = {r["run"]: r for r in runs}
    if full not in by_id or bounded not in by_id:
        continue
    baseline, candidate = by_id[full], by_id[bounded]
    common = sorted(outputs[full].keys() & outputs[bounded].keys())
    complete = len(outputs[full]) == len(outputs[bounded]) == 11
    comparisons.append({"eleven_worker_run": full, "two_worker_run": bounded,
        "completion_timing_comparable": complete,
        "slowdown_ratio": candidate["elapsed_s"] / baseline["elapsed_s"] if complete else None,
        "slower_percent": (candidate["elapsed_s"] / baseline["elapsed_s"] - 1) * 100 if complete else None,
        "compared_datasets": len(common),
        "identical_numerical_outputs": [name for name in common if outputs[full][name] == outputs[bounded][name]],
        "different_numerical_outputs": [name for name in common if outputs[full][name] != outputs[bounded][name]]})

result = {"runs": runs, "pairs": comparisons,
          "all_completed_runs_identical_to_first": {
              str(run): outputs[run] == outputs[1] for run in outputs}}
(HERE / "analysis.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
