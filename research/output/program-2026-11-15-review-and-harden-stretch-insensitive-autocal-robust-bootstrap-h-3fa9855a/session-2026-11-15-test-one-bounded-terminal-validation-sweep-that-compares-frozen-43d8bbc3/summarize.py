"""Summarize terminal runs and the frozen original-selection counterfactual."""
import hashlib
import json
from pathlib import Path
from autocal.stage_artifacts import read_stage_artifact
from autocal.history_selection import evaluate_history_candidates, select_history_plan
from autocal.predictive_validation import score_future_sweeps
from autocal.tools.sanity_battery import anchor_errors
root = Path(__file__).resolve().parent / "evidence"
rows = []
for name in ("hp3", "hp3-soft", "hp4"):
    folder = root / name
    if not (folder / "summary.json").exists(): continue
    summary = json.loads((folder / "summary.json").read_text())
    manifest = json.loads((folder / "sweeps.headless/manifest.json").read_text())
    events = [json.loads(line) for line in (folder / "sweeps.full_auto_log.jsonl").read_text().splitlines()]
    event = next(event for event in events if event.get("event") == "terminal_validation")
    artifact = read_stage_artifact(folder / "sweeps.terminal_validation.stage.json")["payload"]
    original, _, _ = select_history_plan(evaluate_history_candidates(artifact["candidates"]))
    original_score = score_future_sweeps(artifact["validation_dataset"], original["anchors"], (),
        home_offsets_mm=original["home_offsets_mm"], spool_params=original["spool_model_params"])[0]
    old = anchor_errors(manifest["firmware_initial"]["M669"], event["original_selection"]["m669"])
    rows.append({"case": name, "revision": manifest["git_revision"],
        "normal_completion": summary["normal_completion"], "accuracy_gate_passed": summary["passed"],
        "original_error_mm": old["max_abs_coordinate_mm"],
        "applied_error_mm": summary["anchors"]["max_abs_coordinate_mm"],
        "applied_aligned_error_mm": summary["anchors"]["max_aligned_abs_coordinate_mm"],
        "original_prediction_score": original_score,
        "selected_prediction_score": event["selected"]["heldout_prediction_score"],
        "selected_iteration": event["selected"]["iteration"], "selected_run": event["selected"]["run_id"],
        "training_points": sum(len(s["data_points"]) for s in json.loads((folder / "sweeps.json").read_text())["sweeps"]),
        "validation_points": sum(len(s["data_points"]) for s in artifact["validation_dataset"]["sweeps"])})
(root / "results.json").write_text(json.dumps(rows, indent=2)+"\n")
large = [{"path": str(path.resolve()), "bytes": path.stat().st_size,
          "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
          "availability": "local Git-ignored journal only; no off-host backup"}
         for path in root.rglob("events.jsonl") if path.parent.parent.name in [row["case"] for row in rows]]
(root / "journals.json").write_text(json.dumps(large, indent=2)+"\n")
print(json.dumps(rows, indent=2))
