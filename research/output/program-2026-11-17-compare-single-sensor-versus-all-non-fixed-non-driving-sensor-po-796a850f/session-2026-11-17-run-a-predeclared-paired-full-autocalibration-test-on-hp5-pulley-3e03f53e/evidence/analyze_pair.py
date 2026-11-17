"""Compare the predeclared baseline and treatment full runs.

Scores final applied and latest estimated anchors against the nominal firmware
M669 anchors. Reports both absolute coordinate/Euclidean and translation-aligned
metrics, plus budget, residuals, accepted extra ranges and hashes.
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np

SESSION = Path(__file__).resolve().parent.parent
AXES = ["A", "B", "C", "D", "I"]
NOMINAL = np.array([[0.0, -1900.0, -280.0], [1900.0, 0.0, -280.0],
                    [0.0, 1900.0, -280.0], [-1900.0, 0.0, -280.0],
                    [0.0, 0.0, 1900.0]])

M669_RE = re.compile(r"([A-Z])([-+]?\d+(?:\.\d+)?):([-+]?\d+(?:\.\d+)?):([-+]?\d+(?:\.\d+)?)")


def parse_m669(text: str | None) -> dict:
    if not text:
        return {}
    return {match.group(1): [float(match.group(2)), float(match.group(3)), float(match.group(4))]
            for match in M669_RE.finditer(text)}


def load_json(path: Path, default=None):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return default


def nominal_from_dataset(dataset_path: Path):
    data = load_json(dataset_path, {}) or {}
    m669 = (data.get("config") or {}).get("m669") or {}
    anchors = [m669.get(axis) for axis in AXES]
    if all(isinstance(anchor, list) and len(anchor) == 3 for anchor in anchors):
        return np.asarray(anchors, dtype=float)
    return NOMINAL


def error_metrics(fitted: np.ndarray, truth: np.ndarray) -> dict:
    diff = np.asarray(fitted, dtype=float) - np.asarray(truth, dtype=float)
    shift = diff.mean(axis=0)
    aligned = diff - shift
    return {
        "max_abs_coordinate_mm": float(np.max(np.abs(diff))),
        "max_euclidean_mm": float(np.max(np.linalg.norm(diff, axis=1))),
        "max_aligned_coordinate_mm": float(np.max(np.abs(aligned))),
        "max_aligned_euclidean_mm": float(np.max(np.linalg.norm(aligned, axis=1))),
        "translation_mm": shift.tolist(),
        "per_anchor_mm": np.linalg.norm(diff, axis=1).tolist(),
        "per_anchor_aligned_mm": np.linalg.norm(aligned, axis=1).tolist(),
        "fitted_anchors_mm": np.asarray(fitted, dtype=float).tolist(),
        "truth_anchors_mm": np.asarray(truth, dtype=float).tolist(),
    }


def last_summary(jsonl_path: Path):
    summary = None
    last_iteration = None
    for line in jsonl_path.read_text().splitlines() if jsonl_path.exists() else []:
        try:
            event = json.loads(line)
        except ValueError:
            continue
        if event.get("event") == "summary":
            summary = event
        if "iteration" in event:
            last_iteration = event
    return summary, last_iteration


def run_metrics(arm: str) -> dict:
    run_dir = SESSION / "runs" / arm
    dataset_path = run_dir / "sweeps.json"
    record = load_json(run_dir / "run.json", {}) or {}
    manifest = load_json(dataset_path.with_suffix(".headless") / "manifest.json", {}) or {}
    summary, last_iteration = last_summary(dataset_path.with_suffix(".full_auto_log.jsonl"))
    dataset = load_json(dataset_path, {}) or {}
    truth = nominal_from_dataset(dataset_path)
    applied = parse_m669((manifest.get("firmware_final") or {}).get("M669"))
    if not applied and summary:
        applied = parse_m669(summary.get("m669"))
    result = {
        "arm": arm,
        "run_present": bool(record or manifest),
        "dataset_path": str(dataset_path),
        "returncode": record.get("returncode"),
        "wall_s": record.get("wall_s"),
        "manifest_status": manifest.get("status"),
        "stop_reason": manifest.get("stop_reason"),
        "normal_completion": manifest.get("normal_completion"),
        "retained_sweeps": len(dataset.get("sweeps", [])),
        "retained_points": sum(len(s.get("data_points", [])) for s in dataset.get("sweeps", [])),
        "acquisition_rejections": len(dataset.get("acquisition_rejections", [])),
        "physics_steps": record.get("clock", {}).get("total_steps", manifest.get("total_steps")),
        "simulated_s": record.get("clock", {}).get("simulated_s", manifest.get("simulated_s")),
        "collector_time_s": record.get("clock", {}).get("collector_time_s", manifest.get("collector_time_s")),
        "service_wall_s": manifest.get("service_wall_s"),
        "source_sha256": manifest.get("source_sha256"),
        "autocal_source_sha256": manifest.get("autocal_source_sha256"),
        "git_revision": manifest.get("git_revision"),
        "scene_sha256": manifest.get("scene_sha256"),
        "firmware_config_sha256": manifest.get("firmware_config_sha256"),
        "rrf_sha256": manifest.get("rrf_sha256"),
        "physics_artifact": manifest.get("physics_artifact"),
        "baked_scene_sha256": manifest.get("baked_scene_sha256"),
        "applied_m669": applied,
        "latest_summary": {
            key: (summary or {}).get(key) for key in ("m669", "m666", "anchors", "fitted_anchors", "selection")
        } if summary else None,
        "last_iteration": last_iteration,
    }
    if applied:
        applied_ordered = np.asarray([applied.get(axis) for axis in AXES], dtype=float)
        result["final_applied_error"] = error_metrics(applied_ordered, truth)
    if summary and summary.get("fitted_anchors"):
        result["latest_estimate_error"] = error_metrics(np.asarray(summary["fitted_anchors"], dtype=float), truth)
    if summary and summary.get("anchors"):
        result["latest_summary_anchor_error"] = error_metrics(np.asarray(summary["anchors"], dtype=float), truth)
    log_text = (run_dir / "run.log").read_text(errors="replace") if (run_dir / "run.log").exists() else ""
    rms_matches = re.findall(r"pose_fit:.*?rms=([0-9.]+)mm", log_text)
    result["last_pose_fit_rms_mm"] = float(rms_matches[-1]) if rms_matches else None
    steps = re.findall(r"stable_stop step_mm=([0-9.eE+-]+) limit_mm=([0-9.eE+-]+) streak=([0-9]+)/([0-9]+)", log_text)
    result["stable_stop_events"] = [
        {"step_mm": float(step), "limit_mm": float(limit), "streak": int(streak), "needed": int(needed)}
        for step, limit, streak, needed in steps
    ]
    result["extra_range_lines"] = len(re.findall(r"extra-range experiment", log_text))
    validity = load_json(dataset_path.with_suffix(".extra_range_validity.json"), None)
    if validity:
        gate = validity.get("gate") or {}
        result["extra_range_validity"] = {
            "path": str(dataset_path.with_suffix(".extra_range_validity.json")),
            "accepted": gate.get("accepted"),
            "rejected": gate.get("rejected"),
            "no_policy": gate.get("no_policy"),
            "constants": gate.get("constants"),
            "sweeps": {
                sweep_id: {
                    "accepted": record.get("accepted"),
                    "status": record.get("status"),
                    "axis": record.get("axis"),
                    "segments": [
                        {key: segment.get(key) for key in (
                            "drive_travel_mm", "sensor_travel_mm", "extra_travel_mm",
                            "extra_drive_correlation", "cubic_rms_mm", "cubic_max_mm", "status")}
                        for segment in record.get("segments", [])
                    ],
                }
                for sweep_id, record in (gate.get("sweeps") or {}).items()
            },
        }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=str(SESSION / "pair-result.json"))
    args = parser.parse_args()
    results = {arm: run_metrics(arm) for arm in ("baseline", "treatment")}
    payload = {"session": str(SESSION), "runs": results}
    Path(args.output).write_text(json.dumps(payload, indent=2) + "\n")
    rows = []
    for arm, result in results.items():
        applied = result.get("final_applied_error") or {}
        latest = result.get("latest_estimate_error") or {}
        rows.append(
            f"| {arm} | {result.get('stop_reason')} | {result.get('normal_completion')} | "
            f"{result.get('retained_sweeps')} | {result.get('retained_points')} | "
            f"{applied.get('max_abs_coordinate_mm', float('nan')):.2f} | "
            f"{applied.get('max_euclidean_mm', float('nan')):.2f} | "
            f"{applied.get('max_aligned_coordinate_mm', float('nan')):.2f} | "
            f"{applied.get('max_aligned_euclidean_mm', float('nan')):.2f} | "
            f"{latest.get('max_aligned_coordinate_mm', float('nan')):.2f} | "
            f"{result.get('physics_steps')} | {result.get('simulated_s')} | {result.get('wall_s')} |"
        )
    markdown = "\n".join([
        "| arm | stop | normal | sweeps | points | applied max coord | applied max eucl | applied aligned coord | applied aligned eucl | latest est aligned coord | physics steps | simulated s | wall s |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        *rows,
        "",
    ])
    (SESSION / "pair-result.md").write_text(markdown)
    print(markdown)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
