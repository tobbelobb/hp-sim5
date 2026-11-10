"""Inspect archived inputs/results without repeating selection or prediction."""
from pathlib import Path
import gzip
import hashlib
import json
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[4]
SESSION = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

import numpy as np

from autocal.stage_artifacts import _encode, read_stage_artifact


def digest(value):
    return hashlib.sha256(json.dumps(_encode(value), sort_keys=True, allow_nan=False).encode()).hexdigest()


manifest = json.loads((SESSION / "inputs.json").read_text())
results = json.loads((SESSION / "results.json").read_text())
assert results["script_sha256"] == hashlib.sha256((SESSION / "probe.py").read_bytes()).hexdigest()
assert results["inputs_manifest_sha256"] == hashlib.sha256((SESSION / "inputs.json").read_bytes()).hexdigest()
rows = []
with tempfile.TemporaryDirectory(prefix="consensus-audit-") as temporary:
    for entry, result in zip(manifest, results["rows"], strict=True):
        assert entry["fixture"] == result["fixture"]
        raw = gzip.decompress((SESSION / entry["input"]).read_bytes())
        assert hashlib.sha256(raw).hexdigest() == entry["sha256"] == result["input_sha256"]
        path = Path(temporary) / "history.json"
        path.write_bytes(raw)
        payload = read_stage_artifact(path, expected_kind="history")["payload"]
        candidates = payload["candidates"]
        dataset = candidates[-1]["plan"]["dataset"]
        # Ignore file paths/metadata when identifying the actual observations.
        sweeps = dataset["sweeps"]
        candidate_rows = []
        for candidate in candidates:
            anchors = np.asarray(candidate["plan"]["anchors"])
            truth = np.asarray(entry["truth_anchors_mm"])[:, :anchors.shape[1]]
            candidate_rows.append({"iteration": candidate["iteration"],
                                   "sum_anchor_error_mm": float(np.linalg.norm(anchors-truth, axis=1).sum()),
                                   "training_sweep_ids": list(candidate["training_sweep_ids"])})
        rows.append({"fixture": entry["fixture"], "sweeps_sha256": digest(sweeps),
                     "sweeps": len(sweeps), "points": sum(len(s["data_points"]) for s in sweeps),
                     "settings_sha256": digest([c.get("settings") for c in candidates]),
                     "candidates": candidate_rows})
assert len({row["sweeps_sha256"] for row in rows}) == len(rows), "Duplicate observation dataset"
audit = {"command": ".venv/bin/python output/research/sessions/2026-10-23-consensus-membership/audit.py",
         "unique_observation_datasets": len(rows), "archived_inputs_and_script_match_results": True,
         "selection_or_prediction_evaluations": 0, "rows": rows}
(SESSION / "audit.json").write_text(json.dumps(audit, indent=2, allow_nan=False) + "\n")
print(json.dumps({key: value for key, value in audit.items() if key != "rows"}, indent=2))
