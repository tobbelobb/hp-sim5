import json, sys
from pathlib import Path
import numpy as np
from autocal.tests.test_disturbed_robustness import TRUTH
for d in map(Path, sys.argv[1:]):
    m = d.name.split("-", 1)[1]
    rows = [json.loads(l) for l in (d / "sweeps.full_auto_log.jsonl").read_text().splitlines() if l.strip()]
    rows = [r for r in rows if r.get("event") == "ellipsoid_fit"]
    man = json.loads((d / "sweeps.headless/manifest.json").read_text())
    truth = np.asarray(TRUTH[m], float)
    errs = [float(np.max(np.abs(np.asarray(r["runs"][0]["metrics"]["pose_fit_anchors"]) - truth))) for r in rows]
    print(f"{m:30s} visits {len(rows)} final {errs[-1] if errs else float('nan'):.2f} mm  per-visit {[round(e,2) for e in errs]}  sim {man.get('simulated_s', 0):.0f} s  {man.get('stop_reason')}")
