"""E12a: fold information ratios on live runs (which folds lose a constraint)."""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from lowz_common import fit_default
from structural import fold_ratios
for run in map(Path, sys.argv[1:]):
    data = json.loads((run / "sweeps.json").read_text())
    anchors = fit_default(data)[0]["anchors"]
    ratios, full = fold_ratios(data, anchors)
    worst = ratios.max(axis=(1, 2)); cell = [np.unravel_index(r.argmax(), r.shape) for r in ratios]
    print(run.name, "fold max ratio", np.round(worst, 2).tolist(), "cells", [tuple(int(v) for v in c) for c in cell], flush=True)
