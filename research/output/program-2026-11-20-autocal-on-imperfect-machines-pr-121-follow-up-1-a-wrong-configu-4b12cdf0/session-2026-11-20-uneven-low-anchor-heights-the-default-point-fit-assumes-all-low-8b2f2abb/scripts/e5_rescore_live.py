"""E5: rescore finished live runs: equal-z vs free-warp fit (4+ low anchors), F statistic, jackknife."""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from lowz_common import fit_default, refit, rotation_aligned, anchor_error, level_frame_ls

rows = []
for run in sys.argv[1:]:
    run = Path(run)
    truth = np.asarray(json.loads((run / "input.json").read_text())["truth"])
    data = json.loads((run / "sweeps.json").read_text())
    fit, shifted = fit_default(data)
    row = dict(run=run.name, sweeps=len(data["sweeps"]), default_err=anchor_error(fit["anchors"], truth),
               default_err_rot=anchor_error(rotation_aligned(fit["anchors"], truth), truth))
    if len(truth) > 4:
        eq, c_eq, N, _, _ = refit(shifted, fit["anchors"], False)
        fr, c_fr, _, p, k = refit(shifted, fit["anchors"], True)
        row.update(free_err=anchor_error(fr, truth), free_err_rot=anchor_error(rotation_aligned(fr, truth), truth),
                   free_err_vs_level=anchor_error(fr, level_frame_ls(truth)),
                   F=(c_eq - c_fr) / k / (c_fr / (N - p)), eq_rms=float(np.sqrt(c_eq / N)),
                   free_rms=float(np.sqrt(c_fr / N)), free_low_z=fr[:-1, 2].round(2).tolist())
    rows.append(row); print(json.dumps(row), flush=True)
