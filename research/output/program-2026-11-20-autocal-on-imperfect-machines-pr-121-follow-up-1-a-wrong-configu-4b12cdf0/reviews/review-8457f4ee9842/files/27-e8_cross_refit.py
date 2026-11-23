"""E8: refit recorded live datasets with whichever autocal is on PYTHONPATH (old or new gauge).

Usage: e8_cross_refit.py OUT.json RUN_DIR...   Error vs the run's convention-frame truth.
"""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from lowz_common import fit_default, anchor_error
import autocal.theoretical_ellipse as te
rows = []
for run in map(Path, sys.argv[2:]):
    truth = np.asarray(json.loads((run / "input.json").read_text())["truth"])
    fit, _ = fit_default(json.loads((run / "sweeps.json").read_text()))
    a = fit["anchors"]
    rows.append(dict(run=run.name, code=te.__file__, err=anchor_error(a, truth),
                     low_z_err=(a[:-1, 2] - truth[:-1, 2]).round(3).tolist(), dz_D=float(a[-1, 2] - truth[-1, 2])))
    print(json.dumps(rows[-1]), flush=True)
Path(sys.argv[1]).write_text(json.dumps(rows, indent=1))
