"""E12 check: production sweep_jackknife on live runs and the review's two misses, vs E12's stored values."""
import copy, json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from lowz_common import TRUTH, load, fit_default, anchor_error, resynthesise, point_positions
from autocal._autocal_common import _normalize_dataset_point_roles
from autocal.anchor_uncertainty import sweep_jackknife, coordinate_names, uncertainty_label
rows = []
def check(label, data, truth):
    a = fit_default(data)[0]["anchors"]
    value, single = sweep_jackknife(data, a)
    rows.append(dict(case=label, err=anchor_error(a, truth), jackknife=value, label=uncertainty_label(value),
                     single_unit=coordinate_names(single, len(a))))
    print(json.dumps(rows[-1]), flush=True)
runs = Path(__file__).resolve().parents[1] / "runs"
for name in ("new_hp5_base_0", "new_hp5_seed1", "new_hp5_seed2", "new_hp5_seed3", "new_hp5_seed4",
             "new_hp5r_base_0", "new_hp5r_seed1", "new_hp5r_seed2", "new_hp5r_seed3", "new_base_0", "new_hp4_A_p10"):
    data = json.loads((runs / name / "sweeps.json").read_text()); _normalize_dataset_point_roles(data)
    check(name, data, np.asarray(json.loads((runs / name / "input.json").read_text())["truth"]))
for name, sweep, d_mm in (("benchmark_923c2baa_hangprinter_3", 8, 10.0), ("benchmark_923c2baa_hangprinter_5_pulley_reduced", 0, 20.0)):
    base = load(name); _normalize_dataset_point_roles(base); truth = TRUTH[base["machine_type"]]
    moved = truth.copy()
    for anchor in [int(a) for a in base["sweeps"][sweep]["fixed_anchors"] if int(a) != len(truth) - 1]:
        moved[anchor, :2] += d_mm * moved[anchor, :2] / np.linalg.norm(moved[anchor, :2])
    whole = resynthesise(base, truth, moved, positions=point_positions(base, truth))
    data = copy.deepcopy(base); data["sweeps"][sweep] = whole["sweeps"][sweep]
    check(f"{name}#{sweep} {d_mm:g} mm", data, truth)
Path(sys.argv[1]).write_text(json.dumps(rows, indent=1))
