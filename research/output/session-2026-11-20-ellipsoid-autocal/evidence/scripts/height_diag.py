import json, sys
import numpy as np
from autocal.dataset_roles import normalize_dataset_point_roles, mesh_metadata
from autocal.ellipsoid_autocal import fit, SEED_SCALES
from autocal.anchor_seed import build_anchor_initial_guess
from autocal.tests.test_disturbed_robustness import TRUTH
import autocal.pose_fit as pf
machine, path = sys.argv[1], sys.argv[2]
d = json.load(open(path)); normalize_dataset_point_roles(d)
truth = np.asarray(TRUTH[machine], float)
def h(s): return round(mesh_metadata(s)["held_length_mm"] / 50) * 50
heights = sorted({h(s) for s in d["sweeps"]})
def run(sub, label, starts=None):
    seed = build_anchor_initial_guess(sub, machine_type=machine)
    r = fit(sub, [seed * s for s in SEED_SCALES] + [truth], None, True, "ellipsoid")
    e = r["anchors"] - truth
    print(f"{label:40s} err {np.max(np.abs(e)):.2f} rms {r['rms_mm']:.3f} offsets {np.round(r['home_offsets_mm'],2).tolist()} worst {np.unravel_index(np.argmax(np.abs(e)), e.shape)}", flush=True)
run(d, "all")
for x in heights:
    run(dict(d, sweeps=[s for s in d["sweeps"] if h(s) != x]), f"without {x}")
for x in heights:
    run(dict(d, sweeps=[s for s in d["sweeps"] if h(s) >= x]), f"heights >= {x}")
orig = pf.dataset_stretch
pf.dataset_stretch = lambda dataset: None
import autocal.ellipse_fit as ef; ef.dataset_stretch = pf.dataset_stretch
run(d, "all, no stretch correction")
