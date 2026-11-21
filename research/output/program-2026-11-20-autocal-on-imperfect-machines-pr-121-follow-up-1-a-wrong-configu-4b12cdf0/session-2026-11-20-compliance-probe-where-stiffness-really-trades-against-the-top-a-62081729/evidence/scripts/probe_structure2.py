"""Per-step probe k against pose, sensor line and force level, using the true anchors.
usage: probe_structure2.py RUN_DIR ... ; prints one line per step and a per-run summary."""
import json, sys
import numpy as np
sys.path.insert(0, __file__.rsplit('/', 1)[0])
import probe
from autocal.pose_fit import fit_point_positions
from autocal.dataset_roles import normalize_dataset_point_roles

for run in sys.argv[1:]:
    imp = json.load(open(f'{run}/imperfection.json')); ds = json.load(open(f'{run}/sweeps.json'))
    ds.setdefault('machine_type', imp['machine']); normalize_dataset_point_roles(ds)
    truth = np.array(list(imp['truth_mm'].values())); k_true = 1 / imp['stiffness_scale']
    s, n, u, y = probe.line_description(ds)
    rows = []
    for sensor, held, forces, lengths in probe.probe_rows(ds):
        # pose from the true anchors; home offsets unknown here -> use zero (the encoder origin is home)
        pos = fit_point_positions(truth, lengths[:1], np.zeros(len(truth)))[0][0]
        for j in (1, 2):
            a, b = probe.step_terms(truth, pos, sensor, held, forces[j] - forces[0], lengths[j] - lengths[0], s, n, u, y)
            rows.append((sensor, j, forces[j] - forces[0], *pos, (lengths[j][sensor] - lengths[0][sensor] - a) / b / k_true))
    r = np.array(rows)
    print(f'{run.split("/")[-1]}: k_true {k_true:.3f}; ratio k_step/k_true mean {r[:, -1].mean():.3f} median {np.median(r[:, -1]):.3f}')
    for sensor in sorted(set(r[:, 0])):
        m = r[r[:, 0] == sensor]
        print(f'   sensor {int(sensor)}: n {len(m)}, ratio {np.median(m[:, -1]):.3f}')
    for lo, hi in ((-1e9, 100), (100, 400), (400, 800), (800, 1e9)):
        m = r[(r[:, 5] >= lo) & (r[:, 5] < hi)]
        if len(m): print(f'   z in [{lo:.0f},{hi:.0f}): n {len(m)}, ratio median {np.median(m[:, -1]):.3f}')
    two, three = r[r[:, 1] == 1, -1], r[r[:, 1] == 2, -1]
    print(f'   2x level {np.median(two):.3f}, 3x level {np.median(three):.3f}, corr {np.corrcoef(two, three)[0, 1]:.2f}')
