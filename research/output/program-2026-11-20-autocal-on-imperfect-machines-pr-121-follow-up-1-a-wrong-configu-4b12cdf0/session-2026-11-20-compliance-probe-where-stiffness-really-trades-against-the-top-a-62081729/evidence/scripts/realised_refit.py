"""Refit a run with the replay's realised forces in place of the commanded ones.

usage: realised_refit.py RUN_DIR REPLAY.jsonl

Every data point's force-mode lines (sensor_force_n and the HP5 extra sensors' force_n) are replaced
by the effector-side force the exact replay realised there (points without a replay match keep the
commanded value). Fits at k = 1 (configured) and at the true k, with commanded and realised forces,
and reports the top-anchor z error in the convention frame and the k that zeroes it.
"""
import copy, json, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from refit2 import fit_at
from probe_replay_attrib import line_paths
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.tools.imperfect_machine import convention_frame


def realised_dataset(ds, letters, reads):
    angles = np.array([r['angles_deg'] for r in reads])
    _, long = line_paths(reads[0]['joints'])
    force = lambda R, line: float(np.mean([sum(r['joints'][n][0] for n in long[letters[line]]) for r in R]))
    out, matched, total = copy.deepcopy(ds), 0, 0
    for sweep in out['sweeps']:
        for p in sweep['data_points']:
            total += 1
            mu = np.asarray(p.get('mu') or p['raw_angles_deg'], float)
            hit = np.flatnonzero(np.max(np.abs(angles - mu), axis=1) < 0.005)
            if not len(hit):
                continue
            matched += 1
            R = [reads[j] for j in hit]
            p['sensor_force_n'] = force(R, int(p.get('source_sensor_anchor', sweep['sensor_anchor'])))
            for e in p.get('extra_sensor_ranges', []):
                if 'force_n' in e:
                    e['force_n'] = force(R, int(e['anchor']))
    return out, matched, total


def main():
    run, replay = Path(sys.argv[1]), sys.argv[2]
    imp = json.loads((run / 'imperfection.json').read_text())
    fitter = 'ellipsoid' if imp.get('ellipsoid_shape') else 'sweep'
    ds = json.loads((run / 'sweeps.json').read_text()); ds.setdefault('machine_type', imp['machine']); normalize_dataset_point_roles(ds)
    letters = list(imp['truth_mm'])
    live = np.asarray(json.loads((run / 'result.json').read_text())['fitted_mm'], float)
    truth_frame = convention_frame(np.array(list(imp['truth_mm'].values())))[0]
    k_true = 1 / imp['stiffness_scale']
    reads = [json.loads(line) for line in open(replay)]
    realised, matched, total = realised_dataset(ds, letters, reads)
    out = dict(run=run.name, k_true=k_true, points_matched=matched, points=total)
    for name, data in (('commanded', ds), ('realised', realised)):
        ks = k_true * np.array([0.85, 1.0, 1.15])
        z = []
        for k in ks:
            f = convention_frame(np.asarray(fit_at(data, fitter, k, live)['anchors']))[0] - truth_frame
            z.append(float(f[-1, 2]))
            if k == ks[1]:
                out[f'{name}_true_k'] = dict(top_z_frame_mm=float(f[-1, 2]), max_abs_frame_mm=float(np.abs(f).max()),
                                             low_z_frame_mm=float(f[:-1, 2].mean()))
        slope = np.polyfit(ks, z, 1)
        out[f'{name}_k_zero_top_z_over_true'] = float(-slope[1] / slope[0] / k_true)
        out[f'{name}_dDz_dk_mm'] = float(slope[0])
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
