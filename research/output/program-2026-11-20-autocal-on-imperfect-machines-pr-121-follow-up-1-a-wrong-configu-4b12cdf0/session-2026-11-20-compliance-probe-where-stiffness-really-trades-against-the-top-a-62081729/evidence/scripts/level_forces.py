"""Realised sensor-line force at every probe level, from an exact replay.

usage: level_forces.py RUN_DIR REPLAY.jsonl [--json OUT]

Per level: commanded force, realised effector-side force (sum of long-strand tensions), spool-side
force (sum over paths of the first segment's tension x U: equal to the effector side when the
pinholes/wheels are frictionless), and the motor angle. Fits realised = alpha * commanded + delta_level.
"""
import json, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
import probe
from probe_replay_attrib import line_paths
from autocal.dataset_roles import normalize_dataset_point_roles


def main():
    run, replay = Path(sys.argv[1]), sys.argv[2]
    imp = json.loads((run / 'imperfection.json').read_text())
    ds = json.loads((run / 'sweeps.json').read_text()); ds.setdefault('machine_type', imp['machine']); normalize_dataset_point_roles(ds)
    letters = list(imp['truth_mm'])
    _, N, U, _ = probe.line_description(ds)
    reads = [json.loads(line) for line in open(replay)]
    angles = np.array([r['angles_deg'] for r in reads])
    paths, long = line_paths(reads[0]['joints'])
    rows = []
    for sweep in ds['sweeps']:
        for pr in (sweep.get('metadata') or {}).get('compliance_probes', []):
            s = int(pr['sensor_anchor']); letter = letters[s]
            first = [sorted(names, key=lambda n: (not n.endswith('_0'), n))[0] for names in paths[letter].values()]
            levels = []
            for level in pr['levels']:
                mu = np.asarray(level['mu_deg'], float)
                hit = np.flatnonzero(np.max(np.abs(angles - mu), axis=1) < 0.005)
                if not len(hit):
                    break
                R = [reads[j] for j in hit]
                eff = np.mean([sum(r['joints'][n][0] for n in long[letter]) for r in R])
                spool = np.mean([sum(r['joints'][n][0] for n in first) for r in R]) * U[s]
                levels.append(dict(cmd=level['force_n'], eff=float(eff), spool=float(spool), motor_deg=float(mu[s])))
            if len(levels) == len(pr['levels']):
                rows.append(dict(sensor=letter, levels=levels))
    cmd = np.array([l['cmd'] for r in rows for l in r['levels']]); eff = np.array([l['eff'] for r in rows for l in r['levels']])
    spool = np.array([l['spool'] for r in rows for l in r['levels']])
    alpha, delta = np.polyfit(cmd, eff, 1)
    resid = eff - (alpha * cmd + delta)
    step = lambda r, j: (r['levels'][j]['eff'] - r['levels'][0]['eff']) / (r['levels'][j]['cmd'] - r['levels'][0]['cmd'])
    s2, s3 = np.array([step(r, 1) for r in rows]), np.array([step(r, 2) for r in rows])
    off = np.array([[l['eff'] - l['cmd'] for l in r['levels'][:3]] for r in rows])
    summary = dict(run=run.name, probes=len(rows), fit_alpha=float(alpha), fit_delta_n=float(delta), resid_sd_n=float(resid.std()),
                   spool_minus_effector_max_n=float(np.max(np.abs(spool - eff))),
                   offset_n_mean_by_level=off.mean(0).round(4).tolist(), offset_n_sd_by_level=off.std(0).round(4).tolist(),
                   step_ratio_2x_median=float(np.median(s2)), step_ratio_3x_median=float(np.median(s3)),
                   step_ratio_corr=float(np.corrcoef(s2, s3)[0, 1]))
    print(json.dumps(summary, indent=1))
    if '--json' in sys.argv:
        Path(sys.argv[sys.argv.index('--json') + 1]).write_text(json.dumps(dict(summary, rows=rows), indent=1))


if __name__ == '__main__':
    main()
