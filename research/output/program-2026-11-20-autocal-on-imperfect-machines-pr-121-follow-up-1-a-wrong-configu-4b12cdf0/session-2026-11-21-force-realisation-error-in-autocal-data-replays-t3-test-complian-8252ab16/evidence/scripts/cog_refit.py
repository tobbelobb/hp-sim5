"""Refit a recorded run with force-mode line forces corrected by a cogging map.

usage: cog_refit.py RUN_DIR [amp_spool_n=0.3333] [harmonic=50]

The map is the sim's own torque-mode cogging (oracle; derived in cogging_regress.py):
realised effector force = commanded + U * amp * sin(harmonic * encoder angle),
U = the line's mechanical advantage (M666 U). It bounds what any cogging
compensation (calibrated map, phase averaging) can recover from the same data.
Fits at the true and at the configured compliance; errors are in the print frame.
"""
import copy, gzip, json, sys
from pathlib import Path
import numpy as np

PROBE = Path(__file__).resolve().parents[3] / 'session-2026-11-20-compliance-probe-where-stiffness-really-trades-against-the-top-a-62081729/evidence/scripts'
sys.path.insert(0, str(PROBE))
from refit2 import fit_at  # noqa: E402
from autocal.dataset_roles import normalize_dataset_point_roles  # noqa: E402
from autocal.tools.imperfect_machine import convention_frame  # noqa: E402


def load(run):
    path = run / 'sweeps.json'
    return json.loads(path.read_text()) if path.exists() else json.load(gzip.open(run / 'sweeps.json.gz'))


def corrected(ds, amp, harmonic, scale=1.0):
    out = copy.deepcopy(ds)
    U = out['config']['m666_before_data_collection']['U']
    for sweep in out['sweeps']:
        for p in sweep['data_points']:
            th = np.deg2rad(np.asarray(p.get('mu') or p['raw_angles_deg'], float))
            cog = lambda a: scale * U[a] * amp * np.sin(harmonic * th[a])
            s = int(p.get('source_sensor_anchor', sweep['sensor_anchor']))
            p['sensor_force_n'] = float(p['sensor_force_n'] + cog(s))
            for e in p.get('extra_sensor_ranges', []):
                if 'force_n' in e:
                    e['force_n'] = float(e['force_n'] + cog(int(e['anchor'])))
    return out


def main():
    run = Path(sys.argv[1])
    amp = float(sys.argv[2]) if len(sys.argv) > 2 else 0.01 / 0.03
    harmonic = float(sys.argv[3]) if len(sys.argv) > 3 else 50
    imp = json.loads((run / 'imperfection.json').read_text())
    fitter = 'ellipsoid' if imp.get('ellipsoid_shape') else 'sweep'
    ds = load(run)
    ds.setdefault('machine_type', imp['machine'])
    normalize_dataset_point_roles(ds)
    truth = convention_frame(np.array(list(imp['truth_mm'].values())))[0]
    live = np.asarray(json.loads((run / 'result.json').read_text())['fitted_mm'], float)
    k_true = 1 / imp['stiffness_scale']
    out = dict(run=run.name, machine=imp['machine'], seed=imp['seed'], k_true=k_true, amp=amp, harmonic=harmonic)
    for name, data in (('commanded', ds), ('cog_map', corrected(ds, amp, harmonic)),
                       ('cog_map_wrong_sign', corrected(ds, amp, harmonic, -1.0))):
        for kname, k in (('true_k', k_true), ('config_k', 1.0)):
            f = convention_frame(np.asarray(fit_at(data, fitter, k, live)['anchors']))[0] - truth
            out[f'{name}_{kname}'] = dict(top_z_mm=round(float(f[-1, 2]), 4),
                                          max_abs_mm=round(float(np.abs(f).max()), 4))
    print(json.dumps(out))


if __name__ == '__main__':
    main()
