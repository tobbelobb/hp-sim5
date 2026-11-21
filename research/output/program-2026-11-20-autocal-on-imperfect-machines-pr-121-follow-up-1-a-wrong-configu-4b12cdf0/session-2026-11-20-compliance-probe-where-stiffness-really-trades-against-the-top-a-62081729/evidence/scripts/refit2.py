"""Refit one imperfect-machine run whose fit uses configured compliance (HP5r sweeps, ellipsoid shape).

usage: refit2.py RUN_DIR [sweep|ellipsoid]   (default: ellipsoid if imperfection.json says so)

Every compliance the fit takes from config.g (FlexModel.from_m666 and HP4's ABC home strain)
is multiplied by an assumed compliance scale k (k = configured EA / true EA):
  config   k = 1 (what the default autocal does today)
  probe    k from the fixed-pose force-step probes (probe.estimate on the config fit's anchors)
  oracle   k = 1 / the scene's true stiffness scale
  joint    k minimising the fit rms over a grid (sweep data alone; no probe)
Errors are in the print convention frame (top x = y = 0, A.x = 0, origin at home).
"""
import copy, json, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
import probe
from probe_time import probe_seconds
import autocal.pose_fit as pf
import autocal.ellipse_fit as ef
from autocal.flex import FlexModel
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.anchor_seed import build_anchor_initial_guess
from autocal.ellipsoid_autocal import fit as ellipsoid_fit, SEED_SCALES
from autocal.tools.imperfect_machine import convention_frame
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, shift_dataset_home_offsets

GRID = np.round(np.arange(0.75, 1.351, 0.05), 3)
FROM_M666, HOME_STRAIN = FlexModel.from_m666.__func__, pf._hp4_home_strain


def fit_at(dataset, fitter, k, live):
    """The production fit at compliance scale k, also started from the live run's final anchors."""
    FlexModel.from_m666 = classmethod(lambda cls, m666, *, num_axes, spring_k_multiplier=1.0:
                                      FROM_M666(cls, m666, num_axes=num_axes, spring_k_multiplier=spring_k_multiplier / k))
    pf._hp4_home_strain = ef._hp4_home_strain = lambda d: (lambda h: None if h is None else (h[0] * k, h[1]))(HOME_STRAIN(d))
    try:
        data = copy.deepcopy(dataset)
        seed = build_anchor_initial_guess(data, machine_type=data['machine_type'])
        if fitter == 'sweep':
            options = dict(home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
            offsets = pf.fit_pose_least_squares(data, seed, fit_home_offsets=True, extra_initial_anchors=[live], **options)
            result = pf.fit_pose_least_squares(shift_dataset_home_offsets(data, offsets['home_offsets_mm']),
                                               offsets['anchors'], fit_home_offsets=False, **options)
            return dict(result, home_offsets_mm=offsets['home_offsets_mm'])
        return ellipsoid_fit(data, [seed * s for s in SEED_SCALES] + [live], True, 'ellipsoid')
    finally:
        FlexModel.from_m666, pf._hp4_home_strain, ef._hp4_home_strain = classmethod(FROM_M666), HOME_STRAIN, HOME_STRAIN


def main():
    run = Path(sys.argv[1])
    imperfection = json.loads((run / 'imperfection.json').read_text())
    fitter = sys.argv[2] if len(sys.argv) > 2 else ('ellipsoid' if imperfection.get('ellipsoid_shape') else 'sweep')
    dataset = json.loads((run / 'sweeps.json').read_text())
    dataset.setdefault('machine_type', imperfection['machine'])
    normalize_dataset_point_roles(dataset)
    truth = np.array(list(imperfection['truth_mm'].values()))
    truth_frame = convention_frame(truth)[0]

    def summary(result, k):
        f = convention_frame(np.asarray(result['anchors']))[0] - truth_frame
        return dict(k=float(k), max_abs_frame_mm=float(np.abs(f).max()), top_z_frame_mm=float(f[-1, 2]),
                    low_z_frame_mm=float(f[:-1, 2].mean()), frame_error_mm=f.round(3).tolist(), rms_mm=float(result['rms_mm']))

    result = json.loads((run / 'result.json').read_text())
    live = np.asarray(result['fitted_mm'], float)
    fit = lambda k: fit_at(dataset, fitter, k, live)
    config = fit(1.0)
    home = config['home_offsets_mm']
    k_probe, rows = probe.estimate(dataset, config['anchors'], home)
    scan = [summary(fit(k), k) for k in GRID]
    rms = np.array([s['rms_mm'] for s in scan])
    i = int(rms.argmin())
    # parabola through the grid minimum and its neighbours
    j = min(max(i, 1), len(GRID) - 2)
    c = np.polyfit(GRID[j - 1:j + 2], rms[j - 1:j + 2], 2)
    k_joint = float(-c[1] / (2 * c[0])) if c[0] > 0 else float(GRID[i])
    k_true = 1 / imperfection['stiffness_scale']
    out = dict(run=run.name, machine=imperfection['machine'], seed=imperfection['seed'], fitter=fitter,
               true_stiffness_scale=imperfection['stiffness_scale'], k_true=k_true, k_probe=k_probe, k_joint=k_joint,
               k_probe_steps=[round(r['k'], 4) for r in rows], num_probe_steps=len(rows),
               config=summary(config, 1.0), probe=summary(fit(k_probe), k_probe) if k_probe else None,
               oracle=summary(fit(k_true), k_true), joint=summary(fit(k_joint), k_joint),
               rms_scan=[dict(k=s['k'], rms_mm=round(s['rms_mm'], 6), top_z_frame_mm=round(s['top_z_frame_mm'], 4),
                              max_abs_frame_mm=round(s['max_abs_frame_mm'], 4)) for s in scan])
    out['live'] = {key: result.get(key) for key in ('max_abs_frame_error_mm', 'frame_error_mm', 'simulated_s', 'wall_s', 'stop_reason')}
    out['probe_s'], out['num_probes'] = probe_seconds(run)
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
