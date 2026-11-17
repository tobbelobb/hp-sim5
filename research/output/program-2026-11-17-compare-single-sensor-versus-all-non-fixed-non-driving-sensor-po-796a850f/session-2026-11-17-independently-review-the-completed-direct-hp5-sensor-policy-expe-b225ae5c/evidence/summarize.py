"""Aggregate the preserved HP5-direct runs into one results.json."""
from __future__ import annotations

import glob
import json
import statistics as st

SEG = {'A': ('AL', 'AR'), 'B': ('BL', 'BR'), 'C': ('CL', 'CR'),
       'D': ('DL', 'DR'), 'I': ('I0', 'I1', 'I2', 'I3')}


def load(path):
    d = json.load(open(path))
    recs = [r for r in d['records'] if 'samples' in r]
    pre = next(r for r in d['records'] if r.get('phase') == 'pre')
    return d, recs, pre


def error(axis, snap, pre):
    return ((snap['virt'][axis] - pre['virt'][axis])
            - (snap['geom'][axis] - pre['geom'][axis]))


def one(path, axis):
    d, recs, pre = load(path)
    m = d['manifest']
    errs = [st.mean(error(axis, s, pre) for s in r['samples']) for r in recs]
    fmin = [min(min(s['force'][k] for k in SEG[axis]) for s in r['samples']) for r in recs]
    fmean = [st.mean([st.mean([s['force'][k] for k in SEG[axis]]) for s in r['samples']]) for r in recs]
    std = [st.pstdev([s['virt'][axis] for s in r['samples']]) for r in recs]
    partner = 'A' if axis in ('C', 'D') else ('B' if axis == 'A' else 'D')
    partner_force = [st.mean([st.mean([s['force'][k] for k in SEG[partner]]) for s in r['samples']]) for r in recs]
    roles = m.get('roles') or dict(redundant='C', sensor='D')
    force = m['c_force_n'] if axis == roles.get('redundant') else m['d_force_n']
    return dict(file=path, policy=m['policy'], direction=m.get('direction'), axis=axis,
                force_n=force, speed=m.get('drive_speed_rad_s'), poses=len(recs),
                slack_poses=sum(1 for e in errs if e > 0.5),
                err_mean_mm=st.mean(errs), err_max_mm=max(errs),
                err_recovered_max_mm=max(error(axis, r['recovered'], pre) for r in recs),
                force_min_n=min(fmin), force_mean_n=st.mean(fmean),
                partner_force_mean_n=st.mean(partner_force),
                encoder_std_mm=st.mean(std),
                x_span_mm=1000 * (max(st.mean(s['pos'][0] for s in r['samples']) for r in recs)
                                  - min(st.mean(s['pos'][0] for s in r['samples']) for r in recs)))


if __name__ == '__main__':
    out = {'taut_C': [], 'catalog_C': [], 'role_check': [], 'fast_reversal': []}
    for path in sorted(glob.glob('taut_down_*.json')):
        out['taut_C'].append(one(path, 'C'))
    for path in sorted(glob.glob('taut_up_*.json')):
        out['taut_C'].append(one(path, 'C'))
    for path in sorted(glob.glob('slow2_down_*.json')):
        out['catalog_C'].append(one(path, 'C'))
    for path in sorted(glob.glob('altrole_*.json')):
        out['role_check'].append(one(path, 'D'))
    for path in sorted(glob.glob('rec_*.json')):
        out['fast_reversal'].append(one(path, 'C'))
    out['geometry_information'] = json.load(open('info.json'))['results']
    out['monte_carlo_fit'] = json.load(open('mc_fit.json'))
    out['commands'] = {
        'taut_scene': '.venv/bin/python make_taut_scene.py',
        'sweep': '.venv/bin/python sweep_experiment.py --scene "$PWD/hp5_taut.usda" '
                 '--policy 0.3,1.0 --steps 6 --drive-step-rad 0.5 --drive-speed-rad-s 0.8 '
                 '--settle 800 --samples 6 --recover-steps 3000 --direction down '
                 '--out taut_down_0.3,1.0.json',
        'info': '.venv/bin/python info.py',
        'fit': '.venv/bin/python mc_fit.py',
        'summarize': '.venv/bin/python summarize.py',
    }
    with open('results.json', 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps({k: v for k, v in out.items() if k in ('taut_C', 'catalog_C', 'role_check')}, indent=1))
