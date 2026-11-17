"""Independent recomputation of the HP5-direct sensor-policy aggregates.

Reads the source raw runs (read-only, passed with --source) and the reviewer's
own replayed runs (repro_*.json), recomputes every headline aggregate from the
raw snapshots, and prints a paired 0.05 vs 0.3 N comparison.

Definitions are restated here rather than imported from the source summarizer:
  error(axis) = (virt(axis) - virt_pre(axis)) - (geom(axis) - geom_pre(axis))
  slack pose  = mean error over that pose's samples > 0.5 mm
  force_min   = min over samples and the axis' physical segments of joint force
"""
from __future__ import annotations

import argparse
import glob
import json
import statistics as st
from pathlib import Path

SEG = {'A': ('AL', 'AR'), 'B': ('BL', 'BR'), 'C': ('CL', 'CR'),
       'D': ('DL', 'DR'), 'I': ('I0', 'I1', 'I2', 'I3')}


def load(path):
    d = json.load(open(path))
    recs = [r for r in d['records'] if 'samples' in r]
    pre = next(r for r in d['records'] if r.get('phase') == 'pre')
    return d, recs, pre


def err(axis, snap, pre):
    return ((snap['virt'][axis] - pre['virt'][axis])
            - (snap['geom'][axis] - pre['geom'][axis]))


def summarize(path, axis='C'):
    d, recs, pre = load(path)
    m = d['manifest']
    pose_err = [st.mean(err(axis, s, pre) for s in r['samples']) for r in recs]
    forces = [f for r in recs for s in r['samples']
              for k in SEG[axis] for f in [s['force'][k]]]
    fmin = min(forces)
    fmean = st.mean(forces)
    rec_max = max(err(axis, r['recovered'], pre) for r in recs)
    enc_std = st.mean(st.pstdev([s['virt'][axis] for s in r['samples']]) for r in recs)
    xs = [st.mean(s['pos'][0] for s in r['samples']) for r in recs]
    return dict(
        file=Path(path).name, direction=m.get('direction'), force_n=m['c_force_n'],
        poses=len(recs), slack_poses=sum(e > 0.5 for e in pose_err),
        err_mean_mm=st.mean(pose_err), err_max_mm=max(pose_err),
        err_min_mm=min(pose_err), err_recovered_max_mm=rec_max,
        force_min_n=fmin, force_mean_n=fmean,
        force_p05_n=st.quantiles(forces, n=20)[0],
        force_p95_n=st.quantiles(forces, n=20)[-1],
        encoder_std_mm=enc_std, x_span_mm=1000 * (max(xs) - min(xs)),
        pose_err_mm=[round(e, 4) for e in pose_err])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', required=True,
                    help='directory with the source evidence/ raw runs')
    ap.add_argument('--out', default='recompute.json')
    args = ap.parse_args()
    here = Path(__file__).resolve().parent
    src = Path(args.source)
    out = {'source': {}, 'repro': {}, 'all_axis_source_taut_down_0.3': {}}

    for label, pattern, root in (
            ('source_taut', 'taut_*.json', src),
            ('source_catalog', 'slow2_*.json', src),
            ('source_role', 'altrole_*.json', src),
            ('source_fast', 'rec_*.json', src),
            ('repro', 'repro_*.json', here)):
        for path in sorted(root.glob(pattern)):
            axis = 'D' if label == 'source_role' else 'C'
            out['source' if label != 'repro' else 'repro'][Path(path).name] = summarize(path, axis)

    # Per-axis model-bias check on one 0.3 N taut run: the report claims C max
    # ~0.07 mm and the longitudinal pair B/D up to ~0.8 mm against the
    # five-anchor midpoint geometry model.
    d, recs, pre = load(src / 'taut_down_0.3,1.0.json')
    for axis in 'ABCDI':
        es = [st.mean(err(axis, s, pre) for s in r['samples']) for r in recs]
        out['all_axis_source_taut_down_0.3'][axis] = dict(
            min_mm=min(es), max_mm=max(es), mean_mm=st.mean(es))

    # Paired comparison: same pose, same direction, 0.05 vs 0.30 N.
    paired = {}
    for direction in ('down', 'up'):
        a = Path(f'repro_{direction}_0.05.json')
        b = Path(f'repro_{direction}_0.3.json')
        if a.exists() and b.exists():
            da, ra, pa = load(a)
            db, rb, pb = load(b)
            ea = [st.mean(err('C', s, pa) for s in r['samples']) for r in ra]
            eb = [st.mean(err('C', s, pb) for s in r['samples']) for r in rb]
            paired[direction] = dict(
                err_0_05_mm=ea, err_0_3_mm=eb,
                diff_0_3_minus_0_05_mm=[round(y - x, 4) for x, y in zip(ea, eb)])
    out['paired_repro_errorC'] = paired

    # Reproducibility: are replayed per-pose errors identical to the source?
    same = {}
    for direction in ('down', 'up'):
        for force in ('0.05', '0.3'):
            rp = here / f'repro_{direction}_{force}.json'
            sp = src / f'taut_{direction}_{force},1.0.json'
            if rp.exists() and sp.exists():
                _, rr, rpre = load(rp)
                _, sr, spre = load(sp)
                re = [st.mean(err('C', s, rpre) for s in r['samples']) for r in rr]
                se = [st.mean(err('C', s, spre) for s in r['samples']) for r in sr]
                same[f'{direction}_{force}'] = dict(
                    max_abs_delta_mm=max(abs(x - y) for x, y in zip(re, se)),
                    source_err_mm=se, repro_err_mm=re)
    out['replay_vs_source'] = same

    with open(args.out, 'w') as f:
        json.dump(out, f, indent=1)

    def show(items):
        for k, v in items.items():
            print(f"{k:34s} dir={str(v['direction']):4s} F={v['force_n']:.2f} "
                  f"slack={v['slack_poses']}/{v['poses']} "
                  f"mean={v['err_mean_mm']:+.3f} max={v['err_max_mm']:+.3f} "
                  f"recMax={v['err_recovered_max_mm']:+.3f} "
                  f"fMin={v['force_min_n']:.3f} fMean={v['force_mean_n']:.3f} "
                  f"encStd={v['encoder_std_mm']:.4f} span={v['x_span_mm']:.2f}")

    for label in ('source', 'repro'):
        print(f'=== {label}')
        show(out[label])
    print('=== per-axis error, source taut_down_0.3')
    for axis, v in out['all_axis_source_taut_down_0.3'].items():
        print(f"  {axis}: mean={v['mean_mm']:+.3f} min={v['min_mm']:+.3f} max={v['max_mm']:+.3f} mm")
    print('=== replay vs source (per-pose C error)')
    for k, v in out['replay_vs_source'].items():
        print(f"  {k}: max|delta|={v['max_abs_delta_mm']:.6f} mm")
    print('=== paired repro C error per pose (mm)')
    for direction, v in out['paired_repro_errorC'].items():
        print(f"  {direction}: 0.05={[round(x,3) for x in v['err_0_05_mm']]}")
        print(f"         0.30={[round(x,3) for x in v['err_0_3_mm']]}")


if __name__ == '__main__':
    main()
