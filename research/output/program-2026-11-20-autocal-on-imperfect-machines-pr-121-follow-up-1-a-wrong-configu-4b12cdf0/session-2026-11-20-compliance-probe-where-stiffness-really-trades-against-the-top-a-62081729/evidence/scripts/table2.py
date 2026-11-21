"""Markdown tables (per run and per case spread) from results/live/*_refit.json.  usage: table2.py RESULTS_DIR"""
import glob, json, sys
import numpy as np

runs = [json.load(open(p)) for p in sorted(glob.glob(f'{sys.argv[1]}/*_refit.json'))]
V = ('config', 'probe', 'joint', 'oracle')
print('| Run | k true | k probe | k joint | top z: config / probe / joint / oracle | max: config / probe / joint / oracle | probe s / sim s |')
print('|---|---|---|---|---|---|---|')
for d in runs:
    kt = d['k_true']
    print(f"| {d['run']} | {kt:.3f} | {d['k_probe']:.3f} ({d['k_probe'] / kt - 1:+.1%}) | {d['k_joint']:.3f} ({d['k_joint'] / kt - 1:+.1%}) | "
          + ' / '.join(f"{d[v]['top_z_frame_mm']:+.2f}" for v in V) + ' | ' + ' / '.join(f"{d[v]['max_abs_frame_mm']:.2f}" for v in V)
          + f" | {d['probe_s']:.0f} / {d['live']['simulated_s']:.0f} ({d['probe_s'] / d['live']['simulated_s']:.1%}) |")
print()
print('| Case | n | k error probe | k error joint | variant | top z mean ± sd | abs top z mean (worst) | max mean ± sd (worst) |')
print('|---|---|---|---|---|---|---|---|')
for case in sorted({d['run'].split('-')[0] for d in runs}):
    group = [d for d in runs if d['run'].startswith(case + '-')]
    kp = np.array([d['k_probe'] / d['k_true'] - 1 for d in group]) * 100
    kj = np.array([d['k_joint'] / d['k_true'] - 1 for d in group]) * 100
    for v in V:
        z = np.array([d[v]['top_z_frame_mm'] for d in group]); m = np.array([d[v]['max_abs_frame_mm'] for d in group])
        print(f"| {case} | {len(group)} | {kp.mean():+.1f} ± {kp.std(ddof=1) if len(kp) > 1 else 0:.1f}% | {kj.mean():+.1f} ± {kj.std(ddof=1) if len(kj) > 1 else 0:.1f}% | {v} | "
              f"{z.mean():+.2f} ± {z.std(ddof=1) if len(z) > 1 else 0:.2f} | {np.abs(z).mean():.2f} ({np.abs(z).max():.2f}) | {m.mean():.2f} ± {m.std(ddof=1) if len(m) > 1 else 0:.2f} ({m.max():.2f}) |")
