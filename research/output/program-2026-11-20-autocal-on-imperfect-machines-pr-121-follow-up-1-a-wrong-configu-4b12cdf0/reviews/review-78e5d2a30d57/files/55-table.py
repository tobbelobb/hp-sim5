"""Markdown table and spread over refit.py outputs (convention-frame errors, mm)."""
import json, sys
import numpy as np
rows = [json.load(open(p)) for p in sys.argv[1:]]
print('| Run | EA scale | k true | k probe | D.z today | D.z config S | D.z probe | D.z oracle | low z today | low z config | low z probe | max today | max probe | probe time | sim time |')
print('|' + '---|' * 15)
for r in rows:
    t = lambda v, key: r[v][key]
    print(f"| {r['run']} | {r['true_stiffness_scale']:.2f} | {r['k_true']:.3f} | {r['k_probe']:.3f} ({(r['k_probe']/r['k_true']-1)*100:+.1f}%) | "
          f"{t('today','Dz_frame_mm'):+.2f} | {t('config','Dz_frame_mm'):+.2f} | {t('probe','Dz_frame_mm'):+.2f} | {t('oracle','Dz_frame_mm'):+.2f} | "
          f"{t('today','low_z_frame_mm'):+.2f} | {t('config','low_z_frame_mm'):+.2f} | {t('probe','low_z_frame_mm'):+.2f} | "
          f"{t('today','max_abs_frame_mm'):.2f} | {t('probe','max_abs_frame_mm'):.2f} | {r.get('probe_s', float('nan')):.0f} s | {r['simulated_s']:.0f} s |")
print()
def stat(v, key, f=lambda r, x: x):
    x = np.array([f(r, r[v][key]) for r in rows]); return f'mean {x.mean():+.3f} sd {x.std(ddof=1):.3f} max|.| {np.abs(x).max():.3f}'
kerr = np.array([r['k_probe'] / r['k_true'] - 1 for r in rows])
print(f'k probe error: mean {kerr.mean()*100:+.1f}% sd {kerr.std(ddof=1)*100:.1f}% range {kerr.min()*100:+.1f}..{kerr.max()*100:+.1f}%  (n={len(rows)})')
for v in ('today', 'config', 'probe', 'oracle'):
    print(f'{v:7s} D.z: {stat(v, "Dz_frame_mm")}; low z: {stat(v, "low_z_frame_mm")}; max: {stat(v, "max_abs_frame_mm")}')
for v in ('today', 'config', 'probe'):
    d = np.array([r[v]['Dz_frame_mm'] - r['oracle']['Dz_frame_mm'] for r in rows])
    print(f'{v:7s} D.z minus oracle D.z: mean {d.mean():+.3f} sd {d.std(ddof=1):.3f} max|.| {np.abs(d).max():.3f}')
