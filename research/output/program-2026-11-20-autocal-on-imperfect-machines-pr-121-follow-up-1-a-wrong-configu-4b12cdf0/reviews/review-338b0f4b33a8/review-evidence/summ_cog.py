"""Summarise cog_refit outputs: usage summ_cog.py FILES..."""
import json, sys
import numpy as np
rows = [json.load(open(f)) for f in sys.argv[1:]]
for m in sorted({r['machine'] for r in rows}):
    R = [r for r in rows if r['machine'] == m]
    for v in [k for k in R[0] if isinstance(R[0][k], dict)]:
        z = np.array([r[v]['top_z_mm'] for r in R]); mx = np.array([r[v]['max_abs_mm'] for r in R])
        print(f'{m:30s} n={len(R)} {v:28s} top z {z.mean():+.2f} ± {z.std():.2f} (|z| {abs(z).mean():.2f})  max {mx.mean():.2f} ± {mx.std():.2f} worst {mx.max():.2f}')
