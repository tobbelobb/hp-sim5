"""One-line-per-variant view of refit.py outputs."""
import json, sys
import numpy as np
for path in sys.argv[1:]:
    r = json.load(open(path))
    steps = [s for s in r['probe_steps'] if s['d_force'] > 0]
    print(f"{r['run']}: k_true {r['k_true']:.3f} k_probe {r['k_probe']:.3f} ({(r['k_probe']/r['k_true']-1)*100:+.1f}%), "
          f"per-step k {np.min([s['k'] for s in steps]):.2f}..{np.max([s['k'] for s in steps]):.2f}, sim {r['simulated_s']:.0f} s")
    for v in ('today', 'config', 'probe', 'oracle'):
        x = r[v]
        print(f"   {v:7s} max gauge {x['max_abs_gauge_mm']:.3f}  D.z {x['Dz_gauge_mm']:+.3f}  low z {x['low_z_gauge_mm']:+.3f}  D.z-lowz {x['Dz_gauge_mm']-x['low_z_gauge_mm']:+.3f}")
