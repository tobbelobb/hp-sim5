"""k error of reduced probe protocols, from the full probe data of each run (refit.py JSON)."""
import json, sys
import numpy as np
print(f'{"run":8s} {"all":>7s} {"1x only":>8s} {"first 2 sweeps":>15s} {"first sweep, 1x":>16s}')
for path in sys.argv[1:]:
    r = json.load(open(path)); steps = [s for s in r['probe_steps'] if s['d_force'] > 0]
    def k(sel):
        a = np.array([s['a'] for s in sel]); b = np.array([s['b'] for s in sel]); m = np.array([s['measured'] for s in sel])
        return np.sum(b * (m - a)) / np.sum(b * b) / r['k_true'] - 1
    ones = steps[0::2]
    print(f"{r['run']:8s} {k(steps)*100:+6.1f}% {k(ones)*100:+7.1f}% {k(steps[:8])*100:+14.1f}% {k(ones[:2])*100:+15.1f}%")
