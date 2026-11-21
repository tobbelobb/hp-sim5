"""Per run: the k where top-z error crosses zero (from the refit's rms scan), and the D.z slope per unit k."""
import glob, json, sys
import numpy as np
for p in sorted(glob.glob(f'{sys.argv[1]}/*_refit.json')):
    d = json.load(open(p)); k = np.array([r['k'] for r in d['rms_scan']]); z = np.array([r['top_z_frame_mm'] for r in d['rms_scan']])
    slope = np.polyfit(k, z, 1)[0]; k0 = float(np.interp(0, z if slope > 0 else z[::-1], k if slope > 0 else k[::-1]))
    print(f"{d['run']}: dDz/dk {slope:+.2f} mm, k(top z = 0) / k_true {k0 / d['k_true']:.3f}, probe {d['k_probe'] / d['k_true']:.3f}, joint {d['k_joint'] / d['k_true']:.3f}")
