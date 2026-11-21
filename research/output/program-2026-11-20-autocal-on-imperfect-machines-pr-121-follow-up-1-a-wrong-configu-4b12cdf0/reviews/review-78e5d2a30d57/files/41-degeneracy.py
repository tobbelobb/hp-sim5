"""D.z of the production fit on synthetic HP3 data vs true stiffness scale."""
import sys, numpy as np
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from synth import *
ds = load(sys.argv[1])
pos = true_poses(ds)
rec, _ = production_fit(ds)
print("recorded sim data:", errors(rec["anchors"]))
for scale in [1e9, 2.0, 1.2, 1.0, 0.8, 0.5]:
    syn, t = synthesize(ds, pos, scale)
    fit, off = production_fit(syn)
    e = errors(fit["anchors"])
    print(f"S x{scale:g}: max {e['max_coord']:.3f} Dz {e['Dz']:+.3f} lowz {e['low_z']:+.3f} offD {off[3]:+.3f} rms {fit['rms_mm']:.4f}  tension range {t.min():.2f}..{t.max():.2f} N")
