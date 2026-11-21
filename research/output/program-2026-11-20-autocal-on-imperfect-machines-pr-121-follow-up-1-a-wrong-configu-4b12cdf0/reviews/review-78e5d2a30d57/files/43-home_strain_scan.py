"""HP4-style ABC home-strain offsets on HP3 data: anchor error and fit rms vs assumed compliance.

usage: home_strain_scan.py sweeps.json [replay.npz true_compliance_scale ...]
With a replay npz, lengths are regenerated from the replay's true poses and tensions
with stretch scaled by each true compliance scale (synthetic, exact model);
otherwise the recorded lengths are used.
"""
import sys, copy
import numpy as np
sys.path.insert(0, __file__.rsplit('/', 1)[0])
import synth
import autocal.pose_fit as pf
from replay_attrib import substitute

ds = synth.load(sys.argv[1])
ds['machine_type_for_strain'] = 'hangprinter_3'
n, u, y, w = synth.line_params(ds)
target = float(ds['sweeps'][0]['metadata']['strand_tension_n'])
original = pf._hp4_home_strain


def run(data, assumed):
    """assumed: compliance multiple relative to configured S (None = no home strain)."""
    pf._hp4_home_strain = (lambda d: None) if assumed is None else (lambda d: (assumed * target / 20000.0, y[:3] / u[:3]))
    try:
        fit, off = synth.production_fit(data)
    finally:
        pf._hp4_home_strain = original
    return fit, off


datasets = [('recorded', ds)]
if len(sys.argv) > 2:
    z = np.load(sys.argv[2])
    A = synth.TRUTH
    d = np.linalg.norm(A[None] - z['E'][:, None], axis=2); d0 = np.linalg.norm(A, axis=1)
    for s in map(float, sys.argv[3:]):
        L = (d - s * z['T'] * (d + y) / 20000) - (d0 - s * z['T0'] * (d0 + y) / 20000)
        datasets.append((f'replay-synthetic true compliance x{s:g}', substitute(ds, L)))
for label, data in datasets:
    print(f'== {label} (strand target {target:.3f} N)')
    for assumed in [None, 0.0, 0.6, 0.8, 0.9, 1.0, 1.1, 1.2, 1.25, 1.4, 1.6]:
        fit, off = run(data, assumed)
        e = synth.errors(fit['anchors'])
        print(f'  assumed compliance {"none" if assumed is None else f"x{assumed:.2f}"}: max {e["max_coord"]:.3f} Dz {e["Dz"]:+.3f} lowz {e["low_z"]:+.3f} '
              f'off {np.round(off, 3).tolist()} rms {fit["rms_mm"]:.5f}')
