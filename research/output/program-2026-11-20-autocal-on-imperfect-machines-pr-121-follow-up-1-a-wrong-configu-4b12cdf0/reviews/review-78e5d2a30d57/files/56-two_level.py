"""Does repeating the sweeps at a second strand tension identify the compliance scale k?

Synthetic, exact RRF flex model: recorded HP3 poses (replay), equilibrium tensions from
the recorded sensor forces (level 1) and the same forces x2 (level 2), true k = 1.25.
Both levels share the encoder origin (ABC slack, so ABC home strain is k*T_level/S*(d0+Y)).
For each trial k the level-2 ABC lengths are shifted by the home-strain difference, both
levels are fitted jointly with home strain at level 1, and the fit rms and errors are printed.
"""
import sys, copy
import numpy as np
sys.path.insert(0, __file__.rsplit('/', 1)[0])
import synth
import autocal.pose_fit as pf
from replay_attrib import substitute

ds = synth.load(sys.argv[1]); z = np.load(sys.argv[2]); k_true = 1.25
n, u, y, w = synth.line_params(ds); S = 20000.0
A = synth.TRUTH; E = z['E']; d = np.linalg.norm(A[None] - E[:, None], axis=2); d0 = np.linalg.norm(A, axis=1)
s, F = synth.sensor_lines(ds), synth.sensor_forces(ds)
target = float(ds['sweeps'][0]['metadata']['strand_tension_n'])
levels = []
for m in (1.0, 2.0):
    t = synth.equilibrium_tensions(A, E, s, F * m, w) / n
    levels.append((m, substitute(ds, (d - k_true * t * (d + y) / S) - d0)))
orig = pf._hp4_home_strain
for k in [1.0, 1.1, 1.2, 1.25, 1.3, 1.4, 1.5]:
    second = copy.deepcopy(levels[1][1])
    shift = k * target * (2.0 - 1.0) / S * (d0[:3] + y[:3])
    for sweep in second['sweeps']:
        fixed = [int(a) for a in sweep['fixed_anchors']]
        sweep['fixed_lengths'] = [v + (shift[a] if a < 3 else 0) for a, v in zip(fixed, sweep['fixed_lengths'])]
        for p in sweep['data_points']:
            for key, a in (('l_drive', sweep['drive_anchor']), ('l_sensor', sweep['sensor_anchor'])):
                if a < 3:
                    p[key] += shift[a]; p[key + '_mu'] = p[key]
    joint = copy.deepcopy(levels[0][1]); joint['sweeps'] += second['sweeps']
    pf._hp4_home_strain = lambda _d: (k * target / S, y[:3] / u[:3])
    try:
        fit, off = synth.production_fit(joint)
    finally:
        pf._hp4_home_strain = orig
    e = synth.errors(fit['anchors'])
    print(f'trial k {k:.2f}: rms {fit["rms_mm"]:.5f} max {e["max_coord"]:.3f} Dz {e["Dz"]:+.3f} lowz {e["low_z"]:+.3f}')
