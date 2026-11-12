# Planned line tension recovered from CAN streams vs realized sim tension (HP4).
# flex_eff = (cmd_X - cmd_noflex) * r / MA  [effector-distance m]
# firmware: flex = -F / (k*MA*N_cfg), k = S/(MA*d + Y)  => F = -flex*k*MA*N_cfg ; physical line tension T = F/(MA*N)
import json, numpy as np
S, r = 20000., 0.03
MA = np.array([2, 2, 2, 4.]); N = np.array([2, 2, 2, 3.]); Y = np.array([.5283, .5283, .5283, .3989])
anch = np.array([[0, -1.9, -.28], [1.64544826719, .95, -.28], [-1.64544826719, .95, -.28], [0, 0, 1.9]])
nf = json.load(open('runs/t_noflex.json'))['sub']
groups = {'A': [0, 1], 'B': [2, 3], 'C': [4, 5], 'D': [6, 7, 8]}
for lab, ncfg in [('t_base', N), ('t_fix', N * MA)]:
    d = json.load(open(f'runs/{lab}.json'))['sub']
    n = min(len(d['cmd']), len(nf['cmd']))
    cmd = np.array(d['cmd'][:n]); cmd0 = np.array(nf['cmd'][:n])
    p = np.array(d['pos'][:n]); p = p - p[0]
    dist = np.linalg.norm(p[:, None, :] - anch[None], axis=2)
    k = S / (MA * dist + Y)
    flex = (cmd - cmd0) * r / MA
    F = -flex * k * MA * ncfg
    Tplan = F / (MA * N)
    ten = np.array(d['tension'][:n])
    Treal = np.stack([ten[:, g].mean(1) for g in groups.values()], 1)
    sl = slice(n // 10, n)  # skip start-up
    print(lab, 'flex mm mean', np.round(flex[sl].mean(0) * 1e3, 3))
    print('   planned F_axis N   ', np.round(F[sl].mean(0), 2), ' min', np.round(F[sl].min(0), 2), ' max', np.round(F[sl].max(0), 2))
    print('   T plan per line N  ', np.round(Tplan[sl].mean(0), 2))
    print('   T real per line N  ', np.round(Treal[sl].mean(0), 2))
    print('   ratio real/plan    ', np.round((Treal[sl] / Tplan[sl]).mean(0), 3), ' per-line real', np.round(ten[sl].mean(0), 2))
nT = np.array(nf['tension']); print('t_noflex T real per line', np.round(nT[len(nT)//10:].mean(0), 2))
