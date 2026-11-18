import json, sys, numpy as np
sys.path.insert(0, '.')
from geom import HP4
from analyze import load
d, rows, home = load(sys.argv[1], 'hp4_poses.jsonl')
g = HP4('/home/torbjorn/repos/hp-sim5/public/usd_scenes/hp4.usda', home); h = g.lengths(np.zeros(3), None)
F0 = json.loads(open('hp4_poses.jsonl').readline())['forces']
def lineT(F, ax):
    return np.array([F[f'Joint{ax}{s}_anchor_to_pinhole'] for s in 'LR']) if ax != 'D' else np.array([F[f'JointD{i}_1'] for i in range(3)])
out = {ax: [] for ax in 'ABCD'}
for r in rows:
    Lg = g.lengths(r['e'], r['q'])
    for i, ax in enumerate('ABCD'):
        geo = Lg[ax] - h[ax]          # per line, anchor units
        T = lineT(r['forces'], ax); T0 = lineT(F0, ax)
        out[ax].append((r['L'][i], geo, T, T0, Lg[ax], r['sweep']))
for ax in 'ABCD':
    meas = np.array([o[0] for o in out[ax]]); geo = np.array([o[1] for o in out[ax]]); T = np.array([o[2] for o in out[ax]])
    T0 = out[ax][0][3]; Lab = np.array([o[4] for o in out[ax]]); hh = h[ax]
    # per-line stretch prediction (anchor units): T*len/EA_eff - T0*len0/EA_eff
    for EA in (20000., 10000.):
        stretch = (T * Lab - T0 * hh) / EA
        pred = geo - stretch          # payout predicted by each line
        err = meas[:, None] - pred
        taut = np.argmax(T, axis=1)
        e_taut = err[np.arange(len(err)), taut]
        e_mean = meas - pred.mean(1)
        print(ax, 'EA', EA, 'per-line err std', err.std(0).round(3), 'taut-line err mean %.3f std %.3f' % (e_taut.mean(), e_taut.std()),
              '| mean-of-lines err std %.3f' % e_mean.std(), 'min T', T.min().round(3), 'frac pts with a line <0.1N %.2f' % (T.min(1) < 0.1).mean())
print('--- centre point model (what autocal assumes) + mean stretch, split by slackness of the weaker parallel line')
m669 = d['config']['m669']; A = np.array([m669[k] for k in 'ABCD'], float)
for i, ax in enumerate('ABCD'):
    meas = np.array([o[0] for o in out[ax]]); T = np.array([o[2] for o in out[ax]]); T0 = out[ax][0][3]
    E = np.array([r['e'] for r in rows]); dist = np.linalg.norm(A[i] - E, axis=1)
    pm = dist - np.linalg.norm(A[i])
    err = meas - (pm - (T.mean(1) * dist - T0.mean() * np.linalg.norm(A[i])) / 20000.)
    tmin = T.min(1); ratio = T.min(1) / T.max(1)
    for lo, hi in [(0, .1), (.1, .3), (.3, 9)]:
        s = (tmin >= lo) & (tmin < hi)
        if s.any(): print(ax, f'weaker line T in [{lo},{hi})  n={s.sum():3d}  err mean {err[s].mean():+.3f} std {err[s].std():.3f}  min/max ratio {ratio[s].mean():.2f}')
    print(ax, 'corr(mean T, min/max ratio)', np.corrcoef(T.mean(1), ratio)[0, 1].round(2))
