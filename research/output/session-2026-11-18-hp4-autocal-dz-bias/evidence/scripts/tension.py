import json, sys
import numpy as np
sys.path.insert(0, '/home/torbjorn/repos/hp-sim5'); sys.path.insert(0, '.')
from geom import HP4
from analyze import load
dataset_path, poses = sys.argv[1], sys.argv[2]
d, rows, home = load(dataset_path, poses)
m = d['config']['m669']; A = np.array([m[k] for k in 'ABCD'], float)
g = HP4('/home/torbjorn/repos/hp-sim5/public/usd_scenes/hp4.usda', [0, 0, 90.2957]); h = g.lengths(np.zeros(3), None)
EA = 20000.
reads = [json.loads(l) for l in open(poses)]
F0 = reads[0]['forces']
def axis_T(F):
    T = [np.mean([F[f'Joint{a}{s}_anchor_to_pinhole'] for s in 'LR']) for a in 'ABC']
    T.append(np.mean([F[f'JointD{i}_1'] for i in range(3)]))
    return np.array(T)
T0 = axis_T(F0)
print('home tensions per strand', T0.round(3))
out = []
for r in rows:
    e, q = r['e'], r['q']
    geo = np.array([(g.lengths(e, q)[k] - h[k]).mean() for k in 'ABCD'])
    resid = r['L'] - geo
    dist = np.linalg.norm(A - e, axis=1)
    T = axis_T(r['forces'])
    pred = (T * dist - T0 * np.linalg.norm(A, axis=1)) / EA
    out.append((r['sweep'], r['sensor'], resid, pred, T, dist))
res = np.array([o[2] for o in out]); pred = np.array([o[3] for o in out]); T = np.array([o[4] for o in out])
print('n', len(out))
print('resid mean', res.mean(0).round(3), 'pred mean', pred.mean(0).round(3))
print('resid-pred mean', (res - pred).mean(0).round(3), 'std', (res - pred).std(0).round(3))
print('corr per axis', [round(np.corrcoef(res[:, i], pred[:, i])[0, 1], 3) for i in range(4)])
print('strand tension per axis: mean', T.mean(0).round(3), 'std', T.std(0).round(3), 'min', T.min(0).round(3), 'max', T.max(0).round(3))
# weighted mean strand tension target check per sweep
for s in sorted(set(o[0] for o in out)):
    sel = [o for o in out if o[0] == s]
    Ts = np.array([o[4] for o in sel])
    print('sweep', s, 'sensor', sel[0][1], 'T mean per axis', Ts.mean(0).round(2), 'T std', Ts.std(0).round(2))
np.savez('tension.npz', res=res, pred=pred, T=T, sweep=np.array([o[0] for o in out]), dist=np.array([o[5] for o in out]))
