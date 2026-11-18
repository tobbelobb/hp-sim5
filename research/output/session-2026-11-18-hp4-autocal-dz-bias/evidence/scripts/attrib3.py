import sys, numpy as np, json
sys.argv = [sys.argv[0], sys.argv[1], 'full.npz']
exec(open('attrib.py').read().split("if __name__")[0])
from analyze import load
d_, rows, home = load(sys.argv[1], 'hp4_poses.jsonl')
g = HP4('/home/torbjorn/repos/hp-sim5/public/usd_scenes/hp4.usda', home); h = g.lengths(np.zeros(3), None)
F0 = json.loads(open('hp4_poses.jsonl').readline())['forces']
def lineT(F, ax):
    return np.array([F[f'Joint{ax}{s}_anchor_to_pinhole'] for s in 'LR']) if ax != 'D' else np.array([F[f'JointD{i}_1'] for i in range(3)])
taut_geo, taut_full, mean_geo = [], [], []
for r in rows:
    Lg = g.lengths(r['e'], r['q']); a, b, c = [], [], []
    for ax in 'ABCD':
        T = lineT(r['forces'], ax); T0 = lineT(F0, ax); j = np.argmax(T)
        geo = Lg[ax] - h[ax]; s = (T * Lg[ax] - T0 * h[ax]) / 20000.
        a.append(geo[j]); b.append(geo[j] - s[j]); c.append(geo.mean())
    taut_geo.append(a); taut_full.append(b); mean_geo.append(c)
L = z['L']
fit(np.array(mean_geo), 'mean-of-lines geometry (rotation, no stretch)')
fit(np.array(taut_geo), 'taut-line geometry (rotation, no stretch)')
fit(np.array(taut_full), 'taut-line geometry + its stretch (model)')
fit(L, 'measured')
tg = np.array(taut_geo); tg[:, 3] = z['model'][:, 3]
fit(tg, 'taut-line geometry on ABC, ideal D')
tg = z['model'].copy(); tg[:, 3] = np.array(taut_geo)[:, 3]
fit(tg, 'ideal ABC, taut-line geometry on D')
