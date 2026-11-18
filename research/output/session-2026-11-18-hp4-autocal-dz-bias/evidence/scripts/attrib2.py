import sys, numpy as np
sys.argv = [sys.argv[0], sys.argv[1], 'full.npz']
exec(open('attrib.py').read().split("if __name__")[0])
t = np.load('tension.npz'); L = z['L']; T = t['T']; dist = t['dist']; EA = 20000.
g = HP4('/home/torbjorn/repos/hp-sim5/public/usd_scenes/hp4.usda', [0, 0, 90.2957]); h = g.lengths(np.zeros(3), None)
rot = np.array([[(g.lengths(e, q)[k] - h[k]).mean() for k in 'ABCD'] for e, q in zip(z['E'], z['Q'])])
fit(L, 'measured (all 256)')
fit(z['model'], 'truth')
for name, X in [('T*d', T * dist / EA), ('T', T / EA)]:
    corr = np.zeros_like(L)
    for i in range(4):
        A_ = np.column_stack([X[:, i], np.ones(len(L))]); c, *_ = np.linalg.lstsq(A_, (L - rot)[:, i], rcond=None)
        corr[:, i] = c[0] * X[:, i]
    fit(L - corr, f'measured - fitted per-axis {name} stretch')
    c2 = corr.copy(); c2[:, 3] = 0
    fit(L - c2, f'measured - fitted {name} stretch, ABC only')
    c3 = np.zeros_like(corr); c3[:, 3] = corr[:, 3]
    fit(L - c3, f'measured - fitted {name} stretch, D only')
