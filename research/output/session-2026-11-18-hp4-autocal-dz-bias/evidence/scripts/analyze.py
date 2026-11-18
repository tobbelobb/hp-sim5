"""Match dataset points to replayed true poses; compare measured lengths with the point model."""
import json, sys
import numpy as np
sys.path.insert(0, '/home/torbjorn/repos/hp-sim5')
from autocal.pose_fit import dataset_line_lengths

def load(dataset_path, poses_path):
    d = json.load(open(dataset_path))
    reads = [json.loads(l) for l in open(poses_path)]
    home = np.array(reads[0]['pos']) * 1000
    from scipy.spatial import cKDTree
    tree = cKDTree(np.array([r['angles_deg'] for r in reads]))
    L = dataset_line_lengths(d)['lengths_mm']
    pts = [(si, s, p) for si, s in enumerate(d['sweeps']) for p in s['data_points']]
    rows = []
    for k, (si, s, p) in enumerate(pts):
        dist, j = tree.query(p['raw_angles_deg'], p=np.inf)
        if dist > 0.0051: continue
        r = reads[j]
        rows.append(dict(k=k, sweep=si, e=np.array(r['pos']) * 1000 - home, q=np.array(r['q']), L=L[k], forces=r['forces'],
                         sensor=s['sensor_anchor'], drive=s['drive_anchor'], fixed=s['fixed_anchors'], F=p.get('sensor_force_n')))
    return d, rows, home

if __name__ == '__main__':
    d, rows, home = load(sys.argv[1], sys.argv[2])
    m = d['config']['m669']; A = np.array([m[k] for k in 'ABCD'], float)
    print('matched', len(rows), 'home', home)
    E = np.array([r['e'] for r in rows]); L = np.array([r['L'] for r in rows]); Q = np.array([r['q'] for r in rows])
    model = np.linalg.norm(A[None] - E[:, None], axis=2) - np.linalg.norm(A, axis=1)[None]
    res = L - model
    print('effector range mm min', E.min(0).round(1), 'max', E.max(0).round(1))
    ang = 2 * np.degrees(np.arcsin(np.linalg.norm(Q[:, :3], axis=1)))
    print('effector rotation deg: median %.4f max %.4f' % (np.median(ang), ang.max()))
    print('residual (measured - point model) per axis mm: mean', np.nanmean(res, 0).round(3), 'std', np.nanstd(res, 0).round(3),
          'min', np.nanmin(res, 0).round(3), 'max', np.nanmax(res, 0).round(3))
    np.savez(sys.argv[3] if len(sys.argv) > 3 else 'matched.npz', E=E, L=L, Q=Q, res=res, model=model,
             sweep=np.array([r['sweep'] for r in rows]), sensor=np.array([r['sensor'] for r in rows]),
             drive=np.array([r['drive'] for r in rows]), F=np.array([r['F'] or np.nan for r in rows]), k=np.array([r['k'] for r in rows]))
