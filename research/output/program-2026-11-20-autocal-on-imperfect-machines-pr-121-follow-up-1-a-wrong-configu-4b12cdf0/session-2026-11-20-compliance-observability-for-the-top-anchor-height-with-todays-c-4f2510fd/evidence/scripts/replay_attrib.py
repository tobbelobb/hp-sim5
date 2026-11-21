"""Match an HP3 dataset's points to replayed true poses and forces; attribute the anchor error.

usage: replay_attrib.py sweeps.json replay.jsonl [stiffness_n]

Prints: measured length minus rigid point model per line, the stretch the replay's
strand tensions predict with T*(d+Y)/EA, and production refits on substituted lengths.
"""
import json, sys
import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, __file__.rsplit('/', 1)[0])
import synth
from autocal.pose_fit import dataset_line_lengths

STRANDS = {'A': ['AL', 'AR'], 'B': ['BL', 'BR'], 'C': ['CL', 'CR'], 'D': ['D0', 'D1', 'D2']}


def axis_tension(forces):
    return np.array([np.mean([forces[f'Joint{s}_1'] for s in STRANDS[a]]) for a in 'ABCD'])


def match(dataset, reads):
    tree = cKDTree(np.array([r['angles_deg'] for r in reads]))
    home = np.array(reads[0]['pos']) * 1000
    pts = [p for s in dataset['sweeps'] for p in s['data_points']]
    idx = []
    for p in pts:
        dist, j = tree.query(p['raw_angles_deg'], p=np.inf)
        idx.append(j if dist <= 0.0051 else -1)
    return np.array(idx), home


def main():
    ds = synth.load(sys.argv[1])
    reads = [json.loads(l) for l in open(sys.argv[2])]
    ea = float(sys.argv[3]) if len(sys.argv) > 3 else 20000.0
    idx, home = match(ds, reads)
    assert np.all(idx >= 0), f'{np.sum(idx < 0)} unmatched points'
    # The encoder origin: the last 4-axis read with all angles ~0 before the first sweep point.
    first = idx.min()
    zero = max(j for j in range(first) if np.max(np.abs(reads[j]['angles_deg'])) < 0.05)
    origin = np.array(reads[zero]['pos']) * 1000
    T0 = axis_tension(reads[zero]['forces'])
    print(f'home (first read) {home.round(3)}; encoder-zero read #{zero} pos {origin.round(3)}; strand tensions there {T0.round(3)} N')
    E = np.array([reads[j]['pos'] for j in idx]) * 1000 - origin
    Q = np.array([reads[j]['q'] for j in idx])
    T = np.array([axis_tension(reads[j]['forces']) for j in idx])
    L = dataset_line_lengths(ds)['lengths_mm']
    A = synth.TRUTH
    n, u, y, w = synth.line_params(ds)
    d = np.linalg.norm(A[None] - E[:, None], axis=2)
    d0 = np.linalg.norm(A, axis=1)
    model = d - d0
    res = L - model
    pred = -(T * (d + y) - T0 * (d0 + y)) / ea
    rot = 2 * np.degrees(np.arcsin(np.linalg.norm(Q[:, :3], axis=1)))
    print(f'points {len(L)}; effector rotation median {np.median(rot):.4f} max {rot.max():.4f} deg')
    print('strand tension per axis mean', T.mean(0).round(3), 'min', T.min(0).round(3), 'max', T.max(0).round(3))
    print('measured - rigid model  mean', res.mean(0).round(4), 'std', res.std(0).round(4))
    print('stretch predicted       mean', pred.mean(0).round(4), 'std', pred.std(0).round(4))
    print('difference              mean', (res - pred).mean(0).round(4), 'std', (res - pred).std(0).round(4))
    print('corr(res, pred) per axis', [round(float(np.corrcoef(res[:, i], pred[:, i])[0, 1]), 3) for i in range(4)])
    # fit EA per axis: res ~ a_i + pred_i * (20000/EA_i)
    for i in range(4):
        X = np.column_stack([np.ones(len(L)), pred[:, i]])
        c = np.linalg.lstsq(X, res[:, i], rcond=None)[0]
        print(f'  axis {"ABCD"[i]}: res = {c[0]:+.4f} + {c[1]:.3f} * pred  (EA_eff ~ {ea / c[1] if c[1] else np.inf:.0f} N)')
    np.savez(sys.argv[2].replace('.jsonl', '.npz'), E=E, Q=Q, T=T, T0=T0, L=L, res=res, pred=pred, model=model)

    def refit(lengths, label):
        syn = substitute(ds, lengths)
        fit, off = synth.production_fit(syn)
        e = synth.errors(fit['anchors'])
        print(f'{label:46s} max {e["max_coord"]:.3f} Dz {e["Dz"]:+.3f} lowz {e["low_z"]:+.3f} offD {off[3]:+.3f} rms {fit["rms_mm"]:.4f}')
    refit(L, 'measured')
    refit(model, 'true rigid lengths (from encoder-zero pose)')
    refit(model + pred, 'rigid + predicted stretch')
    refit(L - pred, 'measured - predicted stretch')
    for k in range(4):
        p = np.zeros_like(pred); p[:, k] = pred[:, k]
        refit(L - p, f'measured - predicted stretch on {"ABCD"[k]} only')
    refit(L - (pred - pred.mean(0)), 'measured - (stretch - its mean)')
    refit(L - pred.mean(0), 'measured - mean stretch only')


def substitute(ds, lengths):
    import copy
    out = copy.deepcopy(ds)
    k = 0
    mm = ds['config']['mm_per_degree']
    for sweep in out['sweeps']:
        fixed = [int(a) for a in sweep['fixed_anchors']]
        dr, se = int(sweep['drive_anchor']), int(sweep['sensor_anchor'])
        sweep['fixed_lengths'] = [float(lengths[k, a]) for a in fixed]
        for point in sweep['data_points']:
            point['fixed_error_angles_deg'] = [None] * 4
            for a, base in zip(fixed, sweep['fixed_lengths']):
                point['fixed_error_angles_deg'][a] = (lengths[k, a] - base) / mm[a]
            point['fixed_error_angles_mu_deg'] = list(point['fixed_error_angles_deg'])
            point['l_drive'] = point['l_drive_mu'] = float(lengths[k, dr])
            point['l_sensor'] = point['l_sensor_mu'] = float(lengths[k, se])
            k += 1
    return out


if __name__ == '__main__':
    main()
