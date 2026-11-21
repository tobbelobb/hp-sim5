"""Attribute the compliance probe's k error with an exact replay of the run (replay_probe.mjs output).

usage: probe_replay_attrib.py RUN_DIR REPLAY.jsonl [--json OUT]

For every probe level the replay reads whose encoder angles match the level's mean are averaged.
Per line i (effector-distance units, U = mechanical advantage, paths = parallel lines N):
  F_i  effector-side line force = sum over long strands of their tension (strands reaching the effector)
  s_i  stretch  = mean over paths of (sum of segment distance - rest) / U
  g_i  geometric length = mean over paths of (sum of segment distance) / U
  m_i  encoder length (dataset) ; replay check: dm_i ~= d(g_i - s_i)
For each force step:
  r_i     = ds_i / (k_true c_i dF_i)          simulator compliance / probe line model (with true tensions)
  dF_s / dF_cmd                               sensor force realisation
  dF_held / dT_model                          held-line tension split (point-mass equilibrium)
  dg_i - (-u_i . dp_true)                     geometry not in the point model (rotation, attachment offsets)
and k re-estimated with true pieces substituted one at a time.
"""
import json, re, sys
from collections import defaultdict
from pathlib import Path
import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
import probe
from autocal.dataset_roles import normalize_dataset_point_roles

LONG = {  # segment names that run between frame and effector, per scene family
    'hp3': re.compile(r'^Joint([A-Z])(\w*?)_1$'),
    'hp5r_low': re.compile(r'^Joint([A-D])([LR])_(anchor_to_pinhole|pinhole_to_anchor)$'),
    'hp5r_top': re.compile(r'^Joint(I)(\d)_([1-4])$'),
}
PATH = re.compile(r'^Joint([A-Z])([A-Z]?\d?|[LR])_')


def line_paths(names):
    """{line letter: {path id: [joint names]}}, and the long-strand joints per line."""
    paths, long = defaultdict(lambda: defaultdict(list)), defaultdict(list)
    for name in names:
        m = re.match(r'^Joint([A-Z])([LR]|\d)_', name)
        if not m:
            continue
        paths[m[1]][m[2]].append(name)
        if any(p.match(name) for p in LONG.values()):
            long[m[1]].append(name)
    return paths, long


def state(reads, letters, advantage, paths, long):
    """Mean over matched reads: position (mm), quaternion, F, s, g per line."""
    pos = np.mean([r['pos'] for r in reads], axis=0) * 1000
    q = np.mean([r['q'] for r in reads], axis=0)
    F, s, g = [], [], []
    for i, letter in enumerate(letters):
        joints = lambda r, names: np.array([r['joints'][n] for n in names])
        F.append(np.mean([joints(r, long[letter])[:, 0].sum() for r in reads]))
        per_path_s, per_path_g = [], []
        for names in paths[letter].values():
            J = np.mean([joints(r, names) for r in reads], axis=0)
            per_path_s.append((J[:, 2] - J[:, 1]).sum() * 1000 / advantage[i])
            per_path_g.append(J[:, 2].sum() * 1000 / advantage[i])
        s.append(np.mean(per_path_s)); g.append(np.mean(per_path_g))
    return pos, q, np.array(F), np.array(s), np.array(g)


def main():
    run, replay = Path(sys.argv[1]), sys.argv[2]
    imp = json.loads((run / 'imperfection.json').read_text())
    ds = json.loads((run / 'sweeps.json').read_text()); ds.setdefault('machine_type', imp['machine']); normalize_dataset_point_roles(ds)
    truth = np.array(list(imp['truth_mm'].values())); letters = list(imp['truth_mm'])
    k_true = 1 / imp['stiffness_scale']
    S, N, U, Y = probe.line_description(ds)
    mm = np.asarray(ds['config']['mm_per_degree'], float)
    reads = [json.loads(line) for line in open(replay)]
    angles = np.array([r['angles_deg'] for r in reads])
    home = np.array(reads[0]['pos']) * 1000
    paths, long = line_paths(reads[0]['joints'])
    rows = []
    for sweep in ds['sweeps']:
        for pr in (sweep.get('metadata') or {}).get('compliance_probes', []):
            sensor = int(pr['sensor_anchor'])
            held = sorted({int(a) for a in sweep['fixed_anchors']} | {int(pr['drive_anchor'])})
            constant = [i for i in range(len(letters)) if i != sensor and i not in held]
            levels = []
            for level in pr['levels']:
                mu = np.asarray(level['mu_deg'], float)
                err = np.max(np.abs(angles - mu), axis=1)
                for tol in (0.005, 0.01, 0.02, 0.05):
                    hit = np.flatnonzero(err < tol)
                    if len(hit):
                        break
                levels.append((level, mu, hit, tol, state([reads[j] for j in hit], letters, U, paths, long) if len(hit) else None))
            if any(l[4] is None for l in levels):
                continue
            pos0, q0, F0, s0, g0 = levels[0][4]
            p0 = pos0 - home
            to = truth - p0; dist = np.linalg.norm(to, axis=1); u = to / dist[:, None]
            c = (U * dist + Y) / (S * U ** 2 * N)
            for j in (1, 2):
                level, mu, hit, tol, (pos, q, F, s, g) = levels[j]
                dF_cmd = level['force_n'] - levels[0][0]['force_n']
                dm = (mu - levels[0][1]) * mm
                dF, dsr, dg = F - F0, s - s0, g - g0
                dp = pos - pos0
                dT_model = np.zeros(len(letters)); dT_model[held] = np.linalg.solve(u[held].T, -dF_cmd * u[sensor])
                rot = 2 * np.degrees(np.arcsin(min(1.0, np.linalg.norm((q - q0)[:3]))))
                # k with true pieces: sensor equation  dm_s = dg_s - ds_s ; probe models dg_s via held lines and ds_s = k c_s dF
                def k_est(d_tension, d_force, comp_scale, geometry_true):
                    rows_ = held + constant
                    basis = u[rows_]
                    if geometry_true:
                        a = dg[sensor]  # true geometric change of the sensor line
                        b_held = 0.0
                    else:
                        p_ = np.linalg.lstsq(basis, -dm[rows_], rcond=None)[0]
                        qv = np.linalg.lstsq(basis, -(comp_scale * c)[rows_] * d_tension[rows_], rcond=None)[0]
                        a = -u[sensor] @ p_; b_held = -u[sensor] @ qv
                    b = b_held - comp_scale[sensor] * c[sensor] * d_force
                    return (dm[sensor] - a) / b
                ones = np.ones(len(letters))
                dT_true = dF.copy(); dT_true[constant] = 0.0
                r = np.where(np.abs(dF) > 1e-3, dsr / (k_true * c * np.where(np.abs(dF) > 1e-3, dF, 1)), np.nan)
                ratio = np.where(np.isfinite(r), r, 1.0)
                rows.append(dict(
                    sensor=letters[sensor], level=j, z=float(p0[2]), dF_cmd=dF_cmd, matched=(len(levels[0][2]), len(hit)), tol=tol,
                    replay_check_mm=float(np.max(np.abs(dm - (dg - dsr)))),
                    sensor_force_ratio=float(dF[sensor] / dF_cmd),
                    held_tension_ratio={letters[i]: round(float(dF[i] / dT_model[i]), 4) for i in held},
                    constant_dF={letters[i]: round(float(dF[i]), 4) for i in constant},
                    compliance_ratio={letters[i]: round(float(r[i]), 4) for i in range(len(letters)) if np.isfinite(r[i])},
                    geometry_residual_mm={letters[i]: round(float(dg[i] - (-u[i] @ dp)), 4) for i in range(len(letters))},
                    rotation_deg=rot,
                    k_probe=k_est(dT_model, dF_cmd, ones, False) / k_true,
                    k_true_tensions=k_est(dT_true, dF[sensor], ones, False) / k_true,
                    k_all_true_tensions=k_est(dF, dF[sensor], ones, False) / k_true,
                    k_true_geometry=k_est(dT_model, dF_cmd, ones, True) / k_true,
                    k_true_geometry_tension=k_est(dT_true, dF[sensor], ones, True) / k_true))
    out = dict(run=run.name, k_true=k_true, steps=rows)
    keys = ('k_probe', 'k_true_tensions', 'k_all_true_tensions', 'k_true_geometry', 'k_true_geometry_tension', 'sensor_force_ratio')
    print(f"{run.name}: {len(rows)} steps; medians (ratio to k_true):", {k: round(float(np.median([r[k] for r in rows])), 4) for k in keys})
    print('  replay check max |dm - d(g - s)| mm:', round(max(r['replay_check_mm'] for r in rows), 4))
    for letter in letters:
        vals = [r['compliance_ratio'][letter] for r in rows if letter in r['compliance_ratio']]
        held = [r['held_tension_ratio'][letter] for r in rows if letter in r['held_tension_ratio']]
        geo = [r['geometry_residual_mm'][letter] for r in rows]
        print(f'  line {letter}: compliance sim/model median {np.median(vals):.4f} (n {len(vals)}), held dT true/model median '
              f'{np.median(held) if held else float("nan"):.4f}, geometry residual median {np.median(geo):+.4f} mm')
    if '--json' in sys.argv:
        Path(sys.argv[sys.argv.index('--json') + 1]).write_text(json.dumps(out, indent=1, default=float))


if __name__ == '__main__':
    main()
