#!/usr/bin/env python3
"""Constrained calibration ablation for COPacabana (DaRUS-4614).

Progressively relaxes model parameters while training ONLY on `20_Std_Load13900`
PlaneA (P1..P5, 5 holds) and evaluating blind on:
  1. All 7 held-out PlaneA experiments (35 holds across 13.9 & 29.1 kg, STD/APC25/50/75),
  2. Both held-out `CircPath` trajectories (`20_Std_Load13900` and `10_Std_Load29100`,
     24 evenly spaced active trajectory points each = 48 points total).

Objective & cheat-proof force metric:
  - Position residual: 3D bottom-rail reflector displacement relative to P0, weighted
    at 1.0 mm per residual unit (`dp_err * 1000.0`).
  - Force residual: ALWAYS evaluated in logged-sensor units (`F_pred_act / g_f - f_log`),
    weighted at 12.0 N per residual unit (`f_err_log / 12.0`), so the optimizer cannot
    artificially shrink force error by reducing `g_f`.
  - Parameter bounds (11-vector `[EA, g_f, eta_lo, eta_up, d0x, d0y, d0z, s_enc, sx, sy, sz]`):
      EA     in [30, 300] kN
      g_f    in [0.8, 2.0]
      eta_lo in [0.0, 0.35]
      eta_up in [0.0, 0.35]
      d0     in [-0.30, +0.30] m per axis
      s_enc  in [0.90, 1.10]
      (sx, sy, sz) in [0.85, 1.15] per axis
"""
import argparse
import csv
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts' / 'reference_data'))
from summarize_4614 import P0_TAR, dataset_root, load_registration, read_scope, read_tracker  # noqa: E402

SQRT2 = math.sqrt(2.0)
B_REFL = np.array([0.006831, -0.198033, 0.0225])

# Explicit objective weights and parameter bounds.
POS_WEIGHT_M = 1e-3      # 1.0 mm per unit residual
FORCE_WEIGHT_N = 12.0    # 12.0 N (logged sensor units) per unit residual
LOWER_BOUNDS = np.array([30000.0, 0.8, 0.0, 0.0, -0.3, -0.3, -0.3, 0.90, 0.85, 0.85, 0.85])
UPPER_BOUNDS = np.array([300000.0, 2.0, 0.35, 0.35, 0.3, 0.3, 0.3, 1.10, 1.15, 1.15, 1.15])

STAGES = [
    ('A0: XML + EA=83.8kN, g=1.0 (baseline)',       np.array([83800.0,       1.0,   0.0,  0.0,  0, 0, 0, 1.0, 1, 1, 1]), []),
    ('A1: XML + EA=118.5kN (5171*sqrt2), g=sqrt2',  np.array([83800 * SQRT2, SQRT2, 0.0,  0.0,  0, 0, 0, 1.0, 1, 1, 1]), []),
    ('B : + fit EA (g=sqrt2 fixed)',                np.array([118500.0,      SQRT2, 0.0,  0.0,  0, 0, 0, 1.0, 1, 1, 1]), [0]),
    ('C : + fit EA, g_f, eta_lo, eta_up',           np.array([118500.0,      SQRT2, 0.04, 0.12, 0, 0, 0, 1.0, 1, 1, 1]), [0, 1, 2, 3]),
    ('D : + fit home offset d0(x,y,z) (g=sqrt2)',   np.array([118500.0,      SQRT2, 0.04, 0.12, 0, 0, 0, 1.0, 1, 1, 1]), [0, 2, 3, 4, 5, 6]),
    ('E : + fit encoder scale s_enc (g=sqrt2)',     np.array([118500.0,      SQRT2, 0.04, 0.12, 0, 0, 0, 1.0, 1, 1, 1]), [0, 2, 3, 4, 5, 6, 7]),
    ('F2: + fit anchor scale ONLY (s_enc=1, g=sqrt2)', np.array([118500.0,   SQRT2, 0.04, 0.12, 0, 0, 0, 1.0, 1, 1, 1]), [0, 2, 3, 4, 5, 6, 8, 9, 10]),
    ('F1: + fit anchor scale (sx,sy,sz) (g=sqrt2)', np.array([118500.0,      SQRT2, 0.04, 0.12, 0, 0, 0, 1.0, 1, 1, 1]), [0, 2, 3, 4, 5, 6, 7, 8, 9, 10]),
]


def load_xml_geometry(xml_path: Path):
    xml_root = ET.parse(xml_path).getroot()
    chains = sorted(xml_root.findall('.//geometry/chain'), key=lambda c: int(c.get('id')))
    a_xml = np.array([[float(c.find('base').get(k)) for k in 'xyz'] for c in chains])
    b_xml = np.array([[float(c.find('platform').get(k)) for k in 'xyz'] for c in chains])
    u_xml = np.array([[float(c.find('base').get(k)) for k in ('a31', 'a32', 'a33')] for c in chains])
    r_pul = np.array([float(c.find('base').get('radius')) for c in chains])
    return a_xml, b_xml, u_xml, r_pul


def pulley_ik(pos, rot_mat, a_pts, b_pts, u_axes, radii):
    attach = pos[None, :] + b_pts @ rot_mat.T
    delta = attach - a_pts
    axial = np.sum(delta * u_axes, axis=1, keepdims=True)
    radial = delta - axial * u_axes
    dist = np.linalg.norm(radial, axis=1, keepdims=True)
    r = radii[:, None]
    d_r = dist - r
    theta_t = np.arctan2(axial, d_r) + np.arccos(np.clip(r / np.hypot(d_r, axial), -1.0, 1.0))
    beta = np.pi - theta_t
    tangent = a_pts + r * ((1.0 + np.cos(theta_t)) * radial / dist + np.sin(theta_t) * u_axes)
    span = tangent - attach
    l_span = np.linalg.norm(span, axis=1)
    l_tot = l_span + radii * beta[:, 0]
    unit = span / l_span[:, None]
    return l_tot, l_span, unit, tangent


def solve_static_pose(p_init, l0, a_pts, b_pts, u_axes, radii, mass_kg, ea):
    g_vec = np.array([0.0, 0.0, -mass_kg * 9.81])

    def residual(q):
        pos = q[:3]
        rot = Rotation.from_rotvec(q[3:]).as_matrix()
        l_tot, _, unit, _ = pulley_ik(pos, rot, a_pts, b_pts, u_axes, radii)
        forces = ea * np.maximum(0.0, (l_tot - l0) / l0)
        arms = b_pts @ rot.T
        return np.concatenate([(unit * forces[:, None]).sum(0) + g_vec,
                               np.cross(arms, unit * forces[:, None]).sum(0)])

    sol = least_squares(
        residual, np.concatenate([p_init, np.zeros(3)]),
        xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=150,
    )
    pos = sol.x[:3]
    rot = Rotation.from_rotvec(sol.x[3:]).as_matrix()
    l_tot, _, _, _ = pulley_ik(pos, rot, a_pts, b_pts, u_axes, radii)
    forces = ea * np.maximum(0.0, (l_tot - l0) / l0)
    return pos, rot, forces


def verify_pulley_ik_targets(csv_path: Path, geom):
    """Verify Python swivel-pulley IK matches logged motor targets `xtar[0..7]` across P0..P5."""
    a_xml, b_xml, u_xml, r_pul = geom
    l_origin, _, _, _ = pulley_ik(np.zeros(3), np.eye(3), a_xml, b_xml, u_xml, r_pul)
    max_err_mm = 0.0
    with csv_path.open() as f:
        for row in csv.DictReader(f):
            if row['experiment'] != '20_Std_Load13900':
                continue
            tar = np.array([float(row['tar_x']), float(row['tar_y']), float(row['tar_z'])])
            xtar = np.array([float(row[f'xtar{i}']) for i in range(8)])
            l_ik, _, _, _ = pulley_ik(tar, np.eye(3), a_xml, b_xml, u_xml, r_pul)
            err_mm = float(np.max(np.abs((l_ik - l_origin) - xtar)) * 1000.0)
            max_err_mm = max(max_err_mm, err_mm)
    if max_err_mm >= 0.2:
        raise AssertionError(f'Pulley IK mismatch vs logged x_tar: {max_err_mm:.4f} mm (expected < 0.2 mm)')
    print(f'Verified swivel-pulley IK vs logged x_tar (P0..P5 x 8 cables): max err = {max_err_mm:.4f} mm', file=sys.stderr)
    return max_err_mm


def load_planea_fixture(csv_path: Path):
    with csv_path.open() as f:
        rows = list(csv.DictReader(f))
    exps = list(dict.fromkeys(r['experiment'] for r in rows))
    planea = {}
    for exp in exps:
        er = sorted([r for r in rows if r['experiment'] == exp], key=lambda r: int(r['pose'][1:]))
        raw = np.array([[float(r['raw_x']), float(r['raw_y']), float(r['raw_z'])] for r in er])
        planea[exp] = dict(
            mass=float(er[0]['mass_kg']),
            mode=er[0]['mode'],
            dp_meas=raw - raw[0],
            tar=np.array([[float(r['tar_x']), float(r['tar_y']), float(r['tar_z'])] for r in er]),
            x_enc=np.array([[float(r[f'x{i}']) for i in range(8)] for r in er]),
            f_log=np.array([[float(r[f'f{i}']) for i in range(8)] for r in er]),
        )
    return exps, planea


def load_circpath_sets(base_dir: Path):
    r_reg, t_reg = load_registration(base_dir / 'tracker')
    circ_sets = {}
    for exp, mass in [('20_Std_Load13900', 13.9), ('10_Std_Load29100', 29.1)]:
        scope = read_scope(base_dir / 'robot' / exp / 'CircPath' / 'Export 1.csv')
        pos_tr, t_tr = read_tracker(base_dir / 'tracker' / exp / 'CircPath.csv')
        tr_reg = pos_tr @ r_reg.T + t_reg
        p_rob = np.column_stack([scope[f'p[{i}]'] for i in range(3)])
        x_enc = np.column_stack([scope[f'x[{i}]'] for i in range(8)])
        f_log = np.column_stack([scope[f'f[{i}]'] for i in range(8)])
        v_enc = np.column_stack([scope[f'v[{i}]'] for i in range(8)])
        t_r = scope.t - scope.t[0]
        t_t = t_tr - t_tr[0]
        dp_r = np.linalg.norm(p_rob - p_rob[0], axis=1)
        start_r = np.flatnonzero(dp_r > 0.02)[0]
        dp_t = np.linalg.norm(tr_reg - tr_reg[0], axis=1)
        start_t = np.flatnonzero(dp_t > 0.02)[0]
        dt_est = t_t[start_t] - t_r[start_r]
        best_off, best_err = dt_est, 1e9
        grid_t = t_r[::5]
        for off in np.linspace(dt_est - 0.5, dt_est + 0.5, 201):
            tr_i = np.column_stack([np.interp(grid_t + off, t_t, tr_reg[:, k]) for k in range(3)])
            err = np.mean(np.sum(((tr_i - tr_i[0]) - (p_rob[::5] - p_rob[0])) ** 2, axis=1))
            if err < best_err:
                best_err, best_off = err, off
        tr_aligned = np.column_stack([np.interp(t_r + best_off, t_t, tr_reg[:, k]) for k in range(3)])
        active = np.flatnonzero(dp_r > 0.2)
        picks = active[np.linspace(0, len(active) - 1, 24, dtype=int)]
        circ_sets[exp] = dict(
            mass=mass,
            p0_rob=p_rob[0],
            x0=x_enc[0],
            f0=f_log[0],
            p_rob=p_rob[picks],
            x_enc=x_enc[picks],
            v_enc=v_enc[picks],
            f_log=f_log[picks],
            dp_meas=tr_aligned[picks] - tr_aligned[0],
        )
        print(
            f'Loaded CircPath {exp}: offset={best_off:.4f}s, {len(picks)} samples, '
            f'max f_log={f_log[picks].max():.1f}N',
            file=sys.stderr,
        )
    return circ_sets


def eval_planea(params, exp_list, planea, geom):
    a_xml, b_xml, u_xml, r_pul = geom
    ea, g_f, eta_lo, eta_up, d0x, d0y, d0z, s_enc, sx, sy, sz = params
    a_scaled = a_xml.copy()
    a_scaled[:, 0] *= sx
    a_scaled[:, 1] *= sy
    a_scaled[:, 2] *= sz
    p0 = P0_TAR + np.array([d0x, d0y, d0z])
    l_p0, _, _, _ = pulley_ik(p0, np.eye(3), a_scaled, b_xml, u_xml, r_pul)
    refl_p0 = p0 + B_REFL
    pos_errs, force_errs = [], []
    eta = np.array([eta_up] * 4 + [eta_lo] * 4)
    for exp in exp_list:
        d = planea[exp]
        f0 = g_f * d['f_log'][0]
        l0_0 = l_p0 / (1.0 + f0 / ea)
        for k in range(1, 6):
            dx = s_enc * (d['x_enc'][k] - d['x_enc'][0])
            l0_k = l0_0 + dx / (1.0 + f0 / ea)
            p_sim, r_sim, f_plat = solve_static_pose(
                d['tar'][k] + np.array([d0x, d0y, d0z]), l0_k, a_scaled, b_xml, u_xml, r_pul, d['mass'], ea,
            )
            pos_errs.append((p_sim + r_sim @ B_REFL - refl_p0) - d['dp_meas'][k])
            sign_move = np.sign(d['x_enc'][k] - d['x_enc'][k - 1])
            f_pred_act = f_plat * np.where(sign_move < 0, 1.0 + eta, 1.0 / (1.0 + eta))
            force_errs.append(f_pred_act / g_f - d['f_log'][k])
    pos_errs = np.array(pos_errs)
    force_errs = np.array(force_errs)
    return 1000.0 * np.sqrt(np.mean(np.sum(pos_errs ** 2, axis=1))), np.sqrt(np.mean(force_errs ** 2))


def eval_circ(params, circ_sets, geom):
    a_xml, b_xml, u_xml, r_pul = geom
    ea, g_f, eta_lo, eta_up, d0x, d0y, d0z, s_enc, sx, sy, sz = params
    a_scaled = a_xml.copy()
    a_scaled[:, 0] *= sx
    a_scaled[:, 1] *= sy
    a_scaled[:, 2] *= sz
    pos_errs, force_errs = [], []
    eta = np.array([eta_up] * 4 + [eta_lo] * 4)
    for d in circ_sets.values():
        p0 = d['p0_rob'] + np.array([d0x, d0y, d0z])
        l_p0, _, _, _ = pulley_ik(p0, np.eye(3), a_scaled, b_xml, u_xml, r_pul)
        refl_p0 = p0 + B_REFL
        f0 = g_f * d['f0']
        l0_0 = l_p0 / (1.0 + f0 / ea)
        for k in range(len(d['p_rob'])):
            dx = s_enc * (d['x_enc'][k] - d['x0'])
            l0_k = l0_0 + dx / (1.0 + f0 / ea)
            p_sim, r_sim, f_plat = solve_static_pose(
                d['p_rob'][k] + np.array([d0x, d0y, d0z]), l0_k, a_scaled, b_xml, u_xml, r_pul, d['mass'], ea,
            )
            pos_errs.append((p_sim + r_sim @ B_REFL - refl_p0) - d['dp_meas'][k])
            sign_v = np.where(np.abs(d['v_enc'][k]) > 1e-3, np.sign(d['v_enc'][k]), 0.0)
            f_pred_act = f_plat * np.where(sign_v < 0, 1.0 + eta, np.where(sign_v > 0, 1.0 / (1.0 + eta), 1.0))
            force_errs.append(f_pred_act / g_f - d['f_log'][k])
    pos_errs = np.array(pos_errs)
    force_errs = np.array(force_errs)
    return 1000.0 * np.sqrt(np.mean(np.sum(pos_errs ** 2, axis=1))), np.sqrt(np.mean(force_errs ** 2))


def default_base_dir() -> Path:
    cache = ROOT / 'output' / 'reference_cache' / '4614'
    if (cache / 'COPacabana.xml').exists():
        return cache
    return dataset_root('4614') / 'exp-apc_darus-upload' / 'experiments'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, default=None,
                        help='Directory with COPacabana.xml, robot/, and tracker/ (default: cache or DaRUS-4614 root)')
    parser.add_argument('--planea-csv', type=Path,
                        default=ROOT / 'tests' / 'fixtures' / 'reference_data' / 'darus4614_planeA.csv')
    parser.add_argument('--planea-only', action='store_true',
                        help='Skip held-out CircPath evaluation (uses only COPacabana.xml + PlaneA fixture)')
    args = parser.parse_args()

    base_dir = args.base if args.base is not None else default_base_dir()
    xml_path = base_dir / 'COPacabana.xml'
    if not xml_path.exists():
        xml_path = base_dir.parent / 'COPacabana.xml'
    geom = load_xml_geometry(xml_path)
    verify_pulley_ik_targets(args.planea_csv, geom)
    a_xml, b_xml, u_xml, r_pul = geom
    exps, planea = load_planea_fixture(args.planea_csv)
    train_exp = ['20_Std_Load13900']
    test_exps = [e for e in exps if e != train_exp[0]]

    circ_sets = None
    if not args.planea_only:
        circ_probe = base_dir / 'robot' / '20_Std_Load13900' / 'CircPath' / 'Export 1.csv'
        if circ_probe.exists():
            circ_sets = load_circpath_sets(base_dir)
        else:
            print(f'CircPath not found under {base_dir}; pass --base to evaluate CircPath.', file=sys.stderr)

    print(f"{'Stage':48s} | {'TrPos':>6s} {'TrF':>6s} | {'7PlPos':>6s} {'7PlF':>6s} | {'CircPos':>7s} {'CircF':>6s} | Params")
    print('-' * 160)
    for name, p0_vec, free_idx in STAGES:
        params = p0_vec.copy()
        if free_idx:
            def objective(theta):
                pp = p0_vec.copy()
                for j, idx in enumerate(free_idx):
                    pp[idx] = theta[j]
                ea, g_f, eta_lo, eta_up, d0x, d0y, d0z, s_enc, sx, sy, sz = pp
                a_scaled = a_xml.copy()
                a_scaled[:, 0] *= sx
                a_scaled[:, 1] *= sy
                a_scaled[:, 2] *= sz
                p0 = P0_TAR + np.array([d0x, d0y, d0z])
                l_p0, _, _, _ = pulley_ik(p0, np.eye(3), a_scaled, b_xml, u_xml, r_pul)
                refl_p0 = p0 + B_REFL
                d = planea[train_exp[0]]
                f0 = g_f * d['f_log'][0]
                l0_0 = l_p0 / (1.0 + f0 / ea)
                eta = np.array([eta_up] * 4 + [eta_lo] * 4)
                out = []
                for k in range(1, 6):
                    dx = s_enc * (d['x_enc'][k] - d['x_enc'][0])
                    l0_k = l0_0 + dx / (1.0 + f0 / ea)
                    p_sim, r_sim, f_plat = solve_static_pose(
                        d['tar'][k] + np.array([d0x, d0y, d0z]), l0_k, a_scaled, b_xml, u_xml, r_pul, d['mass'], ea,
                    )
                    dp_err = (p_sim + r_sim @ B_REFL - refl_p0) - d['dp_meas'][k]
                    sign_move = np.sign(d['x_enc'][k] - d['x_enc'][k - 1])
                    f_pred_act = f_plat * np.where(sign_move < 0, 1.0 + eta, 1.0 / (1.0 + eta))
                    f_err_log = f_pred_act / g_f - d['f_log'][k]
                    out.extend((dp_err / POS_WEIGHT_M).tolist())
                    out.extend((f_err_log / FORCE_WEIGHT_N).tolist())
                return np.array(out)

            sol = least_squares(
                objective, p0_vec[free_idx],
                bounds=(LOWER_BOUNDS[free_idx], UPPER_BOUNDS[free_idx]),
                xtol=1e-6, ftol=1e-6,
            )
            for j, idx in enumerate(free_idx):
                params[idx] = sol.x[j]

        tr_p, tr_f = eval_planea(params, train_exp, planea, geom)
        te_p, te_f = eval_planea(params, test_exps, planea, geom)
        if circ_sets is not None:
            ci_p, ci_f = eval_circ(params, circ_sets, geom)
            circ_str = f'{ci_p:6.2f}m {ci_f:5.1f}N'
        else:
            circ_str = f"{'n/a':>7s} {'n/a':>6s}"
        pstr = (
            f'EA={params[0]/1e3:.1f}k g={params[1]:.3f} '
            f'eta=({params[2]*100:.1f}%,{params[3]*100:.1f}%) '
            f'd0=({params[4]*1e3:+.0f},{params[5]*1e3:+.0f},{params[6]*1e3:+.0f}) '
            f's={params[7]:.3f} sc=({params[8]:.3f},{params[9]:.3f},{params[10]:.3f})'
        )
        print(f'{name:48s} | {tr_p:5.2f}m {tr_f:5.1f}N | {te_p:5.2f}m {te_f:5.1f}N | {circ_str} | {pstr}')


if __name__ == '__main__':
    main()
