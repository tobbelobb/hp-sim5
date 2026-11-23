"""Compare local joint force and compliance models on frozen simulator sweeps."""
import argparse
from copy import deepcopy
import gzip
import json
from dataclasses import replace
from pathlib import Path
from time import perf_counter

import numpy as np
from scipy.optimize import least_squares

from autocal.anchor_seed import build_anchor_initial_guess
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.flex import FlexModel
from autocal.pose_fit import (HOME_OFFSET_PRIOR_SIGMA_MM, _home_strain, _at_far_bound,
    consistency_residuals, dataset_line_lengths, dataset_stretch, fit_point_positions,
    fit_pose_least_squares, shift_dataset_home_offsets, stretch_corrected_lengths)
from autocal.theoretical_ellipse import (anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec,
    get_anchor_opt_bounds)
from autocal.tools.imperfect_machine import convention_frame


def evaluate(run, variant, mode):
    raw = run / 'sweeps.json'
    data = json.loads(raw.read_text()) if raw.exists() else json.loads(gzip.decompress(
        (run / 'sweeps.json.gz').read_bytes()))
    normalize_dataset_point_roles(data)
    imp = json.loads((run/'imperfection.json').read_text())
    truth = convention_frame(np.array(list(imp['truth_mm'].values())))[0]
    cfg = data['config']['m666_before_data_collection']
    machine = data['machine_type']
    n, d = data['num_anchors'], data['dimensions']
    k_true = 1 / imp['stiffness_scale']
    is_hp3 = n == 4
    lines = dataset_line_lengths(data)
    L, group = lines['lengths_mm'], lines['sweep_index']
    observed = np.isfinite(L)
    entire = observed.copy()
    points = [(s, p) for s in data['sweeps'] for p in s['data_points']]
    th = np.deg2rad(np.array([p.get('mu') or p['raw_angles_deg'] for s,p in points]))
    stretch = dataset_stretch(data)
    home = _home_strain(data)
    if stretch is None and home is None:
        raise ValueError('No stretch model')
    if stretch is None:
        stretch = dict(model=FlexModel.from_m666(cfg, num_axes=n), weight=cfg['W']*9.81,
            passive=0, known=np.full(L.shape, np.nan),
            sensors=np.array([p.get('source_sensor_anchor',s['sensor_anchor']) for s,p in points]),
            forces=np.array([p['sensor_force_n'] for s,p in points]))
    cmd = stretch['known'].copy()
    cmd[np.arange(len(L)), stretch['sensors']] = stretch['forces']
    entry = np.isfinite(cmd)
    motors = np.flatnonzero(entry.any(axis=0))
    nominal = imp.get('cogging_scale')
    nominal = 1 if nominal is None else nominal
    oracle = np.asarray(cfg['U']) * nominal/3 * np.sin((imp.get('cogging_freq') or 50) * th)
    rng = np.random.default_rng(540)
    recorded_shift = np.zeros_like(L)
    if mode == 'extra':
        recorded_shift = -np.asarray(cfg['U']) / 3 * np.sin((imp.get('cogging_freq') or 50) * th)
    if mode == 'offset':
        recorded_shift[:,1] = -0.5
    if mode == 'off200':
        recorded_shift = -np.asarray(cfg['U']) / 3 * np.sin(200 * th + rng.uniform(0,2*np.pi,n))
        recorded_shift[:,1] -= 0.5
    if mode == 'no-ripple':
        recorded_shift = oracle.copy()
    if variant == 'differential' and mode != 'nominal':
        raise ValueError('Force-label perturbation does not exercise differential HP3')
    if np.any((cmd+recorded_shift)[entry] <= 0):
        raise ValueError('Nonpositive force in perturbation')
    cmd += recorded_shift
    baseline = data
    if mode != 'nominal':
        baseline = deepcopy(data)
        baseline_points = [p for sweep in baseline['sweeps'] for p in sweep['data_points']]
        for index, point in enumerate(baseline_points):
            point['sensor_force_n'] = float(cmd[index, stretch['sensors'][index]])
            for extra in point.get('extra_sensor_ranges', []):
                extra['force_n'] = float(cmd[index, extra['anchor']])
    start = perf_counter()
    seed = build_anchor_initial_guess(baseline, machine_type=machine)
    opts = dict(home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    first = fit_pose_least_squares(baseline, seed, fit_home_offsets=True, **opts)
    fits = fit_pose_least_squares(shift_dataset_home_offsets(baseline, first['home_offsets_mm']),
        first['anchors'], fit_home_offsets=False, **opts)
    baseline_s = perf_counter()-start
    start = perf_counter()
    anchor0 = fits['anchors']
    offset0 = first['home_offsets_mm'].copy()
    observed[fits['rejected_points']] = False

    def metrics(a, r):
        diff = convention_frame(a)[0] - truth
        return dict(max=float(np.abs(diff).max()), top=float(diff[-1, 2]),
            low=float(np.mean(diff[:-1,2])), rms=float(np.sqrt(np.mean(r**2))))
    result = dict(run=run.name, n=len(L), sweeps=len(set(group)), k_true=k_true,
        amplitude_scale=nominal, frequency=imp.get('cogging_freq') or 50,
        baseline=metrics(anchor0, fits['point_residuals_mm']),
        first_skipped=first.get('home_offsets_skipped'), baseline_s=baseline_s)
    base_x = anchors_matrix_to_opt_vec(anchor0, machine)
    lo, hi = get_anchor_opt_bounds(machine, n, d)
    m = len(base_x)

    def geometry(x, k, differential):
        anchors = anchor_opt_vec_to_matrix(x[:m], machine, n, d)
        offsets = np.r_[np.zeros(d), x[m:]]
        if differential:
            strain, extra = home
            offsets[:3] = strain * (np.linalg.norm(anchors[:3], axis=1) + extra)
        return anchors, offsets

    def force_stretch(k, correction):
        f = cmd + correction
        return dict(stretch, model=replace(stretch['model'], spring_k_multiplier=1/k),
            known=np.where(np.isfinite(stretch['known']), f, np.nan),
            forces=f[np.arange(len(L)), stretch['sensors']])

    zeros = np.zeros_like(L)
    def residual(x, k, correction, differential):
        anchors, offsets = geometry(x, k, differential)
        st = force_stretch(k, correction)
        if not differential:
            return consistency_residuals(anchors, L, offsets, stretch=st)[observed]
        pos, absolute = fit_point_positions(anchors, L, offsets)
        delta = stretch_corrected_lengths(anchors,pos,absolute,entire,st) - stretch_corrected_lengths(
            anchors,pos,absolute,entire,force_stretch(k, zeros))
        return consistency_residuals(anchors,L+delta,offsets)[observed]

    def refine(x, k, force, differential):
        fit = least_squares(residual, np.clip(x, np.r_[lo, np.full(n-d,-np.inf)], np.r_[hi, np.full(n-d,np.inf)]),
            bounds=(np.r_[lo, np.full(n-d,-np.inf)], np.r_[hi, np.full(n-d,np.inf)]),
            args=(k, force, differential), x_scale='jac', loss='soft_l1', f_scale=0.02, max_nfev=30)
        return fit.x, fit.fun

    differential = is_hp3 and variant == 'differential'
    x = np.r_[base_x, offset0[d:] if differential or not is_hp3 else 0.0]
    xc, rc = refine(x,k_true,zeros,differential)
    result['full_cmd_true'] = metrics(geometry(xc,k_true,differential)[0],rc)
    xo, ro = refine(x,k_true,oracle - recorded_shift,differential)
    result['oracle'] = metrics(geometry(xo,k_true,differential)[0],ro)
    point_owner = np.where(observed)[0]
    folds = group[point_owner]
    period_bank = np.arange(1.,401.)
    phase = (th % (2*np.pi))[:,:,None] * period_bank

    # Local derivatives of the production point residual. A force change affects
    # just the owning point, so uniform perturbations give all its derivatives.
    k = 1.
    r = residual(x,k,zeros,differential)
    jac = np.column_stack([(residual(x+step,k,zeros,differential)-r)/.01
        for step in np.eye(len(x))*0.01])
    jac_k = (residual(x,1.01,zeros,differential)-r)/.01
    jac_force = np.column_stack([(residual(x,k,.01*np.eye(n)[a]*entry,differential)-r)/.01 for a in motors])
    weights = 1 / np.sqrt(1+(r/0.15)**2)
    phi = np.stack([np.sin(phase[:,motors]),np.cos(phase[:,motors])],axis=-1)
    bank = (jac_force[:, :, None, None] * phi[point_owner]).transpose(0,2,1,3).reshape(len(r),400,-1)
    force_columns = []
    for val in (np.ones_like(cmd), np.nan_to_num(cmd)):
        for a in motors[:-1]:
            coef = (motors == a) - np.full(len(motors), 1/len(motors))
            force_columns.append(val[:,motors] * coef)
    contrast = np.stack(force_columns,axis=2)
    contrast_jac = np.einsum('rm,rmc->rc',jac_force,contrast[point_owner])

    def linear_model(X, train, penalized):
        A = X[train]*weights[train,None]
        penalty = np.r_[np.full(X.shape[1]-penalized, 1e-12), np.full(penalized, 1e-6)]
        return np.linalg.solve(A.T @ A + np.diag(penalty), -A.T @ (r[train]*weights[train]))

    def period_scan(X, train):
        A = X[train]*weights[train,None]
        y = r[train]*weights[train]
        Q, sing, _ = np.linalg.svd(A, full_matrices=False)
        Q = Q[:,sing>1e-8]
        y -= Q @ (Q.T @ y)
        B = bank[train]*weights[train,None,None]
        stacked = B.reshape(len(B), -1)
        stacked -= Q @ (Q.T @ stacked)
        lhs = np.einsum('ihj,ihk->hjk',B,B) + 1e-6*np.eye(bank.shape[2])
        rhs = np.einsum('i,ihj->hj',y,B)
        beta = np.linalg.solve(lhs,rhs[...,None])[...,0]
        gain = np.einsum('hj,hj->h',beta,rhs)
        i = gain.argmax()
        return int(i), float(gain[i]/(y@y))

    training = np.ones(len(r),dtype=bool)
    fit_k = not differential and variant != 'fixed'
    base = np.c_[jac, jac_k] if fit_k else jac
    result.update(variant=variant, mode=mode)
    for variant, contrasts in [('plain',False),('ripple',False),('full',True)]:
        is_ripple = variant != 'plain'
        X = np.c_[base,contrast_jac] if contrasts else base
        scores = []
        for g in np.unique(folds):
            tests = folds == g
            j, _ = period_scan(X, ~tests)
            M = np.c_[X,bank[:,j]] if is_ripple else X
            b = linear_model(base, ~tests, 0)
            c = linear_model(M,~tests,M.shape[1]-base.shape[1])
            scores.append((j+1, np.sum((r[tests]+base[tests]@b)**2),np.sum((r[tests]+M[tests]@c)**2)))
        i, train_gain = period_scan(X, training)
        fitmat = np.c_[X,bank[:,i]] if is_ripple else X
        coef = linear_model(fitmat,training,fitmat.shape[1]-base.shape[1])
        k_hat = float(1+coef[base.shape[1]-1]) if fit_k else 1.
        force = np.zeros_like(L)
        if is_ripple:
            force[:,motors] = np.einsum('pmk,mk->pm',phi[:,:,i],coef[-2*len(motors):].reshape(-1,2))
        if contrasts:
            force[:,motors] += contrast @ coef[base.shape[1]:base.shape[1]+contrast.shape[2]]
        xm = x+coef[:len(x)]
        if k_hat > 0:
            force /= k_hat
        stats = dict(k=k_hat, periods=[row[0] for row in scores], period=i+1, trial_gain=train_gain,
            cv_gain=float(1-sum(row[2] for row in scores)/sum(row[1] for row in scores)),
            min_force=np.min((cmd+force)[entry]), max_delta=np.max(np.abs(force[entry])),
            force_rms_by_motor=np.sqrt(np.nanmean(np.where(entry,force**2,np.nan)[:,motors],axis=0)).tolist(),
            force_mean_by_motor=np.nanmean(np.where(entry,force,np.nan)[:,motors],axis=0).tolist())
        if .1 < k_hat < 3 and stats['min_force'] > 0:
            xm, fm = refine(xm,k_hat,force,differential)
            stats.update(metrics(geometry(xm,k_hat,differential)[0],fm))
            stats['bound'] = _at_far_bound(geometry(xm,k_hat,differential)[0], data)
        result[variant] = stats
    result['research_s']=perf_counter()-start
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('runs',type=Path,nargs='+')
    p.add_argument('--variant',choices=['full','differential','fixed'],default='full')
    p.add_argument('--mode',choices=['nominal','extra','offset','off200','no-ripple'],default='nominal')
    ARGS=p.parse_args()
    for run in ARGS.runs:
        try:
            print(json.dumps(evaluate(run, ARGS.variant, ARGS.mode)),flush=True)
        except (ValueError, FileNotFoundError) as exc:
            print(json.dumps(dict(run=run.name,error=str(exc))),flush=True)
