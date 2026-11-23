"""Blind force-model fit: per-motor period scan, offsets, scales and direction terms, jointly with anchors and k.

usage: blind_force_fit.py RUN_DIR [OUT.json]        (PERTURB as in perturb.py; JOBS=n over the k grid)

Nothing about the ripple's period, phase or amplitude is given. Per force-mode motor m the
effector-force correction is
    dF = sum_selected (a sin(h th) + b cos(h th))      h in 1..HMAX per turn, chosen blind
       + o_m                                            per-motor offset   } contrasts: sum_m o_m = 0 and
       + s_m F_cmd                                      per-motor scale    } sum_m s_m = 0. The common part
                                                        trades with k (exactly so for a scale, and for an offset
                                                        under constant-force meshes, where k (F + o) is all the
                                                        data see); it is left to k.
       + d_m dir                                        offset flipping with motion direction, only on
                                                        lines that move both ways (>= 20% each)
th = the motor's encoder angle, dir = sign of that line's length change since the sweep's
previous point. The production residual (ellipse/ellipsoid misfit or point consistency,
stretch-corrected) is linearised in every point's force on every force-mode line (a point's
force only moves its own residuals, so one finite difference per motor), with the fit
parameters (anchors, free home offsets) as nuisance columns. A ridge (prior sd TAU_N) keeps
the force terms bounded. Harmonics enter greedily: per motor the h with the lowest
leave-one-sweep-out squared prediction error, kept while it cuts that error by >= CV_GAIN.
Selected harmonics that no longer earn CV_GAIN once the others are in are pruned. At each k
of the grid the harmonics are selected afresh at that k's commanded-force fit and the force
terms refitted (two linearise / apply / production-refit rounds); k is the vertex of the
production rms, where the selection and refit are repeated (three rounds). Variants:
'ripple' (harmonics only) and 'full' (harmonics + offset/scale contrasts + direction).
"""
import copy, json, multiprocessing, os, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from cog_refit import PROBE, load  # noqa: E402,F401
from refit2 import fit_at, FROM_M666  # noqa: E402
from perturb import perturb, point_directions  # noqa: E402
import autocal.pose_fit as pf  # noqa: E402
import autocal.ellipse_fit as ef  # noqa: E402
from autocal.flex import FlexModel  # noqa: E402
from autocal.theoretical_ellipse import anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec  # noqa: E402
from autocal.dataset_roles import normalize_dataset_point_roles  # noqa: E402
from autocal.tools.imperfect_machine import convention_frame  # noqa: E402

GRID = np.round(np.arange(0.75, 1.351, 0.05), 3)
HMAX, TAU_N, NOISE_MM, CV_GAIN, MAX_HARMONICS = 400, 1.0, 1e-3, 0.05, 3


def stretch_at(ds, k):
    FlexModel.from_m666 = classmethod(lambda cls, m666, *, num_axes, spring_k_multiplier=1.0:
                                      FROM_M666(cls, m666, num_axes=num_axes, spring_k_multiplier=spring_k_multiplier / k))
    try:
        return pf.dataset_stretch(ds)
    finally:
        FlexModel.from_m666 = classmethod(FROM_M666)


class Linear:
    """The production residual at a fit, linearised in the fit parameters and every point's forces."""

    def __init__(self, ds, fitter, k, fit):
        self.ds, self.fitter = ds, fitter
        mt, n, d = str(ds['machine_type']), int(ds['num_anchors']), int(ds['dimensions'])
        self.mt, self.n, self.d = mt, n, d
        self.L = pf.dataset_line_lengths(ds)['lengths_mm']
        self.stretch = stretch_at(ds, k)
        self.units = ef._sweep_ellipses(ds, self.L) if fitter == 'ellipsoid' else None
        offsets = np.asarray(fit.get('home_offsets_mm', np.zeros(n)), float)
        self.offset_lines = [a for a in range(d, n) if abs(offsets[a]) > 0]
        self.base_offsets = offsets
        self.x0 = np.concatenate([anchors_matrix_to_opt_vec(np.asarray(fit['anchors'], float), mt),
                                  offsets[self.offset_lines]])
        self.na = self.x0.size - len(self.offset_lines)
        # Force entries: (point, anchor) for the sensor and every known extra force.
        known = self.stretch['known']
        self.entries = [(i, int(s)) for i, s in enumerate(self.stretch['sensors'])]
        self.entries += [(i, a) for i, a in zip(*np.nonzero(np.isfinite(known)))]
        r0, self.owner = self.residual(self.x0, {})
        self.r0 = r0
        self.Jp = np.column_stack([(self.residual(self.x0 + e, {})[0] - r0) / 1e-3
                                   for e in np.eye(self.x0.size) * 1e-3])
        self.G = np.zeros((r0.size, len(self.entries)))
        motors = sorted({a for _, a in self.entries})
        for m in motors:
            cols = [j for j, (_, a) in enumerate(self.entries) if a == m]
            dr = (self.residual(self.x0, {m: 0.05})[0] - r0) / 0.05
            for j in cols:
                i = self.entries[j][0]
                rows = self.owner == i
                self.G[rows, j] = dr[rows]
        self.motors = motors

    def residual(self, x, delta):
        """Residuals with every point's force on motor m raised by delta[m] N."""
        anchors = anchor_opt_vec_to_matrix(x[:self.na], self.mt, self.n, self.d)
        offsets = self.base_offsets.copy()
        offsets[self.offset_lines] = x[self.na:]
        st = dict(self.stretch, forces=self.stretch['forces'].copy(), known=self.stretch['known'].copy())
        for m, v in delta.items():
            st['forces'] = np.where(st['sensors'] == m, st['forces'] + v, st['forces'])
            st['known'][:, m] += v
        if self.fitter == 'ellipsoid':
            lengths = ef._corrected_lengths(anchors, self.L, offsets, st)
            values, owner = ef.ellipse_misfits(anchors, self.units, lengths + offsets)
            return values, owner
        res = pf.consistency_residuals(anchors, self.L, offsets, stretch=st)
        measured = np.isfinite(self.L)
        return res[measured], np.nonzero(measured)[0]


def features(ds, entries):
    """Per force entry: encoder angle (rad), commanded force, direction, owning sweep."""
    rows, dirs, sweep_of = [], [], []
    for si, sweep in enumerate(ds['sweeps']):
        d = point_directions(sweep)
        for p, dd in zip(sweep['data_points'], d):
            rows.append(np.deg2rad(np.asarray(p.get('mu') or p['raw_angles_deg'], float)))
            dirs.append(dd)
            sweep_of.append(si)
    return rows, dirs, np.asarray(sweep_of)


def design(lin, ds, harmonics, full):
    """Force-term columns (in residual space) and their labels for the given harmonic set."""
    angles, dirs, _ = features(ds, lin.entries)
    forces = np.array([lin.stretch['forces'][i] if a == lin.stretch['sensors'][i] else lin.stretch['known'][i, a]
                       for i, a in lin.entries])
    cols, labels = [], []
    motors = lin.motors
    for m in motors:
        sel = np.array([a == m for _, a in lin.entries])
        th = np.array([angles[i][a] for i, a in lin.entries])
        for h in harmonics.get(m, []):
            for name, f in (('sin', np.sin), ('cos', np.cos)):
                cols.append(np.where(sel, f(h * th), 0.0)); labels.append((m, f'{name}{h:g}'))
        if full:
            dv = np.array([dirs[i].get(a, 1.0) for i, a in lin.entries])
            if sel.any() and min(np.mean(dv[sel] > 0), np.mean(dv[sel] < 0)) >= 0.2:
                cols.append(np.where(sel, dv, 0.0)); labels.append((m, 'direction'))
    if full and len(motors) > 1:
        # Offset and scale contrasts (term_m - mean over motors); the common parts are k's job.
        for name, value in (('offset', np.ones(len(forces))), ('scale', forces)):
            per = [np.where(np.array([a == m for _, a in lin.entries]), value, 0.0) for m in motors]
            mean = np.mean(per, axis=0)
            for m, col in zip(motors[:-1], per[:-1]):
                cols.append(col - mean); labels.append((m, name))
    Phi = np.column_stack(cols) if cols else np.zeros((len(lin.entries), 0))
    return lin.G @ Phi, Phi, labels


def solve(lin, A, rows=None):
    """Ridge LS for [dp, beta] minimising |r0 + Jp dp + A beta|^2 + lambda |beta|^2 (on rows)."""
    X = np.column_stack([lin.Jp, A])
    y = lin.r0
    if rows is not None:
        X, y = X[rows], y[rows]
    # Gaussian prior: force terms sd TAU_N against residual noise NOISE_MM; fit parameters free.
    lam = np.full(X.shape[1], 1e-12)
    lam[lin.Jp.shape[1]:] = (NOISE_MM / TAU_N) ** 2
    gamma = np.linalg.solve(X.T @ X + np.diag(lam), -X.T @ y)
    return gamma


def cv_sse(lin, A, folds):
    X = np.column_stack([lin.Jp, A])
    sse = 0.0
    for test in folds:
        g = solve(lin, A, ~test)
        sse += float(np.sum((lin.r0[test] + X[test] @ g) ** 2))
    return sse


def select(lin, ds, full):
    """Greedy blind harmonic selection per motor by leave-one-sweep-out error."""
    _, _, sweep_of = features(ds, lin.entries)
    point_sweep = sweep_of[lin.owner]
    folds = [point_sweep == s for s in np.unique(point_sweep)]
    harmonics = {m: [] for m in lin.motors}
    base = cv_sse(lin, design(lin, ds, harmonics, full)[0], folds)
    trace = []
    for _ in range(MAX_HARMONICS * len(lin.motors)):
        best = None
        for m in lin.motors:
            if len(harmonics[m]) >= MAX_HARMONICS:
                continue
            for h in range(1, HMAX + 1):
                if h in harmonics[m]:
                    continue
                trial = {**harmonics, m: harmonics[m] + [h]}
                e = cv_sse(lin, design(lin, ds, trial, full)[0], folds)
                if best is None or e < best[0]:
                    best = (e, m, h)
        if best is None or best[0] > (1 - CV_GAIN) * base:
            break
        trace.append(dict(motor=best[1], h=best[2], cv_sse_before=base, cv_sse=best[0]))
        harmonics[best[1]].append(best[2])
        base = best[0]
    for m in lin.motors:
        for h in list(harmonics[m]):
            trial = {**harmonics, m: [x for x in harmonics[m] if x != h]}
            e = cv_sse(lin, design(lin, ds, trial, full)[0], folds)
            if e <= (1 + CV_GAIN) * base:
                harmonics, base = trial, e
                trace.append(dict(motor=m, pruned=h, cv_sse=e))
    return harmonics, trace


def apply(ds, lin, Phi, beta):
    """Dataset with the fitted force correction added to the recorded forces (kept >= 0.05 N: a wild
    trial correction then scores badly instead of failing the stretch model)."""
    out = copy.deepcopy(ds)
    dF = Phi @ beta
    flat = []
    for sweep in out['sweeps']:
        for p in sweep['data_points']:
            flat.append(p)
    for (i, a), v in zip(lin.entries, dF):
        p = flat[i]
        if a == int(lin.stretch['sensors'][i]):
            p['sensor_force_n'] = max(float(p['sensor_force_n'] + v), 0.05)
        else:
            for e in p.get('extra_sensor_ranges', []):
                if int(e['anchor']) == a and 'force_n' in e:
                    e['force_n'] = max(float(e['force_n'] + v), 0.05)
    return out


def refine(ds, fitter, k, live, harmonics, full, rounds=2):
    """Production fit at k with the force terms refitted `rounds` times; returns fit, terms."""
    fit = fit_at(ds, fitter, k, live)
    data, terms = ds, []
    beta_total = None
    for _ in range(rounds):
        lin = Linear(data, fitter, k, fit)
        A, Phi, labels = design(lin, data, harmonics, full)
        if not labels:
            break
        g = solve(lin, A)
        beta = g[lin.Jp.shape[1]:]
        data = apply(data, lin, Phi, beta)
        beta_total = beta if beta_total is None else beta_total + beta
        terms = labels
        fit = fit_at(data, fitter, k, live)
    return fit, (list(zip(terms, np.round(beta_total, 4).tolist())) if beta_total is not None else [])


def constant_terms(ds, fitter, k, fit, harmonics, full, terms):
    """Per motor, the mean non-periodic force correction (offset + scale * F + direction * dir), N,
    and the same relative to the motors' mean (a common part trades with k and the anchors)."""
    if not full:
        return None
    lin = Linear(ds, fitter, k, fit)
    _, Phi, labels = design(lin, ds, harmonics, full)
    value = dict(terms)
    beta = np.array([value.get(label, 0.0) if not label[1].startswith(('sin', 'cos')) else 0.0 for label in labels])
    dF = Phi @ beta
    mean = {m: float(np.mean([v for (_, a), v in zip(lin.entries, dF) if a == m])) for m in lin.motors}
    common = float(np.mean(list(mean.values())))
    return {str(m): dict(mean=round(v, 4), relative=round(v - common, 4)) for m, v in mean.items()}


_CTX = {}


def _grid(k):
    c = _CTX
    harmonics = {}
    if c['harmonics']:
        harmonics, _ = select(Linear(c['ds'], c['fitter'], k, fit_at(c['ds'], c['fitter'], k, c['live'])), c['ds'], c['full'])
    fit, _ = refine(c['ds'], c['fitter'], k, c['live'], harmonics, c['full'])
    return float(fit['rms_mm']), {str(m): h for m, h in harmonics.items()}


def vertex(grid, rms):
    i = int(np.argmin(rms))
    j = min(max(i, 1), len(grid) - 2)
    c = np.polyfit(grid[j - 1:j + 2], rms[j - 1:j + 2], 2)
    return float(-c[1] / (2 * c[0])) if c[0] > 0 else float(grid[i])


def grid_k(ds, fitter, live, select_harmonics, full):
    """k by the production rms, with the force model selected and refitted at every grid k."""
    _CTX.update(ds=ds, fitter=fitter, live=live, harmonics=select_harmonics, full=full)
    jobs = int(os.environ.get('JOBS', 1))
    if jobs > 1:
        with multiprocessing.get_context('fork').Pool(jobs) as pool:
            points = pool.map(_grid, GRID)
    else:
        points = [_grid(k) for k in GRID]
    rms = np.array([p[0] for p in points])
    return vertex(GRID, rms), rms, [p[1] for p in points]


def main():
    run = Path(sys.argv[1])
    imp = json.loads((run / 'imperfection.json').read_text())
    fitter = 'ellipsoid' if imp.get('ellipsoid_shape') else 'sweep'
    ds = load(run)
    ds.setdefault('machine_type', imp['machine'])
    normalize_dataset_point_roles(ds)
    ds = perturb(ds)
    truth = convention_frame(np.array(list(imp['truth_mm'].values())))[0]
    live = np.asarray(json.loads((run / 'result.json').read_text())['fitted_mm'], float)
    k_true = 1 / imp['stiffness_scale']

    def score(fit, k):
        f = convention_frame(np.asarray(fit['anchors']))[0] - truth
        return dict(k=round(float(k), 4), k_over_true=round(float(k / k_true), 4), top_z_mm=round(float(f[-1, 2]), 3),
                    max_abs_mm=round(float(np.abs(f).max()), 3), rms_mm=round(float(fit['rms_mm']), 5))

    out = dict(run=run.name, machine=imp['machine'], seed=imp['seed'], k_true=k_true, perturb=os.environ.get('PERTURB'),
               cogging_freq=imp.get('cogging_freq'), hmax=HMAX, tau_n=TAU_N, noise_mm=NOISE_MM, cv_gain=CV_GAIN)
    k_plain, rms_plain, _ = grid_k(ds, fitter, live, False, False)
    out['plain'] = score(fit_at(ds, fitter, k_plain, live), k_plain)
    out['commanded_true_k'] = score(fit_at(ds, fitter, k_true, live), k_true)
    for variant, full in (('ripple', False), ('full', True)):
        k, rms, per_k = grid_k(ds, fitter, live, True, full)
        harmonics, trace = select(Linear(ds, fitter, k, fit_at(ds, fitter, k, live)), ds, full)
        history = [dict(k=round(k, 4), harmonics={str(m): h for m, h in harmonics.items()}, trace=trace),
                   dict(grid_harmonics=per_k)]
        fit, terms = refine(ds, fitter, k, live, harmonics, full, rounds=3)
        out[variant] = dict(score(fit, k), selection=history, terms=[[int(m), name, v] for (m, name), v in terms],
                            rms_grid=np.round(rms, 6).tolist(), constant_n=constant_terms(ds, fitter, k, fit, harmonics, full, terms))
    out['rms_plain_grid'] = np.round(rms_plain, 6).tolist()
    text = json.dumps(out)
    if len(sys.argv) > 2:
        Path(sys.argv[2]).write_text(text + '\n')
    print(text)


if __name__ == '__main__':
    main()
