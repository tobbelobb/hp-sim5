"""Fold-structure diagnostics for the sweep jackknife (research helper, mirrors the production rule).

For each leave-one-unit-out fold, the analytic information the kept points carry about every
anchor coordinate (pose_planning.point_informations at the full fit's poses, home-offset prior
as in the fit) gives an information-implied std. ratio[i, c] = std_fold_i[c] / std_full[c].
A cell with ratio > R lost most of the constraint on that coordinate when unit i was left out.
"""
import numpy as np
from autocal.dataset_roles import measurement_groups
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, dataset_line_lengths, fit_point_positions
from autocal.pose_planning import point_informations
import autocal.theoretical_ellipse as te


def fold_ratios(data, anchors):
    anchors = np.asarray(anchors, float); n, d = anchors.shape; machine = data["machine_type"]
    rec = dataset_line_lengths(data); lengths, sweep_of = rec["lengths_mm"], rec["sweep_index"]
    positions, _ = fit_point_positions(anchors, lengths, np.zeros(n))
    infos = point_informations(anchors, lengths, positions, machine_type=machine, sigma_mm=1.0)
    no = n - d; m = infos.shape[1] - no
    prior = np.zeros(infos.shape[1:]); prior[m:, m:] = np.eye(no) / HOME_OFFSET_PRIOR_SIGMA_MM ** 2
    vec = te.anchors_matrix_to_opt_vec(anchors, machine)
    mapping = np.stack([(te.anchor_opt_vec_to_matrix(vec + e, machine, n, d)
                         - te.anchor_opt_vec_to_matrix(vec - e, machine, n, d)).ravel() / 2e-3
                        for e in np.eye(m) * 1e-3], axis=1)

    def coord_std(mask):
        info = infos[mask].sum(axis=0) + prior
        cov = np.linalg.pinv(info, rcond=1e-12)[:m, :m]
        return np.sqrt(np.clip(np.diag(mapping @ cov @ mapping.T), 0, None)).reshape(n, d)

    full = coord_std(np.ones(len(lengths), bool))
    groups = measurement_groups(data)
    ratios = []
    for group in groups:
        keep = ~np.isin(sweep_of, list(group))
        with np.errstate(divide="ignore", invalid="ignore"):
            ratios.append(np.where(full > 1e-9, coord_std(keep) / np.where(full > 1e-9, full, 1), 1.0))
    return np.stack(ratios), full


def classical(folds):
    f = np.asarray(folds); k = len(f)
    return np.sqrt((k - 1) / k * np.sum((f - f.mean(0)) ** 2, axis=0))


def structural_spread(folds, ratios, limit, min_kept=3, max_lost=1):
    """Classical spread per coordinate, leaving out a fold only where it alone lost that coordinate.

    A coordinate whose constraint rests on one unit has exactly one fold with ratio > limit.
    That fold is excluded for that coordinate when at least `min_kept` folds remain. Every
    other coordinate, including ones that several folds lose (weak everywhere), keeps the
    classical spread over all folds.
    """
    folds = np.asarray(folds); flagged = np.asarray(ratios) > limit; n = len(folds)
    spread = classical(folds); limited = []
    for idx in np.ndindex(*folds.shape[1:]):
        lost = flagged[(slice(None), *idx)]
        if 1 <= lost.sum() <= max_lost and n - lost.sum() >= min_kept:
            spread[idx] = classical(folds[(slice(None), *idx)][~lost][:, None])[0]
            limited.append(idx)
    return float(spread.max()), limited
