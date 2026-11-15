"""Independent joint range least squares for offline Hangprinter comparisons.

Unlike the production point fit, positions are optimization variables, not the
result of fixed-count inner Gauss-Newton steps. Fit raw observations and its own
offsets; never consume offsets estimated by the ellipse pipeline.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import coo_matrix

from autocal.pose_fit import dataset_line_lengths, HOME_OFFSET_PRIOR_SIGMA_MM
from autocal.projection_mode import resolve_projection_mode
from autocal.theoretical_ellipse import (
    anchors_matrix_to_opt_vec, anchor_opt_vec_to_matrix, get_anchor_opt_bounds,
)


def fit_joint_ranges(dataset, initial_anchors, *, fit_home_offsets=True, use_noise_mean=True):
    machine = dataset["machine_type"]
    if machine not in ("hangprinter_3", "hangprinter_4"):
        raise ValueError("Joint baseline currently supports Hangprinter only")
    if resolve_projection_mode(dataset) is True:
        raise ValueError("Joint baseline comparison of projected data is not validated")
    n, d = int(dataset["num_anchors"]), int(dataset["dimensions"])
    low_z = dataset.get("_low_anchor_z")
    lengths = dataset_line_lengths(dataset, use_noise_mean=use_noise_mean)["lengths_mm"]
    if not lengths.size or not np.all(np.isfinite(lengths)):
        raise ValueError("Joint baseline needs finite point measurements")
    count = len(lengths)
    lb, ub = get_anchor_opt_bounds(machine, n, d, low_z)
    vector = np.clip(anchors_matrix_to_opt_vec(initial_anchors, machine, low_z), lb, ub)
    m, o = len(vector), n - d if fit_home_offsets else 0

    def anchors_of(x):
        return anchor_opt_vec_to_matrix(x[:m], machine, n, d, low_z)

    anchors = anchors_of(vector)
    mapping = np.stack([anchors_of(vector + step) - anchors for step in np.eye(m)], axis=-1)
    absolute = np.linalg.norm(anchors, axis=1) + lengths
    squared = absolute**2 - np.sum(anchors**2, axis=1)
    positions = np.linalg.lstsq(-2 * (anchors[1:] - anchors[0]),
                               (squared[:, 1:] - squared[:, :1]).T, rcond=None)[0].T
    start = np.concatenate([vector, positions.ravel(), np.zeros(o)])
    bounds = (np.r_[lb, np.full(count*d + o, -np.inf)],
              np.r_[ub, np.full(count*d + o, np.inf)])
    rows = np.arange(count*n)
    a_rows = np.repeat(rows, m)
    a_cols = np.tile(np.arange(m), count*n)
    p_rows = np.repeat(rows, d)
    p_cols = m + np.repeat(np.arange(count)*d, n*d) + np.tile(np.arange(d), count*n)
    o_rows = np.tile(np.arange(n-o, n), count) + np.repeat(np.arange(count)*n, o)
    o_cols = m + count*d + np.tile(np.arange(o), count)

    def residuals(x, prior_weight):
        a = anchors_of(x)
        p = x[m:m+count*d].reshape(count, d)
        k = np.r_[np.zeros(n-o), x[m+count*d:]]
        misfit = np.linalg.norm(a[None] - p[:, None], axis=2) - np.linalg.norm(a, axis=1) - k - lengths
        return np.r_[misfit.ravel(), x[m+count*d:] * prior_weight]

    def jacobian(x, prior_weight):
        a = anchors_of(x)
        p = x[m:m+count*d].reshape(count, d)
        delta = a[None] - p[:, None]
        direction = delta / np.maximum(np.linalg.norm(delta, axis=2)[:, :, None], 1e-12)
        anchor_part = np.einsum("pnd,ndk->pnk", direction - a / np.linalg.norm(a, axis=1)[:, None], mapping)
        return coo_matrix((np.r_[anchor_part.ravel(), -direction.ravel(), -np.ones(count*o), np.full(o, prior_weight)],
                           (np.r_[a_rows, p_rows, o_rows, np.arange(count*n, count*n+o)],
                            np.r_[a_cols, p_cols, o_cols, np.arange(m+count*d, len(x))])),
                          shape=(count*n+o, len(x))).tocsr()

    def loss(kind):
        def evaluate(z):
            if kind == "soft_l1":
                root = np.sqrt(1 + z)
                rho = np.array([2*(root-1), 1/root, -.5/root**3])
            else:
                rho = np.array([np.log1p(z), 1/(1+z), -1/(1+z)**2])
            if o:
                rho[:, -o:] = np.array([z[-o:], np.ones(o), np.zeros(o)])
            return rho
        return evaluate

    options = dict(jac=jacobian, bounds=bounds, x_scale=np.r_[np.full(m+count*d, 100.), np.ones(o)],
                   max_nfev=1000, ftol=1e-8, xtol=1e-8, gtol=1e-8,
                   tr_options={"atol": 1e-12, "btol": 1e-12})
    first = least_squares(residuals, start, args=(.02/HOME_OFFSET_PRIOR_SIGMA_MM,),
                          loss=loss("soft_l1"), f_scale=1., **options)
    errors = first.fun[:count*n]
    scale = max(.02, 1.4826 * np.median(np.abs(errors - np.median(errors))))
    fit = least_squares(residuals, first.x, args=(scale/HOME_OFFSET_PRIOR_SIGMA_MM,),
                        loss=loss("cauchy"), f_scale=3*scale, **options)
    return {"anchors": anchors_of(fit.x), "home_offsets_mm": np.r_[np.zeros(n-o), fit.x[m+count*d:]],
            "rms_mm": float(np.sqrt(np.mean(np.sum(fit.fun[:count*n].reshape(count, n)**2, axis=1)))),
            "success": bool(fit.success), "message": str(fit.message), "nfev": first.nfev+fit.nfev,
            "optimality": float(fit.optimality),
            "first_pass": {"success": bool(first.success), "message": str(first.message),
                           "nfev": int(first.nfev), "optimality": float(first.optimality)}}
