"""How hidden pose error grows when one low anchor is off the shared plane (shared-low-z fit)."""
from itertools import product
import json
import numpy as np
from scipy.optimize import least_squares
from autocal import pose_fit
from autocal.theoretical_ellipse import anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec
from autocal.tools.regress_calibration_logs import BENCHMARK_MACHINES

M = "hangprinter_5_pulley_reduced"
nominal = np.asarray(dict(BENCHMARK_MACHINES)[M])
train = np.array(list(product((-300., -100., 100., 300.), (-250., 0., 250.), (100., 200.))))
evaluate = np.array(list(product((-600., 0., 600.), (-500., 0., 500.), (450., 850.))))
out = []
for offset in (2., 5., 10., 25.):
    anchors = nominal.copy(); anchors[1, 2] += offset
    rng = lambda p: np.linalg.norm(p[:, None] - anchors, axis=2) - np.linalg.norm(anchors, axis=1)
    unpack = lambda x: (anchor_opt_vec_to_matrix(x[:11], M, 5, 3), np.r_[np.zeros(3), x[11:13]])
    res = lambda x, L: pose_fit.consistency_residuals(unpack(x)[0], L, unpack(x)[1]).ravel()
    fit = least_squares(res, np.r_[anchors_matrix_to_opt_vec(nominal, M), 0., 0.], args=(rng(train),),
                        max_nfev=120, gtol=1e-11)
    geometry, offsets = unpack(fit.x)
    recovered, _ = pose_fit.fit_point_positions(geometry, rng(evaluate), offsets)
    out.append({"anchor_height_offset_mm": offset,
                "training_range_rms_mm": float(np.sqrt(np.mean(fit.fun ** 2))),
                "max_eval_pose_error_mm": float(np.max(np.linalg.norm(recovered - evaluate, axis=1)))})
print(json.dumps(out, indent=1))
