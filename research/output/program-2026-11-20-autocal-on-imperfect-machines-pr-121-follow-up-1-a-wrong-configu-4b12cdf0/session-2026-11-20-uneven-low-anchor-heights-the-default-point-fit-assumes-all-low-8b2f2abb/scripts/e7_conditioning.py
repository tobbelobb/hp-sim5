"""E7: conditioning of the point fit and the D.z / stretch-scale coupling, per gauge.

Runs with whichever autocal is on PYTHONPATH (old equal-z or new vertical-top gauge).
Jacobian at truth of the range residuals over: the code's reduced anchor vector, the n-d
home offsets (5 mm prior, as the fit), and a global stretch scale k that multiplies the
gravity-aware stretch correction of every range (forced on, with recorded sensor forces).
Reports singular values, per-coordinate std, D.z std with k fixed and with k free
(k prior sigma 0.2 and free), corr(D.z, k), and dD.z/dk along the profiled direction,
at the fit's noise scale. Truth is level, so the old gauge is not biased here.
"""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from lowz_common import TRUTH, load, fit_default
from e2_identifiability import forced_stretch
import autocal.pose_fit as pf
import autocal.theoretical_ellipse as te

STEP = 1e-3
base_correct = pf.stretch_corrected_lengths
state = {"k": 1.0}

def scaled(anchors, positions, absolute, measured, stretch):
    corrected = base_correct(anchors, positions, absolute, measured, stretch)
    return absolute + state["k"] * (corrected - absolute)

pf.stretch_corrected_lengths = scaled


def analyse(machine):
    data = load(f"benchmark_923c2baa_{machine}"); truth = TRUTH[machine]; n, d = truth.shape; no = n - d
    lengths = pf.dataset_line_lengths(data)["lengths_mm"]
    stretch = forced_stretch(data)
    sigma = fit_default(data)[0]["robust_scale_mm"]
    vec = te.anchors_matrix_to_opt_vec(truth, machine)
    m = vec.size

    def res(x):
        a = te.anchor_opt_vec_to_matrix(vec + x[:m], machine, n, d)
        off = np.zeros(n); off[n - no:] = x[m:m + no]
        state["k"] = 1.0 + x[-1]
        r = pf.consistency_residuals(a, lengths, off, stretch=stretch).ravel()
        state["k"] = 1.0
        return r

    size = m + no + 1
    J = np.stack([(res(e) - res(-e)) / (2 * STEP) for e in np.eye(size) * STEP], axis=1)
    info = J.T @ J / sigma ** 2
    info[m:m + no, m:m + no] += np.eye(no) / 5.0 ** 2
    sv = np.linalg.svd(J[:, :m + no], compute_uv=False)
    zD = m - 1 if te.anchor_opt_vector_size(machine, n, d) == 3 * n - 3 else m - 1  # top z is last anchor param
    out = dict(machine=machine, gauge=te.__file__, params=m, sigma_mm=sigma,
               smallest_sv_anchor_offsets=[float(v) for v in sv[-3:]], cond=float(sv[0] / sv[-1]))
    fixed_k = np.linalg.inv(info[:m + no, :m + no])
    out["std_mm_k_fixed"] = np.sqrt(np.diag(fixed_k)[:m]).round(3).tolist()
    out["Dz_std_k_fixed"] = float(np.sqrt(fixed_k[zD, zD]))
    for label, prior in (("k_prior_0.2", 1 / 0.2 ** 2), ("k_free", 0.0)):
        A = info.copy(); A[-1, -1] += prior
        cov = np.linalg.inv(A)
        out[f"Dz_std_{label}"] = float(np.sqrt(cov[zD, zD]))
        out[f"k_std_{label}"] = float(np.sqrt(cov[-1, -1]))
        out[f"corr_Dz_k_{label}"] = float(cov[zD, -1] / np.sqrt(cov[zD, zD] * cov[-1, -1]))
        out[f"dDz_dk_{label}"] = float(cov[zD, -1] / cov[-1, -1])
        out[f"max_std_mm_{label}"] = float(np.sqrt(np.max(np.diag(cov)[:m])))
    return out


if __name__ == "__main__":
    rows = [analyse(m) for m in TRUTH]
    for r in rows:
        print(json.dumps({k: v for k, v in r.items() if k not in ("std_mm_k_fixed",)}))
    Path(sys.argv[1]).write_text(json.dumps(rows, indent=1))
