"""E2: can per-anchor low z be fitted? Jacobian of the point-fit residuals at truth.

Parameters: every anchor coordinate except A.x (yaw gauge), plus the n-d home offsets.
The home point is the origin of every fit, so rotations about it (roll, pitch) are the
candidate null directions once low z is free. Reports the singular values and, with the
other parameters profiled out, the standard deviation of a roll/pitch tilt and of each
single-anchor height, at the dataset's fitted noise scale. `stretch` forces the
gravity-aware stretch correction on (the only term in which absolute z enters).
"""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from lowz_common import TRUTH, load, fit_default
from autocal.flex import FlexModel
from autocal.pose_fit import consistency_residuals, dataset_line_lengths, dataset_stretch

STEP = 1e-3


def forced_stretch(dataset, weight_scale=1.0):
    config = dataset["config"]; m666 = config["m666_before_data_collection"]
    n = int(dataset["num_anchors"])
    points = [(s, p) for s in dataset["sweeps"] for p in s["data_points"]]
    known = np.full((len(points), n), np.nan)
    for i, (_, p) in enumerate(points):
        for e in p.get("extra_sensor_ranges", []):
            known[i, int(e["anchor"])] = float(e["force_n"])
    return dict(model=FlexModel.from_m666(m666, num_axes=n),
                sensors=np.array([int(p.get("source_sensor_anchor", s["sensor_anchor"])) for s, p in points]),
                known=known, forces=np.array([p["sensor_force_n"] for _, p in points]),
                weight=float(m666.get("W", 0.)) * 9.81 * weight_scale,
                passive=float((config.get("force_tuning") or {}).get("force_low_n", 0.)))


def analyse(machine, stretch_mode):
    data = load(f"benchmark_923c2baa_{machine}")
    truth = TRUTH[machine]
    n, d = truth.shape
    lengths = dataset_line_lengths(data)["lengths_mm"]
    stretch = {"none": None, "dataset": dataset_stretch(data), "forced": forced_stretch(data),
               "forced_10xW": forced_stretch(data, 10.0)}[stretch_mode]
    if stretch_mode == "dataset" and stretch is None:
        return None
    fit, _ = fit_default(data)
    sigma = fit["robust_scale_mm"]
    mask = np.ones(n * d, bool); mask[0] = False  # A.x gauge
    no = n - d

    def res(x):
        a = truth.ravel().copy(); a[mask] += x[:mask.sum()]
        off = np.zeros(n); off[n - no:] = x[mask.sum():]
        return consistency_residuals(a.reshape(n, d), lengths, off, stretch=stretch).ravel()

    size = mask.sum() + no
    J = np.zeros((lengths.size, size))
    for i in range(size):
        e = np.zeros(size); e[i] = STEP
        J[:, i] = (res(e) - res(-e)) / (2 * STEP)
    info = J.T @ J / sigma ** 2
    info[-no:, -no:] += np.eye(no) / 5.0 ** 2  # home offset prior as in the fit
    sv = np.linalg.svd(J, compute_uv=False)
    cov = np.linalg.pinv(info, rcond=1e-14)

    out = dict(machine=machine, stretch=stretch_mode, sigma_mm=sigma, points=int(lengths.shape[0]),
               singular_values_smallest=[float(s) for s in sv[-5:]], cond=float(sv[0] / sv[-1]))
    for axis, name in ((0, "roll_x"), (1, "pitch_y")):
        w = np.zeros(3); w[axis] = 1.0
        pattern = np.cross(w, truth)  # rotation about home by 1 rad
        yaw = np.cross([0.0, 0.0, 1.0], truth)
        pattern = pattern - pattern[0, 0] / yaw[0, 0] * yaw  # plus the yaw that keeps A.x = 0
        # report std of the angle, in mrad, and the resulting mm at the top anchor
        J_v = J @ np.concatenate([pattern.ravel()[mask], np.zeros(no)])
        fixed_info = float(J_v @ J_v) / sigma ** 2
        out[name + "_fixed_std_mrad"] = 1e3 / np.sqrt(fixed_info) if fixed_info > 0 else float("inf")
        out[name + "_fixed_std_mm_at_D"] = out[name + "_fixed_std_mrad"] * 1e-3 * np.linalg.norm(truth[-1])
    # single-anchor height, others profiled: std of a_i.z from the covariance
    # pinv silently drops exact null directions, so a rigid model has no finite value here
    out["low_z_profiled_std_mm"] = None if stretch is None else [float(np.sqrt(abs(cov[list(np.flatnonzero(mask)).index(i * d + 2),
                                                        list(np.flatnonzero(mask)).index(i * d + 2)])))
                                    for i in range(n - 1)]
    return out


if __name__ == "__main__":
    rows = []
    for machine in TRUTH:
        for mode in ("none", "dataset", "forced", "forced_10xW"):
            row = analyse(machine, mode)
            if row:
                rows.append(row); print(json.dumps(row), flush=True)
    Path(sys.argv[1]).write_text(json.dumps(rows, indent=1))
