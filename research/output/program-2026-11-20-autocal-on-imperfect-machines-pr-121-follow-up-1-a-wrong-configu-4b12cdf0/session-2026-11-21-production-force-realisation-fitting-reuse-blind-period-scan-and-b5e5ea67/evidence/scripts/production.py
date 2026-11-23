"""Paired replay of the production fit stage at fixed sweep coverage."""
import argparse
import gzip
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from time import perf_counter, process_time

import numpy as np

from autocal.fit_stage import fit_ellipse_dataset
from autocal.force_ripple import _adjust_ranges
from autocal.flex import FlexModel
from autocal.pose_fit import dataset_line_lengths, fit_point_positions, stretch_corrected_lengths
from autocal.tools.imperfect_machine import convention_frame, nominal_anchors


OPTIONS = dict(solve_restarts=1, solve_iterations=200, solve_optimizer="least_squares",
    pointwise_residual_mode="sampson", pointwise_filtering=True, pointwise_global_mad=False,
    sweep_wise_filtering=False, sweep_metric="sum", use_noise_mean=True, sigma_source="auto",
    robust_debug=False, residuals_csv=None, generate_report=False, find_radii="off",
    find_buildup_factor="off", base_radii=None, buildup_factor=None, r0_bounds=None,
    b_bounds=None, r0_prior_sigma_mm=None, b_prior_sigma=None, spool_outer_iters=3,
    theta0_mode="zero", line_width=1.0, sigma_floor_mm=None, fd_eps_mm=1.0,
    regularization=0.0, collector_args=[])


def load_run(run):
    path = run / "sweeps.json"
    if path.exists():
        return json.loads(path.read_text())
    with gzip.open(run / "sweeps.json.gz", "rt") as stream:
        return json.load(stream)


def synthetic_hp3_ranges(data, anchors, imp, mode):
    """First-order counterfactual at fixed poses with true EA and recorded encoder phases."""
    lengths = dataset_line_lengths(data)["lengths_mm"]
    config = data["config"]
    m666 = config["m666_before_data_collection"]
    points = [(s, p) for s in data["sweeps"] for p in s["data_points"]]
    sensors = np.array([p.get("source_sensor_anchor", s["sensor_anchor"]) for s, p in points])
    angles = np.deg2rad(np.array([p.get("mu") or p["raw_angles_deg"] for _, p in points]))
    angles = angles[np.arange(len(points)), sensors]
    base = np.array([p["sensor_force_n"] for _, p in points])
    phase = (imp.get("cogging_freq") or 50) * angles
    ripple = np.asarray(m666["U"])[sensors] * (imp.get("cogging_scale") or 1) / 3 * np.sin(phase)
    extra = {"double": ripple, "no-ripple": -ripple,
             "second-harmonic": 0.6 * np.sin(200 * angles + 1.0),
             "offset": 0.5 * (sensors == 1)}[mode]
    model = FlexModel.from_m666(m666, num_axes=4, spring_k_multiplier=imp["stiffness_scale"])
    positions, absolute = fit_point_positions(anchors, lengths, np.zeros(4))
    measured = np.isfinite(lengths)
    stretch = dict(model=model, sensors=sensors, known=np.full_like(lengths, np.nan), passive=0,
                   weight=m666["W"] * 9.81)
    original = stretch_corrected_lengths(anchors, positions, absolute, measured,
                                         dict(stretch, forces=base + ripple))
    changed = stretch_corrected_lengths(anchors, positions, absolute, measured,
                                       dict(stretch, forces=base + ripple + extra))
    return _adjust_ranges(data, original - changed, config["mm_per_degree"])


def paired(run, mode, prefix):
    data = json.loads(run.read_text()) if run.is_file() else load_run(run)
    imp = (dict(truth_mm=nominal_anchors(data["machine_type"])) if run.is_file()
           else json.loads((run / "imperfection.json").read_text()))
    anchors = np.asarray(list(imp["truth_mm"].values()))
    truth = convention_frame(anchors)[0]
    if prefix is not None:
        data["sweeps"] = data["sweeps"][:prefix]
    if mode in ("double", "no-ripple", "second-harmonic", "offset"):
        data = synthetic_hp3_ranges(data, anchors, imp, mode)
    if mode == "missing-angle":
        point = data["sweeps"][-1]["data_points"][0]
        point.pop("mu", None)
        point.pop("raw_angles_deg", None)
    result = dict(run=run.name, mode=mode, sweeps=len(data["sweeps"]))
    with TemporaryDirectory() as folder:
        path = Path(folder) / "sweeps.json"
        path.write_text(json.dumps(data))
        for enable in (False, True):
            start, cpu_start = perf_counter(), process_time()
            fit = fit_ellipse_dataset(path, fit_force_ripple=enable, **OPTIONS)
            t = perf_counter() - start
            error = convention_frame(fit["anchors"])[0] - truth
            result["candidate" if enable else "baseline"] = dict(
                max_mm=float(np.abs(error).max()), top_z_mm=float(error[-1, 2]),
                rms_mm=fit["pose_fit"]["rms_mm"], wall_s=t, cpu_s=process_time()-cpu_start, ripple=fit["force_ripple"],
                offsets_mm=fit["home_offsets_mm"].tolist(), anchors_mm=fit["anchors"].tolist())
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("runs", nargs="+", type=Path)
    p.add_argument("--mode", choices=["recorded", "double", "no-ripple", "second-harmonic", "offset",
                                     "missing-angle"], default="recorded")
    p.add_argument("--prefix", type=int)
    args = p.parse_args()
    for run in args.runs:
        print(json.dumps(paired(run, args.mode, args.prefix)), flush=True)
