"""Replay the Fable proof of principle and diagnose its integration gap.

No simulator starts. The archived fitter uses true-anchor initialization and
position seeds; its result is a local diagnostic, not a deployable calibration.
Current point/joint fits start from a fixed perturbation of true anchors and
derive their positions from observed lengths. All errors use simulator truth.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from scipy.optimize import least_squares

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "autocal/pose_fit.py").exists())
ARCHIVE = ROOT / "research/output/autocal_research_with_Fable5p2/evidence"
PROGRAM = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ARCHIVE))
from design2 import bias_k
from elastic import TRUE
from autocal.pose_fit import dataset_line_lengths, fit_pose_least_squares, HOME_OFFSET_PRIOR_SIGMA_MM
from autocal.tools.joint_pose_baseline import fit_joint_ranges

SEED = TRUE + np.array([[0, 30, 10], [-20, 15, 10], [20, -15, 10], [0, 0, 30.]])
SUBSET_SEED = 20261115


def metrics(error, rms, offset):
    return {"absolute_mm": float(np.abs(error).max()),
            "aligned_mm": float(np.abs(error - error.mean(0)).max()),
            "mean_translation_mm": error.mean(0).tolist(),
            "reported_rms_mm": float(rms), "offset_mm": float(offset)}


def original_fit(positions, lengths):
    error, rms, offset = bias_k(positions, lengths)
    return metrics(error, rms, offset)


def point_dataset(points):
    # Preserve all four encoder lengths exactly. The one-point groups are for
    # range fitting, not valid ellipse sweeps or a simulated collection protocol.
    sweeps = []
    for i, point in enumerate(points):
        sensor = "ABCD".index(point["torqueAxis"])
        drive = next(k for k in range(4) if k != sensor)
        fixed = [k for k in range(4) if k not in (drive, sensor)]
        lengths = point["enc_mm"]
        sweeps.append({"id": str(i), "drive_anchor": drive, "sensor_anchor": sensor,
                       "fixed_anchors": fixed, "fixed_lengths": [lengths[k] for k in fixed],
                       "data_points": [{"l_drive": lengths[drive], "l_sensor": lengths[sensor]}]})
    dataset = {"version": "1.0", "machine_type": "hangprinter_3", "num_anchors": 4,
               "dimensions": 3, "sweeps": sweeps}
    np.testing.assert_allclose(dataset_line_lengths(dataset)["lengths_mm"],
                               [p["enc_mm"] for p in points], rtol=0, atol=1e-12)
    return dataset


def current_fits(dataset):
    point = fit_pose_least_squares(dataset, SEED, fit_home_offsets=True,
                                  home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    joint = fit_joint_ranges(dataset, SEED)
    return {name: {**metrics(fit["anchors"] - TRUE, fit["rms_mm"], fit["home_offsets_mm"][3]),
                   "success": fit["success"]}
            for name, fit in [("production_point", point), ("independent_joint", joint)]}


def hash_file(path):
    return {"path": str(path.relative_to(ROOT)), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    inputs = []
    direct, subsets = [], []
    for name in ("v_equal10", "v_equal15", "v_equal25"):
        path = ARCHIVE / "sim_results" / f"{name}.out.json"
        inputs.append(hash_file(path))
        saved = json.loads(path.read_text())
        points = saved["points"]
        positions = np.array([p["pos_mm"] for p in points])
        lengths = np.array([p["enc_mm"] for p in points])
        direct.append({"case": name, "points": len(points), "solver_iterations": saved["iters"],
                       "recorded_z_range_mm": [float(positions[:, 2].min()), float(positions[:, 2].max())],
                       "archived_fable": original_fit(positions, lengths), **current_fits(point_dataset(points))})
        low = np.flatnonzero(positions[:, 2] < 450)
        matched = np.sort(np.random.default_rng(SUBSET_SEED).choice(len(points), size=len(low), replace=False))
        subsets.append({"case": name, "subset_seed": SUBSET_SEED,
                        "below_450_mm": {"indices": low.tolist(), "points": len(low),
                                         **original_fit(positions[low], lengths[low])},
                        "same_count_full_height": {"indices": matched.tolist(), "points": len(matched),
                                                   **original_fit(positions[matched], lengths[matched])}})
    path = PROGRAM / "validation/hp3-stateful/sweeps.json"
    inputs.append(hash_file(path))
    dataset = json.loads(path.read_text())
    lengths = dataset_line_lengths(dataset)["lengths_mm"]
    absolute = lengths + np.linalg.norm(TRUE, axis=1)
    squared = absolute ** 2 - np.sum(TRUE ** 2, axis=1)
    starts = np.linalg.lstsq(-2 * (TRUE[1:] - TRUE[0]), (squared[:, 1:] - squared[:, :1]).T, rcond=None)[0].T
    # Reconstructed from biased lengths, not recorded physical positions.
    positions = np.array([least_squares(lambda p: np.linalg.norm(TRUE - p, axis=1) - row, start).x
                          for row, start in zip(absolute, starts)])
    production = {"points": len(lengths), "reconstructed_z_range_mm": [float(positions[:, 2].min()), float(positions[:, 2].max())],
                  "reconstructed_z_percentiles_mm": np.percentile(positions[:, 2], [0, 25, 50, 75, 100]).tolist(),
                  "planner_max_travel_mm": dataset["config"]["max_travel_mm"],
                  "archived_fable": original_fit(positions, lengths), **current_fits(dataset)}
    output = {"source_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
              "inputs": inputs,
              "fitter_source_hashes": [hash_file(ARCHIVE / name) for name in ("design.py", "design2.py", "elastic.py")]
                                      + [hash_file(ROOT / "autocal" / name) for name in ("pose_fit.py", "tools/joint_pose_baseline.py")],
              "direct_recordings": direct, "height_subsets": subsets, "production_recording": production,
              "scope": "Offline replay only; no fresh physics. No ellipse fit on arbitrary one-point groups. Local solver results; truth-based diagnostic initialization. Height subsets also change XY/force composition."}
    target = Path(__file__).with_name("results.json")
    target.write_text(json.dumps(output, indent=2) + "\n")
    print(target)


if __name__ == "__main__":
    main()
