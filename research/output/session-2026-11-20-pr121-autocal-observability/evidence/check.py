"""Bounded observability checks using the PR's spool and point residuals."""

import argparse
from dataclasses import replace
from itertools import product
import json
from pathlib import Path
import platform

import numpy as np
import scipy
from scipy.optimize import least_squares

from autocal import pose_fit
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.spool_fit import spool_lengths_mm
from autocal.spool_model import WinchSpoolModel
from autocal.theoretical_ellipse import anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec
from autocal.tools.regress_calibration_logs import BENCHMARK_MACHINES


TRUTH = {machine: np.asarray(anchors) for machine, anchors in BENCHMARK_MACHINES}
HP4 = "hangprinter_4"
HP5R = "hangprinter_5_pulley_reduced"
REFERENCE = f"benchmark_923c2baa_{HP5R}.json"


def rms(values):
    return float(np.sqrt(np.mean(np.square(values))))


def spool_scale():
    anchors = TRUTH[HP4]
    points = np.array(list(product((-400.0, 0.0, 400.0), (-350.0, 150.0), (50.0, 400.0, 800.0))))
    lengths = np.linalg.norm(points[:, None] - anchors, axis=2) - np.linalg.norm(anchors, axis=1)
    radius = np.full(len(anchors), 30.0)
    buildup = np.full(len(anchors), 0.12)
    gearing = np.ones(len(anchors))
    advantage = np.array([2., 2., 2., 4.])
    strands = np.array([2., 2., 2., 3.])
    angles = np.empty_like(lengths)
    for axis in range(len(anchors)):
        model = WinchSpoolModel.from_firmware(
            base_radius=radius[axis], buildup_factor=buildup[axis],
            spool_to_motor_gearing_factor=gearing[axis], mechanical_advantage=advantage[axis],
            lines_per_spool=strands[axis],
        )
        angles[:, axis] = [model.linepos_mm_to_theta_deg(length) for length in lengths[:, axis]]
    assert np.max(np.abs(spool_lengths_mm(angles, radius, buildup, gearing, advantage, strands) - lengths)) < 1e-8

    cases = []
    for scale in (0.98, 1., 1.02):
        for scale_buildup in (False, True):
            if scale == 1. and scale_buildup:
                continue
            rebuilt = spool_lengths_mm(angles, radius * scale, buildup * (scale if scale_buildup else 1),
                                       gearing, advantage, strands)
            residual = pose_fit.consistency_residuals(anchors * scale, rebuilt, np.zeros(len(anchors)))
            cases.append({
                "scale": scale,
                "scale_buildup": scale_buildup,
                "point_range_rms_mm": rms(residual),
                "max_anchor_coordinate_error_mm": float(np.max(np.abs(anchors * (scale - 1)))),
                "ab_distance_change_mm": float(np.linalg.norm(anchors[0] - anchors[1]) * (scale - 1)),
                "physical_error_for_1000mm_command_mm": abs(1000 / scale - 1000),
            })
    assert max(row["point_range_rms_mm"] for row in cases if row["scale_buildup"]) < 1e-8
    assert min(row["point_range_rms_mm"] for row in cases if row["scale"] != 1 and not row["scale_buildup"]) > 1e-4
    return cases


def equilibrium(anchors, positions, load):
    directions = anchors - positions[:, None]
    directions /= np.linalg.norm(directions, axis=2)[..., None]
    indices = np.arange(len(positions))
    sensors = load["sensors"]
    known = load["known"].copy()
    known[indices, sensors] = load["forces"]
    assert np.all(np.isfinite(known).sum(axis=1) == 2)
    selectors = np.eye(len(anchors))[np.isfinite(known).nonzero()[1]].reshape(-1, 2, len(anchors))
    weight = np.broadcast_to([0., 0., load["weight"]], (len(positions), 3))
    right_hand_side = np.column_stack([weight, known[np.isfinite(known)].reshape(-1, 2)])
    return np.linalg.solve(np.concatenate([directions.transpose(0, 2, 1), selectors], axis=1),
                           right_hand_side[..., None])[..., 0]


def filter_load(load, keep):
    return {key: load[key][keep] if key in ("sensors", "known", "forces") else load[key] for key in load}


def repeated_load(dataset):
    """Compare equal-sized protocols with a local range-noise model."""
    anchors = TRUTH[HP5R]
    offsets = np.zeros(len(anchors))
    recorded = pose_fit.dataset_line_lengths(dataset)["lengths_mm"]
    load = pose_fit.dataset_stretch(dataset)
    complete = np.all(np.isfinite(recorded), axis=1)
    low = filter_load(load, complete)
    positions, _ = pose_fit.fit_point_positions(anchors, recorded[complete], offsets, stretch=low)
    high = {**low, "forces": 1.25 * low["forces"], "known": 1.25 * low["known"]}
    tension_low, tension_high = (equilibrium(anchors, positions, setting) for setting in (low, high))
    keep = np.min(np.minimum(tension_low, tension_high), axis=1) > 0.1
    low, high = (filter_load(setting, keep) for setting in (low, high))
    positions = positions[keep]
    assert len(positions) >= 12
    distances = np.linalg.norm(positions[:, None] - anchors, axis=2)
    home = np.linalg.norm(anchors, axis=1)
    measured = np.ones_like(distances, dtype=bool)
    x = np.r_[anchors_matrix_to_opt_vec(anchors, HP5R), 0., 0., 1.]

    def jacobian(setting):
        corrected_zero = pose_fit.stretch_corrected_lengths(anchors, positions, np.zeros_like(distances),
                                                            measured, setting)
        corrected_one = pose_fit.stretch_corrected_lengths(anchors, positions, np.ones_like(distances),
                                                           measured, setting)
        material = (distances - corrected_zero) / (corrected_one - corrected_zero)

        def residual(parameters):
            geometry = anchor_opt_vec_to_matrix(parameters[:-3], HP5R, len(anchors), 3)
            # Vary compliance on the low anchors, with the top line held at nominal stiffness.
            multiplier = np.r_[np.full(len(anchors) - 1, 1. / parameters[-1]), 1.].tolist()
            trial = {**setting, "model": replace(setting["model"], spring_k_multiplier=multiplier)}
            trial_offsets = np.r_[np.zeros(3), parameters[-3:-1]]
            return pose_fit.consistency_residuals(geometry, material - home, trial_offsets, stretch=trial).ravel()

        assert np.max(np.abs(residual(x))) < 1e-8
        columns = []
        for i, step in enumerate(np.r_[np.full(x.size - 1, .01), .001]):
            change = np.zeros_like(x)
            change[i] = step
            columns.append((residual(x + change) - residual(x - change)) / (2 * step))
        return np.column_stack(columns)

    ordinary = jacobian(low)
    alternate = jacobian(high)
    cases = {}
    for label, second in (("same_load_repeat", ordinary), ("25_percent_load_change", alternate)):
        joint = np.vstack([ordinary, second])
        geometry_and_home = joint[:, :-1]
        compliance = joint[:, -1]
        adjustment, _, _, _ = np.linalg.lstsq(geometry_and_home, -compliance, rcond=None)
        remaining = compliance + geometry_and_home @ adjustment
        cases[label] = {
            "point_range_rms_mm_for_10_percent_compliance_change_after_refit": rms(.1 * remaining),
            "top_z_change_mm_for_10_percent_compliance_change_after_refit": float(.1 * adjustment[-3]),
            "projected_signal_norm_mm_per_unit_compliance": float(np.linalg.norm(remaining)),
        }
    return {
        "recorded_rows": len(recorded),
        "complete_rows": int(complete.sum()),
        "poses_per_protocol": len(positions),
        "range_observations_per_protocol": 2 * len(positions) * len(anchors),
        "minimum_modeled_tension_n": float(np.min(np.minimum(tension_low[keep], tension_high[keep]))),
        "cases": cases,
    }


def noncoplanar_layout():
    """Compare one shared-plane parameterization to an extra height degree of freedom."""
    nominal = TRUTH[HP5R]
    anchors = nominal.copy()
    anchors[1, 2] += 25.
    training = np.array(list(product((-300., -100., 100., 300.), (-250., 0., 250.), (100., 200.))))
    evaluation = np.array(list(product((-600., 0., 600.), (-500., 0., 500.), (450., 850.))))

    def ranges(positions):
        return np.linalg.norm(positions[:, None] - anchors, axis=2) - np.linalg.norm(anchors, axis=1)

    def unpack(parameters, free_height):
        geometry = anchor_opt_vec_to_matrix(parameters[:11], HP5R, len(anchors), 3)
        if free_height:
            geometry[1, 2] += parameters[-1]
        return geometry, np.r_[np.zeros(3), parameters[11:13]]

    cases = {}
    start = np.r_[anchors_matrix_to_opt_vec(nominal, HP5R), 0., 0.]
    assert len(start) == 13
    training_ranges, evaluation_ranges = ranges(training), ranges(evaluation)
    for label, free_height in (("shared_lower_z", False), ("one_extra_height", True)):
        initial = np.r_[start, 0.] if free_height else start

        def residual(parameters, lengths):
            return pose_fit.consistency_residuals(*unpack_for_fit(parameters, lengths)).ravel()

        def unpack_for_fit(parameters, lengths):
            geometry, offsets = unpack(parameters, free_height)
            return geometry, lengths, offsets

        fit = least_squares(residual, initial, args=(training_ranges,), max_nfev=120, gtol=1e-11)
        assert fit.success
        recovered, _ = pose_fit.fit_point_positions(*unpack_for_fit(fit.x, evaluation_ranges))
        cases[label] = {
            "training_range_rms_mm": rms(fit.fun),
            "evaluation_range_rms_mm": rms(residual(fit.x, evaluation_ranges)),
            "max_evaluation_pose_error_mm": float(np.max(np.linalg.norm(recovered - evaluation, axis=1))),
        }
    return {"training_poses": len(training), "evaluation_poses": len(evaluation), "cases": cases}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference_directory", type=Path)
    directory = parser.parse_args().reference_directory
    dataset = json.loads((directory / REFERENCE).read_text())
    normalize_dataset_point_roles(dataset)
    print(json.dumps({
        "python": platform.python_version(),
        "numpy": np.__version__,
        "scipy": scipy.__version__,
        "reference_file": REFERENCE,
        "spool_scale": spool_scale(),
        "repeated_load": repeated_load(dataset),
        "noncoplanar_layout": noncoplanar_layout(),
    }, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
