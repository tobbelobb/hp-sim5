"""Synthetic HP3 equal-strain data with a chosen true line compliance.

Poses: each recorded point's position fitted at the true anchors (rigid).
Tensions: static equilibrium of the four lines with the recorded sensor force
and the effector weight. Lengths: RRF flex model, F*(U*d+Y)/(S*U^2*N), with
S = 20000 * stiffness_scale. Encoder zero is taken at home with zero tension
unless home_tension_n is given (then all lines carry it at home).
"""
from __future__ import annotations

import ast
import copy
import json

import numpy as np

from autocal._autocal_common import _normalize_dataset_point_roles
from autocal.anchor_seed import build_anchor_initial_guess
from autocal.pose_fit import (HOME_OFFSET_PRIOR_SIGMA_MM, dataset_line_lengths, fit_point_positions,
                              fit_pose_least_squares, shift_dataset_home_offsets)
from autocal.tools.regress_calibration_logs import BENCHMARK_MACHINES

TRUTH = np.asarray(dict(BENCHMARK_MACHINES)["hangprinter_3"], dtype=float)
S_NOMINAL = 20000.0


def load(path):
    dataset = json.load(open(path))
    _normalize_dataset_point_roles(dataset)
    m666 = dataset["config"]["m666_before_data_collection"]
    if isinstance(m666, str):
        dataset["config"]["m666_before_data_collection"] = ast.literal_eval(m666)
    return dataset


def line_params(dataset):
    m = dataset["config"]["m666_before_data_collection"]
    return (np.asarray(m["N"], float), np.broadcast_to(np.asarray(m["U"], float), (4,)),
            np.asarray(m["Y"], float), float(m["W"]) * 9.81)


def true_poses(dataset):
    lengths = dataset_line_lengths(dataset)["lengths_mm"]
    positions, _ = fit_point_positions(TRUTH, lengths, np.zeros(4))
    return positions


def sensor_lines(dataset):
    return np.array([int(p.get("source_sensor_anchor", s["sensor_anchor"])) for s in dataset["sweeps"] for p in s["data_points"]])


def sensor_forces(dataset):
    return np.array([float(p["sensor_force_n"]) for s in dataset["sweeps"] for p in s["data_points"]])


def equilibrium_tensions(anchors, positions, sensors, forces, weight):
    """Four line tensions with one known (the sensor) and gravity on the effector."""
    out = np.zeros((len(positions), 4))
    for k, (p, s, f) in enumerate(zip(positions, sensors, forces)):
        u = anchors - p
        u /= np.linalg.norm(u, axis=1)[:, None]
        others = [i for i in range(4) if i != s]
        rhs = np.array([0, 0, weight]) - f * u[s]
        t = np.linalg.solve(u[others].T, rhs)
        out[k, others], out[k, s] = t, f
    return out


def stretch(tension, distance, n, u, y, stiffness):
    return tension * (u * distance + y) / (stiffness * u ** 2 * n)


def synthesize(dataset, positions, stiffness_scale, *, home_tension_n=0.0, force_scale=1.0,
               force_error_n=None):
    """Replace every recorded length by the model's, keeping the dataset structure."""
    n, u, y, w = line_params(dataset)
    s = S_NOMINAL * stiffness_scale
    sensors, forces = sensor_lines(dataset), sensor_forces(dataset) * force_scale
    tensions = equilibrium_tensions(TRUTH, positions, sensors, forces, w)
    if force_error_n is not None:
        tensions = tensions + force_error_n
    distance = np.linalg.norm(TRUTH[None] - positions[:, None], axis=2)
    home = np.linalg.norm(TRUTH, axis=1)
    lengths = (distance - stretch(tensions, distance, n, u, y, s)) - (home - stretch(home_tension_n, home, n, u, y, s))
    out = copy.deepcopy(dataset)
    k = 0
    for sweep in out["sweeps"]:
        fixed = [int(a) for a in sweep["fixed_anchors"]]
        d, se = int(sweep["drive_anchor"]), int(sweep["sensor_anchor"])
        # Held lengths are per sweep; use the first point's value (held lines barely change).
        sweep["fixed_lengths"] = [float(lengths[k, a]) for a in fixed]
        for point in sweep["data_points"]:
            for key in ("fixed_error_angles_deg", "fixed_error_angles_mu_deg"):
                point[key] = None
            # Per-point held length deviation goes into the error angles.
            mm = dataset["config"]["mm_per_degree"]
            point["fixed_error_angles_deg"] = [None] * 4
            for a, base in zip(fixed, sweep["fixed_lengths"]):
                point["fixed_error_angles_deg"][a] = (lengths[k, a] - base) / mm[a]
            point["fixed_error_angles_mu_deg"] = list(point["fixed_error_angles_deg"])
            for key in ("l_drive", "l_drive_mu"):
                point[key] = float(lengths[k, d])
            for key in ("l_sensor", "l_sensor_mu"):
                point[key] = float(lengths[k, se])
            k += 1
    return out, tensions


def production_fit(dataset):
    """The sweep autocal's two-stage point fit (as in test_disturbed_robustness._fit)."""
    dataset = copy.deepcopy(dataset)
    seed = build_anchor_initial_guess(dataset, machine_type=dataset["machine_type"])
    offsets = fit_pose_least_squares(dataset, seed, fit_home_offsets=True,
                                     home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    fit = fit_pose_least_squares(shift_dataset_home_offsets(dataset, offsets["home_offsets_mm"]),
                                 offsets["anchors"], fit_home_offsets=False,
                                 home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    return fit, offsets["home_offsets_mm"]


def errors(anchors):
    e = np.asarray(anchors) - TRUTH
    return dict(max_coord=float(np.max(np.abs(e))), Dz=float(e[3, 2]), low_z=float(np.mean(e[:3, 2])),
                err=e.round(3).tolist())
