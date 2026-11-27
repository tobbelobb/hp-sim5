import sys
from pathlib import Path

import numpy as np
import pytest

from autocal.compliance import estimate_compliance_scale
from autocal.flex import FlexModel
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, fit_pose_least_squares
from autocal.swivel_pulley import (
    home_line_lengths,
    line_geometry,
    swivel_geometry_from_m669,
    swivel_length_and_exit,
)

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "flex-compensation-dev/hangprinter-flex-compensation"))
from test_swivel import reference as swivel_reference  # noqa: E402

# COPacabana (DaRUS-4614) anchors, swivel axes, pulley radii, and home tensions (mm / N)
ENTRIES = np.array([
    [-2427.578, 1959.987, 2157.96],
    [2411.243, 1944.128, 2170.735],
    [2408.63, -1976.237, 2149.285],
    [-2433.437, -1965.415, 2158.603],
    [-2298.856, 1609.864, -925.174],
    [2315.327, 1615.126, -848.102],
    [2308.631, -1613.281, -925.031],
    [-2385.985, -1614.363, -846.768],
])
ATTACHMENTS = np.array([
    [-150, 236.5, 22.5], [150, 236.5, 22.5], [150, -236.5, 22.5], [-150, -236.5, 22.5],
    [-200, 150, 575], [200, 150, 575], [200, -150, 575], [-200, -150, 575],
], dtype=float)
ANCHORS = ENTRIES - ATTACHMENTS
AXES = np.array([[-1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]] * 2)
RADII = np.array([37.0] * 4 + [33.0] * 4)
HOME_TENSION = np.array([90.51, 84.99, 90.51, 84.15, 40.16, 34.93, 35.64, 30.83])


def test_swivel_length_and_exit_matches_reference_implementation():
    rng = np.random.default_rng(4614)
    positions = rng.uniform([-700, -700, -300], [700, 700, 800], size=(25, 3))
    lengths, exits = swivel_length_and_exit(ANCHORS, AXES, RADII, positions)
    for k, pos in enumerate(positions):
        ref = [swivel_reference(a, u, r, pos) for a, u, r in zip(ANCHORS, AXES, RADII)]
        ref_len = np.array([row[0] for row in ref])
        ref_exit = np.array([row[1] for row in ref])
        np.testing.assert_allclose(lengths[k], ref_len, atol=1e-8)
        np.testing.assert_allclose(exits[k], ref_exit, atol=1e-8)


def test_swivel_envelope_theorem_gradient_matches_dir_to_anchor():
    m669 = {"R": RADII.tolist(), "V": AXES.ravel().tolist()}
    swivel = swivel_geometry_from_m669(m669, len(ANCHORS))
    assert swivel is not None
    p = np.array([[320.0, -240.0, 180.0]])
    _, dir_to_anchor = line_geometry(ANCHORS, p, swivel=swivel)
    eps = 1e-4
    fd_grad = np.zeros((len(ANCHORS), 3))
    for j in range(3):
        dp = np.zeros((1, 3))
        dp[0, j] = eps
        lp, _ = line_geometry(ANCHORS, p + dp, swivel=swivel)
        lm, _ = line_geometry(ANCHORS, p - dp, swivel=swivel)
        fd_grad[:, j] = (lp[0] - lm[0]) / (2.0 * eps)
    np.testing.assert_allclose(fd_grad, -dir_to_anchor[0], atol=1e-7)


def test_swivel_geometry_from_m669_handles_zero_and_invalid():
    assert swivel_geometry_from_m669(None, 4) is None
    assert swivel_geometry_from_m669({"R": [0.0] * 4, "V": [1.0, 0.0, 0.0] * 4}, 4) is None
    assert swivel_geometry_from_m669({"R": [30.0] * 4, "V": [0.0] * 12}, 4) is None
    assert swivel_geometry_from_m669({"R": [30.0] * 4, "V": [1.0, 0.0]}, 4) is None


def test_swivel_and_home_tension_pose_fit_recovers_copacabana_anchors():
    """Synthetic 8-anchor COPacabana dataset with swivel pulleys, home tension, and flex."""
    m669 = {"R": RADII.tolist(), "V": AXES.ravel().tolist()}
    m666 = {
        "S": 118500.0,
        "W": 13.9,
        "U": [1.0] * 8,
        "N": [1.0] * 8,
        "Y": [200.0] * 8,
        "T": HOME_TENSION.tolist(),
    }
    swivel = swivel_geometry_from_m669(m669, 8)
    model = FlexModel.from_m666(m666, num_axes=8)
    assert swivel is not None and model is not None
    home = home_line_lengths(ANCHORS, swivel=swivel)

    rng = np.random.default_rng(7)
    positions = rng.uniform([-600, -600, -200], [600, 600, 600], size=(48, 3))
    dist, dir_to_a = line_geometry(ANCHORS, positions, swivel=swivel)

    # Build 6 synthetic sweeps of 8 poses each with 2 fixed + 1 drive (3 position lines)
    # and 1 sensor + 4 extra force-mode lines (exact statically determinate 8-line equilibrium).
    sweeps = []
    for s_idx in range(6):
        pts = []
        fixed = [s_idx % 4, (s_idx + 1) % 4]
        drive = (s_idx + 2) % 4
        sensor = 4 + (s_idx % 4)
        extras = [a for a in range(8) if a not in fixed and a not in (drive, sensor)]
        for k in range(8):
            p_idx = s_idx * 8 + k
            sensor_f = 35.0
            extra_f = 30.0
            known = np.full(8, np.nan)
            known[sensor] = sensor_f
            for ex in extras:
                known[ex] = extra_f
            sel = np.eye(8) * np.isfinite(known)[:, None]
            mat = np.concatenate([dir_to_a[p_idx].T, sel], axis=0)
            load = np.array([0.0, 0.0, 13.9 * 9.81])
            rhs = np.concatenate([load, np.nan_to_num(known)])
            tensions = np.linalg.lstsq(mat, rhs, rcond=None)[0]
            # Forward measured length delta including elastic flex and home tension.
            flex = np.array([
                max(tensions[a], 0.0) * (dist[p_idx, a] + 200.0) / 118500.0
                - model.home_flex_mm(float(home[a]), axis=a)
                for a in range(8)
            ])
            l_meas = dist[p_idx] - home - flex
            err_deg = [float(l_meas[a]) if a in fixed else None for a in range(8)]
            pts.append({
                "l_drive": float(l_meas[drive]),
                "l_sensor": float(l_meas[sensor]),
                "sensor_force_n": sensor_f,
                "fixed_error_angles_deg": err_deg,
                "extra_sensor_ranges": [
                    {"anchor": int(ex), "length_mm": float(l_meas[ex]), "force_n": extra_f, "accepted": True}
                    for ex in extras
                ],
            })
        sweeps.append({
            "id": f"sweep_{s_idx + 1:03d}",
            "fixed_anchors": fixed,
            "fixed_lengths": [0.0] * len(fixed),
            "drive_anchor": drive,
            "sensor_anchor": sensor,
            "data_points": pts,
        })

    dataset = {
        "machine_type": "cubecorners",
        "num_anchors": 8,
        "dimensions": 3,
        "config": {"mm_per_degree": [1.0] * 8, "m666_before_data_collection": m666, "m669": m669},
        "sweeps": sweeps,
    }
    seed = ANCHORS + rng.uniform(-35.0, 35.0, size=ANCHORS.shape)
    fit_swivel = fit_pose_least_squares(
        dataset, seed, fit_home_offsets=False, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM
    )
    assert fit_swivel["success"]
    assert fit_swivel["rms_mm"] < 1e-4

    def procrustes_err(est, ref):
        u, _, vt = np.linalg.svd(est.T @ ref)
        rot = u @ vt
        if np.linalg.det(rot) < 0:
            u[:, -1] *= -1
            rot = u @ vt
        return float(np.max(np.abs(est @ rot - ref)))

    assert procrustes_err(fit_swivel["anchors"], ANCHORS) < 0.05

    # Compare against ignoring swivel geometry: pinhole fit is biased by several mm.
    ds_no_swivel = {**dataset, "config": {"mm_per_degree": [1.0] * 8, "m666_before_data_collection": m666}}
    fit_pinhole = fit_pose_least_squares(
        ds_no_swivel, seed, fit_home_offsets=False, home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM
    )
    assert fit_pinhole["rms_mm"] > 0.5
    assert procrustes_err(fit_pinhole["anchors"], ANCHORS) > 5.0


def test_compliance_probe_works_with_swivel_pulleys():
    m669 = {"R": RADII.tolist(), "V": AXES.ravel().tolist()}
    swivel = swivel_geometry_from_m669(m669, 8)
    home = home_line_lengths(ANCHORS, swivel=swivel)
    mm_per_deg = np.full(8, 0.5)
    ea_cfg, ea_true = 118500.0, 95000.0
    pos = np.array([250.0, -180.0, 120.0])
    dist, dir_to_a = line_geometry(ANCHORS, pos[None, :], swivel=swivel)
    dist, unit = dist[0], dir_to_a[0]
    c_true = (dist + 200.0) / ea_true
    held = [0, 1, 2, 3, 4]
    sensor = 5
    levels = []
    for f in (30.0, 60.0, 90.0):
        df = f - 30.0
        d_t = np.zeros(8)
        d_t[held] = np.linalg.lstsq(unit[held].T, -df * unit[sensor], rcond=None)[0]
        d_t[sensor] = df
        dp = np.linalg.lstsq(unit[held], -c_true[held] * d_t[held], rcond=None)[0]
        dl = np.zeros(8)
        dl[sensor] = float(-unit[sensor] @ dp - c_true[sensor] * df)
        for j in (6, 7):
            dl[j] = float(-unit[j] @ dp)
        levels.append({"force_n": f, "mu_deg": ((dist - home + dl) / mm_per_deg).tolist()})
    ds = {
        "machine_type": "cubecorners",
        "num_anchors": 8,
        "dimensions": 3,
        "config": {
            "mm_per_degree": mm_per_deg.tolist(),
            "m666_before_data_collection": {"S": ea_cfg, "U": [1.0] * 8, "N": [1.0] * 8, "Y": [200.0] * 8},
            "m669": m669,
        },
        "sweeps": [{
            "fixed_anchors": [0, 1, 2, 3],
            "fixed_lengths": [0.0] * 4,
            "drive_anchor": 4,
            "sensor_anchor": sensor,
            "metadata": {"compliance_probes": [{"drive_anchor": 4, "sensor_anchor": sensor, "levels": levels}]},
            "data_points": [],
        }],
    }
    est = estimate_compliance_scale(ds, ANCHORS)
    assert est is not None
    assert est["stiffness_n"] == pytest.approx(ea_true, rel=1e-3)
