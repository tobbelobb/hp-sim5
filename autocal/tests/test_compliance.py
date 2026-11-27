import numpy as np
import pytest

from autocal import autocal as ac
from autocal._autocal_common import _m666_from_plan
from autocal.compliance import (
    estimate_compliance_scale,
    has_compliance_probes,
    resolve_measured_stiffness,
)
from autocal.headless_sim import HeadlessSimulation
from autocal.swivel_pulley import home_line_lengths, line_geometry, swivel_geometry_from_m669

HP3_ANCHORS = np.array([
    [0.0, -1900.0, -280.0],
    [1645.45, 950.0, -280.0],
    [-1645.45, 950.0, -280.0],
    [0.0, 0.0, 2400.0],
])


def _synthetic_probe_dataset(
    anchors,
    *,
    ea_cfg=20000.0,
    ea_true=16000.0,
    ma=None,
    strands=None,
    guy=None,
    m669=None,
    outlier_step=False,
):
    anchors = np.asarray(anchors, dtype=float)
    n, d = anchors.shape
    ma_arr = np.ones(n) if ma is None else np.asarray(ma, dtype=float)
    strands_arr = np.ones(n) if strands is None else np.asarray(strands, dtype=float)
    guy_arr = np.zeros(n) if guy is None else np.asarray(guy, dtype=float)
    swivel = swivel_geometry_from_m669(m669, n)
    home = home_line_lengths(anchors, swivel=swivel)
    mm_per_deg = np.full(n, 0.5)

    def solve_probe_pose(held, sensor, pos_base, f_base, force_levels, corrupt=False):
        dist0, dir0 = line_geometry(anchors, pos_base[None, :], swivel=swivel)
        dist0 = dist0[0]
        unit = dir0[0]
        c_true = (ma_arr * dist0 + guy_arr) / (ea_true * (ma_arr ** 2) * strands_arr)
        l0 = dist0 - home
        levels = []
        for idx, f in enumerate(force_levels):
            df = float(f - f_base)
            d_t = np.zeros(n)
            d_t[held] = np.linalg.lstsq(unit[held].T, -df * unit[sensor], rcond=None)[0]
            d_t[sensor] = df
            # Held lines keep constant encoder length dm_held = 0 -> u_h . dp = -c_h * dT_h
            dp = np.linalg.lstsq(unit[held], -c_true[held] * d_t[held], rcond=None)[0]
            # Sensor encoder change: dm_s = -u_s . dp - c_s * dF
            dl = np.zeros(n)
            dl[sensor] = float(-unit[sensor] @ dp - c_true[sensor] * df)
            for j in range(n):
                if j not in held and j != sensor:
                    dl[j] = float(-unit[j] @ dp)
            if corrupt and idx == 2:
                dl[sensor] += 15.0
            levels.append({
                "force_n": float(f),
                "mu_deg": ((l0 + dl) / mm_per_deg).tolist(),
            })
        return levels

    poses = [
        (np.array([150.0, -120.0, 200.0]), [0, 3], 1, 2),
        (np.array([-180.0, 140.0, 350.0]), [1, 3], 2, 0),
        (np.array([90.0, 210.0, 100.0]), [2, 3], 0, 1),
    ]
    sweeps = []
    for k, (pos, fixed, drive, sensor) in enumerate(poses):
        held = sorted(set(fixed) | {drive})
        levels = solve_probe_pose(held, sensor, pos, 2.0, [2.0, 4.0, 6.0, 2.0], corrupt=(outlier_step and k == 0))
        sweeps.append({
            "id": f"sweep_{k + 1:03d}",
            "fixed_anchors": fixed,
            "fixed_lengths": [0.0] * len(fixed),
            "drive_anchor": drive,
            "sensor_anchor": sensor,
            "metadata": {
                "compliance_probes": [{
                    "direction": 0,
                    "drive_anchor": drive,
                    "sensor_anchor": sensor,
                    "levels": levels,
                }]
            },
            "data_points": [],
        })
    return {
        "machine_type": "hangprinter_3" if n == 4 else "cubecorners",
        "num_anchors": n,
        "dimensions": d,
        "config": {
            "mm_per_degree": mm_per_deg.tolist(),
            "m666_before_data_collection": {
                "S": ea_cfg,
                "U": ma_arr.tolist(),
                "N": strands_arr.tolist(),
                "Y": guy_arr.tolist(),
            },
            **({"m669": m669} if m669 is not None else {}),
        },
        "sweeps": sweeps,
    }


@pytest.mark.parametrize("ea_true", [16000.0, 20000.0, 25000.0, 40000.0])
def test_estimate_compliance_scale_recovers_true_stiffness(ea_true):
    ds = _synthetic_probe_dataset(
        HP3_ANCHORS,
        ea_cfg=20000.0,
        ea_true=ea_true,
        ma=[2.0, 2.0, 2.0, 4.0],
        strands=[2.0, 2.0, 2.0, 4.0],
        guy=[300.0, 300.0, 300.0, 150.0],
    )
    assert has_compliance_probes(ds)
    est = estimate_compliance_scale(ds, HP3_ANCHORS)
    assert est is not None
    assert est["source"] == "compliance_probe"
    assert est["stiffness_n"] == pytest.approx(ea_true, rel=1e-3)
    assert est["k_hat"] == pytest.approx(20000.0 / ea_true, rel=1e-3)
    assert est["spring_k_multiplier"] == pytest.approx(ea_true / 20000.0, rel=1e-3)
    assert est["num_steps"] == 6


def test_estimate_compliance_scale_resists_outlier_probe_step():
    ds = _synthetic_probe_dataset(HP3_ANCHORS, ea_cfg=20000.0, ea_true=16000.0, outlier_step=True)
    est = estimate_compliance_scale(ds, HP3_ANCHORS)
    assert est is not None
    assert est["stiffness_n"] == pytest.approx(16000.0, rel=0.05)


def test_resolve_measured_stiffness_prefers_probe_and_falls_back_to_varpro():
    ds = _synthetic_probe_dataset(HP3_ANCHORS, ea_cfg=20000.0, ea_true=16000.0)
    vp = {"k_grav_identifiable": True, "k_grav_hat": 1.0, "ea_estimated_n": 20000.0, "ea_cfg_n": 20000.0}
    resolved = resolve_measured_stiffness(ds, HP3_ANCHORS, varpro=vp)
    assert resolved is not None and resolved["source"] == "compliance_probe"
    assert resolved["stiffness_n"] == pytest.approx(16000.0, rel=1e-3)

    # Without probes, falls back to gravity-separated VarPro estimate.
    ds_no_probe = {**ds, "sweeps": [{**s, "metadata": {}} for s in ds["sweeps"]]}
    assert not has_compliance_probes(ds_no_probe)
    vp_good = {"k_grav_identifiable": True, "k_grav_hat": 1.25, "ea_estimated_n": 16000.0, "ea_cfg_n": 20000.0}
    resolved_vp = resolve_measured_stiffness(ds_no_probe, HP3_ANCHORS, varpro=vp_good)
    assert resolved_vp is not None and resolved_vp["source"] == "varpro"
    assert resolved_vp["stiffness_n"] == pytest.approx(16000.0)

    # Unidentifiable VarPro returns None.
    assert resolve_measured_stiffness(ds_no_probe, HP3_ANCHORS, varpro={**vp_good, "k_grav_identifiable": False}) is None


def test_m666_from_plan_emits_measured_stiffness_standalone_and_combined():
    standalone = _m666_from_plan({"measured_stiffness": {"stiffness_n": 16123.45}})
    assert standalone == "M666 S16123.5"

    combined = _m666_from_plan({
        "length_model": {
            "find_radii_mode": "global",
            "find_buildup_mode": "off",
            "effective_radii_mm": [30.123, 30.123, 30.123],
            "modeled_buildup_factor": [0.0, 0.0, 0.0],
        },
        "measured_stiffness": {"stiffness_n": 16123.45},
    })
    assert combined == "M666 R30.12:30.12:30.12 Q0 S16123.5"


def test_headless_apply_parameters_verifies_m666_stiffness_and_runs_m500():
    service = HeadlessSimulation.__new__(HeadlessSimulation)
    service.manifest = {}
    sent = []
    replies = {
        "M669": "A:0, -1900, -280",
        "M666": "M666 A1 S16000.0 Q0.0000 K0.0150 V1",
        "M500": "Parameters saved",
    }

    def fake_request(kind, line):
        sent.append((kind, line))
        return {"result": {"reply": replies.get(line, "")}}

    service.request = fake_request
    service.apply_parameters(["M669 A0:-1900:-280", "M666 S16000.0"])
    assert ("gcode", "M666 S16000.0") in sent
    assert ("gcode", "M500") in sent
    assert service.manifest["saved_parameters_permanent"] is True

    # If firmware ignores M666 S..., apply_parameters must fail loudly.
    with pytest.raises(RuntimeError, match="did not accept M666 S24000.0"):
        service.apply_parameters(["M666 S24000.0"])


def test_full_auto_prints_and_persists_measured_stiffness_with_m500(tmp_path, monkeypatch, capsys):
    dataset = tmp_path / "sweeps.json"
    dataset.write_text(
        '{"version":"1.0","machine_type":"slideprinter","num_anchors":3,"dimensions":2,'
        '"sweeps":[{"id":"s1","fixed_anchors":[0],"fixed_lengths":[10],"drive_anchor":1,"sensor_anchor":2,'
        '"data_points":[{"l_drive":1,"l_sensor":2}]}]}'
    )
    sent = []

    def fake_plan(path, **_kwargs):
        return {
            "anchors": np.asarray([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]], dtype=float),
            "machine_type": "slideprinter",
            "cost": 1.0,
            "cost_noise_normalized": 1.0,
            "covariance_scaled": np.eye(6, dtype=float),
            "dataset": ac._load_json(path),
            "measured_stiffness": {
                "source": "compliance_probe",
                "stiffness_n": 16250.0,
                "k_hat": 20000.0 / 16250.0,
                "num_steps": 6,
            },
        }

    from autocal.stopping import StopDecision
    monkeypatch.setattr(ac, "plan_next_ellipse_sweep", fake_plan)
    monkeypatch.setattr(ac, "_print_ellipse_plan", lambda *a, **kw: None)
    monkeypatch.setattr(ac, "_plan_hits_underconstrained_penalty", lambda *a: False)
    monkeypatch.setattr(ac.StoppingPolicy, "update", lambda *a, **kw: StopDecision(1, True, False, None, "accept"))
    monkeypatch.setattr(ac.RRFFirmwareProvider, "send_gcode", lambda self, srv, cmd: sent.append(cmd) or "")

    rc = ac.main([
        "--sim", "--no-collect", "--machine-type", "slideprinter", "--dataset", str(dataset),
        "--max-steps", "2", "--collector-args", "--server", "http://127.0.0.1:12345",
    ])
    assert rc == 0
    assert sent == [
        "M669 A0.00:0.00:0.00 B1.00:0.00:0.00 C0.00:1.00:0.00",
        "M666 S16250.0",
        "M500",
    ]
    out = capsys.readouterr().out
    assert "Line model (M666): M666 S16250.0" in out
    assert "Stiffness (M666 S): S16250.0 N (source=compliance_probe" in out
