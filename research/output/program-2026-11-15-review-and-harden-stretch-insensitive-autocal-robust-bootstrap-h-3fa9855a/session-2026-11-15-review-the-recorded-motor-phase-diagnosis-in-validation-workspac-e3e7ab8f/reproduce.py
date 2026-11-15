"""Read-only phase-diagnosis reproduction; writes only beside this helper.

Run from the repository with .venv/bin/python <this file>.
The optional --rrd check rereads recorded strand forces and window samples.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


OUT = Path(__file__).resolve().parent
ROOT = next(p for p in OUT.parents if (p / "autocal/pose_fit.py").is_file())
INPUT = OUT.parent / "validation/workspace-expansion"


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            h.update(block)
    return {"sha256": h.hexdigest(), "size_bytes": path.stat().st_size}


def metrics(y, prediction):
    residual = y - prediction
    return {
        "raw_rms_n": float(np.sqrt(np.mean(y**2))),
        "residual_rms_n": float(np.sqrt(np.mean(residual**2))),
        "residual_mean_n": float(np.mean(residual)),
        "explained_variance": float(1 - np.var(residual) / np.var(y)),
        "r2_test_centered": float(1 - np.mean(residual**2) / np.var(y)),
        "mse_reduction_from_zero": float(1 - np.mean(residual**2) / np.mean(y**2)),
    }


def fit_rows(sensor, sweep, angles, error, frequency=50, split="parity"):
    phase = np.deg2rad(angles) * frequency
    x = np.array([np.sin(phase), np.cos(phase), np.ones(len(phase))]).T
    rows = []
    for axis in range(3):
        if split == "parity":
            train_sweeps = sweep % 2 == 0
        elif split == "reverse":
            train_sweeps = sweep % 2 == 1
        elif split == "chronological":
            train_sweeps = sweep < 5
        train = (sensor == axis) & train_sweeps
        test = (sensor == axis) & ~train_sweeps
        coef = np.linalg.lstsq(x[train], error[train], rcond=None)[0]
        rows.append({
            "axis": axis,
            "train_points": int(train.sum()),
            "test_points": int(test.sum()),
            "train_sweep_indices": sorted(set(sweep[train].tolist())),
            "test_sweep_indices": sorted(set(sweep[test].tolist())),
            "coefficient_n": coef.tolist(),
            "amplitude_n": float(np.hypot(*coef[:2])),
            "phase_shift_electrical_deg": float(np.rad2deg(np.arctan2(coef[1], coef[0]))),
            "design_condition": float(np.linalg.cond(x[train])),
            "test": metrics(error[test], x[test] @ coef),
            "constant_train_mean_test": metrics(error[test], np.full(test.sum(), error[train].mean())),
        })
    return rows


def raw_force_check(folder, points, sensors, advantage, cached):
    # Independent extraction from existing recordings; no recorder or physics.
    from rerun.chunk import RrdReader
    import re

    rrd, = (folder / "sweeps.headless").glob("*.rrd")
    buffers = {}
    for chunk in RrdReader(rrd).stream().filter(content=["/cable_forces/**"]).to_chunks():
        match = re.search(r"/CablePath(AL|AR|BL|BR|CL|CR|D0|D1|D2)_", str(chunk.entity_path))
        if not match:
            continue
        batch = chunk.to_record_batch()
        if "sim_time" not in batch.schema.names or "Scalars:scalars" not in batch.schema.names:
            continue
        times = np.asarray(batch.column("sim_time").cast("int64").to_pylist()) * 1e-9
        values = batch.column("Scalars:scalars").to_pylist()
        rows = [(t, np.asarray(v, dtype=float).ravel()) for t, v in zip(times, values) if v is not None]
        buffers.setdefault(match[1], []).extend(rows)
    series = {}
    for key, rows in buffers.items():
        rows.sort(key=lambda p: p[0])
        series[key] = (np.array([r[0] for r in rows]), np.array([r[1] for r in rows]))

    start = np.array([p["noise_clock_start_ms"] / 1000 for p in points])
    end = np.array([p["noise_clock_end_ms"] / 1000 for p in points])
    mid = (start + end) / 2
    proxy = np.zeros_like(cached)
    first_proxy = np.zeros_like(cached)
    last_proxy = np.zeros_like(cached)
    mean_proxy = np.zeros_like(cached)
    lags = []
    max_segment_differences = {}
    min_samples = 999
    for axis, group in enumerate([("AL", "AR"), ("BL", "BR"), ("CL", "CR"), ("D0", "D1", "D2")]):
        for key in group:
            t, v = series[key]
            index = np.searchsorted(t, mid, side="right") - 1
            assert index.min() >= 0
            lags.extend((mid - t[index]).tolist())
            proxy[:, axis] += advantage[axis] * v[index, 0]
            first_proxy[:, axis] += advantage[axis] * v[np.searchsorted(t, start, side="right") - 1, 0]
            last_proxy[:, axis] += advantage[axis] * v[np.searchsorted(t, end, side="right") - 1, 0]
            for j, (a, b) in enumerate(zip(start, end)):
                inside = (t >= a) & (t <= b)
                min_samples = min(min_samples, int(inside.sum()))
                assert inside.any()
                mean_proxy[j, axis] += advantage[axis] * v[inside, 0].mean()
            selected = np.unique(np.concatenate([np.searchsorted(t, a, side="right") - 1 for a in [start, mid, end]]))
            max_segment_differences[key] = {
                "segments": v.shape[1],
                "max_segment_minus_first_n": float(np.max(abs(v[selected] - v[selected, :1]))),
            }
    ix = np.arange(len(points)), sensors
    return {
        "recording": str(rrd.relative_to(ROOT)),
        "raw_force_series": {k: {"rows": len(t), "segments": v.shape[1]} for k, (t, v) in series.items()},
        "cached_proxy_max_abs_diff_n": float(np.max(abs(proxy - cached))),
        "force_latest_at_lag_range_s": [float(min(lags)), float(max(lags))],
        "minimum_recorded_samples_in_noise_window": min_samples,
        "sensor_start_minus_mid_rms_n": float(np.sqrt(np.mean((first_proxy[ix] - proxy[ix])**2))),
        "sensor_end_minus_mid_rms_n": float(np.sqrt(np.mean((last_proxy[ix] - proxy[ix])**2))),
        "sensor_window_mean_minus_mid_rms_n": float(np.sqrt(np.mean((mean_proxy[ix] - proxy[ix])**2))),
        "segment_difference_at_window_start_mid_end": max_segment_differences,
    }, mean_proxy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rrd", action="store_true")
    args = parser.parse_args()
    result = {"scope": "Independent saved-data diagnostic; zero-index even sweeps train, zero-index odd sweeps test; no new physics.", "machines": {}}
    manifest = {"review_source_revision": "5251f749b24eaf0db851a8a3174ed6023f511e83", "files": {}}
    source_paths = ["src/wasm/cable_solver/src/engine.rs", "src/wasm/cable_solver/src/lib.rs", "src/js/cable_joints_3d/rust_physics_engine.js", "hp-sim-3d/app/torqueModeSystem.js", "hp-sim-3d/app/externalCommandSocket.js", "hp-sim-3d/app/flightRecorderSnapshot.js", "hp-sim-3d/app/scene/machineEntityBuilders.js", "scripts/hangprinter_flight_recorder.py", "RRF/ReprapFirmware/src/Movement/Kinematics/HangprinterKinematics.cpp", "RRF/host/can/CanInterfaceHost.cpp", "autocal/control/behaviors/sweep_data_collection.mjs", "autocal/control/primitives/encoder_noise.mjs", "autocal/control/primitives/encoder_utils.mjs"]
    paths = [ROOT / p for p in source_paths] + [INPUT / p for p in ["report.md", "force_phase.py", "analyze_recording.py"]]
    for name in ["hp3-final", "hp4-final"]:
        folder = INPUT / name
        paths.extend(folder / p for p in ["sweeps.json", "physics-samples.npz", "physics-diagnostics.json", "force-phase.json", "input-scene.usda", "sweeps.headless/manifest.json", "sweeps.headless/baked-scene.usda"])
        paths.extend((folder / "sweeps.headless").glob("*.rrd"))
        data = json.loads((folder / "sweeps.json").read_text())
        recorded = json.loads((folder / "sweeps.headless/manifest.json").read_text())
        observations = [(i, s, p) for i, s in enumerate(data["sweeps"]) for p in s["data_points"]]
        points = [p for _, _, p in observations]
        sensors = np.array([p.get("source_sensor_anchor", s["sensor_anchor"]) for _, s, p in observations])
        sweeps = np.array([i for i, _, _ in observations])
        raw = np.array([p["raw_angles_deg"] for p in points])
        mean = np.array([p["mu"] for p in points])
        ix = np.arange(len(points)), sensors
        angle = mean[ix]
        with np.load(folder / "physics-samples.npz") as saved:
            proxy = saved["effective_force_proxy_n"]
            times = saved["times_s"]
        commanded = np.array([p["sensor_force_n"] for p in points])
        error = proxy[ix] - commanded
        expected_mid = np.array([(p["noise_clock_start_ms"] + p["noise_clock_end_ms"]) / 2000 for p in points])
        line = data["config"]["m666_before_data_collection"]
        advantage = np.array(line["U"])
        prediction = advantage[sensors] * .01 / .03 * np.sin(50 * np.deg2rad(angle))
        original = json.loads((folder / "force-phase.json").read_text())["rows"]
        reproduced = fit_rows(sensors, sweeps, angle, error)
        deltas = []
        for got, old in zip(reproduced, original):
            assert got["axis"] == old["sensor_anchor"]
            pairs = [(got["amplitude_n"], old["amplitude_n"]), (got["test"]["raw_rms_n"], old["heldout_raw_rms_n"]), (got["test"]["residual_rms_n"], old["heldout_phase_residual_rms_n"]), (got["test"]["explained_variance"], old["heldout_explained_variance"])]
            deltas.extend(abs(a - b) for a, b in pairs)
        r = {
            "recorded_revision": recorded["git_revision"],
            "physics_engine": recorded["physics_engine"],
            "recorded_build_source_checks": {file: digest(ROOT / "src/wasm/cable_solver" / file)["sha256"] == recorded["physics_artifact"]["build"]["sources"][file] for file in ["src/engine.rs", "src/lib.rs"]},
            "scene_hash_matches_manifest": digest(folder / "input-scene.usda")["sha256"] == recorded["scene_sha256"],
            "baked_scene_hash_matches_manifest": digest(folder / "sweeps.headless/baked-scene.usda")["sha256"] == recorded["baked_scene_sha256"],
            "points": len(points),
            "saved_sweeps": len(data["sweeps"]),
            "sweeps": [{"index": i, "id": s["id"], "points": len(s["data_points"])} for i, s in enumerate(data["sweeps"])],
            "noncanonical_source_sensor_points": sum("source_sensor_anchor" in p for p in points),
            "all_simulation_timestamps": all(p["clock_source"] == "simulation" for p in points),
            "npz_midpoint_time_max_abs_diff_s": float(np.max(abs(times - expected_mid))),
            "minimum_time_separation_s": float(np.min(np.diff(times))),
            "raw_mean_sensor_max_abs_diff_deg": float(np.max(abs(raw[ix] - angle))),
            "raw_mean_sensor_rms_diff_deg": float(np.sqrt(np.mean((raw[ix] - angle)**2))),
            "sensor_mad_sigma_max_deg": float(max(p["sigma"][a] for p, a in zip(points, sensors))),
            "all_mu_present": all("mu" in p and len(p["mu"]) == 4 for p in points),
            "reeving_U": line["U"],
            "strand_count_N": line["N"],
            "source_amplitude_resultant_n": (advantage * .01 / .03).tolist(),
            "reproduction_max_abs_metric_diff": max(deltas),
            "original_split_mean_angles": reproduced,
            "original_split_raw_angles": fit_rows(sensors, sweeps, raw[ix], error),
            "reverse_parity": fit_rows(sensors, sweeps, angle, error, split="reverse"),
            "chronological_first_five_sweeps_train": fit_rows(sensors, sweeps, angle, error, split="chronological"),
            "source_amplitude_zero_phase_zero_offset_no_fit": [{"axis": axis, "test": metrics(error[(sensors == axis) & (sweeps % 2 == 1)], prediction[(sensors == axis) & (sweeps % 2 == 1)])} for axis in range(3)],
            "frequency_negative_controls": {str(f): fit_rows(sensors, sweeps, angle, error, frequency=f) for f in [1, 25, 49, 51, 100]},
        }
        if args.rrd:
            r["rrd_check"], mean_proxy = raw_force_check(folder, points, sensors, advantage, proxy)
            r["window_mean_force_original_split"] = fit_rows(sensors, sweeps, angle, mean_proxy[ix] - commanded)
        result["machines"][name] = r
    for path in paths:
        manifest["files"][str(path.relative_to(ROOT))] = digest(path)
    (OUT / "review-results.json").write_text(json.dumps(result, indent=2) + "\n")
    (OUT / "evidence-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    for name, r in result["machines"].items():
        print(name, "metric delta", r["reproduction_max_abs_metric_diff"])
        for key in ["original_split_mean_angles", "original_split_raw_angles", "chronological_first_five_sweeps_train", "source_amplitude_zero_phase_zero_offset_no_fit"]:
            print(key, [(row["axis"], round(row["test"]["residual_rms_n"], 7), round(row["test"]["explained_variance"], 7)) for row in r[key]])
        if args.rrd:
            print("rrd_check", json.dumps(r["rrd_check"]))


if __name__ == "__main__":
    main()
