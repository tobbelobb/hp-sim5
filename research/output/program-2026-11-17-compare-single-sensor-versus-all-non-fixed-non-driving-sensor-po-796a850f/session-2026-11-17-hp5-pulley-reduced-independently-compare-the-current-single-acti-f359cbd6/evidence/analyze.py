"""Analyze the isolated HP5 pulley-reduced sensor-policy runs.

Diagnostics (truth is used only as a diagnostic, never as a fit input):
  * nominal anchors come from the firmware M669 recorded in each dataset;
  * pose is trilaterated from the three position-controlled lines
    (the two fixed anchors and the drive);
  * encoder-minus-point-geometry residuals expose sensor slack/stall;
  * a per-sub-sweep tracking slope separates "follows the mover" from "frozen".

Fit comparison at matched budget (same points, same starts, same optimizer):
  * current: only fixed + drive + sensor lengths constrain geometry;
  * extra:  the redundant free line's encoder range is added.
Both use the same static force model (sensor and redundant line carry known
torques), so the only difference is the extra measured range.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SESSION = HERE.parent
REPO = next(p for p in SESSION.parents if (p / "autocal/pose_fit.py").exists())
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(HERE))

import ext_fit  # noqa: E402
from autocal.pose_fit import fit_pose_least_squares, HOME_OFFSET_PRIOR_SIGMA_MM  # noqa: E402
from autocal.theoretical_ellipse import anchor_opt_vec_to_matrix, anchors_matrix_to_opt_vec  # noqa: E402
from autocal.theoretical_ellipse import get_anchor_opt_bounds  # noqa: E402


def nominal_anchors(dataset: dict) -> np.ndarray:
    m669 = dataset["config"]["m669"]
    return np.array([m669[key] for key in ("A", "B", "C", "D", "I")], dtype=float)


def point_rows(dataset: dict):
    rows = []
    mm_per_degree = np.asarray(dataset["config"]["mm_per_degree"], dtype=float)
    segment = 0
    for sweep_index, sweep in enumerate(dataset["sweeps"]):
        fixed = [int(a) for a in sweep["fixed_anchors"]]
        drive, sensor = int(sweep["drive_anchor"]), int(sweep["sensor_anchor"])
        redundant = next((a for a in range(int(dataset["num_anchors"]))
                          if a not in set(fixed) | {drive, sensor}), None)
        previous_step = None
        for point in sweep["data_points"]:
            angles = point.get("mu") or point.get("raw_angles_deg")
            lengths = np.asarray(angles, dtype=float) * mm_per_degree
            errors = (point.get("fixed_error_angles_mu_deg")
                      if point.get("fixed_error_angles_mu_deg") is not None
                      else point.get("fixed_error_angles_deg")) or [None] * len(lengths)
            # step_index restarts in every sub-sweep; that is the reliable
            # boundary even when the next sub-sweep starts farther out.
            step_index = point.get("step_index")
            if previous_step is not None and isinstance(step_index, int) and step_index <= previous_step:
                segment += 1
            if step_index is not None:
                previous_step = step_index
            rows.append(SimpleRow(
                sweep=sweep_index, segment=segment, fixed=fixed, drive=drive, sensor=sensor,
                redundant=redundant, lengths=lengths, errors=errors,
                sensor_force=float(point.get("sensor_force_n", np.nan)),
                clock=point.get("clock_time_ms"), setpoint=point.get("drive_setpoint_mm"),
                sigma=point.get("sigma"),
            ))
        segment += 1
    return rows


class SimpleRow:
    __slots__ = ("sweep", "segment", "fixed", "drive", "sensor", "redundant", "lengths",
                 "errors", "sensor_force", "clock", "setpoint", "sigma")

    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


def trilaterate(anchors: np.ndarray, lines, absolute, start=None) -> np.ndarray:
    from scipy.optimize import least_squares
    lines = list(lines)
    a = anchors[lines]
    target = np.asarray(absolute, dtype=float)[lines]
    p = np.zeros(3) if start is None else np.asarray(start, dtype=float)
    for _ in range(6):
        residual = np.linalg.norm(p[None, :] - a, axis=1) - target
        jacobian = (p[None, :] - a) / np.maximum(np.linalg.norm(p[None, :] - a, axis=1), 1e-9)[:, None]
        step, *_ = np.linalg.lstsq(jacobian, -residual, rcond=None)
        p = p + step
        if np.linalg.norm(step) < 1e-9:
            break
    return p


def geometry_table(dataset: dict, anchors: np.ndarray):
    rows = point_rows(dataset)
    out = []
    previous_pose = np.zeros(3)
    for row in rows:
        position_lines = [*row.fixed, row.drive]
        # Every fixed/drive/sensor/redundant "length" is the loaded encoder length
        # change from the homed state, so geometric distance = |a| + length.
        absolute = np.linalg.norm(anchors, axis=1) + row.lengths.copy()
        position = trilaterate(anchors, position_lines, absolute, previous_pose)
        previous_pose = position
        geometric = np.linalg.norm(anchors - position[None, :], axis=1) - np.linalg.norm(anchors, axis=1)
        out.append(dict(row=row, position=position, geometric=geometric,
                        encoder_minus_geometry=row.lengths - geometric))
    return out


def tracking(geometry, sweep: int, segment: int):
    points = [g for g in geometry if g["row"].sweep == sweep and g["row"].segment == segment]
    if len(points) < 3:
        return None
    axes = {g["row"].redundant for g in points}
    if len(axes) != 1 or next(iter(axes)) is None:
        return None
    redundant = next(iter(axes))
    measured = np.array([g["row"].lengths[redundant] for g in points])
    geometric = np.array([g["geometric"][redundant] for g in points])
    span = geometric[-1] - geometric[0]
    if abs(span) < 1.0:
        return None
    slope = float(np.polyfit(geometric, measured, 1)[0])
    return dict(sweep=sweep, segment=segment, points=len(points), axis=int(redundant),
                geometric_range_mm=float(span),
                measured_range_mm=float(measured[-1] - measured[0]),
                tracking_slope=slope, start_residual_mm=float(measured[0] - geometric[0]))


def information(fit: dict, dataset: dict, *, measure_extra: bool,
                redundant_force_mode: str = "sensor") -> dict:
    machine = str(dataset["machine_type"])
    n, d = int(dataset["num_anchors"]), int(dataset["dimensions"])
    anchors = fit["anchors"]
    vector = anchors_matrix_to_opt_vec(anchors, machine, None)
    table = ext_fit.line_lengths(dataset, measure_extra=measure_extra)
    lengths = table["lengths_mm"]
    measured = np.isfinite(lengths)
    config = dataset["config"]
    model = ext_fit.FlexModel.from_m666(config["m666_before_data_collection"], num_axes=n)
    weight = float(config["m666_before_data_collection"].get("W", 0)) * 9.81
    passive = float((config.get("force_tuning") or {}).get("force_low_n", 0.0))
    redundant = table["redundant"]
    points = lengths.shape[0]
    force_lines = np.column_stack([ext_fit.point_sensor_anchors(dataset, table), redundant])
    forces = np.column_stack([
        table["sensor_force_n"],
        (table["sensor_force_n"] if redundant_force_mode == "sensor"
         else np.full(points, passive if redundant_force_mode == "passive"
                      else float(redundant_force_mode))),
    ])

    def residuals(x):
        moved = anchor_opt_vec_to_matrix(x, machine, n, d, None)
        absolute = np.linalg.norm(moved, axis=1) + np.where(measured, lengths, 0.0)

        def tension_fn(positions):
            return ext_fit.corrected_lengths(moved, positions, absolute, force_lines, forces, weight)

        positions = ext_fit.solve_positions(moved, absolute, measured, model, tension_fn)
        tensions = tension_fn(positions)
        return ext_fit.consistency_residuals(moved, positions, absolute, measured, tensions, model)[measured]

    # Central differences in optimizer coordinates; the prior/offset block is not
    # part of this observability probe (anchors only, as in the rough checks).
    steps = np.eye(vector.size) * 0.1
    jacobian = np.column_stack([
        (residuals(vector + step) - residuals(vector - step)) / 0.2 for step in steps
    ])
    singular = np.linalg.svd(jacobian, compute_uv=False)
    rank = int(np.linalg.matrix_rank(jacobian))
    sigma = max(0.02, 3.0 * float(fit["robust_scale_mm"]))
    return dict(parameters=int(vector.size), observations=int(measured.sum()),
                rank=rank, singular_min=float(singular[-1]),
                condition=float(singular[0] / singular[-1]) if singular[-1] > 0 else None,
                max_anchor_std_mm=float(np.max(np.linalg.norm(np.linalg.pinv(jacobian), axis=1)) * sigma)
                if rank == vector.size else None)


def fit_both(dataset: dict, truth: np.ndarray, redundant_force_mode: str):
    results = {}
    contexts = {}
    variants = [("current", False, redundant_force_mode), ("extra", True, redundant_force_mode)]
    if redundant_force_mode != "passive":
        # Integration hazard: keep the production d+1 loader (redundant line
        # assumed at force_low) while the collector has pretensioned it.
        variants.append(("production_loader_with_pretension", False, "passive"))
    for label, measure_extra, mode in variants:
        offset = ext_fit.fit_extra(dataset, truth, measure_extra=measure_extra,
                                   fit_home_offsets=True, extra_initial_anchors=(truth * 1.05,),
                                   redundant_force_mode=mode)
        shifted = ext_fit.shift_by_offsets(dataset, offset["home_offsets_mm"], measure_extra=measure_extra)
        primary = ext_fit.fit_extra(shifted, offset["anchors"], measure_extra=measure_extra,
                                    fit_home_offsets=False, redundant_force_mode=mode)
        results[label] = dict(
            offset_fit_success=bool(offset["success"]),
            anchor_fit_success=bool(primary["success"]),
            rms_mm=primary["rms_mm"], robust_scale_mm=primary["robust_scale_mm"],
            home_offsets_mm=offset["home_offsets_mm"].tolist(),
            anchors=primary["anchors"].tolist(),
            error=ext_fit.anchor_error(primary["anchors"], truth),
            information=information(primary, shifted, measure_extra=measure_extra,
                                    redundant_force_mode=mode),
        )
        contexts[label] = dict(measure_extra=measure_extra, offsets=offset["home_offsets_mm"],
                               start=offset["anchors"], force_mode=mode)
    # External control: production fitter, current columns only.
    prod_offset = fit_pose_least_squares(dataset, truth, fit_home_offsets=True,
                                         home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM,
                                         extra_initial_anchors=(truth * 1.05,))
    from autocal.pose_fit import shift_dataset_home_offsets
    prod_shifted = shift_dataset_home_offsets(dataset, prod_offset["home_offsets_mm"])
    prod_primary = fit_pose_least_squares(prod_shifted, prod_offset["anchors"],
                                          fit_home_offsets=False,
                                          home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
    results["production"] = dict(
        offset_fit_success=bool(prod_offset["success"]),
        anchor_fit_success=bool(prod_primary["success"]),
        rms_mm=prod_primary["rms_mm"], home_offsets_mm=prod_offset["home_offsets_mm"].tolist(),
        anchors=prod_primary["anchors"].tolist(),
        error=ext_fit.anchor_error(prod_primary["anchors"], truth),
    )
    results["current_vs_production_max_anchor_delta_mm"] = float(np.max(np.abs(
        np.asarray(results["current"]["anchors"]) - np.asarray(results["production"]["anchors"]))))
    return results, contexts


def _held_out_residual(anchors, dataset_slice: dict, measure_extra: bool, weight: float,
                       passive: float, model, redundant_force_mode: str) -> np.ndarray:
    table = ext_fit.line_lengths(dataset_slice, measure_extra=measure_extra)
    lengths = table["lengths_mm"]
    measured = np.isfinite(lengths)
    points = lengths.shape[0]
    force_lines = np.column_stack([ext_fit.point_sensor_anchors(dataset_slice, table),
                                   table["redundant"]])
    forces = np.column_stack([
        table["sensor_force_n"],
        (table["sensor_force_n"] if redundant_force_mode == "sensor"
         else np.full(points, passive if redundant_force_mode == "passive"
                      else float(redundant_force_mode))),
    ])
    absolute = np.linalg.norm(anchors, axis=1) + np.where(measured, lengths, 0.0)

    def tension_fn(positions):
        return ext_fit.corrected_lengths(anchors, positions, absolute, force_lines, forces, weight)

    positions = ext_fit.solve_positions(anchors, absolute, measured, model, tension_fn)
    residual = ext_fit.consistency_residuals(anchors, positions, absolute, measured,
                                             tension_fn(positions), model)
    return np.linalg.norm(np.where(measured, residual, 0.0), axis=1)


def leave_one_sweep_out(dataset: dict, contexts: dict):
    """Fit anchors on all but one sweep, then score the held-out sweep's residual.

    Home offsets are learned once on all data and shared by both policies, so the
    comparison isolates the anchor fit; no held-out parameter is refitted.
    """
    config = dataset["config"]
    n = int(dataset["num_anchors"])
    model = ext_fit.FlexModel.from_m666(config["m666_before_data_collection"], num_axes=n)
    weight = float(config["m666_before_data_collection"].get("W", 0)) * 9.81
    passive = float((config.get("force_tuning") or {}).get("force_low_n", 0.0))
    out = {}
    for label, context in contexts.items():
        measure_extra = context["measure_extra"]
        force_mode = context["force_mode"]
        shifted = ext_fit.shift_by_offsets(dataset, context["offsets"], measure_extra=measure_extra)
        held = []
        for index in range(len(dataset["sweeps"])):
            subset = dict(shifted)
            subset["sweeps"] = [s for i, s in enumerate(shifted["sweeps"]) if i != index]
            fit = ext_fit.fit_extra(subset, context["start"], measure_extra=measure_extra,
                                    fit_home_offsets=False, redundant_force_mode=force_mode)
            evaluation = dict(shifted)
            evaluation["sweeps"] = [shifted["sweeps"][index]]
            per_point = _held_out_residual(fit["anchors"], evaluation, measure_extra,
                                           weight, passive, model, force_mode)
            held.append(dict(
                sweep=index, points=int(len(per_point)),
                anchor_error=ext_fit.anchor_error(fit["anchors"], np.asarray(dataset["_truth"])),
                median_point_residual_mm=float(np.median(per_point)),
                max_point_residual_mm=float(np.max(per_point))))
        out[label] = held
    return out


def truth_residuals(dataset: dict, truth: np.ndarray, redundant_force_mode: str) -> dict:
    """Consistency residual at the known anchors: are the data compatible with truth?

    This is a diagnostic of measurement/model consistency, never a fit input.
    """
    config = dataset["config"]
    n = int(dataset["num_anchors"])
    model = ext_fit.FlexModel.from_m666(config["m666_before_data_collection"], num_axes=n)
    weight = float(config["m666_before_data_collection"].get("W", 0)) * 9.81
    passive = float((config.get("force_tuning") or {}).get("force_low_n", 0.0))
    out = {}
    for label, measure_extra in (("four_line", False), ("five_line", True)):
        per_point = _held_out_residual(truth, dataset, measure_extra, weight, passive, model,
                                       redundant_force_mode)
        out[label] = dict(rms_mm=float(np.sqrt(np.mean(per_point ** 2))),
                          median_mm=float(np.median(per_point)),
                          max_mm=float(np.max(per_point)))
    return out


def run_metrics(run_dir: Path):
    dataset = json.loads((run_dir / "sweeps.json").read_text())
    invocation = json.loads((run_dir / "invocation.json").read_text())
    summary = json.loads((run_dir / "run-summary.json").read_text())
    truth = nominal_anchors(dataset)
    dataset["_truth"] = truth.tolist()
    geometry = geometry_table(dataset, truth)
    rows = [g["row"] for g in geometry]
    redundant_axes = [row.redundant for row in rows]
    tracks = []
    for sweep in range(len(dataset["sweeps"])):
        segments = sorted({row.segment for row in rows if row.sweep == sweep})
        for segment in segments:
            result = tracking(geometry, sweep, segment)
            if result:
                tracks.append(result)
    extra = np.array([g["encoder_minus_geometry"][g["row"].redundant] for g in geometry]) if rows else np.array([])
    shape = []
    for sweep in range(len(dataset["sweeps"])):
        for segment in sorted({row.segment for row in rows if row.sweep == sweep}):
            residual = np.array([g["encoder_minus_geometry"][g["row"].redundant]
                                 for g in geometry
                                 if g["row"].sweep == sweep and g["row"].segment == segment])
            if residual.size > 2:
                shape.append(float(np.std(residual - residual.mean())))
    fixed_errors = {}
    for anchor in (0, 4):
        values = [row.errors[anchor] * dataset["config"]["mm_per_degree"][anchor]
                  for row in rows if row.errors[anchor] is not None]
        fixed_errors[str(anchor)] = dict(
            median_mm=float(np.median(values)) if values else None,
            p5_mm=float(np.percentile(values, 5)) if values else None,
            p95_mm=float(np.percentile(values, 95)) if values else None)
    sensor_lengths = [row.lengths[row.sensor] for row in rows]
    metrics = dict(
        run=run_dir.name,
        policy=invocation.get("collector", "").split("/")[-1],
        force_low_n=invocation["force_low_n"], sensor_force_n=invocation["sensor_force_n"],
        fixed_targets_mm=invocation["fixed_targets_mm"],
        sweeps=len(dataset["sweeps"]), points=len(rows),
        resolutions=(dataset.get("acquisition_rejections") or []),
        drive_span_per_sweep_mm=[float(s["drive_range"]["end"] - s["drive_range"]["start"])
                                 for s in dataset["sweeps"]],
        redundant_axis=sorted(set(int(a) for a in redundant_axes if a is not None)),
        redundant_encoder_span_mm=(float(np.ptp([row.lengths[row.redundant] for row in rows]))
                                   if rows else None),
        redundant_residual_span_mm=float(np.ptp(extra)) if extra.size else None,
        redundant_shape_error_mm_median=float(np.median(shape)) if shape else None,
        redundant_tracking=tracks,
        redundant_tracking_slope_median=(float(np.median([t["tracking_slope"] for t in tracks]))
                                         if tracks else None),
        redundant_encoder_minus_geometry_p5_p50_p95_mm=(
            np.percentile(extra, [5, 50, 95]).tolist() if extra.size else None),
        fixed_error_mm=fixed_errors,
        sensor_length_span_mm=float(np.ptp(sensor_lengths)) if sensor_lengths else None,
        sensor_sigma_deg_median=(float(np.nanmedian([np.nanmax(row.sigma) for row in rows
                                                     if row.sigma is not None
                                                     and np.any(np.isfinite(row.sigma))]))
                                 if any(row.sigma is not None
                                        and np.any(np.isfinite(row.sigma)) for row in rows) else None),
        wall_s=summary.get("wall_s"), simulated_s=summary.get("simulated_s"),
        sim_s_per_point=(summary.get("simulated_s") or 0) / max(len(rows), 1),
        wall_s_per_point=(summary.get("wall_s") or 0) / max(len(rows), 1),
        dataset_sha256=summary.get("dataset_sha256"),
    )
    fits, contexts = fit_both(dataset, truth, invocation.get("redundant_force_mode", "sensor"))
    metrics["fits"] = fits
    metrics["truth_residuals"] = truth_residuals(dataset, truth,
                                                 invocation.get("redundant_force_mode", "sensor"))
    metrics["cross_validation"] = leave_one_sweep_out(dataset, contexts)
    return metrics


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="*", help="Run directories (default: every runs/*)")
    parser.add_argument("--output", default=str(SESSION / "analysis.json"))
    args = parser.parse_args()
    dirs = [Path(r) for r in args.runs] or sorted(
        p for p in (SESSION / "runs").iterdir()
        if (p / "sweeps.json").exists() and (p / "invocation.json").exists()
        and not p.name.startswith("smoke")
    )
    output = {}
    for directory in dirs:
        print(f"analyzing {directory.name} ...", file=sys.stderr)
        output[directory.name] = run_metrics(directory)
    Path(args.output).write_text(json.dumps(output, indent=2) + "\n")
    print(json.dumps({name: {key: value for key, value in metrics.items()
                            if key not in ("fits", "cross_validation")}
                      for name, metrics in output.items()}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
