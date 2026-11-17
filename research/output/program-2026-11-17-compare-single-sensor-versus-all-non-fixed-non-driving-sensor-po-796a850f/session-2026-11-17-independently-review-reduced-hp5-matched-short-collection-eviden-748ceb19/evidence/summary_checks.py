"""Compact scalar checks of the claims in the original report.

Every value here is recomputed from the saved datasets or from the saved
analysis outputs; no simulation is started.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import review_fit as R  # noqa: E402
import ext_fit  # noqa: E402
from analyze import geometry_table  # noqa: E402

OUT = {}


def load_analysis():
    return json.loads((R.SOURCE / "analysis.json").read_text())


def main() -> int:
    analysis = load_analysis()
    for run in ("ctl-1", "twosensor-1", "ctl-low1p0"):
        dataset, truth = R.load(run)
        inv = json.loads((R.SOURCE / "runs" / run / "invocation.json").read_text())
        summary = json.loads((R.SOURCE / "runs" / run / "run-summary.json").read_text())
        rows = []
        geometry = geometry_table(dataset, truth)
        for g in geometry:
            rows.append(g["row"])
        fixed = {}
        for axis in (0, 4):
            values = [row.errors[axis] * dataset["config"]["mm_per_degree"][axis]
                      for row in rows if row.errors[axis] is not None]
            fixed[str(axis)] = dict(
                median_mm=float(np.median(values)), p5_mm=float(np.percentile(values, 5)),
                p95_mm=float(np.percentile(values, 95)),
                max_abs_mm=float(np.max(np.abs(values))))
        extra = np.array([g["encoder_minus_geometry"][g["row"].redundant] for g in geometry])
        start_residuals = [t["start_residual_mm"] for t in analysis[run]["redundant_tracking"]]
        OUT[run] = dict(
            force_low_n=inv["force_low_n"], labelled_mode=inv["redundant_force_mode"],
            drive_span_per_sweep_mm=analysis[run]["drive_span_per_sweep_mm"],
            simulated_s=summary.get("simulated_s"), wall_s=summary.get("wall_s"),
            fixed_error=fixed,
            tracking_start_residual_min_max_mm=[float(min(start_residuals)), float(max(start_residuals))],
            tracking_slope_min_max=[float(min(t["tracking_slope"] for t in analysis[run]["redundant_tracking"])),
                                    float(max(t["tracking_slope"] for t in analysis[run]["redundant_tracking"]))],
            measured_range_mm=[float(t["measured_range_mm"]) for t in analysis[run]["redundant_tracking"]],
            geometric_range_mm=[float(t["geometric_range_mm"]) for t in analysis[run]["redundant_tracking"]],
            encoder_minus_geometry_p5_p50_p95_mm=np.percentile(extra, [5, 50, 95]).tolist(),
            sensor_payout_mm=[s["sensor_payout_mm"] for s in dataset["sweeps"][0]["metadata"]["spool_reserves"]],
        )

    # Force-model correction: correct mode is passive for both control arms.
    corrected = {}
    for run, mode in (("ctl-1", "passive"), ("ctl-low1p0", "passive")):
        dataset, truth = R.load(run)
        corrected[run] = {}
        for measure_extra in (False, True):
            fit = R.pipeline(dataset, truth, measure_extra=measure_extra, offset_lines=[3, 4],
                             redundant_force_mode=mode, start=truth,
                             extra_starts=(truth * 1.05,))
            corrected[run]["five_line" if measure_extra else "four_line"] = dict(
                max_aligned_abs_coordinate_mm=fit["max_aligned_abs_coordinate_mm"],
                rms_mm=fit["rms_mm"])
    OUT["corrected_control_force"] = corrected

    # Ratios quoted in the report.
    b = analysis["twosensor-1"]
    OUT["ratios"] = dict(
        max_error_22p13_over_1p85=analysis["twosensor-1"]["fits"]["current"]["error"]["max_aligned_abs_coordinate_mm"]
        / analysis["twosensor-1"]["fits"]["extra"]["error"]["max_aligned_abs_coordinate_mm"],
        sigma_15p56_over_2p30=analysis["twosensor-1"]["fits"]["current"]["information"]["max_anchor_std_mm"]
        / analysis["twosensor-1"]["fits"]["extra"]["information"]["max_anchor_std_mm"],
        sigma_16p37_over_2p30=analysis["ctl-1"]["fits"]["current"]["information"]["max_anchor_std_mm"]
        / analysis["twosensor-1"]["fits"]["extra"]["information"]["max_anchor_std_mm"],
        coverage_ratio=[a / b for a, b in zip(
            analysis["ctl-1"]["drive_span_per_sweep_mm"], analysis["ctl-low1p0"]["drive_span_per_sweep_mm"])],
        simulated_time_ratio=analysis["twosensor-1"]["simulated_s"] / analysis["ctl-1"]["simulated_s"],
    )
    noise = json.loads((R.SOURCE / "noise-sensitivity.json").read_text())
    OUT["noise"] = {run: {label: dict(median=d["median_mm"], values=[r["max_aligned_mm"] for r in d["rows"]])
                          for label, d in variants.items()}
                    for run, variants in noise.items()}
    Path(HERE / "summary-checks.json").write_text(json.dumps(OUT, indent=2) + "\n")
    print(json.dumps({run: OUT[run] for run in ("ctl-1", "twosensor-1", "ctl-low1p0")},
                     indent=1)[:4000])
    print("corrected", json.dumps(OUT["corrected_control_force"], indent=1))
    print("ratios", json.dumps(OUT["ratios"], indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
