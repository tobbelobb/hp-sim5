#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from typing import Optional, Sequence

from autocal._autocal_common import (
    _add_candidate_args,
    _add_collector_args,
    _add_output_args,
    _add_solver_args,
    _apply_optimizer_mode_env,
    _arg_has_flag,
    _clean_collector_args,
    _parse_csv_floats,
    _print_ellipse_plan,
    _resolve_spool_cli_options,
)
from autocal.planning_pass import plan_next_ellipse_sweep


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plan the next informative ellipse sweep without collecting it."
    )
    parser.add_argument("dataset", type=Path, help="Dataset JSON to analyze")
    parser.add_argument(
        "--write-sweep-config",
        type=Path,
        default=None,
        help="Write the suggested sweep config to this path (default: <dataset>.active_sweep_cfg.txt)",
    )
    parser.add_argument(
        "--collector-output",
        type=Path,
        default=None,
        help="If set, include this output path in the printed collector command.",
    )
    _add_solver_args(parser)
    _add_candidate_args(parser)
    _add_output_args(parser)
    _add_collector_args(parser)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    spool_options = _resolve_spool_cli_options(parser, args)
    _apply_optimizer_mode_env(args.optimizer_mode)
    collector_args = _clean_collector_args(args.collector_args)
    if bool(args.project_zero_tension) and not _arg_has_flag(collector_args, "--project-zero-tension"):
        collector_args.append("--project-zero-tension")
    if bool(args.debug_sweep_actions) and not _arg_has_flag(collector_args, "--debug-sweep-actions"):
        collector_args.append("--debug-sweep-actions")
    plan = plan_next_ellipse_sweep(
        dataset_path=args.dataset,
        solve_restarts=int(args.solve_restarts),
        solve_iterations=int(args.solve_iterations),
        solve_optimizer=str(args.solve_optimizer),
        residual_threshold=float(args.threshold),
        spring_k_multiplier=float(args.spring_k_multiplier),
        use_flex=bool(args.flex),
        pointwise_residual_mode=str(args.pointwise_residual),
        pointwise_filtering=bool(args.pointwise_filtering),
        pointwise_global_mad=bool(args.pointwise_global_mad),
        sweep_wise_filtering=bool(args.sweep_wise_filtering),
        sweep_metric=str(args.sweep_metric),
        use_noise_mean=bool(args.use_noise_mean),
        sigma_source=str(args.sigma_source),
        robust_debug=bool(args.robust_debug),
        residuals_csv=args.residuals_csv,
        generate_report=bool(args.report),
        find_radii=str(spool_options["find_radii"]),
        find_buildup_factor=str(spool_options["find_buildup_factor"]),
        base_radii=_parse_csv_floats(args.base_radii),
        buildup_factor=args.buildup_factor,
        r0_bounds=spool_options["r0_bounds"],
        b_bounds=spool_options["b_bounds"],
        r0_prior_sigma_mm=spool_options["r0_prior_sigma_mm"],
        b_prior_sigma=spool_options["b_prior_sigma"],
        spool_outer_iters=int(spool_options["spool_outer_iters"]),
        spool_inner_iters=int(spool_options["spool_inner_iters"]),
        theta0_mode=str(spool_options["theta0_mode"]),
        line_width=float(spool_options["line_width"]),
        sigma_floor_mm=spool_options["sigma_floor_mm"],
        sigma_used_mm=spool_options["sigma_used_mm"],
        candidate_deltas=_parse_csv_floats(args.candidate_deltas),
        candidate_count=int(args.candidate_count),
        delta_min=args.delta_min,
        delta_max=args.delta_max,
        fd_eps_mm=float(args.fd_eps_mm),
        regularization=float(args.regularization),
        exclude_existing=not bool(args.no_exclude_existing),
        existing_tol_mm=float(args.existing_tol_mm),
        min_fixed_delta_spacing_mm=float(args.min_fixed_delta_spacing_mm),
        top_k=int(args.top_k),
        write_cfg=args.write_sweep_config,
        collector_output=args.collector_output,
        collector_args=collector_args,
        low_anchor_z=args.low_anchor_z,
        filter_schedule=spool_options["filter_schedule"],
        scale_fix=spool_options["scale_fix"],
        fit_structure=spool_options["fit_structure"],
    )
    _print_ellipse_plan(
        plan,
        output_with_explanations=bool(args.output_with_explanations),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
