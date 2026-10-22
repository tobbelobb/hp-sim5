"""Two help views over the same parser; tuning flags always remain available."""
from __future__ import annotations

import argparse
import sys


_EVERYDAY = {
    "Calibration": {
        "help", "help_research", "machine_type", "dataset", "no_collect",
        "max_steps", "patience", "find_radii", "find_buildup_factor",
        "base_radii", "buildup_factor", "line_width", "r0_bounds", "b_bounds",
    },
    "Collection and simulation": {
        "firmware", "config", "sim", "headless_sim", "collector_args", "speedup", "sweep_points",
        "keep_sim_alive", "hp_sim_reset", "physics_substeps", "open_loop_motors",
        "extended_recording", "recording_mode",
    },
    "Output": {"verbose", "report"},
}
_RESEARCH = {
    "Research: solver and recovery": {
        "solve_restarts", "solve_iterations", "solve_optimizer", "optimizer_mode",
        "trf_solver_fallback", "sparse_recovery",
    },
    "Research: spool model and objectives": {
        "sigma_floor_mm", "low_anchor_z", "r0_prior_sigma_mm", "b_prior_sigma",
        "spool_outer_iters", "theta0_mode", "fit_structure", "filter_schedule",
        "objective_schedule",
    },
    "Research: residuals and filtering": {
        "pointwise_residual", "pointwise_filtering", "pointwise_global_mad",
        "sweep_wise_filtering", "sweep_metric", "sigma_source", "use_noise_mean",
    },
    "Research: planning and run comparisons": {
        "candidate_count", "delta_min", "delta_max", "regularization",
        "min_fixed_delta_spacing_mm", "full_auto_run", "shotgun",
    },
    "Diagnostics and advanced configuration": {
        "stage_artifacts", "full_auto_log", "robust_debug", "residuals_csv",
        "output_with_explanations", "write_sweep_config", "rrf_config", "klipper_config",
        "project_zero_tension", "debug_sweep_actions", "extended_reference",
        "extended_reference_ws",
    },
}


class _ResearchHelp(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        parser._print_message(parser.format_help(research=True), sys.stdout)
        parser.exit()


class CalibrationArgumentParser(argparse.ArgumentParser):
    """Keep routine help compact without maintaining a second set of defaults."""

    def __init__(self, **kwargs):
        super().__init__(
            usage="%(prog)s --machine-type TYPE [--dataset PATH] [options]",
            epilog="Use --help-research for all tuning, diagnostics and advanced options. "
                   "Both help views use the same defaults; research options are accepted directly.",
            **kwargs,
        )
        self.add_argument(
            "--help-research", action=_ResearchHelp, nargs=0,
            default=argparse.SUPPRESS, help="Show every option, including tuning and diagnostics.",
        )

    def format_help(self, *, research=False):
        formatter = self._get_formatter()
        formatter.add_usage(self.usage, self._actions, self._mutually_exclusive_groups)
        formatter.add_text(self.description)
        if research:
            machine = next(a for a in self._actions if a.dest == "machine_type")
            formatter.add_text("Accepted machine types: " + ", ".join(machine.choices))
        groups = {**_EVERYDAY, **_RESEARCH} if research else _EVERYDAY
        for title, names in groups.items():
            actions = [a for a in self._actions if a.dest in names]
            if actions:
                formatter.start_section(title)
                formatter.add_arguments(actions)
                formatter.end_section()
        # Future options must remain discoverable in extended help even before
        # their presentation group is chosen.
        if research:
            known = set().union(*groups.values())
            other = [a for a in self._actions if a.dest not in known]
            if other:
                formatter.start_section("Other advanced options")
                formatter.add_arguments(other)
                formatter.end_section()
        formatter.add_text(self.epilog)
        return formatter.format_help()
