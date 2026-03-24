import os

import pytest

import autocal.autocal as ac
from autocal._autocal_common import (
    _apply_optimizer_mode_env,
    _parse_filter_schedule,
    _parse_full_auto_run_spec,
    _parse_objective_schedule,
    _resolve_spool_cli_options,
    build_semi_auto_parser,
)


def _parse_spool_args(*extra_args):
    parser = build_semi_auto_parser()
    args = parser.parse_args(
        [
            "--dataset",
            "dummy.json",
            "--machine-type",
            "slideprinter",
            *extra_args,
        ]
    )
    return parser, args


def test_spool_cli_options_parse_modes_and_bounds():
    parser, args = _parse_spool_args(
        "--find-radii",
        "global",
        "--theta0-mode",
        "infer",
        "--r0-bounds",
        "12,34",
        "--spool-outer-iters",
        "5",
        "--spool-inner-iters",
        "17",
        "--line-width",
        "1.25",
        "--sigma-floor-mm",
        "0.05",
        "--sigma-used-mm",
        "0.8",
    )
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["find_radii"] == "global"
    assert opts["find_buildup_factor"] == "off"
    assert opts["theta0_mode"] == "infer"
    assert opts["r0_bounds"] == (12.0, 34.0)
    assert opts["spool_outer_iters"] == 5
    assert opts["spool_inner_iters"] == 17
    assert opts["filter_schedule"] == ["warmup", "warmup", "warmup", "dynamic"]
    assert opts["objective_schedule"] == [1, 1, 1, 1]
    assert opts["line_width"] == 1.25
    assert opts["sigma_floor_mm"] == 0.05
    assert opts["sigma_used_mm"] == 0.8
    assert opts["fit_structure"] == [3]


def test_spool_cli_flags_without_value_default_to_per_anchor():
    parser, args = _parse_spool_args(
        "--find-radii",
        "--find-buildup-factor",
    )
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["find_radii"] == "per-anchor"
    assert opts["find_buildup_factor"] == "per-anchor"
    assert opts["theta0_mode"] == "zero"


def test_spool_cli_rejects_invalid_bounds():
    parser, args = _parse_spool_args("--r0-bounds", "10")
    with pytest.raises(SystemExit):
        _resolve_spool_cli_options(parser, args)


def test_spool_cli_rejects_negative_line_width():
    parser, args = _parse_spool_args("--line-width", "-0.1")
    with pytest.raises(SystemExit):
        _resolve_spool_cli_options(parser, args)


def test_spool_cli_rejects_nonpositive_sigma_floor_mm():
    parser, args = _parse_spool_args("--sigma-floor-mm", "0")
    with pytest.raises(SystemExit):
        _resolve_spool_cli_options(parser, args)


def test_spool_cli_rejects_nonpositive_sigma_used_mm():
    parser, args = _parse_spool_args("--sigma-used-mm", "0")
    with pytest.raises(SystemExit):
        _resolve_spool_cli_options(parser, args)


def test_spool_cli_parses_scale_fix_levels():
    parser, args = _parse_spool_args("--scale-fix", "1,3")
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["scale_fix"] == [1, 3]


def test_spool_cli_rejects_invalid_scale_fix_level():
    parser, args = _parse_spool_args("--scale-fix", "1,4")
    with pytest.raises(SystemExit):
        _resolve_spool_cli_options(parser, args)


def test_spool_cli_default_enables_scale_fix_2():
    parser, args = _parse_spool_args()
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["scale_fix"] == [3]


def test_spool_cli_parses_filter_schedule_words():
    parser, args = _parse_spool_args("--filter-schedule", "warmup,warmup,dynamic,constant")
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["filter_schedule"] == ["warmup", "warmup", "dynamic", "constant"]


def test_spool_cli_parses_filter_schedule_numbers():
    parser, args = _parse_spool_args("--filter-schedule", "0,0,1,2")
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["filter_schedule"] == ["warmup", "warmup", "dynamic", "constant"]


def test_spool_cli_parses_objective_schedule_numbers():
    parser, args = _parse_spool_args("--objective-schedule", "0,0,1,2")
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["objective_schedule"] == [0, 0, 1, 2]


def test_spool_cli_parses_objective_schedule_words():
    parser, args = _parse_spool_args("--objective-schedule", "prefit,pointwise,position")
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["objective_schedule"] == [0, 1, 2]


def test_spool_cli_rejects_unknown_objective_schedule_alias():
    parser, args = _parse_spool_args("--objective-schedule", "0,3")
    with pytest.raises(SystemExit):
        _resolve_spool_cli_options(parser, args)


def test_spool_cli_rejects_constant_without_dynamic_since_last_warmup():
    parser, args = _parse_spool_args("--filter-schedule", "warmup,constant")
    with pytest.raises(SystemExit):
        _resolve_spool_cli_options(parser, args)


def test_spool_cli_allows_disabling_scale_fixes():
    parser, args = _parse_spool_args("--scale-fix", "off")
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["scale_fix"] == []


def test_spool_cli_parses_fit_structure_levels():
    parser, args = _parse_spool_args("--fit-structure", "1,3")
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["fit_structure"] == [1, 3]


def test_spool_cli_rejects_invalid_fit_structure_level():
    parser, args = _parse_spool_args("--fit-structure", "4")
    with pytest.raises(SystemExit):
        _resolve_spool_cli_options(parser, args)


def test_spool_cli_parses_optimizer_mode():
    _parser, args = _parse_spool_args("--optimizer-mode", "fast-fd")
    assert str(args.optimizer_mode) == "fast-fd"


def test_full_auto_run_spec_parses_optimizer_mode_override():
    tokens, overrides = _parse_full_auto_run_spec("--optimizer-mode fast-fd --solve-iterations 2")
    assert tokens == ["--optimizer-mode", "fast-fd", "--solve-iterations", "2"]
    assert overrides["optimizer_mode"] == "fast-fd"
    assert overrides["solve_iterations"] == 2


def test_full_auto_run_spec_parses_fit_structure_override():
    tokens, overrides = _parse_full_auto_run_spec("--fit-structure 2")
    assert tokens == ["--fit-structure", "2"]
    assert overrides["fit_structure"] == "2"


def test_full_auto_run_spec_parses_filter_schedule_override():
    tokens, overrides = _parse_full_auto_run_spec("--filter-schedule 0,1,2")
    assert tokens == ["--filter-schedule", "0,1,2"]
    assert overrides["filter_schedule"] == "0,1,2"


def test_full_auto_run_spec_parses_objective_schedule_override():
    tokens, overrides = _parse_full_auto_run_spec("--objective-schedule 0,1,2")
    assert tokens == ["--objective-schedule", "0,1,2"]
    assert overrides["objective_schedule"] == "0,1,2"


def test_parse_filter_schedule_rejects_unknown_numeric_pass_alias():
    with pytest.raises(ValueError, match="0,1,2"):
        _parse_filter_schedule("0,1,3")


def test_parse_objective_schedule_rejects_unknown_numeric_pass_alias():
    with pytest.raises(ValueError, match="0,1,2 or prefit,pointwise,position"):
        _parse_objective_schedule("0,1,3")


def test_parse_filter_schedule_rejects_legacy_refreeze_aliases():
    with pytest.raises(ValueError, match="warmup,dynamic,constant or 0,1,2"):
        _parse_filter_schedule("warmup_no_freeze,refreeze_dynamic,refreeze_constant_mask")


def test_apply_optimizer_mode_env_sets_solver_variables(monkeypatch):
    monkeypatch.delenv("AUTOCAL_OPTIMIZER_MODE", raising=False)
    monkeypatch.delenv("AUTOCAL_DISABLE_JAX_OBJECTIVE", raising=False)
    monkeypatch.delenv("AUTOCAL_JAX_LBFGSB_MODE", raising=False)

    assert _apply_optimizer_mode_env("legacy") == "legacy"
    assert os.environ.get("AUTOCAL_OPTIMIZER_MODE") == "legacy"
    assert os.environ.get("AUTOCAL_DISABLE_JAX_OBJECTIVE") == "1"
    assert os.environ.get("AUTOCAL_JAX_LBFGSB_MODE") == "fun"

    assert _apply_optimizer_mode_env("fast") == "fast"
    assert os.environ.get("AUTOCAL_OPTIMIZER_MODE") == "fast"
    assert os.environ.get("AUTOCAL_JAX_LBFGSB_MODE") == "jac"
    assert os.environ.get("AUTOCAL_DISABLE_JAX_OBJECTIVE") is None

    assert _apply_optimizer_mode_env("fast-fd") == "fast-fd"
    assert os.environ.get("AUTOCAL_OPTIMIZER_MODE") == "fast-fd"
    assert os.environ.get("AUTOCAL_JAX_LBFGSB_MODE") == "fun"
    assert os.environ.get("AUTOCAL_DISABLE_JAX_OBJECTIVE") is None


def test_autocal_main_normalizes_hangprinter_alias(monkeypatch, tmp_path):
    captured = {}

    def fake_full_auto_loop(**kwargs):
        captured["machine_type"] = kwargs["machine_type"]
        captured["verbose"] = kwargs["verbose"]
        return 0

    monkeypatch.setattr(ac, "full_auto_loop", fake_full_auto_loop)

    rc = ac.main(
        [
            "--machine-type",
            "hp3",
            "--verbose",
            "--dataset",
            str(tmp_path / "dummy.json"),
        ]
    )

    assert rc == 0
    assert captured["machine_type"] == "hangprinter_4"
    assert captured["verbose"] is True


def test_autocal_main_forwards_top_level_speedup_to_collector_args(monkeypatch, tmp_path):
    captured = {}

    def fake_full_auto_loop(**kwargs):
        captured["collector_args"] = kwargs["collector_args"]
        return 0

    monkeypatch.setattr(ac, "full_auto_loop", fake_full_auto_loop)

    rc = ac.main(
        [
            "--machine-type",
            "slideprinter",
            "--dataset",
            str(tmp_path / "dummy.json"),
            "--speedup",
            "40",
        ]
    )

    assert rc == 0
    assert captured["collector_args"] == ["--speedup", "40"]


def test_autocal_main_keeps_explicit_collector_speedup_untouched(monkeypatch, tmp_path):
    captured = {}

    def fake_full_auto_loop(**kwargs):
        captured["collector_args"] = kwargs["collector_args"]
        return 0

    monkeypatch.setattr(ac, "full_auto_loop", fake_full_auto_loop)

    rc = ac.main(
        [
            "--machine-type",
            "slideprinter",
            "--dataset",
            str(tmp_path / "dummy.json"),
            "--collector-args",
            "--speedup",
            "40",
        ]
    )

    assert rc == 0
    assert captured["collector_args"] == ["--speedup", "40"]


def test_autocal_main_rejects_removed_full_auto_flag(tmp_path):
    with pytest.raises(SystemExit):
        ac.main(
            [
                "--machine-type",
                "slideprinter",
                "--full-auto",
                "--dataset",
                str(tmp_path / "dummy.json"),
            ]
        )


def test_autocal_main_rejects_removed_full_auto_verbose_flag(tmp_path):
    with pytest.raises(SystemExit):
        ac.main(
            [
                "--machine-type",
                "slideprinter",
                "--full-auto-verbose",
                "--dataset",
                str(tmp_path / "dummy.json"),
            ]
        )
