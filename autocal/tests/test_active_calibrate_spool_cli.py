import os

import pytest

from autocal.active_calibrate import (
    _apply_optimizer_mode_env,
    _parse_full_auto_run_spec,
    _resolve_spool_cli_options,
    build_ellipse_parser,
)


def test_spool_cli_options_parse_modes_and_bounds():
    parser = build_ellipse_parser()
    args = parser.parse_args(
        [
            "dummy.json",
            "--machine-type",
            "slideprinter",
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
        ]
    )
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["find_radii"] == "global"
    assert opts["find_buildup_factor"] == "off"
    assert opts["theta0_mode"] == "infer"
    assert opts["r0_bounds"] == (12.0, 34.0)
    assert opts["spool_outer_iters"] == 5
    assert opts["spool_inner_iters"] == 17
    assert opts["line_width"] == 1.25
    assert opts["sigma_floor_mm"] == 0.05
    assert opts["sigma_used_mm"] == 0.8


def test_spool_cli_flags_without_value_default_to_per_anchor():
    parser = build_ellipse_parser()
    args = parser.parse_args(
        [
            "dummy.json",
            "--machine-type",
            "slideprinter",
            "--find-radii",
            "--find-buildup-factor",
        ]
    )
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["find_radii"] == "per-anchor"
    assert opts["find_buildup_factor"] == "per-anchor"
    assert opts["theta0_mode"] == "zero"


def test_spool_cli_rejects_invalid_bounds():
    parser = build_ellipse_parser()
    args = parser.parse_args(
        [
            "dummy.json",
            "--machine-type",
            "slideprinter",
            "--r0-bounds",
            "10",
        ]
    )
    with pytest.raises(SystemExit):
        _resolve_spool_cli_options(parser, args)


def test_spool_cli_rejects_negative_line_width():
    parser = build_ellipse_parser()
    args = parser.parse_args(
        [
            "dummy.json",
            "--machine-type",
            "slideprinter",
            "--line-width",
            "-0.1",
        ]
    )
    with pytest.raises(SystemExit):
        _resolve_spool_cli_options(parser, args)


def test_spool_cli_rejects_nonpositive_sigma_floor_mm():
    parser = build_ellipse_parser()
    args = parser.parse_args(
        [
            "dummy.json",
            "--machine-type",
            "slideprinter",
            "--sigma-floor-mm",
            "0",
        ]
    )
    with pytest.raises(SystemExit):
        _resolve_spool_cli_options(parser, args)


def test_spool_cli_rejects_nonpositive_sigma_used_mm():
    parser = build_ellipse_parser()
    args = parser.parse_args(
        [
            "dummy.json",
            "--machine-type",
            "slideprinter",
            "--sigma-used-mm",
            "0",
        ]
    )
    with pytest.raises(SystemExit):
        _resolve_spool_cli_options(parser, args)


def test_spool_cli_parses_scale_fix_levels():
    parser = build_ellipse_parser()
    args = parser.parse_args(
        [
            "dummy.json",
            "--machine-type",
            "slideprinter",
            "--scale-fix",
            "1,3",
        ]
    )
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["scale_fix"] == [1, 3]


def test_spool_cli_rejects_invalid_scale_fix_level():
    parser = build_ellipse_parser()
    args = parser.parse_args(
        [
            "dummy.json",
            "--machine-type",
            "slideprinter",
            "--scale-fix",
            "1,4",
        ]
    )
    with pytest.raises(SystemExit):
        _resolve_spool_cli_options(parser, args)


def test_spool_cli_default_enables_scale_fix_2():
    parser = build_ellipse_parser()
    args = parser.parse_args(
        [
            "dummy.json",
            "--machine-type",
            "slideprinter",
        ]
    )
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["scale_fix"] == [2]


def test_spool_cli_allows_disabling_scale_fixes():
    parser = build_ellipse_parser()
    args = parser.parse_args(
        [
            "dummy.json",
            "--machine-type",
            "slideprinter",
            "--scale-fix",
            "off",
        ]
    )
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["scale_fix"] == []


def test_spool_cli_parses_optimizer_mode():
    parser = build_ellipse_parser()
    args = parser.parse_args(
        [
            "dummy.json",
            "--machine-type",
            "slideprinter",
            "--optimizer-mode",
            "fast-fd",
        ]
    )
    assert str(args.optimizer_mode) == "fast-fd"


def test_full_auto_run_spec_parses_optimizer_mode_override():
    tokens, overrides = _parse_full_auto_run_spec("--optimizer-mode fast-fd --solve-iterations 2")
    assert tokens == ["--optimizer-mode", "fast-fd", "--solve-iterations", "2"]
    assert overrides["optimizer_mode"] == "fast-fd"
    assert overrides["solve_iterations"] == 2


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
