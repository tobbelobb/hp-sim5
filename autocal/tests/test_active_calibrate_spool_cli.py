import pytest

from autocal.active_calibrate import build_ellipse_parser, _resolve_spool_cli_options


def test_spool_cli_options_parse_modes_and_bounds():
    parser = build_ellipse_parser()
    args = parser.parse_args(
        [
            "dummy.json",
            "--machine-type",
            "slideprinter",
            "--find-radii",
            "global",
            "--r0-bounds",
            "12,34",
            "--spool-outer-iters",
            "5",
            "--spool-inner-iters",
            "17",
        ]
    )
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["find_radii"] == "global"
    assert opts["find_buildup_factor"] == "off"
    assert opts["r0_bounds"] == (12.0, 34.0)
    assert opts["spool_outer_iters"] == 5
    assert opts["spool_inner_iters"] == 17


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
