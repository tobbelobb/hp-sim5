import os

import pytest

import autocal.autocal as ac
from autocal._autocal_common import (
    _apply_optimizer_mode_env,
    _parse_filter_schedule,
    _parse_full_auto_run_spec,
    _parse_objective_schedule,
    _resolve_spool_cli_options,
    _suggested_collect_command,
    build_semi_auto_parser,
)
from autocal.active_learning import SweepConfig


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


def test_home_offsets_and_equal_strain_default_on_with_explicit_ablation():
    parser, args = _parse_spool_args()
    _resolve_spool_cli_options(parser, args)
    assert args.fit_home_offsets and args.equal_strain
    parser, args = _parse_spool_args("--no-fit-home-offsets", "--no-equal-strain")
    _resolve_spool_cli_options(parser, args)
    assert not args.fit_home_offsets and not args.equal_strain
    _, overrides = _parse_full_auto_run_spec("--no-fit-home-offsets")
    assert overrides["fit_home_offsets"] is False


def test_spool_fitting_turns_home_offsets_off_unless_requested():
    parser, args = _parse_spool_args("--find-radii", "global")
    _resolve_spool_cli_options(parser, args)
    assert args.fit_home_offsets is False
    parser, args = _parse_spool_args("--find-radii", "global", "--fit-home-offsets")
    with pytest.raises(SystemExit):
        _resolve_spool_cli_options(parser, args)


def test_planned_collection_cannot_be_redirected_by_forwarded_aliases(tmp_path):
    cfg = SweepConfig((0,), (12.0,), 1, 2)
    command = _suggested_collect_command(
        tmp_path / "planned.txt", cfg, machine_type="slideprinter",
        output_file=tmp_path / "new.json",
        completed_sweeps=3,
        extra_args=["--out", "training.json", "--output=training.json",
                    "--machine-type", "hp4", "--sweepFile=wrong.txt",
                    "--fixedTargets", "90", "--hp-sim-reset", "--completed-sweeps", "90",
                    "--server", "http://example.invalid", "--force-mode", "constant"],
    )
    assert command == [
        "node", "autocal/control/cli/collect_sweep_data.mjs",
        "--machineType", "slideprinter", "--sweep-config-file", str(tmp_path / "planned.txt"),
        "--fixed-targets", "12.0", "--output-file", str(tmp_path / "new.json"),
        "--completed-sweeps", "3",
        "--server", "http://example.invalid", "--force-mode", "constant",
    ]


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
        "--line-width",
        "1.25",
        "--sigma-floor-mm",
        "0.05",
    )
    opts = _resolve_spool_cli_options(parser, args)
    assert opts["find_radii"] == "global"
    assert opts["find_buildup_factor"] == "off"
    assert opts["theta0_mode"] == "infer"
    assert opts["r0_bounds"] == (12.0, 34.0)
    assert opts["spool_outer_iters"] == 5
    assert opts["filter_schedule"] == ["warmup", "warmup", "warmup", "dynamic"]
    assert opts["objective_schedule"] == [1, 1, 1, 1]
    assert opts["line_width"] == 1.25
    assert opts["sigma_floor_mm"] == 0.05


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


def test_parse_filter_schedule_rejects_unknown_numeric_pass_alias():
    with pytest.raises(ValueError, match="0,1,2"):
        _parse_filter_schedule("0,1,3")


def test_parse_objective_schedule_rejects_unknown_numeric_pass_alias():
    with pytest.raises(ValueError, match="0,1,2 or prefit,pointwise,position"):
        _parse_objective_schedule("0,1,3")


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
    assert captured["machine_type"] == "hangprinter_3"
    assert captured["verbose"] is True


@pytest.mark.parametrize("flag, value", [("--speedup", "40"), ("--solver-iterations", "8")])
def test_autocal_main_forwards_top_level_simulation_settings_to_collector_args(monkeypatch, tmp_path, flag, value):
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
            flag,
            value,
        ]
    )

    assert rc == 0
    assert captured["collector_args"] == [flag, value]


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


def test_open_loop_motors_option():
    _, args = _parse_spool_args("--open-loop-motors")
    assert args.open_loop_motors is True
    _, defaults = _parse_spool_args()
    assert defaults.open_loop_motors is False


@pytest.mark.parametrize("positive,negative,key", [
    ("--pointwise-filtering", "--no-pointwise-filtering", "pointwise_filtering"),
    ("--sweep-wise-filtering", "--no-sweep-wise-filtering", "sweep_wise_filtering"),
    ("--pointwise-global-mad", "--pointwise-per-sweep-mad", "pointwise_global_mad"),
    ("--use-noise-mean", "--use-raw-lengths", "use_noise_mean"),
])
def test_filter_variants_preserve_inheritance_and_explicit_reenable(positive, negative, key):
    _, args = _parse_spool_args(negative)
    assert getattr(args, key) is False
    assert _parse_full_auto_run_spec("")[1] == {}
    assert _parse_full_auto_run_spec(positive)[1] == {key: True}
    assert _parse_full_auto_run_spec(negative)[1] == {key: False}
    assert _parse_full_auto_run_spec(f"{negative} {positive}")[1] == {key: True}


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf"])
def test_planning_ridge_requires_a_finite_positive_value(value):
    with pytest.raises(SystemExit):
        _parse_spool_args("--regularization", value)




@pytest.mark.parametrize("args, expected", [
    ([], 1.0),
    (["--precision", "low"], 5.0),
    (["--precision", "medium"], 1.0),
    (["--precision", "high"], 0.5),
    (["--precision", "2.5"], 2.5),
    (["--precision", "0"], 0.0),
    (["--thoroughness", "low"], 5.0),
    (["--thoroughness", "0.25"], 0.25),
    (["--stable-step-mm", "0.2"], 0.2),
    (["--stable-step-mm", "0"], 0.0),
])
def test_stable_step_reaches_the_loop(monkeypatch, tmp_path, args, expected):
    captured = {}
    monkeypatch.setattr(ac, "full_auto_loop", lambda **kwargs: captured.update(kwargs) or 0)
    assert ac.main(["--machine-type", "hp3", "--dataset", str(tmp_path / "d.json"), *args]) == 0
    assert captured["stable_step_mm"] == expected


@pytest.mark.parametrize("args", [
    ["--stable-step-mm", "-0.1"],
    ["--stable-step-mm", "nan"],
    ["--stable-step-mm", "inf"],
    ["--precision", "bogus"],
    ["--precision", "-1"],
    ["--precision", "low", "--stable-step-mm", "0.5"],
])
def test_stable_step_must_be_a_finite_nonnegative_distance(tmp_path, args):
    with pytest.raises(SystemExit):
        ac.main(["--machine-type", "hp3", "--dataset", str(tmp_path / "d.json"), *args])


def test_log_entries_put_wall_and_sim_time_before_the_message():
    line = ac._format_log_entry("; full-auto: stopping", wall_s=1.5, sim_s=None)
    assert line == "     1.50s           -  | ; full-auto: stopping"
    assert ac._format_log_entry("x", wall_s=12.0, sim_s=3.25).startswith("    12.00s       3.25s  | x")


def test_redirected_output_is_logged_one_complete_line_at_a_time():
    lines = []
    sink = ac._LineSink(lines.append)
    sink.write("a\nb")
    sink.write("c\n")
    sink.write("tail")
    assert lines == ["a", "bc"]
    sink.close()
    assert lines == ["a", "bc", "tail"]


def test_no_collect_refuses_a_missing_dataset_without_collecting(tmp_path, monkeypatch, capsys):
    def collection_started(*args, **kwargs):
        raise AssertionError("collection started")

    monkeypatch.setattr(ac.subprocess, "Popen", collection_started)
    monkeypatch.setattr(ac.subprocess, "run", collection_started)
    dataset = tmp_path / "new" / "data.json"
    with pytest.raises(SystemExit) as refused:
        ac.main(["--machine-type", "hangprinter_5", "--dataset", str(dataset), "--no-collect"])
    assert refused.value.code == 2
    assert "--no-collect needs an existing dataset" in capsys.readouterr().err
    assert not dataset.parent.exists()


def test_collector_prompt_without_newline_is_yielded_while_waiting():
    import subprocess
    import sys

    child = subprocess.Popen(
        [sys.executable, "-c",
         "import sys, time; sys.stdout.write('Enter to send'); sys.stdout.flush(); "
         "time.sleep(2); print('done')"],
        stdout=subprocess.PIPE,
    )
    try:
        lines = ac._collector_output_lines(child.stdout)
        assert next(lines) == "Enter to send"
        assert child.poll() is None  # the prompt arrived while the collector was still waiting
        assert list(lines) == ["done"]
    finally:
        child.wait()


def test_collector_output_reader_works_for_high_descriptor_numbers():
    import os
    import subprocess
    import sys

    child = subprocess.Popen(
        [sys.executable, "-c", "print('first'); print('second')"], stdout=subprocess.PIPE,
    )
    high_fd = 2000  # select.select() rejects descriptors of 1024 and above on Linux
    os.dup2(child.stdout.fileno(), high_fd)

    class HighDescriptor:
        def fileno(self):
            return high_fd

    try:
        assert list(ac._collector_output_lines(HighDescriptor())) == ["first", "second"]
    finally:
        os.close(high_fd)
        child.wait()
        child.stdout.close()
