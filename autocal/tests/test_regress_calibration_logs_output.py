import importlib.util
import math
import sys
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "tools" / "regress_calibration_logs.py"
SPEC = importlib.util.spec_from_file_location("regress_calibration_logs", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
rcl = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = rcl
SPEC.loader.exec_module(rcl)


def _parsed(*, score_ref=10.0, score_gen=11.0):
    ref_summary = rcl.Params(
        anchors=[(0.0, -1900.0), (1645.44826719, 950.0), (-1645.44826719, 950.0)],
        radii=[39.184, 39.184, 39.184],
        score=score_ref,
    )
    gen_summary = rcl.Params(
        anchors=[(0.3, -1899.8), (1645.54826719, 949.9), (-1645.44826719, 950.2)],
        radii=[39.185, 39.183, 39.184],
        score=score_gen,
    )
    ref = rcl.ParsedLog(
        summary=ref_summary,
        iterations=[rcl.Iteration(anchors=ref_summary.anchors, radii=ref_summary.radii, score=score_ref)],
        summary_block_lines=["== Calibration summary ==", "Fit quality score: 10.0"],
    )
    gen = rcl.ParsedLog(
        summary=gen_summary,
        iterations=[rcl.Iteration(anchors=gen_summary.anchors, radii=gen_summary.radii, score=score_gen)],
        summary_block_lines=["== Calibration summary ==", "Fit quality score: 11.0"],
    )
    return ref, gen


def test_format_table_keeps_columns_aligned():
    lines = rcl.format_table(
        headers=["col1", "column_two", "c3"],
        rows=[
            ["a", "bbbb", "1"],
            ["longer", "z", "999"],
        ],
    )
    bar_positions = [[i for i, ch in enumerate(line) if ch == "|"] for line in lines if "|" in line]
    assert bar_positions
    assert all(pos == bar_positions[0] for pos in bar_positions)


def test_report_dataset_has_readable_summary_and_verdicts():
    ref, gen = _parsed(score_ref=10.0, score_gen=11.0)
    ok, lines, true_delta = rcl.report_dataset(
        name="demo",
        ref=ref,
        gen=gen,
        tol_mm_total=10.0,
        fail_on_score_mismatch=True,
        color=False,
    )
    text = "\n".join(lines)
    assert ok
    assert "Calibration summary:" in text
    assert "delta(gen-ref)" in text
    assert "verdict" in text
    assert "worse" in text
    assert "RUN_TRACKER: true_err_total delta(gen-ref)=" in text
    assert "true_gen_iter_mean=" in text
    assert "true_gen_iter_std=0.000" in text
    assert "(n=1)" in text
    assert isinstance(true_delta, float)


def test_report_dataset_can_colorize_verdicts():
    ref, gen = _parsed(score_ref=10.0, score_gen=11.0)
    _, lines, _ = rcl.report_dataset(
        name="demo",
        ref=ref,
        gen=gen,
        tol_mm_total=10.0,
        fail_on_score_mismatch=True,
        color=True,
    )
    text = "\n".join(lines)
    assert "\x1b[31mworse\x1b[0m" in text


def test_report_dataset_shows_extra_generated_iterations():
    ref_summary = rcl.Params(
        anchors=[(0.0, -1900.0), (1645.44826719, 950.0), (-1645.44826719, 950.0)],
        radii=[39.184, 39.184, 39.184],
        score=2000.0,
    )
    gen_summary = rcl.Params(
        anchors=[(0.0, -1900.0), (1645.44826719, 950.0), (-1645.44826719, 950.0)],
        radii=[39.184, 39.184, 39.184],
        score=100.0,
    )
    ref = rcl.ParsedLog(
        summary=ref_summary,
        iterations=[
            rcl.Iteration(
                anchors=[(10.0, -1890.0), (1640.0, 955.0), (-1640.0, 955.0)],
                radii=[40.0, 40.0, 40.0],
                score=2000.0,
            ),
        ],
        summary_block_lines=["== Calibration summary ==", "Fit quality score: 2000.0"],
    )
    gen = rcl.ParsedLog(
        summary=gen_summary,
        iterations=[
            rcl.Iteration(
                anchors=[(12.0, -1888.0), (1638.0, 957.0), (-1638.0, 957.0)],
                radii=[40.1, 40.1, 40.1],
                score=2100.0,
            ),
            rcl.Iteration(
                anchors=gen_summary.anchors,
                radii=gen_summary.radii,
                score=100.0,
            ),
        ],
        summary_block_lines=["== Calibration summary ==", "Fit quality score: 100.0"],
    )

    _, lines, _ = rcl.report_dataset(
        name="demo",
        ref=ref,
        gen=gen,
        tol_mm_total=10.0,
        fail_on_score_mismatch=True,
        color=False,
    )
    text = "\n".join(lines)
    assert "ITERATIONS: ref=1 gen=2" in text
    assert "missing" in text
    assert "100.000" in text


def test_prepare_isolated_dataset_copy_creates_unique_paths(tmp_path):
    src = tmp_path / "demo.json"
    src.write_text('{"x": 1}', encoding="utf-8")
    scratch = tmp_path / "scratch"

    first = rcl.prepare_isolated_dataset_copy("demo", src, scratch)
    second = rcl.prepare_isolated_dataset_copy("demo", src, scratch)

    assert first != second
    assert first.read_text(encoding="utf-8") == src.read_text(encoding="utf-8")
    assert second.read_text(encoding="utf-8") == src.read_text(encoding="utf-8")


def test_compute_true_gen_iter_stats_returns_mean_and_std():
    iters = [
        rcl.Iteration(
            anchors=[(0.0, -1900.0), (1645.44826719, 950.0), (-1645.44826719, 950.0)],
            radii=[39.184, 39.184, 39.184],
            score=1.0,
        ),
        rcl.Iteration(
            anchors=[(1.0, -1900.0), (1645.44826719, 951.0), (-1644.44826719, 950.0)],
            radii=[39.184, 39.184, 39.184],
            score=2.0,
        ),
    ]

    mean, std, count = rcl.compute_true_gen_iter_stats(iters)
    assert count == 2
    assert mean is not None
    assert std is not None
    assert math.isclose(mean, 1.5, rel_tol=1e-9, abs_tol=1e-9)
    assert math.isclose(std, 1.5, rel_tol=1e-9, abs_tol=1e-9)


def test_run_active_calibrate_passes_full_auto_log(monkeypatch, tmp_path):
    captured = {}

    class DummyProc:
        returncode = 0
        stdout = "ok"

    def fake_run(cmd, **kwargs):
        captured["cmd"] = cmd
        return DummyProc()

    monkeypatch.setattr(rcl.subprocess, "run", fake_run)
    repo_root = tmp_path
    dataset = tmp_path / "demo.json"
    dataset.write_text("{}", encoding="utf-8")
    jsonl = tmp_path / "demo.full_auto_log.jsonl"

    rc, out = rcl.run_active_calibrate(repo_root, dataset, full_auto_log=jsonl)
    assert rc == 0
    assert out == "ok"
    assert "--full-auto-log" in captured["cmd"]
    idx = captured["cmd"].index("--full-auto-log")
    assert captured["cmd"][idx + 1] == str(jsonl)


def test_main_run_tracker_summary_includes_true_gen_stats(monkeypatch, tmp_path, capsys):
    repo_root = tmp_path
    (repo_root / "autocal").mkdir()
    (repo_root / "autocal" / "active_calibrate.py").write_text("# stub\n", encoding="utf-8")
    (repo_root / "data").mkdir()
    (repo_root / "refs").mkdir()
    (repo_root / "data" / "demo.json").write_text("{}", encoding="utf-8")
    (repo_root / "refs" / "demo.full_auto_reference_run_feb_26.log").write_text("log", encoding="utf-8")

    monkeypatch.setattr(rcl, "DATASETS", ["demo"])

    def fake_run_one_dataset(**kwargs):
        return rcl.DatasetRunResult(
            name="demo",
            ok=True,
            lines=["demo lines"],
            generated_log=repo_root / "generated.log",
            reference_log=repo_root / "refs" / "demo.full_auto_reference_run_feb_26.log",
            true_err_total_delta=1.25,
            true_gen_iter_mean=4.5,
            true_gen_iter_std=0.75,
            true_gen_iter_count=3,
        )

    monkeypatch.setattr(rcl, "run_one_dataset", fake_run_one_dataset)
    monkeypatch.setattr(
        sys,
        "argv",
        ["prog", "--repo-root", str(repo_root), "--data-dir", "data", "--ref-dir", "refs"],
    )

    rc = rcl.main()
    out = capsys.readouterr().out

    assert rc == 0
    assert "RUN_TRACKER: true_err_total delta(gen-ref) and true_gen iter stats by dataset" in out
    assert "true_gen_mean" in out
    assert "true_gen_std" in out
    assert "n_true_gen" in out
    assert "4.500" in out
    assert "0.750" in out
    assert "RUN_TRACKER: final_score=" in out
    assert "3.500" in out


def test_compute_final_score_uses_requested_formula():
    score = rcl.compute_final_score(total_error_sum=8.0, mean_error_sum=6.0)
    assert math.isclose(score, 11.0, rel_tol=1e-9, abs_tol=1e-9)
