import importlib.util
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
    ok, lines = rcl.report_dataset(
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


def test_report_dataset_can_colorize_verdicts():
    ref, gen = _parsed(score_ref=10.0, score_gen=11.0)
    _, lines = rcl.report_dataset(
        name="demo",
        ref=ref,
        gen=gen,
        tol_mm_total=10.0,
        fail_on_score_mismatch=True,
        color=True,
    )
    text = "\n".join(lines)
    assert "\x1b[31mworse\x1b[0m" in text


def test_prepare_isolated_dataset_copy_creates_unique_paths(tmp_path):
    src = tmp_path / "demo.json"
    src.write_text('{"x": 1}', encoding="utf-8")
    scratch = tmp_path / "scratch"

    first = rcl.prepare_isolated_dataset_copy("demo", src, scratch)
    second = rcl.prepare_isolated_dataset_copy("demo", src, scratch)

    assert first != second
    assert first.read_text(encoding="utf-8") == src.read_text(encoding="utf-8")
    assert second.read_text(encoding="utf-8") == src.read_text(encoding="utf-8")


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
