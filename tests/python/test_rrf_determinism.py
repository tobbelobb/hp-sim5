"""Determinism and standstill-correctness tests for the x86_64 host build of
RepRapFirmware (rrf_simulator).

Verifies that solo runs and concurrent parallel runs produce bit-for-bit
identical CAN motion streams (excluding the wall-clock header timestamp) AND
that physical/kinematic invariants hold (endpoints, net CAN step counts,
dwell gaps, input-shaping standstill completion, and queued M-codes).
"""

from concurrent.futures import ThreadPoolExecutor
import csv
import hashlib
import math
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

import pytest


ROOT = Path(__file__).resolve().parents[2]
SIMULATOR = ROOT / "RRF/build/rrf_simulator"
CSV2CAN = ROOT / "RRF/csv2can.py"
POSITION_RE = re.compile(
    r"^X:([-\d.]+) Y:([-\d.]+) Z:([-\d.]+)(?: U:([-\d.]+))?.* Count ([-\d ]+) Machine",
    re.M,
)
CARTESIAN_CONFIG = """\
G21
G90
M584 X40.0 Y41.0 Z42.0 P3
M669 K0
M92 X80 Y80 Z400
M201 X3000 Y3000 Z3000
M203 X18000 Y18000 Z18000
M204 P3000 T3000
M566 X300 Y300 Z300
M564 H0 S0
"""


def _run_capture(
    work: Path,
    gcode_name: str,
    config_name: str,
    run_id: str,
    *,
    allow_queue_drain: bool = False,
) -> tuple[str, list[str], Path, str]:
    capture = work / f"capture_{run_id}.csv"
    result = subprocess.run(
        [
            str(SIMULATOR),
            "--vsd", str(work),
            "--gcode", f"gcodes/{gcode_name}",
            "-c", f"sys/{config_name}",
            "--can-log", str(capture),
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Error:" not in result.stdout, result.stdout
    if not allow_queue_drain:
        assert "Queue drained" not in result.stdout, result.stdout
    lines = capture.read_text().splitlines()[1:]  # skip JSON metadata line
    digest = hashlib.sha256("\n".join(lines).encode()).hexdigest()
    return digest, lines, capture, result.stdout


def _sum_can_steps(lines: list[str]) -> dict[int, int]:
    totals: dict[int, int] = {}
    for row in csv.reader(lines):
        if not row or row[0] == "T":
            continue
        drive = int(row[1])
        totals[drive] = totals.get(drive, 0) + int(float(row[6]))
    return dict(sorted(totals.items()))


def _stream_gaps_ms(lines: list[str]) -> list[float]:
    """Return gaps (in ms) between consecutive distinct DDA execution windows across all drives."""
    windows: list[tuple[int, int]] = []
    for row in csv.reader(lines):
        if not row or row[0] == "T":
            continue
        when = int(row[2])
        end = when + int(row[3]) + int(row[4]) + int(row[5])
        if windows and windows[-1][0] == when:
            windows[-1] = (when, max(windows[-1][1], end))
        else:
            windows.append((when, end))
    gaps: list[float] = []
    for (_, prev_end), (when, _) in zip(windows, windows[1:]):
        if when != prev_end:
            gaps.append((when - prev_end) / 750.0)
    return gaps


def _parse_m114(stdout: str) -> tuple[list[tuple[float, float, float]], list[tuple[int, ...]]]:
    matches = POSITION_RE.findall(stdout)
    positions = [(float(m[0]), float(m[1]), float(m[2])) for m in matches]
    counts = [tuple(int(v) for v in m[4].split()) for m in matches]
    return positions, counts


@pytest.mark.skipif(not SIMULATOR.exists(), reason="build RRF/build/rrf_simulator first")
def test_copacabana_planeA_deterministic_solo_and_parallel():
    with tempfile.TemporaryDirectory(prefix="rrf-det-copa-", dir="/tmp") as tmp:
        work = Path(tmp)
        shutil.copytree(ROOT / "RRF/run/vsd/sys", work / "sys")
        (work / "gcodes").mkdir()
        shutil.copyfile(
            ROOT / "public/gcode/copacabana_planeA.gcode",
            work / "gcodes/copacabana_planeA.gcode",
        )

        solo_digest, solo_lines, solo_csv, _ = _run_capture(
            work, "copacabana_planeA.gcode", "config_copacabana.g", "solo"
        )
        assert len(solo_lines) > 0

        # Verify only the 4 commanded G4 P3000 dwells create gaps between consecutive moves.
        gaps_ms = _stream_gaps_ms(solo_lines)
        assert len(gaps_ms) == 4, f"unexpected pauses: {gaps_ms}"
        for gap in gaps_ms:
            assert 3000.0 <= gap <= 3070.0, f"unexpected dwell gap {gap} ms"

        # Verify identical .csv and .can output across 8 concurrent simulator processes.
        with ThreadPoolExecutor(max_workers=8) as pool:
            parallel = list(
                pool.map(
                    lambda i: _run_capture(
                        work, "copacabana_planeA.gcode", "config_copacabana.g", f"p{i}"
                    ),
                    range(8),
                )
            )

        for idx, (digest, lines, _, _) in enumerate(parallel):
            assert len(lines) == len(solo_lines), f"parallel run {idx} length mismatch"
            assert digest == solo_digest, f"parallel run {idx} output diverged"

        # Also verify binary .can conversion matches byte-for-byte.
        subprocess.run(["python3", str(CSV2CAN), str(solo_csv)], check=True)
        subprocess.run(["python3", str(CSV2CAN), str(parallel[0][2])], check=True)
        assert solo_csv.with_suffix(".can").read_bytes() == parallel[0][2].with_suffix(".can").read_bytes()


@pytest.mark.skipif(not SIMULATOR.exists(), reason="build RRF/build/rrf_simulator first")
def test_draw_squares_parallel_determinism_and_correctness():
    with tempfile.TemporaryDirectory(prefix="rrf-det-sq-", dir="/tmp") as tmp:
        work = Path(tmp)
        shutil.copytree(ROOT / "RRF/run/vsd/sys", work / "sys")
        (work / "gcodes").mkdir()
        draw_squares = (ROOT / "RRF/run/vsd/gcodes/draw_squares.gcode").read_text()
        (work / "gcodes/draw_squares.gcode").write_text(draw_squares + "\nM400\nM114\n")
        (work / "gcodes/draw_squares_f0.gcode").write_text(
            draw_squares.replace("M666 F1", "M666 F0") + "\nM400\nM114\n"
        )

        solo_digest, solo_lines, _, solo_out = _run_capture(
            work, "draw_squares.gcode", "config_hangprinter.g", "solo"
        )
        assert len(solo_lines) > 1000

        # Independent correctness check: draw_squares enables M666 F1 after G28, so returning
        # to (0, 0, 0) keeps the origin pretension stretch (-14, -15, -10, -4 steps) while
        # the flex-aware forward transform still reports (0, 0, 0) within step quantization.
        positions, counts = _parse_m114(solo_out)
        assert len(positions) == 1, solo_out
        for actual, wanted in zip(positions[0], (0.0, 0.0, 0.0)):
            assert math.isclose(actual, wanted, abs_tol=0.05), positions[0]
        net_steps = _sum_can_steps(solo_lines)
        assert tuple(net_steps[d] for d in (40, 41, 42, 43)) == counts[0]
        assert counts[0] == (-14, -15, -10, -4)
        assert _stream_gaps_ms(solo_lines) == []

        # With flex disabled (M666 F0), returning to origin must yield exactly zero net steps.
        _, f0_lines, _, f0_out = _run_capture(
            work, "draw_squares_f0.gcode", "config_hangprinter.g", "f0"
        )
        f0_positions, f0_counts = _parse_m114(f0_out)
        assert f0_positions == [(0.0, 0.0, 0.0)]
        assert f0_counts == [(0, 0, 0, 0)]
        assert _sum_can_steps(f0_lines) == {40: 0, 41: 0, 42: 0, 43: 0}

        with ThreadPoolExecutor(max_workers=8) as pool:
            parallel = list(
                pool.map(
                    lambda i: _run_capture(
                        work, "draw_squares.gcode", "config_hangprinter.g", f"p{i}"
                    ),
                    range(8),
                )
            )

        for idx, (digest, lines, _, _) in enumerate(parallel):
            assert len(lines) == len(solo_lines), f"parallel run {idx} length mismatch"
            assert digest == solo_digest, f"parallel run {idx} output diverged"


@pytest.mark.skipif(not SIMULATOR.exists(), reason="build RRF/build/rrf_simulator first")
@pytest.mark.parametrize(
    "shaper_cmd",
    [
        'M593 P"zv" F40',
        'M593 P"zvd" F20',
        'M593 P"ei2" F20',
        'M593 P"ei3" F15',
    ],
)
def test_input_shaping_standstill_and_parallel_determinism(shaper_cmd):
    """M400 and G4 must wait for delayed input-shaped segments to retire so motor
    counts, reconstructed Cartesian coordinates, and subsequent moves stay exact."""
    gcode = (
        f"G92 X0 Y0 Z0\n{shaper_cmd}\n"
        "G1 X10 F6000\nM400\nM114\n"
        "G4 P500\n"
        "G1 X0 F6000\nM400\nM114\n"
    )
    with tempfile.TemporaryDirectory(prefix="rrf-det-shaper-", dir="/tmp") as tmp:
        work = Path(tmp)
        (work / "sys").mkdir()
        (work / "sys/config.g").write_text(CARTESIAN_CONFIG)
        (work / "gcodes").mkdir()
        (work / "gcodes/test.gcode").write_text(gcode)

        solo_digest, solo_lines, _, solo_out = _run_capture(work, "test.gcode", "config.g", "solo")
        positions, counts = _parse_m114(solo_out)
        assert positions == [(10.0, 0.0, 0.0), (0.0, 0.0, 0.0)], solo_out
        assert counts == [(800, 0, 0), (0, 0, 0)], solo_out
        drive40_steps = [int(r[6]) for r in csv.reader(solo_lines) if r[1] == "40"]
        assert drive40_steps == [800, -800]
        # Gap between move 1 and move 2 must include at least the 500 ms G4 dwell plus shaping tail.
        gaps_ms = _stream_gaps_ms(solo_lines)
        assert len(gaps_ms) == 1 and gaps_ms[0] >= 500.0, gaps_ms

        with ThreadPoolExecutor(max_workers=6) as pool:
            parallel = list(
                pool.map(lambda i: _run_capture(work, "test.gcode", "config.g", f"p{i}"), range(6))
            )
        for idx, (digest, lines, _, _) in enumerate(parallel):
            assert len(lines) == len(solo_lines), f"{shaper_cmd} run {idx} length mismatch"
            assert digest == solo_digest, f"{shaper_cmd} run {idx} output diverged"


@pytest.mark.skipif(not SIMULATOR.exists(), reason="build RRF/build/rrf_simulator first")
def test_gcode_queue_overflow_correctness_and_parallel_determinism():
    m106s = "\n".join(f"M106 P0 S{(i + 1) / 40.0:.3f}" for i in range(40))
    gcode = (
        "G21\nG90\nG92 X0 Y0 Z0\n"
        "G1 X100 Y0 F1200\n"
        f"{m106s}\n"
        "G1 X0 Y0 F1200\n"
        "M400\nM114\n"
    )
    with tempfile.TemporaryDirectory(prefix="rrf-det-qovf-", dir="/tmp") as tmp:
        work = Path(tmp)
        shutil.copytree(ROOT / "RRF/run/vsd/sys", work / "sys")
        (work / "gcodes").mkdir()
        (work / "gcodes/test.gcode").write_text(gcode)

        solo_digest, solo_lines, _, solo_out = _run_capture(
            work, "test.gcode", "config_slideprinter.g", "solo", allow_queue_drain=True
        )
        assert len(solo_lines) > 0
        positions, counts = _parse_m114(solo_out)
        assert positions == [(0.0, 0.0, 0.0)], solo_out
        assert counts == [(0, 0, 0)], solo_out
        # Both outbound (X100) and return (X0) moves executed, cancelling to zero net steps.
        net_steps = _sum_can_steps(solo_lines)
        assert net_steps == {40: 0, 41: 0, 42: 0}
        # Confirm actual non-zero outbound motion occurred before returning to origin.
        max_abs_cum = 0
        cum = 0
        for row in csv.reader(solo_lines):
            if row[1] == "41":
                cum += int(float(row[6]))
                max_abs_cum = max(max_abs_cum, abs(cum))
        assert max_abs_cum > 1000

        with ThreadPoolExecutor(max_workers=6) as pool:
            parallel = list(
                pool.map(
                    lambda i: _run_capture(
                        work, "test.gcode", "config_slideprinter.g", f"p{i}", allow_queue_drain=True
                    ),
                    range(6),
                )
            )
        for idx, (digest, lines, _, _) in enumerate(parallel):
            assert len(lines) == len(solo_lines), f"queue overflow run {idx} length mismatch"
            assert digest == solo_digest, f"queue overflow run {idx} output diverged"


@pytest.mark.skipif(not SIMULATOR.exists(), reason="build RRF/build/rrf_simulator first")
def test_copacabana_flex_correctness_and_parallel_determinism():
    gcode = (
        "G21\nG90\nG92 X0 Y0 Z0\nM666 F1\n"
        "G1 X97.4 Y-2.5 Z-150.6 F30000\nM400\nM114\n"
        "G4 P1000\n"
        "G1 X200 Y100 Z-50\nM400\nM114\n"
        "G4 P1000\n"
        "G1 X0 Y0 Z0\nM400\nM114\n"
    )
    with tempfile.TemporaryDirectory(prefix="rrf-det-flex-", dir="/tmp") as tmp:
        work = Path(tmp)
        shutil.copytree(ROOT / "RRF/run/vsd/sys", work / "sys")
        (work / "gcodes").mkdir()
        (work / "gcodes/test.gcode").write_text(gcode)

        solo_digest, solo_lines, _, solo_out = _run_capture(
            work, "test.gcode", "config_copacabana.g", "solo"
        )
        assert len(solo_lines) > 0
        positions, counts = _parse_m114(solo_out)
        expected_waypoints = [(97.4, -2.5, -150.6), (200.0, 100.0, -50.0), (0.0, 0.0, 0.0)]
        assert len(positions) == 3, solo_out
        for actual_pos, wanted_pos in zip(positions, expected_waypoints):
            for a, w in zip(actual_pos, wanted_pos):
                assert math.isclose(a, w, abs_tol=0.1), (actual_pos, wanted_pos)

        net_steps = _sum_can_steps(solo_lines)
        assert tuple(net_steps[d] for d in range(40, 48)) == counts[-1]

        gaps_ms = _stream_gaps_ms(solo_lines)
        assert len(gaps_ms) == 2, f"expected 2 G4 P1000 dwells, got {gaps_ms}"
        for gap in gaps_ms:
            assert 1000.0 <= gap <= 1070.0, f"unexpected dwell gap {gap} ms"

        with ThreadPoolExecutor(max_workers=6) as pool:
            parallel = list(
                pool.map(
                    lambda i: _run_capture(work, "test.gcode", "config_copacabana.g", f"p{i}"),
                    range(6),
                )
            )
        for idx, (digest, lines, _, _) in enumerate(parallel):
            assert len(lines) == len(solo_lines), f"flex run {idx} length mismatch"
            assert digest == solo_digest, f"flex run {idx} output diverged"
