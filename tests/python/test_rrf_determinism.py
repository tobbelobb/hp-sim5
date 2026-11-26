"""Determinism tests for the x86_64 host build of RepRapFirmware (rrf_simulator).

Verifies that solo runs and concurrent parallel runs produce bit-for-bit
identical CAN motion streams (excluding the wall-clock header timestamp),
with no host-load stalls or extra pauses.
"""

from concurrent.futures import ThreadPoolExecutor
import csv
import hashlib
from pathlib import Path
import shutil
import subprocess
import tempfile

import pytest


ROOT = Path(__file__).resolve().parents[2]
SIMULATOR = ROOT / "RRF/build/rrf_simulator"
CSV2CAN = ROOT / "RRF/csv2can.py"


def _run_capture(
    work: Path,
    gcode_name: str,
    config_name: str,
    run_id: str,
    *,
    allow_queue_drain: bool = False,
) -> tuple[str, list[str], Path]:
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
    return digest, lines, capture


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

        solo_digest, solo_lines, solo_csv = _run_capture(
            work, "copacabana_planeA.gcode", "config_copacabana.g", "solo"
        )
        assert len(solo_lines) > 0

        # Verify only the 4 commanded G4 P3000 dwells create gaps between consecutive moves.
        drive40 = [r for r in csv.reader(solo_lines) if r[1] == "40"]
        prev_end = None
        gaps_ms = []
        for row in drive40:
            when = int(row[2])
            dur = int(row[3]) + int(row[4]) + int(row[5])
            if prev_end is not None and when != prev_end:
                gaps_ms.append((when - prev_end) / 750.0)
            prev_end = when + dur
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

        for idx, (digest, lines, _) in enumerate(parallel):
            assert len(lines) == len(solo_lines), f"parallel run {idx} length mismatch"
            assert digest == solo_digest, f"parallel run {idx} output diverged"

        # Also verify binary .can conversion matches byte-for-byte.
        subprocess.run(["python3", str(CSV2CAN), str(solo_csv)], check=True)
        subprocess.run(["python3", str(CSV2CAN), str(parallel[0][2])], check=True)
        assert solo_csv.with_suffix(".can").read_bytes() == parallel[0][2].with_suffix(".can").read_bytes()


@pytest.mark.skipif(not SIMULATOR.exists(), reason="build RRF/build/rrf_simulator first")
def test_draw_squares_parallel_determinism():
    with tempfile.TemporaryDirectory(prefix="rrf-det-sq-", dir="/tmp") as tmp:
        work = Path(tmp)
        shutil.copytree(ROOT / "RRF/run/vsd/sys", work / "sys")
        (work / "gcodes").mkdir()
        shutil.copyfile(
            ROOT / "RRF/run/vsd/gcodes/draw_squares.gcode",
            work / "gcodes/draw_squares.gcode",
        )

        solo_digest, solo_lines, _ = _run_capture(
            work, "draw_squares.gcode", "config_hangprinter.g", "solo"
        )
        assert len(solo_lines) > 1000

        with ThreadPoolExecutor(max_workers=8) as pool:
            parallel = list(
                pool.map(
                    lambda i: _run_capture(
                        work, "draw_squares.gcode", "config_hangprinter.g", f"p{i}"
                    ),
                    range(8),
                )
            )

        for idx, (digest, lines, _) in enumerate(parallel):
            assert len(lines) == len(solo_lines), f"parallel run {idx} length mismatch"
            assert digest == solo_digest, f"parallel run {idx} output diverged"


@pytest.mark.skipif(not SIMULATOR.exists(), reason="build RRF/build/rrf_simulator first")
@pytest.mark.parametrize(
    ("name", "config", "gcode"),
    [
        (
            "input_shaping_zv",
            "config_slideprinter.g",
            'G21\nG90\nG92 X0 Y0 Z0\nM593 P"zv" F40\n'
            "G1 X50 Y0 F6000\nG1 X50 Y50\nG1 X0 Y50\nG1 X0 Y0\nG4 P500\nG1 X25 Y25 F3000\nM400\n",
        ),
        (
            "gcode_queue_overflow",
            "config_slideprinter.g",
            "G21\nG90\nG92 X0 Y0 Z0\nG1 X100 Y0 F1200\n"
            + "\n".join(f"M106 P0 S{(i % 255) / 255.0:.3f}" for i in range(40))
            + "\nG1 X0 Y0 F1200\nM400\n",
        ),
        (
            "copacabana_flex_f1",
            "config_copacabana.g",
            "G21\nG90\nG92 X0 Y0 Z0\nM666 F1\n"
            "G1 X97.4 Y-2.5 Z-150.6 F30000\nG4 P1000\nG1 X200 Y100 Z-50\nG4 P1000\nG1 X0 Y0 Z0\nM400\n",
        ),
    ],
)
def test_edge_cases_parallel_determinism(name, config, gcode):
    with tempfile.TemporaryDirectory(prefix=f"rrf-det-{name}-", dir="/tmp") as tmp:
        work = Path(tmp)
        shutil.copytree(ROOT / "RRF/run/vsd/sys", work / "sys")
        (work / "gcodes").mkdir()
        (work / "gcodes/test.gcode").write_text(gcode)

        allow_drain = name == "gcode_queue_overflow"
        solo_digest, solo_lines, _ = _run_capture(
            work, "test.gcode", config, "solo", allow_queue_drain=allow_drain
        )
        assert len(solo_lines) > 0

        with ThreadPoolExecutor(max_workers=6) as pool:
            parallel = list(
                pool.map(
                    lambda i: _run_capture(
                        work, "test.gcode", config, f"p{i}", allow_queue_drain=allow_drain
                    ),
                    range(6),
                )
            )

        for idx, (digest, lines, _) in enumerate(parallel):
            assert len(lines) == len(solo_lines), f"{name} run {idx} length mismatch"
            assert digest == solo_digest, f"{name} run {idx} output diverged"
