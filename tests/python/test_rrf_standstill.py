import csv
import re
import subprocess
from pathlib import Path

import pytest


SIMULATOR = Path(__file__).resolve().parents[2] / "RRF/build/rrf_simulator"
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


@pytest.mark.slow
@pytest.mark.parametrize(
    ("moves", "expected_steps", "expected_x"),
    [
        ("G1 X0.05 F600\nM400", [4], 0.05),
        ("G1 X-0.05 F600\nM400", [-4], -0.05),
        ("G1 X10 F600\nG1 X0\nM400\nG1 X1\nM400", [800, -800, 80], 1.0),
    ],
)
def test_cartesian_standstill_matches_can_steps(tmp_path, moves, expected_steps, expected_x):
    (tmp_path / "sys").mkdir()
    (tmp_path / "sys/config.g").write_text(CARTESIAN_CONFIG)
    (tmp_path / "gcodes").mkdir()
    gcode = tmp_path / "gcodes/standstill.gcode"
    gcode.write_text(f"G92 X0 Y0 Z0\n{moves}\nM114\n")
    capture = tmp_path / "can.csv"

    result = subprocess.run(
        [str(SIMULATOR), "--vsd", str(tmp_path), "-c", "sys/config.g",
         "--gcode", str(gcode), "--can-log", str(capture)],
        capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Error:" not in result.stdout, result.stdout
    position = re.search(r"^X:([-\d.]+).* Count (-?\d+) ", result.stdout, re.M)
    assert position is not None, result.stdout

    with capture.open() as stream:
        next(stream)  # JSON metadata
        steps = [int(row[6]) for row in csv.reader(stream) if row[1] == "40"]

    assert (steps, int(position[2]), float(position[1])) == (
        expected_steps, sum(expected_steps), expected_x,
    )
