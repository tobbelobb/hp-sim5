"""Regenerate repro/fixtures/rrf_two_drivers_one_board.{csv,g} and .gcode from
the real firmware.  This is the only fixture-generation script that needs the
simulator; the other fixtures are checked in and their klippy command lines are
documented in the header of the repro that uses them.
"""
import pathlib
import shutil
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from rrf_harness import run_rrf  # noqa: E402

HERE = pathlib.Path(__file__).parent
FIX = HERE / "fixtures"
FIX.mkdir(exist_ok=True)

CONFIG = """\
; Four motors on one CAN board (address 40, drivers 0..3).
G21
G90
M83
M584 X40.0 Y40.1 Z40.2 E40.3 P3
M669 K0
M92 X80 Y80 Z400 E95.922
M201 X3000 Y3000 Z3000 E3000
M203 X18000 Y18000 Z18000 E18000
M204 P3000 T3000
M566 X300 Y300 Z300 E1200
M569 P40.0 S1
M569 P40.1 S1
M569 P40.2 S1
M569 P40.3 S1
M564 H0 S0
"""
GCODE = "G92 X0 Y0 Z0\nG1 X10 Y5 F600\n"

res = run_rrf(GCODE, pathlib.Path("/tmp/opencode/rrf_multidriver_fixture"),
              config_text=CONFIG, config_name="config_probe.g")
assert res["returncode"] == 0, res["errors"]
assert res["final_position"] == {"X": 10.0, "Y": 5.0, "Z": 0.0}, res["final_position"]
shutil.copy(pathlib.Path("/tmp/opencode/rrf_multidriver_fixture/capture.csv"),
            FIX / "rrf_two_drivers_one_board.csv")
(FIX / "rrf_two_drivers_one_board_config.g").write_text(CONFIG)
(FIX / "rrf_two_drivers_one_board.gcode").write_text(GCODE)
print((FIX / "rrf_two_drivers_one_board.csv").read_text())
