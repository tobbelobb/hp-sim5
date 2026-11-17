"""Negative-result check: RRF's move queue does not drop, duplicate or reorder
tiny moves, even far beyond the DDA ring length.

20000 absolute moves of one microstep (0.0125 mm at 80 steps/mm) must produce
exactly 20000 steps and end at 250.000 mm.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from rrf_harness import run_rrf, step_totals, CARTESIAN_CONFIG  # noqa: E402

BASE = pathlib.Path("/tmp/opencode/rrf_queue_stress")
count = 20000
gcode = "G92 X0 Y0 Z0\n" + "".join(
    f"G1 X{k * 0.0125:.4f} F600\n" for k in range(1, count + 1))
res = run_rrf(gcode, BASE, config_text=CARTESIAN_CONFIG, config_name="config_probe.g")
steps = step_totals(res["rows"]).get(40, 0)
final_x = res["final_position"]["X"]
print(f"moves={count} captured_steps={steps} final_X={final_x} segments={len(res['rows'])}")
if steps != count or abs(final_x - count * 0.0125) > 1e-6:
    print(f"\nFAIL: expected {count} steps and X={count * 0.0125} mm", file=sys.stderr)
    sys.exit(1)
print("OK: no steps lost or duplicated across 20000 queued moves")
