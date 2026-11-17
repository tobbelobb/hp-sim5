"""Negative-result check: an M204 acceleration change issued while moves are
already queued applies to the following move (and not retroactively in a way
that breaks the queued plan).  RRF batch mode queues the whole file, so this is
the "command arrives mid-motion" case from the planner's point of view.

Closed form on the cartesian config (80 steps/mm): all three moves cruise at
F600 = 10 mm/s and are blended, so only the ramps that actually happen cost
time:
  move 1  0 -> 10 mm/s            t = 10/10 + 10/(2*3000) = 1.0016667 s
  move 2  10 -> 10 mm/s           t = 10/10              = 1.0000000 s
  move 3  10 -> 0, after M204 300 t = 10/10 + 10/(2*300)  = 1.0166667 s
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from rrf_harness import run_rrf, STEP_CLOCK_HZ, CARTESIAN_CONFIG  # noqa: E402

BASE = pathlib.Path("/tmp/opencode/rrf_midmotion")
gcode = ("G92 X0 Y0 Z0\n"
         "G1 X10 F600\n"
         "G1 X20 F600\n"
         "M204 P300 T300\n"
         "G1 X30 F600\n")
res = run_rrf(gcode, BASE, config_text=CARTESIAN_CONFIG, config_name="config_probe.g")

durations = [
    (row["accel_clocks"] + row["steady_clocks"] + row["decel_clocks"]) / STEP_CLOCK_HZ
    for row in res["rows"] if row["num_drives"] == 1 and row["steps"][0] != 0
]
print("per-segment durations (s):", durations)
print("final position:", res["final_position"])

problems = []
if len(durations) < 3:
    problems.append(f"expected 3 motion segments, got {len(durations)}")
else:
    # third move decelerates from cruise with the NEW acceleration
    if abs(durations[-1] - (10 / 10 + 10 / (2 * 300))) > 5e-4:
        problems.append(
            f"third move took {durations[-1]:.6f}s, closed form at a=300 is "
            f"{10 / 10 + 10 / (2 * 300):.6f}s")
    # first move accelerates from rest with the OLD acceleration
    if abs(durations[0] - (10 / 10 + 10 / (2 * 3000))) > 5e-4:
        problems.append(
            f"first move took {durations[0]:.6f}s, closed form at a=3000 is "
            f"{10 / 10 + 10 / (2 * 3000):.6f}s")
    # second move starts and ends at speed: exactly 10 mm at 10 mm/s
    if abs(durations[1] - 10 / 10) > 5e-3:
        problems.append(
            f"blended middle move took {durations[1]:.6f}s, closed form (no ramp) "
            f"is {10 / 10:.6f}s")

if problems:
    print("\nFAIL", file=sys.stderr)
    for p in problems:
        print("  - " + p, file=sys.stderr)
    sys.exit(1)
print("OK: mid-queue M204 change applies to the following move only, queued moves keep their plan")
