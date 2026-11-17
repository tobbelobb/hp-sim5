"""Repro: the RRF host build corrupts the live motor position at standstill by
one microstep, and that corrupted position becomes the commanded position for
the next move.

Both are visible from outside the firmware, using only the real batch-mode
binary (RRF/build/rrf_simulator) and its own G-code replies:

  case A  slideprinter config, exact return to the start with a standstill:
          G92 X0 Y0 Z0 / G1 X10 F600 / M400 / G1 X0 F600 / M400
          The CAN capture contains +147/-147 steps on motors b/c and then
          -147/+147, i.e. the net step count must be 0/0.  Observed 0/-1/+1.

  case B  cartesian config (80 whole steps per mm), 1 mm commanded after a
          standstill: G1 X10 / G1 X0 / M400 / G1 X1.
          The last move must be exactly 80 steps.  Observed 79.
          Commanded target 1.000 mm, reported final X 0.9875 mm.

  case C  single 0.05 mm move = exactly 4 steps at 80 steps/mm, but M114's
          motor step counter says 3 while the CAN capture says 4.

Firmware code: src/Movement/Move.cpp `Move::FreeOldSegments` (RRF_HOST_BUILD
only, fork commit 7eb7a97bf) advances
    netSteps = (int32_t)(length + distanceCarriedForwards)      // truncation
whereas the real per-step ISR path `DriveMovement::TakeStepsAndCalcStepTimeRarely`
uses
    currentMotorPosition = ... + std::lrint(u*t + 0.5*a*t*t + carry)  // rounding
So after an acceleration/deceleration segment the live motor position can be
one step low/high.  `GCodes::LockMovementSystemAndWaitForStandstill` then does
`UpdateOwnedDriveEndpointsFromMotors()`, folding that offset into the user
position, and the next absolute move is computed from it.

Exit status: 1 when the bug is present (with a one-line reason per failed
invariant), 0 when the firmware behaves exactly.
"""
import pathlib
import re
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from rrf_harness import run_rrf, step_totals, CARTESIAN_CONFIG  # noqa: E402

SLIDE_CONFIG = pathlib.Path(
    "/home/torbjorn/repos/hp-sim5/RRF/run/vsd/sys/config_slideprinter.g").read_text()
BASE = pathlib.Path("/tmp/opencode/rrf_standstill_repro")


def m114_count(stdout):
    for line in stdout.splitlines():
        match = re.search(r"Count (-?\d+) (-?\d+) (-?\d+)", line)
        if match:
            return tuple(int(v) for v in match.groups())
    return None


def m114_user(stdout):
    for line in stdout.splitlines():
        if line.startswith("X:"):
            return line
    return None


def run(gcode, tag, config_text, config_name):
    return run_rrf(gcode, BASE / tag, config_text=config_text,
                   config_name=config_name)


problems = []
evidence = {}

# ---------------------------------------------------------------- case A
res = run("G92 X0 Y0 Z0\nG1 X10 F600\nM400\nG1 X0 F600\nM400\nM114\n",
          "a_slide_roundtrip", SLIDE_CONFIG, "config_slideprinter.g")
net = step_totals(res["rows"])
net_b = net.get(41, 0.0)
net_c = net.get(42, 0.0)
evidence["A_slideprinter_roundtrip_net_steps"] = {
    "motor_a_addr40": net.get(40, 0.0), "motor_b_addr41": net_b,
    "motor_c_addr42": net_c,
    "m114": m114_user(res["stdout"]),
}
if net_b != 0 or net_c != 0:
    problems.append(
        "A: after an exact return to the commanded start point with a standstill, "
        f"the motors end at {net_b:+.0f}/{net_c:+.0f} steps instead of 0/0 "
        f"(1 step = {188.496 / 3200:.4f} mm of line); M114 reports {m114_user(res['stdout'])}")

# ---------------------------------------------------------------- case B
res = run("G92 X0 Y0 Z0\nG1 X10 F600\nG1 X0 F600\nM400\nG1 X1 F600\nM400\nM114\n",
          "b_cart_after_standstill", CARTESIAN_CONFIG, "config_probe.g")
runs = [row["steps"][0] for row in res["rows"] if row["num_drives"] == 1]
last_move_steps = runs[-1] if runs else 0
final_x = res["final_position"]["X"]
evidence["B_cartesian_1mm_after_standstill"] = {
    "segment_steps": runs,
    "last_move_steps": last_move_steps,
    "expected_last_move_steps": 80,
    "reported_final_x_mm": final_x,
    "expected_final_x_mm": 1.0,
    "m114": m114_user(res["stdout"]),
}
if last_move_steps != 80 or abs(final_x - 1.0) > 1e-9:
    problems.append(
        f"B: the 1 mm move commanded after a standstill emitted {last_move_steps:+.0f} steps "
        f"instead of +80 (0.9875 mm instead of 1.0000 mm; error 0.0125 mm = 1 step)")

# control for case B: the same moves without the M400
res_ctl = run("G92 X0 Y0 Z0\nG1 X10 F600\nG1 X0 F600\nG1 X1 F600\nM400\nM114\n",
              "b_control_no_standstill", CARTESIAN_CONFIG, "config_probe.g")
runs_ctl = [row["steps"][0] for row in res_ctl["rows"] if row["num_drives"] == 1]
evidence["B_control_without_M400"] = {
    "segment_steps": runs_ctl,
    "last_move_steps": runs_ctl[-1] if runs_ctl else None,
    "reported_final_x_mm": res_ctl["final_position"]["X"],
}
if not runs_ctl or runs_ctl[-1] != 80:
    problems.append(
        "B-control: without the M400 the same 1 mm move emitted "
        f"{runs_ctl[-1] if runs_ctl else None} steps, expected 80")

# ---------------------------------------------------------------- case C
res = run("G92 X0 Y0 Z0\nG1 X0.05 F600\nM400\nM114\n",
          "c_cart_005mm", CARTESIAN_CONFIG, "config_probe.g")
captured = [row["steps"][0] for row in res["rows"] if row["num_drives"] == 1]
count = m114_count(res["stdout"])
evidence["C_cartesian_0p05mm"] = {
    "captured_steps": captured, "m114_count": count,
    "expected_count": 4,
}
if captured and count and count[0] != captured[0]:
    problems.append(
        f"C: a 0.05 mm move = exactly 4 steps, the CAN stream says {captured[0]:.0f} steps "
        f"but the firmware's own live motor counter says {count[0]} steps")

evidence["determinism"] = {}
res2 = run("G92 X0 Y0 Z0\nG1 X10 F600\nM400\nG1 X0 F600\nM400\nM114\n",
           "a_slide_roundtrip_2", SLIDE_CONFIG, "config_slideprinter.g")
net2 = step_totals(res2["rows"])
evidence["determinism"]["repeated_run_net_steps"] = {
    "motor_b_addr41": net2.get(41), "motor_c_addr42": net2.get(42)}
if (net2.get(41), net2.get(42)) != (net_b, net_c):
    problems.append("determinism: repeated identical run gave different step totals")

import json  # noqa: E402
print(json.dumps(evidence, indent=2))

if problems:
    print("\nBUG REPRODUCED: RRF host-build standstill bookkeeping is off by one microstep", file=sys.stderr)
    for problem in problems:
        print("  - " + problem, file=sys.stderr)
    sys.exit(1)
print("OK: standstill position bookkeeping is exact")
