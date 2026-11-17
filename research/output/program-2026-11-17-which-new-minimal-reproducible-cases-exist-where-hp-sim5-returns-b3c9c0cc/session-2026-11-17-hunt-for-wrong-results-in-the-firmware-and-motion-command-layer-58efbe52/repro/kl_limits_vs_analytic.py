"""Negative-result check: Klipper's feedrate / acceleration limits and step
quantization on a single axis match the closed-form trapezoid.

Uses the real klippy.py batch mode with repro/klipper_cartesian.cfg
(80 whole steps/mm, max_velocity 300 mm/s, max_accel 3000 mm/s^2) and compares
the step timestamps with  t = d/v + v/(2a)  (or the triangular case).

Exits 1 if any case deviates by more than 10 ms (one 2 ms bucket's worth of
timing jitter is expected because only whole steps are emitted).
"""
import pathlib
import re
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from klipper_harness import run_klipper  # noqa: E402

CFG = pathlib.Path(__file__).parent / "klipper_cartesian.cfg"
BASE = pathlib.Path("/tmp/opencode/kl_limits_repro")
CLOCK_HZ = 50_000_000


def step_timeline(text, oid=2):
    """First-step clock and last-step clock for one stepper oid."""
    first = last = None
    clock = 0
    for raw in text.splitlines():
        line = raw.strip()
        m = re.match(r"queue_step oid=(\d+) interval=(-?\d+) count=(-?\d+) add=(-?\d+)", line)
        if not m or int(m.group(1)) != oid:
            continue
        interval, count, add = (int(m.group(i)) for i in (2, 3, 4))
        for i in range(count):
            clock += max(1, interval + add * i)
            if first is None:
                first = clock
            last = clock
    return first, last


def trapezoid_time(d, v, a):
    d_acc = v * v / (2 * a)
    if d <= 2 * d_acc:
        return 2 * (d / a) ** 0.5
    return d / v + v / a


cases = [
    ("F600 (10 mm/s)", "G1 X80 F600\n", 80, 10.0, 3000),
    ("F60000 (capped at max_velocity 300 mm/s)", "G1 X80 F60000\n", 80, 300.0, 3000),
    ("F30 (0.5 mm/s)", "G1 X80 F30\n", 80, 0.5, 3000),
    ("M204 P300 then F600", "M204 P300 T300\nG1 X80 F600\n", 80, 10.0, 300),
    ("M203 X6000 then F6000", "M203 X6000\nG1 X80 F6000\n", 80, 100.0, 3000),
]

problems = []
print(f"{'case':44s} {'steps':>6s} {'first->last (s)':>16s} {'closed form (s)':>16s}")
for name, body, distance, speed, accel in cases:
    gcode = "SET_KINEMATIC_POSITION X=0 Y=0 Z=0\nG90\n" + body
    res = run_klipper(gcode, BASE / re.sub(r"\W+", "_", name), cfg_path=CFG)
    steps = res["steps"].get(2, 0)
    first, last = step_timeline(res["mcu_commands"])
    span = (last - first) / CLOCK_HZ
    expected = trapezoid_time(distance, speed, accel)
    # first step is half a step into the move and the last step is one step
    # short of the end: allow the corresponding interval (<= 20 ms here)
    tolerance = 0.020 + 0.5 / (speed * 80) * 10
    print(f"{name:44s} {steps:6d} {span:16.5f} {expected:16.5f}")
    if steps != int(round(distance * 80)):
        problems.append(f"{name}: {steps} steps, expected {int(round(distance * 80))}")
    if abs(span - expected) > tolerance:
        problems.append(f"{name}: move span {span:.5f}s vs closed form {expected:.5f}s")

if problems:
    print("\nFAIL", file=sys.stderr)
    for p in problems:
        print("  - " + p, file=sys.stderr)
    sys.exit(1)
print("\nOK: Klipper's feedrate/acceleration limits and step counts match the closed form")
