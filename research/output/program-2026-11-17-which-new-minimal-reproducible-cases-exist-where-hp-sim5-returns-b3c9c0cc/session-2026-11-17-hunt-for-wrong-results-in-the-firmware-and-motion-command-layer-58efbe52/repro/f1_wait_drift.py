"""Follow-up F1: does the standstill off-by-one accumulate over many waits?

Cartesian config (80 whole steps/mm, so 1 mm = 80 steps exactly) unless noted.

  A  alternating X0/X1 moves, each followed by M400            (should stay exact)
  B  monotone 1 mm staircase, each move followed by M400       (x1..x100)
  C  monotone 1 mm staircase, each move followed by G4 P1
  D  alternating 10 mm round trips, each move followed by M400
  E  the failing unit (G1 X10; G1 X0; M400; G1 X1; M400) x n   -> does the
     1-step loss compound?
  F  slideprinter: G1 X10; M400; G1 X0; M400 repeated n times -> per-motor net
     steps (the one case that showed a non-zero net in the first pass)

Reports net captured steps, per-move step counts, final X, and (for F) the net
per motor, to distinguish a bounded constant offset from accumulation.
"""
import pathlib
import re
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from rrf_harness import run_rrf, step_totals, CARTESIAN_CONFIG  # noqa: E402

BASE = pathlib.Path("/tmp/opencode/rrf_f1_drift2")
SLIDE = pathlib.Path(
    "/home/torbjorn/repos/hp-sim5/RRF/run/vsd/sys/config_slideprinter.g").read_text()


def run(gcode, tag, config_text=CARTESIAN_CONFIG, config_name="config_probe.g"):
    return run_rrf(gcode, BASE / tag, config_text=config_text, config_name=config_name)


def per_move_steps(res):
    return [row["steps"][0] for row in res["rows"] if row["num_drives"] == 1]


def m114_x(stdout):
    for line in stdout.splitlines():
        if line.startswith("X:"):
            return float(line.split()[0].split(":")[1]), line
    return None, None


problems = []
print("== A: alternating X0/X1, M400 after every move (exact 1 mm moves)")
for n in (10, 100):
    targets = [1 if k % 2 == 1 else 0 for k in range(1, n + 1)]
    body = "".join(f"G1 X{t} F600\nM400\n" for t in targets)
    res = run("G92 X0 Y0 Z0\n" + body + "M114\n", f"A{n}")
    moves = per_move_steps(res)
    final_x, _ = m114_x(res["stdout"])
    expected = targets[-1] * 80
    print(f"   n={n:3d} moves={len(moves)} net={sum(moves):6.1f} expected={expected:5.1f} "
          f"final_X={final_x} per-move={sorted(set(moves))}")
    if sum(moves) != expected or final_x != expected / 80:
        problems.append(f"A n={n}: net {sum(moves)}, final {final_x}")

print("== B/C: monotone 1 mm staircase with a wait after every move")
for tag, wait in (("B", "M400"), ("C", "G4 P1")):
    for n in (10, 100):
        body = "".join(f"G1 X{k} F600\n{wait}\n" for k in range(1, n + 1))
        res = run("G92 X0 Y0 Z0\n" + body + "M114\n", f"{tag}{n}")
        moves = per_move_steps(res)
        final_x, _ = m114_x(res["stdout"])
        print(f"   {tag} n={n:3d} wait={wait:5s} net={sum(moves):6.1f} expected={n * 80:5d} "
              f"final_X={final_x} per-move={sorted(set(moves))}")
        if sum(moves) != n * 80 or final_x != n:
            problems.append(f"{tag} n={n}: net {sum(moves)}, final {final_x}")

print("== D: alternating 10 mm round trips with M400 after every move")
for n in (10, 100):
    body = "".join("G1 X10 F600\nM400\nG1 X0 F600\nM400\n" for _ in range(n))
    res = run("G92 X0 Y0 Z0\n" + body + "M114\n", f"D{n}")
    moves = per_move_steps(res)
    final_x, _ = m114_x(res["stdout"])
    print(f"   n={n:3d} moves={len(moves)} net={sum(moves):6.1f} expected=0 final_X={final_x} "
          f"per-move={sorted(set(moves))}")
    if sum(moves) != 0 or final_x != 0:
        problems.append(f"D n={n}: net {sum(moves)}, final {final_x}")

print("== E: the failing unit repeated (G1 X10 / G1 X0 / M400 / G1 X1 / M400)")
for n in (1, 2, 5, 20, 100):
    gcode = "G92 X0 Y0 Z0\n" + "G1 X10 F600\nG1 X0 F600\nM400\nG1 X1 F600\nM400\n" * n + "M114\n"
    res = run(gcode, f"E{n}")
    moves = per_move_steps(res)
    final_x, _ = m114_x(res["stdout"])
    print(f"   n={n:3d} moves={len(moves)} net={sum(moves):6.1f} final_X={final_x} "
          f"per-move={sorted(set(moves))}")
    # every unit should end at X=1 mm; anything else is a real position error
    if final_x != 1.0:
        problems.append(f"E n={n}: final X {final_x} instead of 1.0 mm")

print("== F: slideprinter G1 X10 / M400 / G1 X0 / M400 repeated n times")
for n in (1, 2, 5, 20, 100):
    gcode = "G92 X0 Y0 Z0\n" + "G1 X10 F600\nM400\nG1 X0 F600\nM400\n" * n + "M114\n"
    res = run(gcode, f"F{n}", SLIDE, "config_slideprinter.g")
    net = step_totals(res["rows"])
    print(f"   n={n:3d} net_steps="
          + ", ".join(f"{k}:{v:+.0f}" for k, v in sorted(net.items()))
          + f"  X={res['final_position']['X']:.4f}")
    if any(abs(v) > 1 for v in net.values()):
        problems.append(f"F n={n}: |net| > 1 step: {net}")

if problems:
    print("\nF1 DRIFT/EFFECT PRESENT", file=sys.stderr)
    for p in problems:
        print("  - " + p, file=sys.stderr)
    sys.exit(1)
print("\nOK: all sequences exact after 100 waits")
