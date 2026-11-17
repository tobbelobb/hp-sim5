"""Follow-up F3: physical impact of the Klipper MCU player's first-batch step
loss on the shipped slideprinter, evaluated with the firmware's own forward
transform (klippy/chelper `winch_forward_solve`, the function behind
`WinchKinematics.calc_position` / M114).

   .venv/bin/python repro/f3_impact_slideprinter.py [stream.txt]

Step totals for the exact firmware stream and for the player are produced by
repro/f3_player_totals.mjs (the real KlipperMcuCommandPlayer).  The winch model
is configured exactly as public/klipper/slideprinter/printer-slideprinter-linux-mcu.cfg.

Default stream: public/mcu_commands/slideprinter/draw_squares_bigger_no_buildup.txt
(klippy.py -i public/gcode/draw_squares_bigger.gcode with the slideprinter
config; the G-code ends with G1 X0 Y0 Z0, i.e. the commanded end pose is the
origin, which for this machine lies in the plane of the three anchors).
"""
import json
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[4]
sys.path.insert(0, str(ROOT / "klipper" / "klippy"))
import chelper  # noqa: E402

DEFAULT_STREAM = ROOT / "public" / "mcu_commands" / "slideprinter" / "draw_squares_bigger_no_buildup.txt"
MM_PER_STEP = 188.496 / 3200.0
ANCHORS = [
    (0.0, -1900.0, 0.0),
    (1645.448267190434, 950.0, 0.0),
    (-1645.448267190434, 950.0, 0.0),
]
ATTACHMENTS = [(0.0, 0.0, 0.0)] * 3


def player_totals(stream: pathlib.Path, fraction=1.0):
    out = subprocess.run(
        ["node", str(HERE / "f3_player_totals.mjs"), str(stream), str(fraction)],
        capture_output=True, text=True, check=True)
    payload = [ln for ln in out.stdout.splitlines() if ln.startswith("{")]
    return json.loads(payload[-1])


def make_solver():
    ffi_main, ffi_lib = chelper.get_ffi()
    ptr = ffi_main.gc(ffi_lib.winch_flex_alloc(), ffi_lib.winch_flex_free)
    n = len(ANCHORS)
    anchors_c = ffi_main.new("double[]", [c for a in ANCHORS for c in a])
    attach_c = ffi_main.new("double[]", [c for a in ATTACHMENTS for c in a])
    min_c = ffi_main.new("double[]", [2.5] * n)
    max_c = ffi_main.new("double[]", [170.0] * n)
    guy_c = ffi_main.new("double[]", [528.3] * n)
    mech_c = ffi_main.new("int[]", [1] * n)
    lines_c = ffi_main.new("int[]", [1] * n)
    ffi_lib.winch_flex_configure(
        ptr, n, anchors_c, attach_c,
        0.0,            # buildup factor
        0.006,          # mover weight kg
        20000.0,        # spring constant N/m
        min_c, max_c, guy_c,
        1,              # qp algorithm
        True,           # ignore_gravity
        False,          # ignore_pretension
        mech_c, lines_c)
    for i in range(n):
        ffi_lib.winch_flex_set_spool_params(ptr, i, 188.496, 3200)
    assert ffi_lib.winch_flex_set_anchor_mode(ptr, 2)   # all_on_top
    ffi_lib.winch_flex_set_enabled(ptr, 1)
    return ffi_main, ffi_lib, ptr


def forward(ffi_main, ffi_lib, ptr, motor_steps, guess=(0.0, 0.0, 0.0)):
    motor_c = ffi_main.new("double[]", list(motor_steps))
    guess_c = ffi_main.new("double[3]", list(guess))
    out_pos = ffi_main.new("double[3]")
    out_cost = ffi_main.new("double[1]")
    out_iters = ffi_main.new("int[1]")
    ok = ffi_lib.winch_forward_solve(
        ptr, motor_c, guess_c, 1e-3, 1e-3, 3, 30, out_pos, out_cost, out_iters)
    return (ok, [out_pos[i] for i in range(3)], out_cost[0], out_iters[0])


def main():
    stream = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_STREAM
    full = player_totals(stream, 1.0)
    quarter = player_totals(stream, 0.25)
    axes = ["A", "B", "C"]
    exact_steps = [full["exact_steps"][a] for a in axes]
    player_steps = [full["player_steps"][a] for a in axes]
    deficit_steps = [full["deficit_steps"][a] for a in axes]

    ffi_main, ffi_lib, ptr = make_solver()
    ok_e, p_exact, cost_e, it_e = forward(ffi_main, ffi_lib, ptr, exact_steps)
    ok_p, p_player, cost_p, it_p = forward(ffi_main, ffi_lib, ptr, player_steps)
    delta = [p_player[i] - p_exact[i] for i in range(3)]

    # Constancy of the deficit: compare a quarter-stream replay with the full one.
    quarter_deficit = [quarter["deficit_steps"][a] for a in axes]
    deficit_constant = all(abs(quarter_deficit[i] - deficit_steps[i]) < 0.05
                          for i in range(3))

    # Algebraic cross-check regardless of whether the flex solver converges.
    # Linearise |p - a_i| = L_i: u_i . dp + dz^2/(2L_i) = dL_i, with u_i the
    # in-plane unit vector from anchor i to the pose.
    u = [[-(a[0]) / 1900.0, -(a[1]) / 1900.0] for a in ANCHORS]
    # dL = player_length - exact_length = -(deficit) * mm_per_step
    rhs = [-d * MM_PER_STEP for d in deficit_steps]
    # unknowns: dx, dy, w = dz^2 ; u_i . (dx,dy) + w/(2*1900) = rhs_i
    import itertools
    A = [[u[i][0], u[i][1], 1.0 / 3800.0] for i in range(3)]
    b = rhs[:]
    # 3x3 solve by Cramer's rule
    def det3(m):
        return (m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
                - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
                + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]))
    d0 = det3(A)
    sol = []
    for col in range(3):
        M = [row[:] for row in A]
        for r in range(3):
            M[r][col] = b[r]
        sol.append(det3(M) / d0)
    lin = {"dx_mm": round(sol[0], 4), "dy_mm": round(sol[1], 4),
           "dz_mm": round((sol[2] ** 0.5 if sol[2] > 0 else float("nan")), 3)}

    report = {
        "algebraic_linearisation": lin,
        "stream": str(stream.relative_to(ROOT)),
        "klipper_config": "public/klipper/slideprinter/printer-slideprinter-linux-mcu.cfg",
        "forward_transform": "klippy/chelper winch_forward_solve (M114 / calc_position)",
        "exact_steps": dict(zip(axes, exact_steps)),
        "player_steps": {a: round(v, 4) for a, v in zip(axes, player_steps)},
        "deficit_steps": {a: round(v, 4) for a, v in zip(axes, deficit_steps)},
        "deficit_line_mm": {a: round(v * MM_PER_STEP, 4) for a, v in zip(axes, deficit_steps)},
        "solver_ok": {"exact": bool(ok_e), "player": bool(ok_p)},
        "exact_effector_mm": [round(v, 4) for v in p_exact],
        "player_effector_mm": [round(v, 4) for v in p_player],
        "effector_delta_mm": [round(v, 4) for v in delta],
        "effector_delta_norm_mm": round(sum(v * v for v in delta) ** 0.5, 4),
        "deficit_at_25pct_of_stream": {a: round(v, 4) for a, v in zip(axes, quarter_deficit)},
        "loss_is_constant": deficit_constant,
        "solver_cost": {"exact": round(cost_e, 8), "player": round(cost_p, 8)},
        "solver_iters": {"exact": it_e, "player": it_p},
        "flex_solver_converged": bool(ok_e and ok_p),
        "note": ("this pose (home point in the plane of the three anchors) has no "
                 "first-order z sensitivity, so the flex forward solver does not "
                 "converge in z; use the planar physics measurement in "
                 "repro/f3_impact_sim.mjs as the physical number"),
    }
    print(json.dumps(report, indent=2))

    problems = []
    if not deficit_constant:
        problems.append("deficit changed between the first quarter and the whole stream")
    lost = [a for a, d in zip(axes, deficit_steps) if abs(d) > 0.05]
    if not lost:
        problems.append("player replayed every anchor step exactly (no F3 loss on this stream)")
    if ok_e and (abs(p_exact[0]) > 1.0 or abs(p_exact[1]) > 1.0 or abs(p_exact[2]) > 1.0):
        problems.append(f"exact stream does not map back to the commanded X0 Y0 Z0: {p_exact}")
    if problems:
        print("\nUNEXPECTED:", "; ".join(problems), file=sys.stderr)
        sys.exit(2 if "no F3 loss" in problems[0] else 1)

    print("\nF3 IMPACT on the shipped slideprinter (forward transform, flex model on):")
    for a, d in zip(axes, deficit_steps):
        if abs(d) > 0.05:
            print(f"  {a}: player is {d:+.3f} steps ({d * MM_PER_STEP:+.4f} mm of line) from the firmware's step stream")
    print(f"  algebraic in-plane offset: dx={lin['dx_mm']:+.4f} dy={lin['dy_mm']:+.4f} mm "
          f"(z not first-order determined at this pose; rigid second-order |dz| would be {lin['dz_mm']} mm)")
    if ok_e and ok_p:
        print(f"  exact effector pose  : {[round(v, 4) for v in p_exact]} mm (commanded X0 Y0 Z0)")
        print(f"  player effector pose : {[round(v, 4) for v in p_player]} mm")
    else:
        print("  flex forward solver did not converge at this singular pose (see note)")
    print(f"  deficit at 25% of the stream == deficit at 100%: {deficit_constant}")
    sys.exit(1)


if __name__ == "__main__":
    main()
