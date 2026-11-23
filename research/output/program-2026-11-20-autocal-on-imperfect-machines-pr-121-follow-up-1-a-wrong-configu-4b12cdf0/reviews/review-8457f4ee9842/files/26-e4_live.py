"""E4: live headless autocal on a scene with moved low anchors.

Usage: e4_live.py NAME [A=dz] [Bx=dx] [By=dy] ... [seed=N] [machine=hangprinter_5]   (mm; + is up)
Moves the frame-side prims of a tower (Spool<L>, Anchor<L>{L,R}[_top|_bottom]) in the
machine's catalog scene. The firmware config keeps its nominal anchors, as a user who
assumed a regular room would. `seed=N` adds a seeded uniform +-20 mm xy move to every low
anchor except A, which only moves in y, so distinct equal-height machines serve as seeds.
Truth = benchmark truth plus the moves, expressed in the convention frame (top above home,
A.x = 0). Scores the applied (firmware final) anchors, an offline refit, the jackknife and
the console quality label.
"""
import json, re, subprocess, sys, os
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from lowz_common import REPO, TRUTH, fit_default, rotation_aligned, anchor_error

SCENES = {"hangprinter_3": "hp3.usda", "hangprinter_4": "hp4.usda", "hangprinter_5": "hp5.usda",
          "hangprinter_5_pulley_reduced": "hp5_pulley_reduced.usda"}
name, args = sys.argv[1], dict(s.split("=") for s in sys.argv[2:])
machine = args.pop("machine", "hangprinter_3")
seed = args.pop("seed", None)
letters = "ABCD" if machine.startswith("hangprinter_5") else "ABC"
moves = {letter: np.zeros(3) for letter in letters}
if seed is not None:
    rng = np.random.default_rng(int(seed))
    for letter in letters:
        moves[letter][:2] += rng.uniform(-20.0, 20.0, 2) * ([0.0, 1.0] if letter == "A" else [1.0, 1.0])
for key, value in args.items():
    moves[key[0]][{"": 2, "x": 0, "y": 1, "z": 2}[key[1:]]] += float(value)
out = Path(__file__).resolve().parents[1] / "runs" / name
out.mkdir(parents=True, exist_ok=False)
scene = (REPO / "public/usd_scenes" / SCENES[machine]).read_text()
for letter, move in moves.items():
    if not move.any():
        continue
    prims = re.findall(r'def \w+ "((?:Spool%s|Anchor%s[LR](?:_top|_bottom)?))"' % (letter, letter), scene)
    assert prims, letter
    for prim in prims:
        pattern = re.compile(r'(def \w+ "%s".*?xformOp:translate = )\(([^,]+), ([^,]+), ([^)]+)\)' % prim, re.S)
        m = pattern.search(scene)
        xyz = np.array([float(m.group(i)) for i in (2, 3, 4)]) + move / 1000.0
        scene = scene[:m.start()] + m.group(1) + "(%r, %r, %r)" % tuple(float(v) for v in xyz) + scene[m.end():]
(out / "input-scene.usda").write_text(scene)
import autocal.theoretical_ellipse as te
truth = TRUTH[machine].copy()
for letter, move in moves.items():
    truth[letters.index(letter)] += move
truth = te.canonicalize_anchor_gauge(machine, truth)
command = [sys.executable, "-m", "autocal.autocal", "--headless-sim", "--machine-type", machine,
           "--dataset", str(out / "sweeps.json"), "--recording-mode", "none"]
(out / "input.json").write_text(json.dumps(dict(command=command, seed=seed, moves_mm={k: v.tolist() for k, v in moves.items()},
                                                truth=truth.tolist()), indent=1))
env = dict(os.environ, HP_SIM5_AUTOCAL_SCENE=str(out / "input-scene.usda"), PYTHONUNBUFFERED="1")
with (out / "stdout.log").open("w") as log:
    code = subprocess.run(command, cwd=REPO, env=env, stdout=log, stderr=subprocess.STDOUT).returncode
manifest = json.loads((out / "sweeps.headless" / "manifest.json").read_text())
log_text = (out / "stdout.log").read_text() + ((out / "sweeps.full_auto.log").read_text() if (out / "sweeps.full_auto.log").exists() else "")
labels = re.findall(r"Found parameters of (\w+) quality", log_text)
console_uncertainty = re.findall(r"Anchor uncertainty \(mm, lower is better\): ([\d.]+|inf|nan)", log_text)
summary = dict(name=name, machine=machine, seed=seed, moves_mm={k: v.tolist() for k, v in moves.items()},
               shifts_mm={k: float(v[2]) for k, v in moves.items() if v[2]}, exit_code=code,
               stop_reason=manifest.get("stop_reason"), simulated_s=manifest.get("simulated_s"),
               wall_s=manifest.get("wall_s"), quality_label=labels[-1] if labels else None,
               console_uncertainty_mm=float(console_uncertainty[-1]) if console_uncertainty else None)
if "firmware_final" in manifest:
    m669 = manifest["firmware_final"]["M669"]
    names = "ABCD" if len(truth) == 4 else "ABCDI"
    applied = np.array([[float(v) for v in re.search(r"%s:([-\d.]+), ([-\d.]+), ([-\d.]+)" % L, m669).groups()] for L in names])
    data = json.loads((out / "sweeps.json").read_text())
    fit, _ = fit_default(data)
    from autocal.anchor_uncertainty import sweep_jackknife_mm
    summary.update(sweeps=len(data["sweeps"]), applied=applied.tolist(), applied_err=anchor_error(applied, truth),
                   applied_err_rot=anchor_error(rotation_aligned(applied, truth), truth),
                   refit=fit["anchors"].tolist(), refit_err=anchor_error(fit["anchors"], truth),
                   refit_dz_D=float(fit["anchors"][-1, 2] - truth[-1, 2]),
                   refit_low_z_err=(fit["anchors"][:-1, 2] - truth[:-1, 2]).round(3).tolist(),
                   rms_mm=fit["rms_mm"], robust_scale_mm=fit["robust_scale_mm"], rejected=len(fit["rejected_points"]),
                   jackknife_mm=sweep_jackknife_mm(data, fit["anchors"]))
(out / "summary.json").write_text(json.dumps(summary, indent=1))
print(json.dumps(summary))
