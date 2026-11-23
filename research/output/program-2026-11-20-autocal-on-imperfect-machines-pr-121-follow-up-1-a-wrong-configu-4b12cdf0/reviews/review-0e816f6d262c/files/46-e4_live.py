"""E4: live headless HP3 autocal on a scene whose low anchors sit at uneven heights.

Usage: e4_live_hp3.py NAME A=10 [B=-5 ...] [machine=hangprinter_5]   (mm, + is up)
Shifts SpoolX, AnchorXL, AnchorXR of hp3.usda by dz; the firmware config still states
equal heights (-280 mm), as a user who assumed a level room would. Truth = benchmark
truth with the same dz. Scores the applied (firmware final) anchors and an offline refit.
"""
import json, re, subprocess, sys, os
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from lowz_common import REPO, TRUTH, fit_default, rotation_aligned, anchor_error, level_frame

name, shifts = sys.argv[1], dict(s.split("=") for s in sys.argv[2:])
machine = shifts.pop("machine", "hangprinter_3")
shifts = {k: float(v) for k, v in shifts.items()}
scene_file = {"hangprinter_3": "hp3.usda", "hangprinter_5": "hp5.usda"}[machine]
out = Path(__file__).resolve().parents[1] / "runs" / name
out.mkdir(parents=True, exist_ok=False)
scene = (REPO / "public/usd_scenes" / scene_file).read_text()
for letter, dz in shifts.items():
    for prim in (f"Spool{letter}", f"Anchor{letter}L", f"Anchor{letter}R"):
        pattern = re.compile(r'(def \w+ "%s".*?xformOp:translate = \(([^,]+), ([^,]+), ([^)]+)\))' % prim, re.S)
        m = pattern.search(scene)
        z = float(m.group(4)) + dz / 1000.0
        scene = scene[:m.start()] + m.group(1)[:m.group(1).rfind("(")] + f"({m.group(2)}, {m.group(3)}, {z!r})" + scene[m.end():]
(out / "input-scene.usda").write_text(scene)
truth = TRUTH[machine].copy()
for letter, dz in shifts.items():
    truth["ABCD".index(letter), 2] += dz
command = [sys.executable, "-m", "autocal.autocal", "--headless-sim", "--machine-type", machine,
           "--dataset", str(out / "sweeps.json"), "--recording-mode", "none"]
(out / "input.json").write_text(json.dumps(dict(command=command, shifts_mm=shifts, truth=truth.tolist()), indent=1))
env = dict(os.environ, HP_SIM5_AUTOCAL_SCENE=str(out / "input-scene.usda"), PYTHONUNBUFFERED="1")
with (out / "stdout.log").open("w") as log:
    code = subprocess.run(command, cwd=REPO, env=env, stdout=log, stderr=subprocess.STDOUT).returncode
manifest = json.loads((out / "sweeps.headless" / "manifest.json").read_text())
m669 = manifest["firmware_final"]["M669"]
applied = np.array([[float(v) for v in re.search(r"%s:([-\d.]+), ([-\d.]+), ([-\d.]+)" % L, m669).groups()] for L in ("ABCD" if machine == "hangprinter_3" else "ABCDI")])
data = json.loads((out / "sweeps.json").read_text())
fit, shifted = fit_default(data)
from autocal.anchor_uncertainty import sweep_jackknife_mm
summary = dict(name=name, machine=machine, shifts_mm=shifts, exit_code=code, stop_reason=manifest.get("stop_reason"),
               simulated_s=manifest.get("simulated_s"), wall_s=manifest.get("wall_s"), sweeps=len(data["sweeps"]),
               applied=applied.tolist(), applied_err=anchor_error(applied, truth),
               applied_err_rot=anchor_error(rotation_aligned(applied, truth), truth),
               applied_err_vs_level=anchor_error(applied, level_frame(truth)) if machine == "hangprinter_3" else None,
               refit=fit["anchors"].tolist(), refit_err=anchor_error(fit["anchors"], truth),
               refit_err_rot=anchor_error(rotation_aligned(fit["anchors"], truth), truth),
               refit_dz_D=float(fit["anchors"][-1, 2] - truth[-1, 2]),
               rms_mm=fit["rms_mm"], robust_scale_mm=fit["robust_scale_mm"], rejected=len(fit["rejected_points"]),
               jackknife_mm=sweep_jackknife_mm(data, fit["anchors"]))
(out / "summary.json").write_text(json.dumps(summary, indent=1))
print(json.dumps(summary))
