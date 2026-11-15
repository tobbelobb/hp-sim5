"""Analytical synthetic geometry; no scene input, simulation, or hardware.

Run from repository root with PYTHONPATH=. .venv/bin/python <this-file>.
The geometry is the existing pose-planning unit-test fixture, used only by this
review. Production workspace bounds receive no anchors or scene geometry.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import types

import numpy as np

from autocal._autocal_common import _suggested_collect_command
from autocal.active_learning import SweepConfig, generate_candidate_sweeps
from autocal.equal_strain import equal_strain_tensions, sensor_force_profiles
from autocal.pose_planning import predict_sweep_positions

CANDIDATE = "6ab979c367c286451d05a06b839ca770c51eeab3"
# Freeze the workspace implementation while the parent fixes the shared checkout.
candidate_workspace = subprocess.check_output(
    ["git", "show", f"{CANDIDATE}:autocal/workspace.py"], text=True)
workspace_module = types.ModuleType("review_candidate_workspace")
exec(compile(candidate_workspace, "candidate-6ab979c3-workspace.py", "exec"), workspace_module.__dict__)
exploration_grid = workspace_module.exploration_grid
subprocess.run(["git", "diff", "--quiet", CANDIDATE, "--", "autocal/active_learning.py",
                "autocal/pose_planning.py", "autocal/equal_strain.py",
                "autocal/_autocal_common.py"], check=True)


anchors = np.array([[0., -1900., -280.], [1645.4, 950., -280.],
                    [-1645.4, 950., -280.], [0., 0., 1900.]])
dataset = {"num_anchors": 4, "config": {"max_travel_mm": 390.}, "sweeps": [
    {"fixed_anchors": [low, 3], "fixed_lengths": [390., 0.], "data_points": [{}]}
    for low in range(3)]}

levels = []
for tested_d in (0., -390., -780., -1170.):
    if tested_d:
        dataset["sweeps"].append({"fixed_anchors": [0, 3],
            "fixed_lengths": [390., tested_d], "data_points": [{}]})
    grid, bounds = exploration_grid(dataset, 9)
    candidates = generate_candidate_sweeps(num_anchors=4, dimensions=3,
        machine_type="hangprinter_3", fixed_delta_values_mm=grid,
        fixed_delta_bounds_by_anchor=bounds)
    summaries = []
    for share in (.25, .05, 0.):
        positions = [predict_sweep_positions(anchors, cfg, 20, share) for cfg in candidates]
        positions = [p for p in positions if p.size]
        summaries.append({"minimum_pretension_share": share,
            "accepted_candidates": len(positions),
            "predicted_min_z_mm": min(float(p[:, 2].min()) for p in positions),
            "predicted_max_z_mm": max(float(p[:, 2].max()) for p in positions)})
    levels.append({"completed_d_mm": tested_d, "grid": grid, "bounds_mm": bounds,
        "generated_candidates": len(candidates), "sampling": summaries})

# A candidate rejected by the unchanged .25 ranking heuristic, yet admitted by
# the existing .05 force-profile sampler. Check positive-tension affine balance
# for the retained synthetic profile poses; this is not a servo feasibility test.
cfg = SweepConfig((0, 3), (780., -1560.), 1, 2)
positions = predict_sweep_positions(anchors, cfg, 60, .05)
tensions = np.array([equal_strain_tensions(anchors, p, [0, 3, 1, 2],
    strand_tension_n=1., mover_weight_n=1., strands=np.ones(4),
    extra_line_length_mm=np.zeros(4)) for p in positions])
feasible = np.all(tensions > 0., axis=1)
force_profiles = sensor_force_profiles(anchors, cfg, strand_tension_n=1.,
    mover_weight_n=1., strands=np.ones(4), extra_line_length_mm=np.zeros(4),
    minimum_force_n=.01)
directions = anchors[[0, 3, 1, 2]][None, :, :] - positions[:, None, :]
directions /= np.linalg.norm(directions, axis=2)[:, :, None]
balance_error = np.max(np.abs((directions * tensions[:, :, None]).sum(axis=1) - [0., 0., 1.]))
assert predict_sweep_positions(anchors, cfg, 20, .25).shape[0] == 0
assert feasible.sum() == 44
assert all(len(profile["drive_mm"]) == 44 for profile in force_profiles)
assert balance_error < 1e-12

command = _suggested_collect_command(Path("review-planned.txt"), cfg,
    machine_type="hangprinter_3", output_file=Path("review-output.json"), completed_sweeps=3,
    extra_args=["--completed-sweeps", "90", "--completed-sweeps=90",
                "--machine-type", "skycam", "--sweepFile=wrong.txt",
                "--fixedTargets", "90", "--out", "wrong.json", "--hp-sim-reset",
                "--server", "http://example.invalid"])
assert command.count("--completed-sweeps") == 1
assert command[command.index("--completed-sweeps") + 1] == "3"
assert not any("90" in value or value in {"skycam", "wrong.json", "--hp-sim-reset"}
               for value in command)

files = ["autocal/workspace.py", "autocal/active_learning.py", "autocal/pose_planning.py",
         "autocal/equal_strain.py", "autocal/_autocal_common.py"]
print(json.dumps({"evidence_type": "synthetic ideal rigid geometry; no simulator or hardware",
    "candidate_revision": CANDIDATE,
    "anchors_mm": anchors.tolist(), "candidate_grid_count": 9,
    "source_sha256": {name: hashlib.sha256(candidate_workspace.encode() if name == "autocal/workspace.py"
                                           else Path(name).read_bytes()).hexdigest() for name in files},
    "frontend_owned_fields_control": {"attempted_completed_sweeps": 90,
        "planned_completed_sweeps": 3, "effective_command": command},
    "levels": levels,
    "conservative_sampling_counterexample": {
        "fixed_anchors": [0, 3], "fixed_deltas_mm": [780., -1560.],
        "ranking_pose_count": 0, "profile_input_pose_count": len(positions),
        "positive_tension_profile_pose_count": int(feasible.sum()),
        "positive_tension_pose_z_range_mm": [float(positions[feasible, 2].min()),
                                             float(positions[feasible, 2].max())],
        "positive_tension_range_n": [float(tensions[feasible].min()), float(tensions[feasible].max())],
        "assumed_mover_weight_n": 1., "assumed_target_strand_tension_n": 1.,
        "max_force_balance_error_n": float(balance_error),
    }}, indent=2))
