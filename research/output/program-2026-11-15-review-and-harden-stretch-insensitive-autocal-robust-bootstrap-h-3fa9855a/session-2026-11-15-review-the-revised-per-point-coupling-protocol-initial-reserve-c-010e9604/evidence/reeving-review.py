"""Frozen 8338be8f analytical checks; no simulator, service, or hardware.

Run from repository root with PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python <this file>.
"""
from __future__ import annotations

import contextlib
import copy
import hashlib
import io
import json
from pathlib import Path
import subprocess
import types
from unittest.mock import patch

import numpy as np

REVISION = "8338be8f"
ROOT = Path.cwd()
DIRECTORY = Path(__file__).resolve().parent
SOURCE_HASHES = {}


def frozen_module(path):
    source = subprocess.check_output(["git", "show", f"{REVISION}:{path}"], text=True)
    SOURCE_HASHES[path] = hashlib.sha256(source.encode()).hexdigest()
    module = types.ModuleType(f"review_{Path(path).stem}")
    module.__file__ = str(ROOT / path)
    exec(compile(source, module.__file__, "exec"), module.__dict__)
    return module


# Dependencies that provide the sampled geometry must match the frozen source.
subprocess.run(["git", "diff", "--quiet", REVISION, "--", "autocal/pose_planning.py",
                "autocal/active_learning.py", "autocal/rough_bootstrap.py",
                "autocal/_autocal_common.py", "autocal/json_schema.py"], check=True)
eq = frozen_module("autocal/equal_strain.py")
workspace = frozen_module("autocal/workspace.py")
ac = frozen_module("autocal/autocal.py")
ac.profile_policy_error = eq.profile_policy_error

from autocal.active_learning import SweepConfig
from autocal.headless_sim import HeadlessSimulation
from autocal.pose_planning import predict_sweep_positions

ANCHORS = np.array([[0., -1900., -280.], [1645.44826719, 950., -280.],
                    [-1645.44826719, 950., -280.], [0., 0., 1900.]])
STRANDS = np.array([2., 2., 2., 3.])
EXTRA = np.array([528.3, 528.3, 528.3, 398.9])
ADVANTAGE = np.array([2., 2., 2., 4.])
WEIGHT_N = .081 * 9.81
START_FORCE_N = .23
TARGET_N = 5 * START_FORCE_N / min(STRANDS * ADVANTAGE)
EA_N = 20000.


def lengths_and_null(position, lines):
    to_anchor = ANCHORS[lines] - position
    lengths = np.linalg.norm(to_anchor, axis=1)
    directions = (to_anchor / lengths[:, None]).T
    null = np.linalg.svd(directions)[2][-1]
    null *= np.sign(sum(null))
    return lengths, directions, null


def summary(tensions, position, lines, advantage, target):
    lengths, directions, null = lengths_and_null(position, lines)
    u, n, y = advantage[lines], STRANDS[lines], EXTRA[lines]
    flex_ea = tensions * (u * lengths + y) / (u ** 2 * n)
    reference = lengths + y / u
    return {"force_balance_residual_n": (directions @ tensions - [0., 0., WEIGHT_N]).tolist(),
            "projected_strain_residual_ea_mm_n": float(null @ (flex_ea - target * reference)),
            "projected_strain_residual_mm_at_ea": float(null @ (flex_ea - target * reference) / EA_N),
            "achieved_weighted_target_n": float(null @ flex_ea / (null @ reference))}


position = np.array([250., -150., 800.])
options = dict(strand_tension_n=TARGET_N, mover_weight_n=WEIGHT_N,
               strands=STRANDS, extra_line_length_mm=EXTRA)
invariant_options = dict(options, strand_tension_n=1.)
corrected = eq.equal_strain_tensions(ANCHORS, position, range(4),
                                    mechanical_advantage=ADVANTAGE, **invariant_options)
old_u1 = eq.equal_strain_tensions(ANCHORS, position, range(4), **invariant_options)
invariant = {"position_mm": position.tolist(), "common_target_n": 1., "corrected_tensions_n": corrected.tolist(),
             "corrected": summary(corrected, position, list(range(4)), ADVANTAGE, 1.),
             "old_u1_tensions_n": old_u1.tolist(),
             "old_u1_rule_evaluated_with_hp4_reeving": summary(old_u1, position, list(range(4)), ADVANTAGE, 1.)}
assert abs(invariant["corrected"]["projected_strain_residual_ea_mm_n"]) < 1e-9
assert max(abs(x) for x in invariant["corrected"]["force_balance_residual_n"]) < 1e-9
assert abs(invariant["old_u1_rule_evaluated_with_hp4_reeving"]["projected_strain_residual_ea_mm_n"]) > 1

clamping = []
for fixed in [(390., -100.), (390., -1170.), (780., -1560.)]:
    cfg = SweepConfig((0, 3), fixed, 1, 2)
    positions = predict_sweep_positions(ANCHORS, cfg, 60, .05)
    lines = [0, 3, 1, 2]
    intended = np.array([eq.equal_strain_tensions(ANCHORS, position, lines,
                                                 mechanical_advantage=ADVANTAGE, **options)
                         for position in positions])
    feasible = np.all(np.isfinite(intended) & (intended > 0), axis=1)
    positions, intended = positions[feasible], intended[feasible]
    profiles = eq.sensor_force_profiles(ANCHORS, cfg, minimum_force_n=START_FORCE_N,
                                       mechanical_advantage=ADVANTAGE, **options)
    for direction, profile in enumerate(profiles):
        sensor_index = lines.index(profile["sensor_anchor"])
        commanded = np.maximum(intended[:, sensor_index], START_FORCE_N)
        order = np.argsort(np.linalg.norm(ANCHORS[profile["drive_anchor"]] - positions, axis=1)
                           - np.linalg.norm(ANCHORS[profile["drive_anchor"]]))
        assert np.allclose(profile["force_n"], commanded[order])
        cases = []
        for index, position in enumerate(positions):
            _, _, null = lengths_and_null(position, lines)
            # With three passive held lines and one torque-controlled sensor,
            # changing that sensor force moves equilibrium along this null vector.
            actual = intended[index] + (commanded[index] - intended[index, sensor_index]) / null[sensor_index] * null
            cases.append({"position_mm": position.tolist(), "intended_sensor_n": float(intended[index, sensor_index]),
                          "commanded_sensor_n": float(commanded[index]), "actual_tensions_n": actual.tolist(),
                          **summary(actual, position, lines, ADVANTAGE, TARGET_N)})
        worst = max(cases, key=lambda x: abs(x["projected_strain_residual_ea_mm_n"]))
        clamping.append({"fixed_anchors": list(cfg.fixed_anchors), "fixed_mm": list(fixed),
                         "drive": profile["drive_anchor"], "sensor": profile["sensor_anchor"],
                         "table_rows": len(positions), "clamped_rows": int(sum(commanded > intended[:, sensor_index])),
                         "intended_target_n": TARGET_N, "worst_case": worst})
assert any(item["clamped_rows"] for item in clamping)
assert max(item["worst_case"]["achieved_weighted_target_n"] for item in clamping) > 1.7 * TARGET_N

config = {"m666_before_data_collection": {"W": .081, "N": STRANDS.tolist(),
                                        "Y": EXTRA.tolist(), "U": ADVANTAGE.tolist()},
          "force_tuning": {"force_low_n": .01, "force_mid_n": START_FORCE_N, "force_max_n": 4.}}
sweep = {"id": "sweep_001", "fixed_anchors": [0, 3], "fixed_lengths": [390., -100.],
         "drive_anchor": 1, "sensor_anchor": 2,
         "data_points": [{"l_drive": -100., "l_sensor": -100.}],
         "metadata": {"sensor_force_profile": True}}
legacy = {"version": "2.0", "machine_type": "hangprinter_4", "num_anchors": 4,
          "dimensions": 3, "config": copy.deepcopy(config), "sweeps": [copy.deepcopy(sweep)]}
corrected_dataset = copy.deepcopy(legacy)
corrected_dataset["sweeps"][0]["metadata"].update(sensor_force_profile_model=eq.PROFILE_STRAIN_MODEL,
                                                  strand_tension_n=TARGET_N)
assert eq.profile_policy_error(legacy)
assert eq.profile_policy_error(corrected_dataset) is None
profile_path = DIRECTORY / "retuned-profile.json"
corrected_dataset["config"]["force_tuning"]["force_mid_n"] = .4
assert eq.write_sensor_force_profile(profile_path, corrected_dataset, ANCHORS, [SweepConfig((0,3), (390.,-100.), 1,2)])
retuned = json.loads(profile_path.read_text())
assert all(p["strand_tension_n"] == TARGET_N for p in retuned)
inconsistent = copy.deepcopy(corrected_dataset)
other = copy.deepcopy(inconsistent["sweeps"][0]); other["id"] = "sweep_002"
other["metadata"]["strand_tension_n"] = TARGET_N * 2
inconsistent["sweeps"].append(other)
assert eq.profile_policy_error(inconsistent)

mixed = copy.deepcopy(legacy)
constant = copy.deepcopy(sweep); constant["id"] = "sweep_002"
constant["metadata"]["sensor_force_profile"] = False
mixed["sweeps"].append(constant)


class BoundaryReached(Exception):
    pass


def check_entry(dataset, name, offline=False):
    path = DIRECTORY / name
    path.write_text(json.dumps(dataset, indent=2) + "\n")
    output = io.StringIO()
    result = {"policy_error": eq.profile_policy_error(dataset)}
    argv = ["--headless-sim", "--machine-type", "hangprinter_4", "--dataset", str(path)]
    if offline:
        argv.append("--no-collect")
    def blocked_entry(self):
        raise BoundaryReached("mocked simulator boundary; no service or physics started")
    with patch.object(HeadlessSimulation, "__init__", lambda *args, **kwargs: None), \
         patch.object(HeadlessSimulation, "__enter__", blocked_entry), \
         patch.object(ac, "_run_full_auto", return_value=73), \
         contextlib.redirect_stderr(output), contextlib.redirect_stdout(output):
        try:
            result["main_return"] = ac.main(argv)
        except BoundaryReached as error:
            result["mock_simulator_boundary_reached"] = str(error)
        except SystemExit as error:
            result["system_exit"] = error.code
    result["last_console_line"] = output.getvalue().strip().splitlines()[-1:]
    return result


legacy_entry = check_entry(legacy, "legacy-profile.json")
offline_entry = check_entry(legacy, "legacy-profile.json", offline=True)
mixed_entry = check_entry(mixed, "mixed-profile.json")
assert legacy_entry["system_exit"] == 2 and "mock_simulator_boundary_reached" not in legacy_entry
assert offline_entry["main_return"] == 73
assert "mock_simulator_boundary_reached" in mixed_entry

frontier = {"num_anchors": 4, "config": {"max_travel_mm": 390.}, "sweeps": []}
_, bounds_initial = workspace.exploration_grid(frontier, 9)
frontier["sweeps"].append({"fixed_anchors": [0,3], "fixed_lengths": [780., -1560.], "data_points": []})
_, bounds_empty = workspace.exploration_grid(frontier, 9)
assert bounds_initial == bounds_empty
frontier["sweeps"][-1]["data_points"] = [{"l_drive": -1., "l_sensor": -2.}]
_, bounds_completed = workspace.exploration_grid(frontier, 9)
assert bounds_completed[3][0] == -1950.

print(json.dumps({"candidate_revision": REVISION, "source_sha256": SOURCE_HASHES,
    "evidence_type": "analytical firmware-equation calculation and mocked application boundary; no physics or hardware",
    "inputs": {"anchors_mm": ANCHORS.tolist(), "strands": STRANDS.tolist(), "extra_mm": EXTRA.tolist(),
               "mechanical_advantage": ADVANTAGE.tolist(), "weight_n": WEIGHT_N,
               "force_mid_n": START_FORCE_N, "default_strand_target_n": TARGET_N,
               "ea_n_for_illustrative_residual_mm": EA_N},
    "uncapped_invariant": invariant, "clamping": clamping,
    "retuning": {"original_strand_target_n": TARGET_N, "new_force_mid_n": .4,
                 "recorded_new_targets_n": [p["strand_tension_n"] for p in retuned]},
    "policy": {"legacy_all_profiled": legacy_entry, "legacy_offline": offline_entry,
               "mixed_legacy_and_constant": mixed_entry,
               "inconsistent_targets_error": eq.profile_policy_error(inconsistent)},
    "frontier": {"initial_bounds": bounds_initial, "empty_failed_bounds": bounds_empty,
                 "nonempty_bounds": bounds_completed}}, indent=2))
