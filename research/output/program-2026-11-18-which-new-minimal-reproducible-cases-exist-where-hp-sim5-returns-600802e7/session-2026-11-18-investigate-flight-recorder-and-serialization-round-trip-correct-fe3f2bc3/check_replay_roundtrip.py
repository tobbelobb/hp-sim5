#!/usr/bin/env python3
"""Negative control: recorded trajectory, clocks and replay round trip exactly.

Checks, on HP4 and five fixed steps:
  * two independent record=True runs produce semantically identical `.rrd` data
    (only the static `/experiment` manifest differs, and it contains run ids);
  * recording does not perturb physics (record=True and record=False telemetry
    rows are identical);
  * reading the `.rrd` back reproduces the telemetry trajectory within Rerun's
    f32 transform precision and keeps sim_step = 0..N and sim_time = N*dt.

Exit code 0 means every check passed; 1 means replay diverged.
"""
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SCENE = 'public/usd_scenes/hp4.usda'
STEPS = 5
TOLERANCE_M = 1e-6


def use_checked_in_native_addon():
    addon = ROOT / 'src/wasm/cable_solver/target/native/physics.node'
    scratch = Path('/tmp/opencode')
    if not scratch.is_dir():
        scratch = Path(tempfile.gettempdir())
    cache = scratch / f'hp_sim5_{hashlib.sha256(addon.read_bytes()).hexdigest()[:12]}.node'
    if not cache.is_file():
        shutil.copyfile(addon, cache)
    os.environ['HP_SIM5_NATIVE_ENGINE'] = str(cache)
    return cache


def main():
    os.environ['HP_SIM5_SESSION_DIR'] = str(HERE)
    use_checked_in_native_addon()
    sys.path.insert(0, str(ROOT / 'src/python'))
    from hp_sim5_research.experiments import run_experiment

    commands = [{'type': 'Move', 'A': 0.1, 'B': 0.1, 'C': 0.1, 'D': 0.1}]
    runs = [run_experiment(str(ROOT), scene=SCENE, steps=STEPS, commands=commands, record=True)
            for _ in range(2)]
    plain = run_experiment(str(ROOT), scene=SCENE, steps=STEPS, commands=commands, record=False)
    a, b = [Path(run['artifacts']['rrd']) for run in runs]
    rows = [[json.loads(line) for line in Path(run['artifacts']['telemetry']).read_text().splitlines()]
            for run in runs + [plain]]

    rerun = Path(sys.executable).with_name('rerun')
    scratch = Path('/tmp/opencode') if Path('/tmp/opencode').is_dir() else Path(tempfile.gettempdir())
    filtered = []
    for path, tag in ((a, 'a'), (b, 'b')):
        target = scratch / f'hp_sim5_replay_{tag}.rrd'
        subprocess.run([str(rerun), 'rrd', 'filter', '--drop-entity', '/experiment',
                        str(path), '-o', str(target)], check=True, capture_output=True)
        filtered.append(target)
    compared = subprocess.run([str(rerun), 'rrd', 'compare', '--unordered', *map(str, filtered)],
                              capture_output=True, text=True)

    from rerun.chunk import RrdReader
    steps, times, poses = [], [], []
    for chunk in RrdReader(a).stream().to_chunks():
        if chunk.entity_path == '/world/machines/default/effector' and chunk.num_rows:
            batch = chunk.to_record_batch()
            steps += batch.column('sim_step').to_pylist()
            times += [value.total_seconds() for value in batch.column('sim_time').to_pylist()]
            poses += [row[0] for row in batch.column('Transform3D:translation').to_pylist()]
    trajectory_error = max((abs(pose[axis] - row['effectors'][0]['position'][axis])
                            for pose, row in zip(poses, rows[0]) for axis in range(3)),
                           default=float('inf'))
    telemetry_same = all(row_a == row_b == row_c for row_a, row_b, row_c in zip(*rows))

    evidence = {
        'runs': [run['run_id'] for run in runs], 'record_false_run': plain['run_id'],
        'rrd_bytes': [path.stat().st_size for path in (a, b)],
        'rrd_sha256': [hashlib.sha256(path.read_bytes()).hexdigest() for path in (a, b)],
        'compare_after_dropping_manifest_exit': compared.returncode,
        'compare_output_tail': (compared.stdout or compared.stderr).strip().splitlines()[-1:] if (compared.stdout or compared.stderr) else [],
        'sim_step': steps, 'sim_time_s': times, 'dt_s': runs[0]['dt_s'],
        'replay_vs_telemetry_max_abs_error_m': trajectory_error,
        'record_true_vs_false_telemetry_identical': telemetry_same,
    }
    (HERE / 'evidence_replay_roundtrip.json').write_text(json.dumps(evidence, indent=2) + '\n')
    print(json.dumps(evidence, indent=2))

    failures = []
    if compared.returncode != 0:
        failures.append('two identical recordings differ semantically after dropping /experiment')
    if trajectory_error > TOLERANCE_M:
        failures.append(f'.rrd replay differs from telemetry by {trajectory_error} m')
    if steps != list(range(STEPS + 1)):
        failures.append(f'sim_step is not 0..{STEPS}: {steps}')
    if any(abs(time - step * runs[0]['dt_s']) > 1e-9 for step, time in zip(steps, times)):
        failures.append(f'sim_time does not match step*dt: {times}')
    if not telemetry_same:
        failures.append('recording perturbs the physics telemetry')
    if failures:
        print('\nFAIL (replay round trip):', file=sys.stderr)
        for failure in failures:
            print('  - ' + failure, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
