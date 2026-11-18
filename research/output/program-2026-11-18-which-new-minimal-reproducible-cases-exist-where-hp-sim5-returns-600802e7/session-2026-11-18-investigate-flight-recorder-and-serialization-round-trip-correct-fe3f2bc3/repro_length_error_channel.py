#!/usr/bin/env python3
"""Reproducer: headless recordings fabricate the line-length error channel.

Runs a five-step HP4 experiment through the documented headless API
(`hp_sim5_research.experiments.run_experiment`), reads the saved `.rrd` back with
Rerun's RrdReader, and compares the recorded `commanded`/`error` values with the
production JS flight-recorder snapshot for the same scene and commands.

Expected (FLIGHT_RECORDER.md, "Length trace" table):
  commanded = predicted paid-out length at the target
  error     = actual - commanded
  `peak_abs_length_error_m` > 0 while motors lag their targets.
Actual: commanded == actual and error == 0.0 everywhere, so the manifest reports
`peak_abs_length_error_m: 0.0` even though the real peak error is millimetres.

Exit code 1 means the documented invariant is violated (the bug is present).
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
MOVE_RAD = 0.1
TOLERANCE_M = 1e-6


def use_checked_in_native_addon():
    """Point the loader at a byte copy outside the checkout.

    The checkout's addon is newer than one Rust source file, and the default
    loader refuses stale builds. A custom path is the loader's supported way to
    use an existing artifact without rebuilding.
    """
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
    addon = use_checked_in_native_addon()
    sys.path.insert(0, str(ROOT / 'src/python'))
    from hp_sim5_research.experiments import run_experiment
    from rerun.chunk import RrdReader

    commands = [{'type': 'Move', **{axis: MOVE_RAD for axis in 'ABCD'}}]
    manifest = run_experiment(str(ROOT), scene=SCENE, steps=STEPS, commands=commands, record=True)
    run = Path(manifest['artifacts']['rrd']).parent
    rrd = Path(manifest['artifacts']['rrd'])
    rows = [json.loads(line) for line in (run / 'telemetry.jsonl').read_text().splitlines()]

    time_rows = []
    for chunk in RrdReader(rrd).stream().to_chunks():
        if chunk.entity_path == '/world/machines/default/effector' and chunk.num_rows:
            batch = chunk.to_record_batch()
            time_rows = list(zip(batch.column('sim_step').to_pylist(),
                                 batch.column('Transform3D:translation').to_pylist()))
    truth = json.loads(subprocess.run(
        ['node', str(HERE / 'snapshot_truth.mjs'), str(ROOT / SCENE), str(STEPS), str(MOVE_RAD)],
        check=True, capture_output=True, text=True).stdout)

    telemetry_zero = sum(1 for row in rows for cable in row['cables']
                         if cable['lengths_m']['error'] == 0.0
                         and cable['lengths_m']['commanded'] == cable['lengths_m']['actual'])
    telemetry_total = sum(len(row['cables']) for row in rows)
    reported = manifest['metrics']['peak_abs_length_error_m']
    real = truth['peak_abs_error_m']

    rrd_errors = []
    for chunk in RrdReader(rrd).stream().to_chunks():
        if chunk.entity_path.startswith('/line_errors/') and chunk.entity_path.endswith('/error'):
            rrd_errors.extend(value for row in chunk.to_record_batch().column('Scalars:scalars').to_pylist()
                              for value in row)

    # Sanity check: the saved recording does reproduce the sampled trajectory.
    trajectory_error = max(
        (abs(recorded[0][axis] - sampled['effectors'][0]['position'][axis])
         for (step, recorded), sampled in zip(time_rows, rows) for axis in range(3)),
        default=float('inf'))
    assert trajectory_error <= TOLERANCE_M, (
        f'replay trajectory mismatch: {trajectory_error} m between .rrd and telemetry')
    assert [step for step, _ in time_rows] == list(range(STEPS + 1)), (
        f'unexpected sim_step values in .rrd: {[step for step, _ in time_rows]}')

    evidence = {
        'run_id': manifest['run_id'],
        'rrd': str(rrd), 'rrd_bytes': rrd.stat().st_size,
        'rrd_sha256': hashlib.sha256(rrd.read_bytes()).hexdigest(),
        'native_addon': str(addon), 'native_addon_sha256': hashlib.sha256(addon.read_bytes()).hexdigest(),
        'metrics': manifest['metrics'],
        'truth_peak_abs_error_m': real,
        'telemetry_cables_with_fabricated_error': telemetry_zero,
        'telemetry_cables_total': telemetry_total,
        'rrd_error_scalars': len(rrd_errors),
        'rrd_max_abs_error': max((abs(value) for value in rrd_errors), default=None),
        'trajectory_roundtrip_max_abs_error_m': trajectory_error,
        'truth_first_nonzero': next((cable for sample in truth['samples'] for cable in sample['cables']
                                     if cable['error'] not in (None, 0.0)), None),
    }
    (HERE / 'evidence_length_error_channel.json').write_text(json.dumps(evidence, indent=2) + '\n')
    print(json.dumps({key: evidence[key] for key in (
        'metrics', 'truth_peak_abs_error_m', 'telemetry_cables_with_fabricated_error',
        'telemetry_cables_total', 'rrd_error_scalars', 'rrd_max_abs_error',
        'trajectory_roundtrip_max_abs_error_m')}, indent=2))

    failures = []
    if reported <= 0.0:
        failures.append(f'peak_abs_length_error_m is {reported!r}; the JS snapshot measures {real:.6g} m')
    if telemetry_zero != telemetry_total:
        failures.append(f'{telemetry_zero}/{telemetry_total} telemetry cable rows are commanded==actual, error==0')
    if rrd_errors and any(value != 0.0 for value in rrd_errors):
        failures.append(f'line_errors/error is not identically zero in the .rrd ({len(rrd_errors)} scalars)')
    if failures:
        print('\nFAIL (documented length-error channel is not recorded):', file=sys.stderr)
        for failure in failures:
            print('  - ' + failure, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
