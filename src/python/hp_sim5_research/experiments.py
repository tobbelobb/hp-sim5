"""Repeatable, bounded native experiments with numeric and Rerun evidence."""
import hashlib
from importlib.metadata import version
import json
import math
import os
from pathlib import Path
import platform
import re
import subprocess
import time
import uuid
import warnings

from .js_physics import JSPhysics
from .mirror import World, apply_state, capture_snapshot, sample  # noqa: F401 - re-exported API
from .recording import RerunSystem, blueprint

DEFAULT_SCENE = 'public/usd_scenes/hp4.usda'
MAX_STEPS = 10_000


def encode(value):
    return json.dumps(value, allow_nan=False, sort_keys=True)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def write_json(path, value):
    path.write_text(encode(value) + '\n')


def repo_path(root, value):
    path = (root / value).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError('Input paths must be inside the repository')
    return path


def run_directory(root, run_id):
    if not re.fullmatch(r'[0-9a-f]{32}', run_id):
        raise ValueError('Use the run_id returned by run_experiment')
    # Launcher worktrees deliberately share output with the original checkout.
    # Inputs stay inside the worktree; run IDs stay inside its approved output.
    # A launcher session owns its experiments; loose runs use shared scratch.
    session = os.environ.get('HP_SIM5_SESSION_DIR')
    base = Path(session) if session else Path(root) / 'research/output/scratch'
    output = (base / 'experiments').resolve()
    directory = (output / run_id).resolve()
    if not directory.is_relative_to(output):
        raise ValueError('Experiment paths must stay inside the experiment output directory')
    return directory


def validate_commands(commands, axes, steps):
    if not isinstance(commands, list) or len(commands) > steps:
        raise ValueError('commands must be an array with at most one record per step')
    for command in commands:
        if not isinstance(command, dict):
            raise ValueError('Each command must be an object; {} holds the current targets')
        kind = command.get('type', '')
        if kind not in ('', 'Move', 'Add to reference', 'SetTorqueMode', 'SetPositionMode'):
            raise ValueError(f'Unsupported command type: {kind}')
        allowed = axes | {'type', 'axes', 'E', 'axis', 'torqueNm'}
        if set(command) - allowed:
            raise ValueError(f'Unknown command fields: {sorted(set(command) - allowed)}')
        nested = command.get('axes', {})
        if not isinstance(nested, dict) or set(nested) - axes:
            raise ValueError(f'axes must map known motor axes: {sorted(axes)}')
        numbers = [v for k, v in command.items() if k in axes | {'E', 'torqueNm'}]
        numbers += list(nested.values())
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in numbers):
            raise ValueError('Motor angles, extrusion and torque must be finite numbers')
        if kind in ('SetTorqueMode', 'SetPositionMode') and command.get('axis') not in axes:
            raise ValueError(f'Mode changes require a known axis: {sorted(axes)}')
        if kind == '' and (nested or set(command) & axes):
            raise ValueError('Motor angle commands require type Move or Add to reference')


def metrics(samples):
    initial, final = samples[0], samples[-1]
    positions = {frame['path']: frame['position'] for frame in initial['effectors']}
    return {
        'effector_displacement_m': {frame['path']: math.dist(positions[frame['path']], frame['position'])
                                   for frame in final['effectors']},
        'peak_cable_force_n': max((force for row in samples for cable in row['cables']
                                   for force in cable['segment_forces_n']), default=0.),
        'peak_abs_length_error_m': max((abs(cable['lengths_m']['error']) for row in samples for cable in row['cables']
                                        if cable['lengths_m']['error'] is not None), default=0.),
        'peak_abs_tracking_error_rad': max((abs(motor['tracking_error_rad']) for row in samples for motor in row['motors']
                                            if motor['tracking_error_rad'] is not None), default=0.),
        'total_missed_step_slips': max(sum(motor['missed_step_slips'] for motor in row['motors']) for row in samples),
    }


def run_experiment(root, scene=DEFAULT_SCENE, *, steps=200, dt=None, commands=None, label='', record=True):
    root = Path(root).resolve()
    if isinstance(steps, bool) or not isinstance(steps, int) or not 1 <= steps <= MAX_STEPS:
        raise ValueError(f'steps must be an integer in [1, {MAX_STEPS}]')
    if dt is not None and (isinstance(dt, bool) or not isinstance(dt, (int, float)) or not math.isfinite(dt) or dt <= 0):
        raise ValueError('dt must be positive and finite')
    scene_path = repo_path(root, scene)
    commands = [] if commands is None else commands
    validate_commands(commands, set('ABCD'), steps)
    frozen_scene = scene_path.read_text()
    started = time.perf_counter()
    run_id = uuid.uuid4().hex
    directory = run_directory(root, run_id)
    directory.mkdir(parents=True)
    (directory / 'scene.usda').write_text(frozen_scene)
    write_json(directory / 'commands.json', commands)
    source = hashlib.sha256()
    for path in sorted((root / 'src/python').rglob('*.py')):
        source.update(str(path.relative_to(root)).encode() + b'\0' + path.read_bytes())
    revision = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=root, capture_output=True, text=True)
    manifest = {'schema_version': 2, 'backend': 'headless-js', 'physics_engine': 'rust-native',
                'run_id': run_id, 'label': label, 'status': 'running',
                'scene': str(scene_path.relative_to(root)), 'scene_generation': 0, 'steps': steps, 'dt_s': dt,
                'commands_sha256': digest(encode(commands).encode()), 'scene_sha256': digest(frozen_scene.encode()),
                'python_source_sha256': source.hexdigest(), 'git_revision': revision.stdout.strip(),
                'python': platform.python_version(), 'platform': platform.platform(),
                'packages': {name: version(name) for name in ('numpy', 'rerun-sdk')},
                'construction_wall_s': time.perf_counter() - started, 'record': record,
                'artifacts': {name: str(directory / file) for name, file in
                              [('manifest', 'manifest.json'), ('scene', 'scene.usda'), ('commands', 'commands.json'),
                               ('telemetry', 'telemetry.jsonl'), ('snapshot', 'final.json')]}}
    recording = None
    physics = None
    write_json(directory / 'manifest.json', manifest)
    try:
        native_started = time.perf_counter()
        physics = JSPhysics(root, directory, scene_path, dt=dt)
        state = physics.request(dt=dt)
        dt = state['dt_s'] if dt is None else dt
        manifest.update(dt_s=dt, scene_generation=state.get('scene_generation', 0),
                        physics_engine=physics.physics_engine, physics_artifact=physics.physics_artifact,
                        js_physics_source_sha256=physics.physics_host_sha256)
        manifest['construction_wall_s'] += time.perf_counter() - native_started
        world = World()
        apply_state(world, state)
        if record:
            import rerun as rr
            recording = rr.RecordingStream('hp-sim5 research', recording_id=run_id)
            recording.set_sinks(rr.FileSink(directory / 'recording.rrd'), default_blueprint=blueprint())
            recording.log('experiment', rr.TextDocument(encode(manifest)), static=True)
            manifest['artifacts']['rrd'] = str(directory / 'recording.rrd')
            recorder = RerunSystem(recording)
        samples = []
        started = time.perf_counter()
        with warnings.catch_warnings(record=True) as caught, (directory / 'telemetry.jsonl').open('w') as output:
            warnings.simplefilter('always')
            for step in range(steps + 1):
                if step:
                    state = physics.request(steps=1, commands=commands if step == 1 else ())
                    apply_state(world, state)
                if record:
                    recorder.update(world, dt if step else 0.)
                row = sample(world, step, dt)
                output.write(encode(row) + '\n')  # rejects non-finite state
                samples.append(row)
            manifest['warnings'] = sorted({str(item.message) for item in caught})
        if recording is not None:
            recorder.flush()
        manifest.update(status='complete', metrics=metrics(samples), simulation_wall_s=time.perf_counter() - started)
        write_json(directory / 'final.json', capture_snapshot(world))
        if recording is not None:
            recording.flush(timeout_sec=5)
    except Exception as error:
        manifest.update(status='failed', error=str(error))
        raise
    finally:
        if physics is not None:
            physics.close()
        if recording is not None:
            recorder.flush()
            recording.disconnect()
        write_json(directory / 'manifest.json', manifest)
    return manifest


def read_run(root, run_id, start_step=None, limit=20):
    directory = run_directory(Path(root), run_id)
    manifest = json.loads((directory / 'manifest.json').read_text())
    if start_step is None:
        return manifest
    if not 0 <= start_step <= manifest['steps'] or not 1 <= limit <= 100:
        raise ValueError('start_step must be inside the run; limit must be in [1, 100]')
    with (directory / 'telemetry.jsonl').open() as file:
        return {'run_id': run_id, 'samples': [json.loads(line) for i, line in enumerate(file)
                                            if start_step <= i < start_step + limit]}


def compare_runs(root, baseline_id, candidate_id):
    baseline, candidate = [read_run(root, run_id) for run_id in (baseline_id, candidate_id)]
    if any(run['status'] != 'complete' for run in (baseline, candidate)):
        raise ValueError('Only complete runs can be compared')
    if (baseline['steps'], baseline['dt_s']) != (candidate['steps'], candidate['dt_s']):
        raise ValueError('Compare runs with identical steps and dt')
    before, after = [read_run(root, run['run_id'], run['steps'], 1)['samples'][0] for run in (baseline, candidate)]
    positions = {frame['path']: frame['position'] for frame in before['effectors']}
    if set(positions) != {frame['path'] for frame in after['effectors']}:
        raise ValueError('Compare runs with identical effector identities')
    return {'baseline_id': baseline_id, 'candidate_id': candidate_id,
            'changed_inputs': [key for key in ('scene_sha256', 'commands_sha256', 'python_source_sha256', 'packages',
                                              'record', 'backend', 'physics_engine', 'physics_artifact',
                                              'js_physics_source_sha256')
                               if baseline.get(key) != candidate.get(key)],
            'metric_delta_candidate_minus_baseline': {key: candidate['metrics'][key] - value
                                                      for key, value in baseline['metrics'].items() if isinstance(value, (int, float))},
            'final_effector_distance_m': {frame['path']: math.dist(positions[frame['path']], frame['position'])
                                          for frame in after['effectors']},
            'interpretation': 'Differences are observations. A smaller error/force is not automatically a better calibration.'}
