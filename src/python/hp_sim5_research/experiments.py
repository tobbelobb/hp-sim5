"""Repeatable, bounded native experiments with numeric and Rerun evidence."""
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import platform
import re
import subprocess
import time
import uuid
import warnings

from cable_joints_3d.__main__ import _blueprint
from cable_joints_3d.ecs import EncoderComponent
from cable_joints_3d.machine_simulation import load_machine_world, register_machine_systems
from cable_joints_3d.machine_snapshot import capture_machine_snapshot, entity_name
from cable_joints_3d.remote_spool_system import RemoteSpoolSystem
from cable_joints_3d.spools import SpoolStateComponent
from cable_joints_3d.stepper_motor import StepperMotorComponent
from usd.cable_scene_loader import open_cable_scene

DEFAULT_SCENE = 'public/usd_scenes/hp4_rigid_body.usda'
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
    return repo_path(root, f'output/research/experiments/{run_id}')


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


def sample(world, step, dt):
    snapshot = capture_machine_snapshot(world)
    motors = []
    for entity in world.query([StepperMotorComponent, SpoolStateComponent]):
        motor = world.get_component(entity, StepperMotorComponent)
        state = world.get_component(entity, SpoolStateComponent)
        encoder = world.get_component(entity, EncoderComponent)
        measured = None if encoder is None else encoder.angle - (motor.missed_step_encoder_offset or 0.)
        target = motor.commanded_angle - motor.delta_angle
        motors.append({'name': entity_name(world, entity), 'axis': state.axis,
                       'mode': 'torque' if motor.torque_mode else 'position',
                       'target_angle_rad': None if motor.torque_mode else target,
                       'encoder_angle_rad': measured,
                       'tracking_error_rad': None if motor.torque_mode or measured is None else measured - target,
                       'peak_missed_steps': motor.missed_steps})
    return {'step': step, 'sim_time_s': step * dt,
            'effectors': [frame for frame in snapshot['frames'] if frame['kind'] == 'effector'],
            'motors': motors,
            'cables': [{'name': f"{cable['machine']}/{cable['name']}", 'lengths_m': cable['lengths'],
                        'segment_forces_n': [segment['force_n'] for segment in cable['segments']]}
                       for cable in snapshot['cables']]}


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
        'peak_total_missed_steps': max(sum(motor['peak_missed_steps'] for motor in row['motors']) for row in samples),
    }


def run_experiment(root, scene=DEFAULT_SCENE, *, steps=200, dt=None, commands=None, label='', record=True):
    root = Path(root).resolve()
    if isinstance(steps, bool) or not isinstance(steps, int) or not 1 <= steps <= MAX_STEPS:
        raise ValueError(f'steps must be an integer in [1, {MAX_STEPS}]')
    if dt is not None and (isinstance(dt, bool) or not isinstance(dt, (int, float)) or not math.isfinite(dt) or dt <= 0):
        raise ValueError('dt must be positive and finite')
    scene_path = repo_path(root, scene)
    commands = [] if commands is None else commands
    # Freeze composed USD inputs, including referenced layers and baked cable initialization.
    started = time.perf_counter()
    frozen_scene = open_cable_scene(scene_path).Flatten().ExportToString()
    world = load_machine_world(frozen_scene)
    dt = world.get_resource('dt') if dt is None else dt
    if not isinstance(dt, (int, float)) or not math.isfinite(dt) or dt <= 0:
        raise ValueError('Authored dt must be positive and finite')
    world.set_resource('dt', dt)
    axes = {world.get_component(entity, SpoolStateComponent).axis
            for entity in world.query([StepperMotorComponent, SpoolStateComponent])}
    validate_commands(commands, axes, steps)
    if not axes or not capture_machine_snapshot(world)['cables']:
        raise ValueError('The scene must contain driven spools and cable paths')
    construction_s = time.perf_counter() - started
    run_id = uuid.uuid4().hex
    directory = run_directory(root, run_id)
    directory.mkdir(parents=True)
    (directory / 'scene.usda').write_text(frozen_scene)
    write_json(directory / 'commands.json', commands)
    source = hashlib.sha256()
    for path in sorted((root / 'src/python').rglob('*.py')):
        source.update(str(path.relative_to(root)).encode() + b'\0' + path.read_bytes())
    revision = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=root, capture_output=True, text=True)
    manifest = {'schema_version': 1, 'run_id': run_id, 'label': label, 'status': 'running',
                'scene': str(scene_path.relative_to(root)), 'steps': steps, 'dt_s': dt,
                'commands_sha256': digest(encode(commands).encode()), 'scene_sha256': digest(frozen_scene.encode()),
                'python_source_sha256': source.hexdigest(), 'git_revision': revision.stdout.strip(),
                'python': platform.python_version(), 'platform': platform.platform(),
                'packages': {name: version(name) for name in ('numpy', 'usd-core', 'rerun-sdk')},
                'construction_wall_s': construction_s, 'record': record,
                'artifacts': {name: str(directory / file) for name, file in
                              [('manifest', 'manifest.json'), ('scene', 'scene.usda'), ('commands', 'commands.json'),
                               ('telemetry', 'telemetry.jsonl'), ('snapshot', 'final.json')]}}
    recording = None
    write_json(directory / 'manifest.json', manifest)
    try:
        if record:
            import rerun as rr
            recording = rr.RecordingStream('hp-sim5 research', recording_id=run_id)
            recording.set_sinks(rr.FileSink(directory / 'recording.rrd'), default_blueprint=_blueprint())
            recording.log('experiment', rr.TextDocument(encode(manifest)), static=True)
            register_machine_systems(world, recording)
            manifest['artifacts']['rrd'] = str(directory / 'recording.rrd')
        world.get_system(RemoteSpoolSystem).commands = commands
        samples = []
        started = time.perf_counter()
        with warnings.catch_warnings(record=True) as caught, (directory / 'telemetry.jsonl').open('w') as output:
            warnings.simplefilter('always')
            for step in range(steps + 1):
                if step:
                    world.update(dt)
                row = sample(world, step, dt)
                output.write(encode(row) + '\n')  # rejects non-finite state
                samples.append(row)
            manifest['warnings'] = sorted({str(item.message) for item in caught})
        manifest.update(status='complete', metrics=metrics(samples), simulation_wall_s=time.perf_counter() - started)
        write_json(directory / 'final.json', capture_machine_snapshot(world))
        if recording is not None:
            recording.flush(timeout_sec=5)
    except Exception as error:
        manifest.update(status='failed', error=str(error))
        raise
    finally:
        if recording is not None:
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
            'changed_inputs': [key for key in ('scene_sha256', 'commands_sha256', 'python_source_sha256', 'packages', 'record')
                               if baseline[key] != candidate[key]],
            'metric_delta_candidate_minus_baseline': {key: candidate['metrics'][key] - value
                                                      for key, value in baseline['metrics'].items() if isinstance(value, (int, float))},
            'final_effector_distance_m': {frame['path']: math.dist(positions[frame['path']], frame['position'])
                                          for frame in after['effectors']},
            'interpretation': 'Differences are observations. A smaller error/force is not automatically a better calibration.'}
