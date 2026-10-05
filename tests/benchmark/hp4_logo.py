"""Benchmark scheduled HP4 logo commands without a browser or real-time pacing.

Run with PYTHONPATH=src/python .venv/bin/python tests/benchmark/hp4_logo.py.
See tests/benchmark/README.md for command generation and comparable runs.
"""
import argparse
import cProfile
import hashlib
import json
from pathlib import Path
import platform
import statistics
import time

from cable_joints_3d.machine_simulation import load_machine_world, register_machine_systems
from cable_joints_3d.machine_snapshot import capture_machine_snapshot
from cable_joints_3d.remote_spool_system import RemoteSpoolSystem


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--commands', type=Path, required=True)
    parser.add_argument('--steps', type=int, help='Prefix length; omit for the complete print')
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--profile', type=Path, help='Profile a separate run after wall-time measurements')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rrd', type=Path, help='Record every step; each repeat gets its own RRD')
    args = parser.parse_args()
    if args.repeats < 1 or (args.steps is not None and args.steps < 1):
        parser.error('repeats and steps must be positive')
    started = time.perf_counter()
    source = args.commands.read_bytes()
    commands = json.loads(source)
    if not isinstance(commands, list) or not commands:
        parser.error('commands must be a nonempty JSON array')
    total_commands = len(commands)
    parse_seconds = time.perf_counter() - started
    steps = min(len(commands), args.steps) if args.steps is not None else len(commands)
    commands = commands[:steps]
    times, loads, snapshots = [], [], []

    def run(index, profile=None):
        started = time.perf_counter()
        world = load_machine_world('public/usd_scenes/hp4_rigid_body.usda')
        loads.append(time.perf_counter() - started)
        recording = None
        if args.rrd:
            import rerun as rr
            path = args.rrd.with_name(f'{args.rrd.stem}-{index}{args.rrd.suffix}')
            path.parent.mkdir(parents=True, exist_ok=True)
            recording = rr.RecordingStream('HP4 logo benchmark')
            recording.set_sinks(rr.FileSink(path))
            register_machine_systems(world, recording)
        remote = world.get_system(RemoteSpoolSystem)
        remote.commands = commands
        dt = world.get_resource('dt')
        started = time.perf_counter()
        try:
            if profile:
                profile.enable()
            for _ in range(steps):
                world.update(dt)
            if profile:
                profile.disable()
            if recording:
                recording.flush(timeout_sec=30)
            elapsed = time.perf_counter() - started
            snapshot = capture_machine_snapshot(world)
        finally:
            if recording:
                recording.disconnect()
        print(f'run {index}: {steps} steps in {elapsed:.3f} s ({steps / elapsed:.1f} steps/s)', flush=True)
        return elapsed, snapshot, dt

    for index in range(args.repeats):
        elapsed, snapshot, dt = run(index)
        times.append(elapsed)
        snapshots.append(snapshot)
    if any(snapshot != snapshots[0] for snapshot in snapshots[1:]):
        raise AssertionError('Repeated runs produced different final snapshots')
    if args.profile:
        profile = cProfile.Profile()
        run('profile', profile)
        profile.dump_stats(args.profile)
    median = statistics.median(times)
    result = {
        'python': platform.python_version(), 'platform': platform.platform(),
        'commands': str(args.commands), 'commands_sha256': hashlib.sha256(source).hexdigest(),
        'total_commands': total_commands, 'steps': steps, 'dt': dt,
        'recording': bool(args.rrd), 'parse_seconds': parse_seconds,
        'load_seconds': loads, 'wall_seconds': times, 'median_seconds': median,
        'steps_per_second': steps / median, 'simulated_seconds': steps * dt,
        'snapshot': snapshots[0],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, allow_nan=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
