#!/usr/bin/env python3
"""Reproducer: a headless NativeSession reset does not start a new generation.

FLIGHT_RECORDER.md says "scene resets restart them and set the scene_generation
timeline" (sim_time/sim_step and scene_generation). This drives the documented
bridge reset through `hp_sim5_research.session.NativeSession`, records both
generations into one `.rrd`, and reads the file back.

Observed: the epoch increments but `scene_generation` stays 1, the `.rrd` has no
`scene_generation` timeline, and sim_step/sim_time restart at 0. Two different
effector poses are stored under the same sim_step index, so a latest-at replay
cannot recover the first generation.

Exit code 1 means the documented invariant is violated (the bug is present).
"""
import asyncio
import hashlib
import json
import math
import os
import shutil
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SCENE = ROOT / 'public/usd_scenes/hp4.usda'


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


def move(value):
    return {'type': 'command', 'command': {'type': 'Move', 'A': value, 'B': value, 'C': value, 'D': value}}


async def record(out):
    sys.path.insert(0, str(ROOT / 'src/python'))
    from hp_sim5_research.session import NativeSession

    session = NativeSession(ROOT, out, SCENE, record=True)
    try:
        await session.handle(move(0.1))
        await session.advance(0.1)
        first = {'epoch': session.epoch, 'step': session.step,
                 'generation': session.world.get_resource('sceneGeneration')}
        await session.handle({'type': 'reset'})
        reset = {'epoch': session.epoch, 'step': session.step,
                 'generation': session.world.get_resource('sceneGeneration')}
        await session.handle(move(-0.2))
        await session.advance(0.1)
        second = {'epoch': session.epoch, 'step': session.step,
                  'generation': session.world.get_resource('sceneGeneration')}
    finally:
        session.close()
    return first, reset, second


def main():
    addon = use_checked_in_native_addon()
    out = HERE / 'repro-output' / 'reset-generation'
    out.mkdir(parents=True, exist_ok=True)
    first, reset, second = asyncio.run(record(out))

    from rerun.chunk import RrdReader
    rrd = out / 'recording.rrd'
    chunks = RrdReader(rrd).stream().to_chunks()
    timeline_names = sorted({name for chunk in chunks for name in chunk.timeline_names})
    steps, poses = [], []
    for chunk in chunks:
        if chunk.entity_path == '/world/machines/default/effector' and chunk.num_rows:
            batch = chunk.to_record_batch()
            steps += batch.column('sim_step').to_pylist()
            poses += [row[0] for row in batch.column('Transform3D:translation').to_pylist()]

    duplicates = {}
    for step, pose in zip(steps, poses):
        duplicates.setdefault(step, []).append(pose)
    collisions = {step: values for step, values in duplicates.items() if len(values) > 1}
    collision_distance = max((math.dist(values[0], values[1])
                              for values in collisions.values() if len(values) > 1), default=0.0)

    events = [json.loads(line) for line in (out / 'events.jsonl').read_text().splitlines()]
    generations = sorted({event['scene_generation'] for event in events if 'scene_generation' in event})
    evidence = {
        'scene': str(SCENE), 'rrd': str(rrd), 'rrd_bytes': rrd.stat().st_size,
        'rrd_sha256': hashlib.sha256(rrd.read_bytes()).hexdigest(),
        'native_addon': str(addon), 'native_addon_sha256': hashlib.sha256(addon.read_bytes()).hexdigest(),
        'before_reset': first, 'after_reset': reset, 'after_second_run': second,
        'scene_generation_timeline': 'scene_generation' in timeline_names,
        'timeline_names': timeline_names,
        'sim_steps': steps, 'duplicate_sim_steps': {str(step): values for step, values in collisions.items()},
        'max_collision_pose_distance_m': collision_distance,
        'event_scene_generations': generations,
    }
    (HERE / 'evidence_reset_generation.json').write_text(json.dumps(evidence, indent=2) + '\n')
    print(json.dumps({key: evidence[key] for key in (
        'before_reset', 'after_reset', 'after_second_run', 'scene_generation_timeline',
        'sim_steps', 'max_collision_pose_distance_m', 'event_scene_generations')}, indent=2))

    failures = []
    if reset['generation'] == first['generation']:
        failures.append(f"reset kept scene_generation={reset['generation']} while epoch went "
                        f"{first['epoch']} -> {reset['epoch']}")
    if 'scene_generation' not in timeline_names:
        failures.append('the .rrd has no scene_generation timeline')
    if collisions:
        failures.append(f'sim_step values repeat across the reset ({sorted(collisions)}); '
                        f'two poses differ by {collision_distance * 1000:.3f} mm at the same index')
    if len(generations) < 2:
        failures.append(f'events.jsonl scene_generation values never change: {generations}')
    if failures:
        print('\nFAIL (reset is not a distinct, replayable generation):', file=sys.stderr)
        for failure in failures:
            print('  - ' + failure, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
