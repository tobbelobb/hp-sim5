#!/usr/bin/env python3
"""Reproducer: headless `.rrd` scene content contradicts FLIGHT_RECORDER.md.

Two documented native-recording invariants are checked on one HP4 recording:

1. "Each machine has a frame tree ... rigid members use transforms relative to
   their parent body" and the Native recording column's "live parent/member
   hierarchy". Actual: every rigid member is written under
   `world/machines/MACHINE/bodies/NAME` (no `/members/` level) at its parent
   body's world pose, so all wheels and pinholes collapse onto the body centre.
2. Native recording lists motor/encoder/velocity plots, tool points, extrusion
   deposits and `extrusion_lengths/MACHINE`. Actual: those entity groups are
   absent from the file.

Exit code 1 means a documented invariant is violated (the bug is present).
"""
import hashlib
import json
import math
import os
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SCENE = ROOT / 'public/usd_scenes/hp4.usda'
MACHINE = 'default'
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


def rotate(quaternion, vector):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    axis = np.array([x, y, z], dtype=float)
    return vector + w * 2 * np.cross(axis, vector) + 2 * np.cross(axis, np.cross(axis, vector))


def main():
    addon = use_checked_in_native_addon()
    sys.path.insert(0, str(ROOT / 'src/python'))
    import rerun as rr
    from rerun.chunk import RrdReader
    from hp_sim5_research.js_physics import JSPhysics
    from hp_sim5_research import mirror
    from hp_sim5_research.recording import RerunSystem, blueprint

    out = HERE / 'repro-output' / 'scene-content'
    out.mkdir(parents=True, exist_ok=True)
    rrd = out / 'recording.rrd'
    physics = JSPhysics(ROOT, out, SCENE, layering=True)
    try:
        state = physics.request(dt=0.002)
        world = mirror.World()
        mirror.apply_state(world, state)
        truth = {}
        for entity in world.query([mirror.PositionComponent]):
            member = world.get_component(entity, mirror.RigidBodyMemberComponent)
            if member is None:
                continue
            info = world.get_component(entity, mirror.SceneEntityInfoComponent)
            body = world.get_component(member.body_entity, mirror.PositionComponent)
            orientation = world.get_component(member.body_entity, mirror.OrientationComponent)
            truth[info.name] = (np.asarray(body.pos) + rotate(orientation.quaternion, np.asarray(member.local_position))).tolist()

        recording = rr.RecordingStream('hp-sim5 repro', recording_id='scene-content')
        recording.set_sinks(rr.FileSink(rrd), default_blueprint=blueprint())
        recorder = RerunSystem(recording)
        recorder.update(world, 0.)
        recorder.flush()
        recording.flush(timeout_sec=5)
        recording.disconnect()
    finally:
        physics.close()

    chunks = RrdReader(rrd).stream().to_chunks()
    paths = {chunk.entity_path for chunk in chunks}
    recorded = {}
    for chunk in chunks:
        prefix = f'/world/machines/{MACHINE}/bodies/'
        if chunk.entity_path.startswith(prefix) and chunk.num_rows:
            name = chunk.entity_path[len(prefix):]
            if name in truth:
                recorded[name] = chunk.to_record_batch().column('Transform3D:translation').to_pylist()[0][0]
    errors = {name: math.dist(position, truth[name]) for name, position in recorded.items()}
    worst = max(errors.items(), key=lambda item: item[1]) if errors else ('none', 0.0)
    missing_groups = {
        'member frame hierarchy': [path for path in paths if '/members/' in path],
        'anchor frames': [path for path in paths if f'/machines/{MACHINE}/anchors/' in path],
        'tool points': [path for path in paths if '/tool_points/' in path],
        'extrusion deposits': [path for path in paths if '/extrusions/' in path],
        'deposited length': [path for path in paths if path.startswith('/extrusion_lengths/')],
        'motors': [path for path in paths if path.startswith('/motors/')],
        'encoders': [path for path in paths if path.startswith('/encoders/')],
        'velocities': [path for path in paths if path.startswith('/velocities/')],
    }

    evidence = {
        'scene': str(SCENE), 'rrd': str(rrd), 'rrd_bytes': rrd.stat().st_size,
        'rrd_sha256': hashlib.sha256(rrd.read_bytes()).hexdigest(),
        'native_addon': str(addon), 'native_addon_sha256': hashlib.sha256(addon.read_bytes()).hexdigest(),
        'rigid_members_recorded': len(recorded), 'rigid_members_in_state': len(truth),
        'max_member_position_error_m': worst[1], 'worst_member': worst[0],
        'worst_member_recorded': recorded.get(worst[0]), 'worst_member_truth': truth.get(worst[0]),
        'sample_members': {name: {'recorded': recorded[name], 'truth': truth[name]}
                           for name in list(recorded)[:3]},
        'entity_groups_present': {key: len(value) for key, value in missing_groups.items()},
        'entity_path_count': len(paths),
    }
    (HERE / 'evidence_scene_content.json').write_text(json.dumps(evidence, indent=2) + '\n')
    print(json.dumps(evidence, indent=2))

    failures = []
    if not missing_groups['member frame hierarchy']:
        failures.append('no rigid-member path contains /members/; the frame tree is flat')
    if worst[1] > TOLERANCE_M:
        failures.append(f'{worst[0]} is recorded {worst[1]:.6f} m away from its true world pose')
    if not missing_groups['anchor frames']:
        failures.append('no anchor path contains /anchors/; anchors are written under /bodies/')
    for name, found in missing_groups.items():
        if name not in ('member frame hierarchy', 'anchor frames') and not found:
            failures.append(f'documented native-recording entity group is absent: {name}')
    if failures:
        print('\nFAIL (headless .rrd content does not match FLIGHT_RECORDER.md):', file=sys.stderr)
        for failure in failures:
            print('  - ' + failure, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
