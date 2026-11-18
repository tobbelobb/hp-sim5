#!/usr/bin/env python3
"""Reproducer (latent): the worker export nulls extrusion deposit positions.

`ExtruderComponent.extrusions` rows are `{pos: [x, y, z], length, machineId,
color}` (hp-sim-3d/app/remoteSpoolSystem.js). `scripts/research_physics.mjs`
serializes them with `xyz(r.pos)`, which reads `.x/.y/.z` from an Array, so every
deposit arrives in Python as `pos: [null, null, null]` (and NaN after
`mirror._decode`). The position is the only place the deposit's location is
recorded, so a deposit round trip loses it.

No current consumer reads this field (the harness's snapshots ignore it), so the
defect is latent rather than a wrong displayed result. Exit 1 means the
documented deposit data did not survive serialization.
"""
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SCENE = ROOT / 'public/usd_scenes/hp4.usda'
STEPS = 10
MOVE = 0.2


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
    addon = use_checked_in_native_addon()
    sys.path.insert(0, str(ROOT / 'src/python'))
    from hp_sim5_research.js_physics import JSPhysics
    from hp_sim5_research import mirror

    out = HERE / 'repro-output' / 'extrusion-position'
    out.mkdir(parents=True, exist_ok=True)
    physics = JSPhysics(ROOT, out, SCENE, layering=True)
    try:
        physics.request(dt=0.002)
        command = {'type': 'Move', 'E': MOVE, **{axis: MOVE for axis in 'ABCD'}}
        state = physics.request(steps=STEPS, commands=[command])
    finally:
        physics.close()
    truth = json.loads(subprocess.run(
        ['node', str(HERE / 'extrusion_truth.mjs'), str(SCENE), str(STEPS), str(MOVE)],
        check=True, capture_output=True, text=True).stdout)

    exported = next(rows for name, rows in state['components'] if name == 'ExtruderComponent')
    records = exported[0][1]['extrusions'] if exported else []
    raw = exported[0][1]
    world = mirror.World()
    mirror.apply_state(world, state)
    extruder_id = world.query([mirror.ExtruderComponent])[0]
    decoded = world.get_component(extruder_id, mirror.ExtruderComponent).extrusions

    evidence = {
        'scene': str(SCENE), 'steps': STEPS, 'move_rad_and_mm': MOVE,
        'native_addon': str(addon), 'native_addon_sha256': hashlib.sha256(addon.read_bytes()).hexdigest(),
        'js_extrusions': truth['extrusions'], 'js_deposited_length': truth['deposited_length'],
        'worker_exported_extrusions': records,
        'python_decoded_positions': [['nan' if math.isnan(value) else value for value in row['pos']]
                                     for row in decoded],
        'exported_positions_are_finite': all(
            isinstance(value, (int, float)) and math.isfinite(value)
            for row in records for value in row.get('pos', [])),
    }
    (HERE / 'evidence_extrusion_position.json').write_text(json.dumps(evidence, indent=2) + '\n')
    print(json.dumps(evidence, indent=2))

    failures = []
    if not truth['extrusions']:
        failures.append('the JS world produced no deposit; increase the command or steps')
    if len(records) != len(truth['extrusions']):
        failures.append(f'worker exported {len(records)} deposits, JS world has {len(truth["extrusions"])}')
    if not evidence['exported_positions_are_finite']:
        failures.append('worker export lost the deposit position (nulls instead of coordinates)')
    if failures:
        print('\nFAIL (extrusion deposit serialization):', file=sys.stderr)
        for failure in failures:
            print('  - ' + failure, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
