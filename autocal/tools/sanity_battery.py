"""Run isolated headless autocal cases and compare final to initial firmware anchors.

Example: python -m autocal.tools.sanity_battery OUTPUT --machine hangprinter_4
Use --scene public/usd_scenes/hp4_worn_bearings.usda or --stiffness 10000
for controlled scene variants. Existing output directories are never overwritten.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys

from autocal._autocal_common import REPO_ROOT, catalogued_machine


def anchor_errors(initial, final):
    def parse(text):
        anchors = []
        for name in 'ABCD':
            match = re.search(rf'\b{name}\s*:?\s*([-+\d.eE]+)[,:]\s*([-+\d.eE]+)[,:]\s*([-+\d.eE]+)', text)
            if not match:
                raise ValueError(f'missing anchor {name}')
            anchors.append([float(v) for v in match.groups()])
        return anchors
    truth, fitted = parse(initial), parse(final)
    diff = [[v - t for v, t in zip(a, b)] for a, b in zip(fitted, truth)]
    shift = [sum(row[i] for row in diff) / len(diff) for i in range(3)]
    aligned = [[v - shift[i] for i, v in enumerate(row)] for row in diff]
    return {'reference_mm': truth, 'fitted_mm': fitted, 'error_mm': diff,
            'mean_translation_mm': shift, 'aligned_error_mm': aligned,
            'anchor_distance_mm': [math.sqrt(sum(v * v for v in row)) for row in diff],
            'aligned_anchor_distance_mm': [math.sqrt(sum(v * v for v in row)) for row in aligned],
            'max_abs_coordinate_mm': max(abs(v) for row in diff for v in row),
            'max_aligned_abs_coordinate_mm': max(abs(v) for row in aligned for v in row)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--machine', choices=['hangprinter_3', 'hangprinter_4'], required=True)
    parser.add_argument('--scene', type=Path)
    parser.add_argument('--stiffness', type=float, help='Set all cable stiffness-per-unit-length values')
    parser.add_argument('--max-steps', type=int, default=20)
    args = parser.parse_args(argv)
    if args.stiffness is not None and args.stiffness <= 0:
        parser.error('stiffness must be positive')
    args.output.mkdir(parents=True, exist_ok=False)
    output = args.output.resolve()
    source = (args.scene or REPO_ROOT / catalogued_machine(args.machine)['scene']).resolve()
    scene = source.read_text()
    if args.stiffness is not None:
        scene, count = re.subn(r'(cablePath:stiffnessPerUnitLength\s*=\s*)[-+\d.eE]+',
                               lambda m: m[1] + str(args.stiffness), scene)
        if not count:
            raise ValueError('no cable stiffness values found')
    scene_path = output / 'input-scene.usda'
    scene_path.write_text(scene)
    command = [sys.executable, '-m', 'autocal.autocal', '--headless-sim', '--machine-type', args.machine,
               '--dataset', str(output / 'sweeps.json'), '--recording-mode', 'none', '--max-steps', str(args.max_steps)]
    env = dict(os.environ, HP_SIM5_AUTOCAL_SCENE=str(scene_path), PYTHONUNBUFFERED='1')
    (output / 'input.json').write_text(json.dumps({'command': command, 'source_scene': str(source),
        'scene_override': str(scene_path), 'stiffness': args.stiffness}, indent=2) + '\n')
    with (output / 'stdout.log').open('w') as log:
        result = subprocess.run(command, cwd=REPO_ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
    manifest_path = output / 'sweeps.headless' / 'manifest.json'
    summary = {'exit_code': result.returncode}
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        summary.update({k: manifest.get(k) for k in ('status', 'stop_reason', 'normal_completion', 'wall_s', 'git_revision')})
        try:
            summary['anchors'] = anchor_errors(manifest['firmware_initial']['M669'], manifest['firmware_final']['M669'])
        except (KeyError, ValueError) as exc:
            summary['anchor_error'] = str(exc)
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
