"""Cross-check catalogued machine presets against the scenes and firmware configs.

Checks, per catalogued scene (the non-layered preset; layered presets
intentionally use different R/Q and are not compared here):
  * scene `machine:catalog` resolves back to the same catalog entry;
  * every firmware M669 anchor equals (scene anchor - the body it feeds) in mm;
  * every firmware M666 R spool radius equals the scene's spool radius;
  * Klipper anchor_x/y/z and rotation_distance agree with RRF and the scene.

Exits nonzero on any disagreement.  This is the control experiment for the
negative result in the report: the shipped presets agree today.
"""
from pathlib import Path
import json
import math
import re
import sys

from pxr import Usd, Sdf

ROOT = Path('/home/torbjorn/repos/hp-sim5')
sys.path.insert(0, str(ROOT))
import machines  # noqa: E402

ANCHOR_TAGS = {'Anchor'}
POS_TOL_MM = 0.02


def prim_pos(prim):
    tr = prim.GetAttribute('xformOp:translate')
    return tuple(tr.Get()) if tr and tr.Get() is not None else None


def text(path):
    return (ROOT / path).read_text()


def parse_m669(config_text):
    out = {}
    for line in config_text.splitlines():
        line = line.split(';')[0].strip()
        m = re.match(r'M669\s+(.*)', line)
        if not m:
            continue
        for axis, x, y, z in re.findall(r'\b([ABCDIJLOK])([-+0-9.eE]+):([-+0-9.eE]+):([-+0-9.eE]+)', m.group(1)):
            out[axis] = (float(x), float(y), float(z))
    return out


def parse_m666(config_text, letter):
    for line in config_text.splitlines():
        line = line.split(';')[0].strip()
        if not line.startswith('M666'):
            continue
        m = re.search(rf'(?:^|\s){letter}([-+0-9.eE:]+)', line[len('M666'):])
        if m:
            return [float(v) for v in m.group(1).split(':')]
    return None


def parse_klipper(path):
    anchors = {}
    rotation = {}
    lines = text(path).splitlines()
    section = None
    for line in lines:
        line = line.split('#')[0].strip()
        m = re.match(r'\[(.+)\]', line)
        if m:
            section = m.group(1)
            continue
        if section and section.startswith('stepper_') and ':' in line:
            key, value = [v.strip() for v in line.split(':', 1)]
            if key.startswith('anchor_'):
                anchors.setdefault(section, {})[key[-1]] = float(value)
            elif key == 'rotation_distance':
                rotation[section] = float(value)
    return anchors, rotation


def main():
    catalog = json.loads(text('machines/catalog.json'))
    failures = []
    checked_anchors = 0
    checked_spools = 0

    scene_machine = {}
    for entry in catalog['machines']:
        scene_machine.setdefault(entry['scene'], []).append(entry)

    for scene_path, scene_entries in scene_machine.items():
        stage = Usd.Stage.Open(str(ROOT / scene_path))
        scene_root = None
        for prim in stage.Traverse():
            if prim.GetAttribute('machine:catalog'):
                scene_root = prim
                break
        identity = scene_root.GetAttribute('machine:catalog').Get() if scene_root else None
        if not identity or machines.machine(identity)['scene'] != scene_path:
            failures.append(f'{scene_path}: machine:catalog={identity!r} does not resolve to this scene')
            continue

        tagged = {p.GetName(): p for p in scene_root.GetChildren()}
        positions = {name: prim_pos(p) for name, p in tagged.items()}

        # Anchor -> the body it feeds (next joint in the cable path).
        feed = {}
        for prim in scene_root.GetChildren():
            if prim.GetTypeName() != 'CableJoint':
                continue
            b0 = prim.GetRelationship('physics:body0').GetTargets()
            b1 = prim.GetRelationship('physics:body1').GetTargets()
            if not b0 or not b1:
                continue
            n0, n1 = b0[0].name, b1[0].name
            if n0 in tagged and 'Anchor' in (tagged[n0].GetAttribute('ecs:tags').Get() or []):
                # Only frame anchors feeding a mover attachment encode the
                # firmware M669 vector. Mounted-motor scenes (slideprinter)
                # feed a spool and use a different idealisation.
                target_tags = tagged[n1].GetAttribute('ecs:tags').Get() or []
                if {'Attachment', 'Pinhole', 'Eyelet'} & set(target_tags):
                    feed[n0] = n1

        entry = scene_entries[0]
        rrf_text = text(entry['rrf_config'])
        rrf_anchors = parse_m669(rrf_text)
        rrf_radius = parse_m666(rrf_text, 'R')
        rrf_u = parse_m666(rrf_text, 'U') or [1.0] * len(rrf_anchors)

        # Firmware axis order is the letters it defines.
        for name, other in feed.items():
            m = re.search(r'Anchor([ABCDIJLO])', name)
            if not m:
                continue
            letter = m.group(1)
            if letter not in rrf_anchors:
                continue
            anchor = positions.get(name)
            target = positions.get(other)
            if anchor is None or target is None:
                continue
            offset = [1000.0 * (a - b) for a, b in zip(anchor, target)]
            expected = rrf_anchors[letter]
            err = max(abs(a - b) for a, b in zip(offset, expected))
            checked_anchors += 1
            if err > POS_TOL_MM:
                failures.append(f'{scene_path}: {name} - {other} = {offset} mm but M669 {letter} = {list(expected)} '
                                f'(error {err:.3f} mm)')

        # Spool radii. Only scenes with a spool are checked.
        spools = [p for p in tagged.values() if 'Spool' in (p.GetAttribute('ecs:tags').Get() or [])]
        if spools and rrf_radius:
            # Every spool in these scenes uses the same authored radius family;
            # compare each spool's radius against the R entry for its axis.
            for spool in spools:
                letter = spool.GetName()[-1]
                axis_index = list(rrf_anchors.keys()).index(letter) if letter in rrf_anchors else None
                if axis_index is None or axis_index >= len(rrf_radius):
                    continue
                r = spool.GetAttribute('radius').Get()
                err = abs(1000.0 * r - rrf_radius[axis_index])
                checked_spools += 1
                if err > 0.01:
                    failures.append(f'{scene_path}: {spool.GetName()} radius {1000*r:.4f} mm vs M666 R {rrf_radius[axis_index]} mm')

        if entry.get('klipper_config') and rrf_radius:
            k_anchors, k_rotation = parse_klipper(entry['klipper_config'])
            order = list(rrf_anchors.keys())
            for idx, section in enumerate([f'stepper_{chr(ord("a") + i)}' for i in range(len(order))]):
                if section not in k_anchors:
                    failures.append(f'{entry["klipper_config"]}: missing [{section}]')
                    continue
                for axis in 'xyz':
                    err = abs(k_anchors[section][axis] - rrf_anchors[order[idx]]['xyz'.index(axis)])
                    if err > 1e-4:
                        failures.append(f'{entry["klipper_config"]}: {section} anchor_{axis} disagrees with M669 {order[idx]}')
                # Klipper folds mechanical advantage into rotation_distance.
                expected_rd = 2 * math.pi * rrf_radius[idx] / rrf_u[idx]
                rd = k_rotation.get(section)
                if rd is not None and abs(rd - expected_rd) > 0.01:
                    failures.append(f'{entry["klipper_config"]}: {section} rotation_distance {rd} != {expected_rd:.4f}')

    print(f'checked {len(scene_machine)} scenes, {checked_anchors} anchor offsets, {checked_spools} spool radii')
    for failure in failures:
        print('MISMATCH:', failure)
    if failures:
        print(f'FAIL: {len(failures)} catalog/scene/firmware disagreements')
        return 1
    print('PASS: catalogued scenes, anchors, spool radii and Klipper configs agree')
    return 0


if __name__ == '__main__':
    sys.exit(main())
