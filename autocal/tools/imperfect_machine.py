"""Run the default headless autocal on one randomized imperfect machine and score it.

The machine's config.g is left as catalogued (nominal anchors, nominal M666 S);
only the simulated scene deviates from it:
  - each low anchor is raised or lowered by its own dz (uniform in +-low_dz_max_mm),
  - the top anchor is moved sideways by an offset of uniform direction and a
    radius uniform in [0, top_offset_max_mm],
  - every line's true stiffness per unit length (EA) is the scene's value times
    stiffness_scale (uniform in 1 +- stiffness_error), so the true compliance is
    1/stiffness_scale times what config.g's M666 S describes.
--cogging-scale multiplies every torque-mode motor's cogging (the engine default,
1% of holding torque) by a factor and --cogging-freq sets its periods per turn (default
numPolePairs), to stress force realisation; neither is drawn.
An anchor moves rigidly with its spool, line pivots and anchor-side wheels (every
frame-side prim; effector rigid-body members stay), so spool-to-anchor
(guy) lengths are unchanged. Truth is the catalogued M669 plus these shifts. Any
override fixes that perturbation; the others are still drawn from the seed, so one
seed's draws are the same whichever overrides are given.

  python -m autocal.tools.imperfect_machine OUT --machine hangprinter_3 --seed 3
  python -m autocal.tools.imperfect_machine OUT --machine hangprinter_3 --seed 3 \\
      --stiffness-scale 1 --top-offset-mm 0:0          # low-anchor heights only
  python -m autocal.tools.imperfect_machine OUT --rescore   # recompute result.json

OUT/result.json holds truth, fit, per-coordinate errors and the fit's own quality
scores (rms, chi2, jackknife anchor uncertainty, split-fit difference) for every
iteration's selected fit. Error columns, each as <key>_error_mm, max_abs_<key>_error_mm
and worst_<key>_coordinate:
  frame  primary: truth and fit each expressed in the print convention (origin at
         home, z through the top anchor, first anchor at x=0, y<0). A true top
         anchor off the home plumb line is a frame tilt (truth_frame_tilt_deg), not error.
  raw    fit minus scene truth in the scene frame.
  gauge  fit minus truth rotated so the low anchors' plane is level (the shared-low-z
         fit's own frame; on three low anchors lengths cannot see that rotation).
  rigid  after the best rotation and translation of the fit onto truth (shape only).
Existing output directories are never overwritten.
"""
import argparse
import json
from pathlib import Path
import re

import numpy as np

from autocal._autocal_common import REPO_ROOT, catalogued_machine
from autocal.tools.autocal_benchmark import split_fit_difference_mm
from autocal.tools.benchmark_history import run_autocal

DEFAULTS = dict(low_dz_max_mm=30.0, top_offset_max_mm=50.0, stiffness_error=0.2)
SUPPORTED = ('hangprinter_3', 'hangprinter_4', 'hangprinter_5', 'hangprinter_5_pulley_reduced')


def nominal_anchors(machine):
    config = (REPO_ROOT / catalogued_machine(machine)['rrf_config']).read_text()
    line = next(line.split(';')[0] for line in config.splitlines() if re.match(r'M669\s+A', line))
    return {name: [float(v) for v in values.split(':')]
            for name, values in re.findall(r'\b([A-Z])([-\d.]+:[-\d.]+:[-\d.]+)', line)}


def draw(machine, seed, low_dz_mm=None, top_offset_mm=None, stiffness_scale=None,
         low_dz_max_mm=DEFAULTS['low_dz_max_mm'], top_offset_max_mm=DEFAULTS['top_offset_max_mm'],
         stiffness_error=DEFAULTS['stiffness_error']):
    """Perturbations of one seed; every draw is made even when overridden."""
    names = list(nominal_anchors(machine))
    rng = np.random.default_rng(seed)
    dz = rng.uniform(-low_dz_max_mm, low_dz_max_mm, len(names) - 1)
    radius, angle = rng.uniform(0, top_offset_max_mm), rng.uniform(0, 2 * np.pi)
    scale = rng.uniform(1 - stiffness_error, 1 + stiffness_error)
    if low_dz_mm is not None:
        dz = np.asarray(low_dz_mm, dtype=float)
        if dz.shape != (len(names) - 1,):
            raise ValueError(f'{machine} needs {len(names) - 1} low-anchor dz values')
    top = np.asarray(top_offset_mm, dtype=float) if top_offset_mm is not None else radius * np.array([np.cos(angle), np.sin(angle)])
    shifts = {name: [0.0, 0.0, float(z)] for name, z in zip(names[:-1], dz)}
    shifts[names[-1]] = [float(top[0]), float(top[1]), 0.0]
    return dict(seed=seed, shifts_mm=shifts,
                stiffness_scale=float(scale if stiffness_scale is None else stiffness_scale),
                ranges=dict(low_dz_max_mm=low_dz_max_mm, top_offset_max_mm=top_offset_max_mm,
                            stiffness_error=stiffness_error))


FRAME_PRIM = re.compile(r'        def \w+ "((?:Spool|Anchor|Wheel|Pinhole|Corner|Attach)([A-Z])\w*)"')


def perturbed_scene(text, shifts_mm, stiffness_scale):
    """Shift every frame-side prim of each anchor and scale cable EA.

    Frame-side prims are the top-level Spool/Anchor/Wheel/Pinhole/Corner/Attach<name>*
    prims that are not members of the effector rigid body: the anchor's spool, its
    line pivots and, on reeved machines, its anchor-side wheels and attachments.
    """
    effector = re.search(r'def RigidBody "Effector".*?rigidBody:members = \[([^\]]*)\]', text, re.S)
    members = set(re.findall(r'/(\w+)>', effector[1])) if effector else set()
    lines = text.splitlines(keepends=True)
    moved = {name: [] for name in shifts_mm}
    current = None
    for index, line in enumerate(lines):
        match = FRAME_PRIM.match(line)
        if match:
            current = match[2] if match[2] in shifts_mm and match[1] not in members else None
            if current:
                moved[current].append(match[1])
        elif line.startswith('        def '):
            current = None
        translate = re.match(r'(\s+double3 xformOp:translate = \()([^)]*)(\).*)', line, re.S)
        if current and translate:
            values = [float(v) + d / 1000 for v, d in zip(translate[2].split(','), shifts_mm[current])]
            lines[index] = translate[1] + ', '.join(repr(v) for v in values) + translate[3]
            current = None
    for name, prims in moved.items():
        if not any(p.startswith('Spool') for p in prims) or len(prims) < 2:
            raise ValueError(f'anchor {name}: expected its spool and line pivots, found {prims}')
    scene, count = re.subn(r'(cablePath:stiffnessPerUnitLength\s*=\s*)([-+\d.eE]+)',
                           lambda m: m[1] + f'{float(m[2]) * stiffness_scale:.12g}', ''.join(lines))
    if not count:
        raise ValueError('no cable stiffness values found')
    return scene, moved


def scaled_cogging(text, scale=1.0, freq=None):
    """Give every stepper an explicit cogging torque of scale x the engine default (1% of holding),
    at freq periods per turn when given."""
    def lines(m):
        extra = f'\n{m[1]}custom double stepper:coggingFreq = {freq:.12g}' if freq is not None else ''
        return f'{m[0]}\n{m[1]}custom double stepper:coggingTorque = {0.01 * float(m[2]) * scale:.12g}{extra}'
    scene, count = re.subn(r'^(\s*)custom double stepper:holdingTorque\s*=\s*([-+\d.eE]+)\s*$', lines,
                           text, flags=re.M)
    if not count:
        raise ValueError('no stepper holding torques found')
    return scene


def _rotate_onto_z(anchors, vector):
    """Rotate about the origin (home) by the smallest rotation taking vector to +z."""
    vector = np.asarray(vector, dtype=float) / np.linalg.norm(vector)
    axis, angle = np.cross(vector, [0, 0, 1]), np.arccos(np.clip(vector[2], -1, 1))
    if np.linalg.norm(axis) > 0:
        k = axis / np.linalg.norm(axis)
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        anchors = anchors @ (np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K).T
    turn = np.arctan2(-anchors[0, 0], -anchors[0, 1])  # then about z: first anchor at x=0, y<0
    c, s = np.cos(turn), np.sin(turn)
    return anchors @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T, float(np.degrees(angle))


def convention_frame(anchors):
    """The agreed print frame: origin at home, z axis through the top anchor, first
    anchor in the x=0 half-plane with y<0. Returns anchors in it and the tilt (deg)
    between the top anchor's direction and the input frame's z."""
    return _rotate_onto_z(np.asarray(anchors, dtype=float), np.asarray(anchors, dtype=float)[-1])


def gauge_truth(truth):
    """Truth as the shared-low-z fit can express it: rotated about home so the low
    anchors' least-squares plane is level, then about z so the first anchor has x=0.
    Line lengths cannot tell these apart on three low anchors."""
    low = truth[:-1]
    normal = np.linalg.svd(low - low.mean(axis=0))[2][-1]
    return _rotate_onto_z(truth, normal * np.sign(normal[2]))


def rigid_error(fitted, truth):
    """Fit minus truth after the best rigid motion (rotation and translation) of the fit."""
    f, t = fitted - fitted.mean(axis=0), truth - truth.mean(axis=0)
    u, _, vt = np.linalg.svd(f.T @ t)
    d = np.sign(np.linalg.det(u @ vt))
    return f @ (u @ np.diag([1, 1, d]) @ vt) - t


def errors(fitted, truth, truth_frame, gauge):
    """Error columns: frame (primary, the print convention), raw, level-plane gauge, rigid."""
    fitted = np.asarray(fitted, dtype=float)
    fitted_frame, fit_tilt = convention_frame(fitted)
    columns = dict(frame=fitted_frame - truth_frame, raw=fitted - truth, gauge=fitted - gauge,
                   rigid=rigid_error(fitted, truth))
    return fitted_frame, fit_tilt, columns


def worst(names, error):
    row, column = np.unravel_index(np.abs(error).argmax(), error.shape)
    return f'{names[row]}.{"xyz"[column]}'


def score(output):
    imperfection = json.loads((output / 'imperfection.json').read_text())
    names = list(imperfection['truth_mm'])
    truth = np.array(list(imperfection['truth_mm'].values()))
    gauge, tilt = gauge_truth(truth)
    truth_frame, truth_tilt = convention_frame(truth)
    dataset = output / 'sweeps.json'
    manifest_path = dataset.with_suffix('.headless') / 'manifest.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    log = dataset.with_suffix('.full_auto_log.jsonl')
    rows = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    iterations = []
    for row in rows:
        if 'selected_run' not in row:
            continue
        run = next(r for r in row['runs'] if r['id'] == row['selected_run'])
        anchors = run['metrics'].get('pose_fit_anchors')
        if anchors is None:
            continue
        _, _, columns = errors(anchors, truth, truth_frame, gauge)
        iterations.append(dict(iteration=row['iteration'], num_sweeps=row.get('num_sweeps'), sim_s=row.get('sim_s'),
                               decision=row.get('decision'), frame_error_mm=columns['frame'].round(4).tolist(),
                               **{f'max_abs_{k}_error_mm': float(np.abs(v).max()) for k, v in columns.items()},
                               anchor_uncertainty_mm=row.get('anchor_uncertainty_mm'), cost=row.get('cost'),
                               **{k: run['metrics'].get(k) for k in ('pose_fit_rms_mm', 'chi2_red', 'max_std_mm')}))
    summary = next((r for r in rows if r.get('event') == 'summary'), None)
    result = dict(machine=imperfection['machine'], seed=imperfection['seed'], imperfection=imperfection,
                  status=manifest.get('status'), stop_reason=manifest.get('stop_reason'),
                  normal_completion=manifest.get('normal_completion'), simulated_s=manifest.get('simulated_s'),
                  wall_s=manifest.get('wall_s'), git_revision=manifest.get('git_revision'),
                  anchor_names=names, truth_mm=truth.tolist(), truth_frame_mm=truth_frame.round(4).tolist(),
                  truth_frame_tilt_deg=truth_tilt, gauge_truth_mm=gauge.round(4).tolist(),
                  low_plane_tilt_deg=tilt, iterations=iterations)
    if summary:
        fitted = np.asarray(summary['anchors'])
        fitted_frame, fit_tilt, columns = errors(fitted, truth, truth_frame, gauge)
        result.update(fitted_mm=fitted.tolist(), fitted_frame_mm=fitted_frame.round(4).tolist(),
                      fitted_frame_tilt_deg=fit_tilt, selection=summary.get('selection') or {})
        for key, error in columns.items():
            result.update({f'{key}_error_mm': error.round(4).tolist(),
                           f'max_abs_{key}_error_mm': float(np.abs(error).max()),
                           f'worst_{key}_coordinate': worst(names, error)})
        try:
            result['split_fit_difference_mm'] = split_fit_difference_mm(json.loads(dataset.read_text()), fitted)
        except (KeyError, ValueError, np.linalg.LinAlgError) as exc:
            result['split_fit_difference_mm'] = None
            result['split_fit_error'] = str(exc)
    (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('output', type=Path)
    parser.add_argument('--machine', choices=SUPPORTED, default='hangprinter_3')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--low-dz-mm', help='override: colon-separated dz per low anchor, e.g. 0:12:-5')
    parser.add_argument('--top-offset-mm', help='override: top anchor x:y offset, e.g. 0:0')
    parser.add_argument('--stiffness-scale', type=float, help='override: true EA / configured EA')
    for key, value in DEFAULTS.items():
        parser.add_argument('--' + key.replace('_', '-'), type=float, default=value)
    parser.add_argument('--cogging-scale', type=float, help='multiply the motors\' default cogging torque')
    parser.add_argument('--cogging-freq', type=float, help='cogging periods per motor turn (default numPolePairs)')
    parser.add_argument('--ellipsoid-shape', action='store_true', help='collect and fit the opt-in ellipsoid shape')
    parser.add_argument('--timeout-s', type=float, default=55 * 60)
    parser.add_argument('--prepare-only', action='store_true', help='write scene and imperfection.json, do not run')
    parser.add_argument('--rescore', action='store_true', help='only recompute OUT/result.json')
    args = parser.parse_args(argv)
    output = args.output.resolve()
    if args.rescore:
        print(json.dumps({k: v for k, v in score(output).items() if k not in ('iterations', 'imperfection')}, indent=2))
        return 0
    floats = lambda text: [float(v) for v in text.split(':')] if text else None
    imperfection = draw(args.machine, args.seed, floats(args.low_dz_mm), floats(args.top_offset_mm), args.stiffness_scale,
                        args.low_dz_max_mm, args.top_offset_max_mm, args.stiffness_error)
    source = REPO_ROOT / catalogued_machine(args.machine)['scene']
    scene, moved = perturbed_scene(source.read_text(), imperfection['shifts_mm'], imperfection['stiffness_scale'])
    if args.cogging_scale is not None or args.cogging_freq is not None:
        scene = scaled_cogging(scene, 1.0 if args.cogging_scale is None else args.cogging_scale, args.cogging_freq)
    truth = {name: (np.add(anchor, imperfection['shifts_mm'][name])).round(6).tolist()
             for name, anchor in nominal_anchors(args.machine).items()}
    output.mkdir(parents=True, exist_ok=False)
    (output / 'input-scene.usda').write_text(scene)
    imperfection.update(machine=args.machine, source_scene=str(source.relative_to(REPO_ROOT)),
                        moved_prims=moved, truth_mm=truth, nominal_mm=nominal_anchors(args.machine),
                        overrides=dict(low_dz_mm=args.low_dz_mm, top_offset_mm=args.top_offset_mm,
                                       stiffness_scale=args.stiffness_scale), ellipsoid_shape=args.ellipsoid_shape,
                        cogging_scale=args.cogging_scale, cogging_freq=args.cogging_freq)
    (output / 'imperfection.json').write_text(json.dumps(imperfection, indent=2) + '\n')
    if args.prepare_only:
        print(json.dumps(imperfection, indent=2))
        return 0
    code = run_autocal(args.machine, output / 'sweeps.json', REPO_ROOT, output / 'autocal.log',
                       args.timeout_s, scene=output / 'input-scene.usda',
                       extra_args=['--ellipsoid-shape'] if args.ellipsoid_shape else ())
    result = score(output)
    print(json.dumps({k: result.get(k) for k in ('machine', 'seed', 'stop_reason', 'max_abs_frame_error_mm',
                      'worst_frame_coordinate', 'frame_error_mm', 'truth_frame_tilt_deg', 'max_abs_raw_error_mm',
                      'max_abs_rigid_error_mm', 'simulated_s', 'wall_s')}, indent=2))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
