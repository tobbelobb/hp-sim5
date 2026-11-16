"""Compare fit options on exactly the same embedded stage dataset, without collection."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from autocal.tools.sanity_battery import anchor_errors
from autocal.stage_artifacts import read_stage_artifact
from autocal.calibrate import build_anchor_initial_guess
from autocal.tools.joint_pose_baseline import fit_joint_ranges
from autocal.projection_mode import resolve_projection_mode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('options', type=Path, nargs='+', help='JSON option objects; one per fit')
    parser.add_argument('--reference-m669', type=Path, help='Reference M669 anchors in millimetres')
    parser.add_argument('--joint', action='store_true',
                        help='Also fit raw observations with independent joint anchor/offset/position least squares (Hangprinter)')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    summary = []
    for index, options in enumerate(args.options):
        target = args.output / f'fit-{index}.json'
        command = [sys.executable, '-m', 'autocal.tools.replay_stage', 'fit', str(args.stage),
                   '--options', str(options), '--output', str(target)]
        result = subprocess.run(command, capture_output=True, text=True)
        (args.output / f'fit-{index}.log').write_text(result.stdout + result.stderr)
        row = {'options': json.loads(options.read_text()), 'command': command, 'exit_code': result.returncode}
        if result.returncode == 0:
            replay = read_stage_artifact(target)['payload']
            fit = replay['fit']
            row.update(anchors=fit['anchors'].tolist(), cost_noise_normalized=fit.get('cost_noise_normalized'),
                       home_offsets_mm=fit['home_offsets_mm'].tolist(),
                       pose_fit_anchors=fit['pose_fit']['anchors'].tolist(),
                       pose_fit_rms_mm=fit['pose_fit']['rms_mm'])
            if args.joint:
                try:
                    if fit.get('spool_model_params') is not None:
                        raise ValueError('Joint comparison with fitted spool lengths is not validated')
                    seed = build_anchor_initial_guess(fit['dataset'], low_anchor_z=fit['dataset'].get('_low_anchor_z'))
                    joint = fit_joint_ranges(
                        fit['dataset'], fit['anchors'] if seed is None else seed,
                        fit_home_offsets=fit['home_offsets_fitted'],
                        use_noise_mean=bool(replay['fit_options'].get('use_noise_mean', True))
                                       and resolve_projection_mode(fit['dataset']) is not True,
                    )
                    row['joint_fit'] = {key: value.tolist() if hasattr(value, 'tolist') else value
                                        for key, value in joint.items()}
                except ValueError as error:
                    row['joint_error'] = str(error)
                    row['exit_code'] = 1
            if args.reference_m669:
                anchors = fit['anchors']
                m669 = ' '.join(name + ':'.join(str(v) for v in anchor)
                                for name, anchor in zip('ABCDIJKLO', anchors))
                row['errors'] = anchor_errors(args.reference_m669.read_text(), m669)
                pose_m669 = ' '.join(name + ':'.join(str(v) for v in anchor)
                                    for name, anchor in zip('ABCDIJKLO', fit['pose_fit']['anchors']))
                row['pose_fit_errors'] = anchor_errors(args.reference_m669.read_text(), pose_m669)
                if 'joint_fit' in row:
                    joint_m669 = ' '.join(name + ':'.join(str(v) for v in anchor)
                                         for name, anchor in zip('ABCDIJKLO', joint['anchors']))
                    row['joint_fit_errors'] = anchor_errors(args.reference_m669.read_text(), joint_m669)
        summary.append(row)
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return int(any(row['exit_code'] or row.get('joint_fit', {}).get('success') is False for row in summary))


if __name__ == '__main__':
    raise SystemExit(main())
