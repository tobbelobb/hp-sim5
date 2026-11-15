"""Compare fit options on exactly the same embedded stage dataset, without collection."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from autocal.tools.sanity_battery import anchor_errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('options', type=Path, nargs='+', help='JSON option objects; one per fit')
    parser.add_argument('--reference-m669', type=Path, help='Reference M669 anchors in millimetres')
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
            fit = json.loads(target.read_text())['payload']['fit']
            row.update({key: fit.get(key) for key in ('anchors', 'cost_noise_normalized', 'home_offsets_mm')})
            if args.reference_m669:
                anchors = fit['anchors']['value']
                m669 = ' '.join(name + ':'.join(str(v) for v in anchor)
                                for name, anchor in zip('ABCD', anchors))
                row['errors'] = anchor_errors(args.reference_m669.read_text(), m669)
        summary.append(row)
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return int(any(row['exit_code'] for row in summary))


if __name__ == '__main__':
    raise SystemExit(main())
