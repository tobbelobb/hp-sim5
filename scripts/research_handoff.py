"""Export selected research evidence or inspect a relocated handoff/saved job offline."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.archive import export_handoff, read_handoff, saved_autocal_status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    export = commands.add_parser('export', help='Copy a session and explicitly selected evidence files')
    export.add_argument('session', type=Path)
    export.add_argument('selection', type=Path, help='JSON claim, scope, decision, source_revision and files')
    export.add_argument('destination', type=Path, help='New directory; existing paths are never overwritten')
    show = commands.add_parser('show', help='Verify and locate a handoff without its original worktree')
    show.add_argument('directory', type=Path)
    job = commands.add_parser('job', help='Read a saved autocal job.json as historical evidence')
    job.add_argument('record', type=Path)
    args = parser.parse_args()
    try:
        if args.command == 'export':
            result = export_handoff(args.session, json.loads(args.selection.read_text()), args.destination)
        elif args.command == 'show':
            result = read_handoff(args.directory)
        else:
            result = saved_autocal_status(args.record)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(2, f'{error}\n')
    print(json.dumps(result, indent=2))
    return 1 if result['diagnostics'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
