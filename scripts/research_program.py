"""Create and inspect durable research programs without starting a physics world."""
import argparse
import json
from pathlib import Path
import sqlite3
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.program import ProgramStore, render_markdown


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--store', type=Path, default=ROOT / '.research-programs', help='Manager-owned store directory')
    commands = parser.add_subparsers(dest='command', required=True)
    create = commands.add_parser('create', help='Create a program from one question')
    create.add_argument('question')
    create.add_argument('--constraints', type=json.loads, default={})
    create.add_argument('--id')
    for name in ('open', 'status', 'overview', 'export'):
        command = commands.add_parser(name)
        command.add_argument('program')
        if name == 'export':
            command.add_argument('--format', choices=('json', 'markdown'), default='json')
    restore = commands.add_parser('import', help='Restore a version-1 JSON export; IDs must be new')
    restore.add_argument('source', type=Path)
    for name in ('question', 'approach', 'assignment', 'artifact', 'decision', 'steering', 'transition'):
        command = commands.add_parser(name)
        command.add_argument('program')
        command.add_argument('--expected-revision', type=int, help='Reject stale manager decisions')
        if name != 'transition':
            command.add_argument('text')
            command.add_argument('--id')
        if name == 'question':
            command.add_argument('--scope', type=json.loads, default={})
        elif name == 'approach':
            command.add_argument('--question', required=True)
            command.add_argument('--reason', required=True)
        elif name == 'assignment':
            command.add_argument('--approach', required=True)
            command.add_argument('--allocation', required=True, type=json.loads, help='Finite JSON limits including positive wall_minutes')
        elif name == 'artifact':
            command.add_argument('--sha256', required=True)
            command.add_argument('--evidence-type', choices=('simulated', 'synthetic', 'analytic', 'hardware', 'imported'), required=True)
            command.add_argument('--source-revision', required=True)
            command.add_argument('--provenance', type=json.loads, default={})
        elif name in ('decision', 'steering'):
            command.add_argument('--subject')
            command.add_argument('--evidence', action='append', default=[])
            command.add_argument('--actor', default='human' if name == 'steering' else 'manager')
        else:
            command.add_argument('record')
            command.add_argument('state')
            command.add_argument('--reason', required=True)
    args = parser.parse_args(argv)
    try:
        with ProgramStore(args.store, create=args.command in ('create', 'import')) as store:
            command = args.command
            options = {'expected_revision': getattr(args, 'expected_revision', None)}
            if command == 'create':
                result = store.create_program(args.question, args.constraints, program_id=args.id)
            elif command == 'import':
                result = store.import_snapshot(json.loads(args.source.read_text()))
            elif command in ('open', 'status', 'overview'):
                result = store.overview(args.program)
            elif command == 'export':
                result = store.snapshot(args.program)
                if args.format == 'markdown':
                    print(render_markdown(result), end='')
                    return 0
            elif command == 'transition':
                result = store.transition(args.program, args.record, args.state, args.reason, **options)
            else:
                options['record_id'] = args.id
                if command == 'question':
                    result = store.add_question(args.program, args.text, scope=args.scope, **options)
                elif command == 'approach':
                    result = store.add_approach(args.program, args.question, args.text, args.reason, **options)
                elif command == 'assignment':
                    result = store.add_assignment(args.program, args.approach, args.text, args.allocation, **options)
                elif command == 'artifact':
                    result = store.add_artifact(args.program, args.text, args.sha256, args.evidence_type,
                                                args.source_revision, provenance=args.provenance, **options)
                else:
                    result = store.record_note(args.program, command, args.text, subject_id=args.subject,
                                               evidence_ids=args.evidence, actor=args.actor, **options)
    except (OSError, ValueError, sqlite3.Error) as error:
        parser.exit(2, f'{error}\n')
    print(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
