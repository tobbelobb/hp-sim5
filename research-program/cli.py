"""A research agenda and portable handoffs. The coding platform owns its agents."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.archive import session_name
from hp_sim5_research.workspace import Sessions, lock, read_json, researcher_handoff, role, write_json
from hp_sim5_research import coding_client


def resolve_program(sessions, resume):
    if resume == 'latest':
        resume = read_json(sessions.state / 'last-program.json')['directory']
    candidate = Path(resume).expanduser()
    if candidate.is_dir():
        return candidate.resolve()
    if candidate.name != resume or resume in ('.', '..'):
        raise ValueError('Supply a program ID or an existing program directory')
    candidate = ROOT / 'research/output' / resume
    if not candidate.is_dir():
        raise ValueError(f'Unknown program: {resume}; supply its directory on this host')
    return candidate.resolve()


def read_program(directory):
    path = directory / 'program.json'
    if not path.is_file():
        raise ValueError('Legacy program has no program.json; start a new program and carry its agenda/evidence forward')
    program = read_json(path)
    if 'program' in program:
        previous = program['program']
        program = dict(program, version=1, id=previous['id'], question=previous['goal'], agents=program.get('agents', []))
    program.setdefault('agents', [])
    if not program.get('id') or not program.get('question'):
        raise ValueError('Invalid program.json: expected id and question')
    return program


def handoff(program, directory):
    return {'program_id': program['id'], 'directory': str(directory),
            'agenda': str(directory / 'agenda.md'), 'agents': program['agents'],
            'instructions': role(ROOT, 'manager'),
            'task': f"{program['question']}\n\nProgram directory: {directory}\n"
                    'Read agenda.md and the research-program skill. Prepare independent researchers with '
                    '`./hp-sim5-research-program --resume DIRECTORY --agent QUESTION`, then delegate '
                    'the returned instructions and task using your platform. Update the agenda when evidence changes direction.'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    entry = parser.add_mutually_exclusive_group(required=True)
    entry.add_argument('--question', help='Start a program with this research question')
    entry.add_argument('--resume', '--program', nargs='?', const='latest', help='Reopen ID or directory; omit value for latest')
    parser.add_argument('--agent', help='Prepare a researcher for this question; the platform starts it')
    parser.add_argument('--agent-id', help='Reopen a previously commissioned researcher on this host')
    parser.add_argument('--review', type=Path, help='Seal a JSON selection for a fresh reviewer; see README')
    parser.add_argument('--verify', type=Path, help='Verify a sealed review selection without running code')
    backend = parser.add_mutually_exclusive_group()
    backend.add_argument('--codex', action='store_const', const='codex', dest='backend')
    backend.add_argument('--claude', action='store_const', const='claude', dest='backend')
    parser.add_argument('--cloud', action='store_true', help='Use the current hosted chat (the default)')
    parser.add_argument('--model')
    parser.add_argument('--conversation', help='Exact backend conversation ID for a standalone manager')
    parser.add_argument('--model-provider', default='openai')
    parser.add_argument('--dry-run', action='store_true')
    args, client_args = parser.parse_known_args(argv)
    if client_args and client_args[0] == '--':
        client_args = client_args[1:]
    if client_args and not args.backend:
        parser.error('Client arguments require --codex or --claude')
    if args.cloud and args.backend:
        parser.error('--cloud uses the current chat; omit --codex and --claude')
    if sum(bool(x) for x in (args.agent, args.agent_id, args.review, args.verify)) > 1:
        parser.error('Choose --agent, --agent-id, --review, or --verify')
    if args.backend and any((args.agent, args.agent_id, args.review, args.verify)):
        parser.error('Delegate this handoff through the current platform; omit the client flag')
    sessions = Sessions(ROOT)
    try:
        if args.question:
            if not args.question.strip():
                raise ValueError('Research question must not be empty')
            identity = 'program-' + session_name(args.question)
            directory = ROOT / 'research/output' / identity
            program = {'version': 1, 'id': identity, 'question': args.question, 'agents': []}
            if not args.dry_run:
                directory.mkdir(parents=True)
                write_json(directory / 'program.json', program)
                (directory / 'agenda.md').write_text(f'# {args.question}\n\n'
                    '## Current understanding\n\nNo findings yet.\n\n'
                    '## Open questions and alternatives\n\n'
                    'Choose consequential, tractable questions; explore before committing.\n\n'
                    '## Next decision\n\nIdentify an investigation that could change our understanding.\n')
        else:
            directory = resolve_program(sessions, args.resume)
            program = read_program(directory)
        if not args.dry_run:
            if not (directory / 'agenda.md').exists():
                (directory / 'agenda.md').write_text(f'# {program["question"]}\n\n'
                    'Read the existing program.json, reports, and evidence before choosing the next investigation.\n')
            write_json(sessions.state / 'last-program.json', {'directory': str(directory)})
        if args.agent_id:
            member = next((a for a in program['agents'] if a['id'] == args.agent_id), None)
            if member is None:
                raise ValueError('Unknown researcher for this program')
            try:
                session = sessions.open(resume=member['id'], dry_run=args.dry_run)
            except ValueError:
                target = directory / member['directory']
                if target.is_dir():
                    session = sessions.open(resume=str(target), dry_run=args.dry_run)
                else:
                    session = sessions.open(directory=target, topic=member['question'], dry_run=args.dry_run)
            result = researcher_handoff(ROOT, session)
        elif args.agent:
            if not args.agent.strip():
                raise ValueError('Researcher question must not be empty')
            identity = 'session-' + session_name(args.agent)
            session = sessions.open(directory=directory / identity, topic=args.agent, dry_run=args.dry_run)
            if not args.dry_run:
                with lock(sessions.state / 'program-locks' / (program['id'] + '.lock')):
                    program = read_program(directory)
                    program['agents'].append({'id': session['id'], 'question': args.agent, 'directory': identity})
                    write_json(directory / 'program.json', program)
            result = researcher_handoff(ROOT, session)
        elif args.review or args.verify:
            from evidence import seal, verify
            result = seal(ROOT, directory, args.review, dry_run=args.dry_run) if args.review else verify(args.verify)
        else:
            result = handoff(program, directory)
        if not args.backend:
            print(json.dumps(result, indent=2, ensure_ascii=False))
            return 0
        # Opening a standalone manager remains a convenience, with ordinary client resume.
        run = sessions.state / 'managers' / program['id']
        conversations = read_json(run / 'conversations.json') if (run / 'conversations.json').is_file() else {}
        if args.conversation:
            conversations[args.backend] = args.conversation
        bound = conversations.get(args.backend)
        resume_client = bool(args.resume and args.backend in conversations)
        bound_instructions = result['instructions'] + f'\nProgram: {directory}\nAgenda: {directory / "agenda.md"}\n'
        if not args.dry_run:
            run.mkdir(parents=True, exist_ok=True)
            (run / 'instructions.md').write_text(bound_instructions)
            (run / 'attachment.json').write_text('{"mcpServers": {}}\n')
        if args.backend == 'codex':
            extra = (['resume', *([bound] if bound else [])] if resume_client else []) + client_args
            command = coding_client.codex_command(ROOT, run, instructions=bound_instructions,
                       goal_mode=True, model=args.model, model_provider=args.model_provider,
                       settings=coding_client.disabled_research_mcp_settings(ROOT), codex_args=extra)
        else:
            if args.resume and bound:
                extra = ['--resume', bound]
            else:
                import uuid
                bound = str(uuid.uuid4())
                conversations[args.backend] = bound
                extra = ['--session-id', bound]
            extra += client_args
            command = coding_client.claude_command(run, goal_mode=True, model=args.model,
                       allowed_tools='Read,Glob,Grep,Bash,Agent,Skill', codex_args=extra)
        if not resume_client:
            command.append(result['task'])
        if args.dry_run:
            print(json.dumps(dict(result, argv=command), indent=2))
            return 0
        conversations.setdefault(args.backend, None)
        write_json(run / 'conversations.json', conversations)
        environment = coding_client.codex_environment(coding_client.forwarded_settings(client_args)[0].get('model_provider', args.model_provider), root=ROOT, env_file=ROOT / '.env') if args.backend == 'codex' else coding_client.claude_environment()
        return subprocess.call(command, cwd=ROOT, env=environment)
    except (OSError, ValueError, TypeError, KeyError, subprocess.SubprocessError) as error:
        parser.exit(2, f'hp-sim5-research-program: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
