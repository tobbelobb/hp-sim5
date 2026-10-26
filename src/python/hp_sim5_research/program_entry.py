"""Use the current chat as manager, or open one local coding-client chat."""
import argparse
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys

from . import coding_client
from .program import ProgramStore, _json
from .program_host import open_manager, read_json, request
from .program_manager import failure


class Parser(argparse.ArgumentParser):
    def error(self, message):
        self.exit(2, _json(failure('invalid_request', message)) + '\n')


def client_plan(root, attachment, backend, *, model=None, model_provider='openai',
                dont_ask=False, client_args=()):
    """Local managers use ordinary program CLI calls, without a manager MCP."""
    if coding_client.research_client() == 'cloud':
        raise ValueError('The current cloud chat is the manager. Use --cloud without --codex or --claude; read scripts/cloud/RESEARCH.md')
    session = Path(attachment['session'])
    instructions = ('Bound execution mode: LOCAL. This is a local installed coding-client manager.\n'
                    'Use local CLI workers and reviewers. Do not use --cloud or the cloud-research-client skill.\n'
                    'Command sandboxing and invisible host PIDs do not make this a cloud run.\n'
                    'Never switch transport to recover a controller error.\n'
                    'Bound manager connection: ' + attachment['connection'] +
                    '\nProgram ID: ' + attachment['program_id'] + '\nManager artifacts: ' + str(session) +
                    '\nUse hp-sim5-research-program tool with this connection; no manager MCP attachment is needed.\n\n' +
                    (root / 'research/prompts/research-manager.txt').read_text())
    if backend == 'codex':
        forwarded, forwarded_model = coding_client.forwarded_settings(client_args)
        command = coding_client.codex_command(
            root, session, instructions=instructions, dont_ask=dont_ask, goal_mode=True,
            model_provider=None if 'model_provider' in forwarded else model_provider,
            model=None if forwarded_model else model,
            settings=coding_client.disabled_research_mcp_settings(root), codex_args=client_args)
        provider = forwarded.get('model_provider', model_provider)
    else:
        command = coding_client.claude_command(session, model=model, dont_ask=dont_ask, goal_mode=True,
                                               allowed_tools='Read,Glob,Grep,Bash', codex_args=client_args)
        provider = None
    return {'argv': command, 'instructions': instructions,
            'backend': backend, 'model_provider': provider, 'physics_world': False}


def main(argv, root):
    root = Path(root).resolve()
    parser = Parser(description=__doc__)
    parser.add_argument('--store', type=Path, default=root / '.research-programs')
    probe = argparse.ArgumentParser(add_help=False)
    probe.add_argument('--store')
    _, remainder = probe.parse_known_args(argv[:argv.index('--')] if '--' in argv else argv)
    if remainder and remainder[0] == 'tool':
        parser.add_argument('command', choices=['tool'])
        parser.add_argument('connection', type=Path)
        parser.add_argument('operation')
        parser.add_argument('--params', default='{}')
        args = parser.parse_args(argv)
        try:
            result = request(read_json(args.connection), args.operation, json.loads(args.params),
                             timeout=60 if args.operation == 'save' else 10)
        except (OSError, ValueError, TypeError, KeyError) as error:
            result = failure('invalid_request', str(error))
        print(_json(result))
        return 0 if result['ok'] else 2
    backend = parser.add_mutually_exclusive_group()
    backend.add_argument('--codex', action='store_const', const='codex', dest='backend')
    backend.add_argument('--claude', action='store_const', const='claude', dest='backend')
    parser.add_argument('--cloud', action='store_true', help='Current authenticated chat is the manager; workers/reviewers use cloud subagents')
    entry = parser.add_mutually_exclusive_group(required=True)
    entry.add_argument('--question', help='Create a program from this question')
    entry.add_argument('--program', help='Open the existing program ID')
    parser.add_argument('--constraints', type=json.loads, default={})
    parser.add_argument('--dry-run', action='store_true', help='Plan without creating files, host, clients or workers')
    parser.add_argument('--model')
    parser.add_argument('--model-provider', default='openai')
    parser.add_argument('--dont-ask', action='store_true')
    split = argv.index('--') if '--' in argv else len(argv)
    args = parser.parse_args(argv[:split])
    client_args = argv[split + 1:]
    if client_args and not args.backend:
        parser.error('Forwarded client arguments require a local --codex or --claude manager')
    if args.backend == 'claude' and any(a.startswith('--model-provider') for a in argv[:split]):
        parser.error('--model-provider requires --codex')
    try:
        if args.cloud:
            os.environ['HP_SIM5_RESEARCH_CLIENT'] = 'cloud'
        transport = coding_client.research_client()
        if transport == 'cloud' and (args.backend or args.model or args.model_provider != 'openai'):
            parser.error('The current cloud chat is the manager. Use --cloud without a coding CLI, model or provider override')
        if args.dry_run:
            attachment = {'program_id': args.program or 'PROGRAM_ID',
                          'session': str(root / 'output/research/managers/dry-run'),
                          'connection': str(args.store.absolute() / 'managers' / (args.program or 'PROGRAM_ID') / 'connection.json')}
            opened = {'attachment': attachment, 'overview': None}
        else:
            with ProgramStore(args.store, create=bool(args.question)) as store:
                program = (store.create_program(args.question, args.constraints) if args.question
                           else store.get_program(args.program))
            response = open_manager(args.store, root, program['id'])
            if not response['ok']:
                print(_json(response))
                return 2
            opened = response['result']
            attachment = opened['attachment']
        plan = client_plan(root, attachment, args.backend, model=args.model, model_provider=args.model_provider,
                           dont_ask=args.dont_ask, client_args=client_args) if args.backend else None
        if not args.dry_run and plan:
            session = Path(attachment['session'])
            (session / 'attachment.json').write_text('{"mcpServers":{}}\n')
            (session / 'instructions.md').write_text(plan['instructions'])
            (session / 'launch.json').write_text(_json(plan) + '\n')
        result = {'ok': True, 'result': dict(opened, client=plan, client_transport=transport,
                                            dry_run=args.dry_run, physics_world=False)}
        if not args.backend or args.dry_run:
            print(_json(result))
            return 0
        print(_json(result), file=sys.stderr, flush=True)
        environment = (coding_client.codex_environment(plan['model_provider'], root=root, env_file=root / '.env')
                       if args.backend == 'codex' else coding_client.claude_environment(env_file=root / '.env'))
        environment['HP_SIM5_RESEARCH_CLIENT'] = transport
        return subprocess.run(plan['argv'] + ['Open the bound program overview and manage this question.'],
                              cwd=root, env=environment).returncode
    except (OSError, ValueError, sqlite3.Error, subprocess.SubprocessError) as error:
        print(_json(failure('operation_failed', str(error))))
        return 2
