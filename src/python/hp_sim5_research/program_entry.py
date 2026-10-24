"""Existing-chat CLI tools and dedicated supported-client manager entry."""
import argparse
import json
from pathlib import Path
import sqlite3
import subprocess
import sys

from . import coding_client
from .program import ProgramError, ProgramStore, _json
from .program_host import open_manager, read_json, request
from .program_manager import failure


class Parser(argparse.ArgumentParser):
    def error(self, message):
        self.exit(2, _json(failure('invalid_request', message)) + '\n')


def attachment_config(root, connection):
    return {'mcpServers': {'hp_sim5_program': {
        'command': sys.executable, 'args': [str(root / 'scripts/research_program_mcp.py'), str(connection)]}}}


def client_plan(root, attachment, backend, *, model=None, model_provider='openai',
                dont_ask=False, client_args=()):
    """Reuse worker/reviewer command construction and existing authentication."""
    session, connection = Path(attachment['session']), Path(attachment['connection'])
    instructions = ((root / 'research/prompts/research-manager.txt').read_text() +
                    '\n\nBound manager attachment: ' + str(connection) +
                    '\nProgram ID: ' + attachment['program_id'] + '\nManager artifacts: ' + str(session) + '\n')
    config = attachment_config(root, connection)
    if backend == 'codex':
        forwarded, forwarded_model = coding_client.forwarded_settings(client_args)
        settings = {'mcp_servers.hp_sim5.enabled': False, 'mcp_servers.rerun.enabled': False,
                    'mcp_servers.hp_sim5_program.command': sys.executable,
                    'mcp_servers.hp_sim5_program.args': config['mcpServers']['hp_sim5_program']['args'],
                    'mcp_servers.hp_sim5_program.cwd': str(root),
                    'mcp_servers.hp_sim5_program.required': True,
                    'mcp_servers.hp_sim5_program.startup_timeout_sec': 30,
                    'mcp_servers.hp_sim5_program.default_tools_approval_mode': 'approve',
                    'mcp_servers.hp_sim5_program.tool_timeout_sec': 30}
        command = coding_client.codex_command(
            root, session, instructions=instructions, dont_ask=dont_ask,
            model_provider=None if 'model_provider' in forwarded else model_provider,
            model=None if forwarded_model else model, settings=settings, codex_args=client_args)
        provider = forwarded.get('model_provider', model_provider)
    else:
        command = coding_client.claude_command(session, model=model, dont_ask=dont_ask,
                                               allowed_tools='mcp__hp_sim5_program,Read,Glob,Grep,Bash',
                                               codex_args=client_args)
        provider = None
    return {'argv': command, 'instructions': instructions, 'attachment': config,
            'backend': backend, 'model_provider': provider, 'physics_world': False}


def main(argv, root):
    root = Path(root).resolve()
    parser = Parser(description=__doc__)
    parser.add_argument('--store', type=Path, default=root / '.research-programs')
    # The legacy store CLI retains create/query/export and its existing outputs.
    probe = argparse.ArgumentParser(add_help=False)
    probe.add_argument('--store')
    _, remainder = probe.parse_known_args(argv[:argv.index('--')] if '--' in argv else argv)
    command = remainder[0] if remainder else ''
    if command == 'tool':
        parser.add_argument('command', choices=['tool'])
        parser.add_argument('connection', type=Path)
        parser.add_argument('operation')
        parser.add_argument('--params', default='{}')
        args = parser.parse_args(argv)
        try:
            result = request(read_json(args.connection), args.operation, json.loads(args.params))
        except (OSError, ValueError, TypeError, KeyError) as error:
            result = failure('invalid_request', str(error))
        print(_json(result))
        return 0 if result['ok'] else 2
    backend = parser.add_mutually_exclusive_group()
    backend.add_argument('--codex', action='store_const', const='codex', dest='backend')
    backend.add_argument('--claude', action='store_const', const='claude', dest='backend')
    entry = parser.add_mutually_exclusive_group(required=True)
    entry.add_argument('--question', help='Create a program from this question')
    entry.add_argument('--program', help='Open the existing program ID')
    parser.add_argument('--constraints', type=json.loads, default={})
    parser.add_argument('--serve', action='store_true', help='Print manager MCP attachment paths; leave the owned host running')
    parser.add_argument('--dry-run', action='store_true', help='Plan without creating files, host, clients or workers')
    parser.add_argument('--model')
    parser.add_argument('--model-provider', default='openai')
    parser.add_argument('--dont-ask', action='store_true')
    split = argv.index('--') if '--' in argv else len(argv)
    args = parser.parse_args(argv[:split])
    client_args = argv[split + 1:]
    if (args.serve and args.backend) or (client_args and not args.backend):
        parser.error('--serve prints attachment configuration; forwarded client arguments require --codex or --claude')
    if args.backend == 'claude' and any(a.startswith('--model-provider') for a in argv[:split]):
        parser.error('--model-provider requires --codex')
    try:
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
        if not args.dry_run:
            session = Path(attachment['session'])
            config = attachment_config(root, attachment['connection'])
            (session / 'attachment.json').write_text(_json(config) + '\n')
            server = config['mcpServers']['hp_sim5_program']
            (session / 'attachment.toml').write_text(
                '[mcp_servers.hp_sim5]\nenabled = false\n[mcp_servers.rerun]\nenabled = false\n'
                '[mcp_servers.hp_sim5_program]\n' + '\n'.join(
                    f'{key} = {coding_client.toml_literal(value)}' for key, value in dict(server, cwd=str(root)).items()) +
                '\nrequired = true\nstartup_timeout_sec = 30\ntool_timeout_sec = 30\ndefault_tools_approval_mode = "approve"\n')
            if plan:
                (session / 'instructions.md').write_text(plan['instructions'])
                (session / 'launch.json').write_text(_json(plan) + '\n')
        result = {'ok': True, 'result': dict(opened, client=plan, dry_run=args.dry_run, physics_world=False)}
        if not args.backend or args.dry_run:
            print(_json(result))
            return 0
        print(_json(result), file=sys.stderr, flush=True)
        environment = (coding_client.codex_environment(plan['model_provider'], root=root, env_file=root / '.env')
                       if args.backend == 'codex' else coding_client.claude_environment(env_file=root / '.env'))
        return subprocess.run(plan['argv'] + ['Open the bound program overview and manage this question.'],
                              cwd=root, env=environment).returncode
    except (OSError, ValueError, sqlite3.Error, subprocess.SubprocessError) as error:
        print(_json(failure('operation_failed', str(error))))
        return 2
