"""Coding-client argv and existing authentication shared by workers and reviewers.

No model API, credential provisioning or physics runtime is introduced here.
"""
import json
import os
from pathlib import Path
import tomllib


CHATGPT_SIGN_IN = 'openai'
STRIPPED_KEYS = ('OPENAI_API_KEY', 'CODEX_API_KEY')


def forwarded_settings(codex_args):
    # Effective overrides and model from forwarded argv; later wins.
    settings, model = {}, None
    arguments = iter(codex_args)
    for argument in arguments:
        if argument in ('-c', '--config'):
            override = next(arguments, None)
        elif argument.startswith('--config='):
            override = argument.split('=', 1)[1]
        else:
            override = None
        if override and '=' in override:
            key, value = override.split('=', 1)
            settings[key.strip()] = value.strip().strip('"')
        elif argument in ('-m', '--model'):
            model = next(arguments, None)
        elif argument.startswith('--model='):
            model = argument.split('=', 1)[1].strip().strip('"')
    return settings, model


def provider_env_key(provider, *, root):
    # A configured provider names the environment variable holding its credential.
    if not provider:
        return None
    candidates = [root / '.codex/config.toml',
                  Path(os.environ.get('CODEX_HOME', Path.home() / '.codex')) / 'config.toml']
    for path in candidates:
        try:
            providers = tomllib.loads(path.read_text()).get('model_providers', {})
        except (OSError, tomllib.TOMLDecodeError):
            continue
        variable = providers.get(provider, {}).get('env_key')
        if variable:
            return variable
    return None


def dotenv_values(path):
    # Read plain KEY=VALUE lines without executing the file or touching the environment.
    values = {}
    try:
        lines = path.read_text().splitlines()
    except OSError:
        return values
    for line in lines:
        line = line.strip()
        if line.startswith('export '):
            line = line[7:].lstrip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, _, value = line.partition('=')
        key, value = key.strip(), value.strip().strip('"').strip("'")
        if key and value:
            values[key] = value
    return values


def codex_environment(provider=CHATGPT_SIGN_IN, *, root, env_file):
    # ChatGPT sign-in needs no OpenAI key. A selected API provider gets its configured
    # env_key from the Git-ignored .env when the caller has not already exported it.
    environment = {key: value for key, value in os.environ.items() if key not in STRIPPED_KEYS}
    variable = provider_env_key(provider, root=root)
    if variable and not environment.get(variable):
        value = dotenv_values(env_file).get(variable)
        if value:
            environment[variable] = value
    return environment


def claude_environment():
    # A key in the repo .env must not silently override Claude Code's account
    # login. API billing is opt-in through an explicitly exported key.
    environment = dict(os.environ)
    # Match the supervisor's long-running experiment tool budget (milliseconds).
    environment['MCP_TOOL_TIMEOUT'] = '14500000'
    return environment


def toml_literal(value):
    if isinstance(value, dict):
        return '{ ' + ', '.join(f'{json.dumps(key)} = {toml_literal(item)}' for key, item in value.items()) + ' }'
    if isinstance(value, list):
        return '[' + ', '.join(toml_literal(item) for item in value) + ']'
    return json.dumps(value)


def disabled_research_mcp_settings(root):
    # Codex validates transport even for disabled servers, including with no user config.
    # Replace each whole table so an inherited HTTP URL cannot conflict with stdio.
    return {'mcp_servers.hp_sim5': {'enabled': False, 'command': str(root / '.venv/bin/python'),
                                     'args': [str(root / 'scripts/hp_sim5_mcp.py')]},
            'mcp_servers.rerun': {'enabled': False, 'command': str(root / '.venv/bin/rerun'),
                                  'args': ['viewer-mcp']},
            'mcp_servers.hp_sim5_program': {'enabled': False, 'command': str(root / '.venv/bin/python'),
                                            'args': [str(root / 'research-program/cli.py')]}}


def writable_roots(root, session):
    from .workspace import common_root
    roots = [(root / 'output').resolve(), (root / 'research/output').resolve(),
             common_root(root) / '.research-state']
    session = session.resolve()
    if not any(session.is_relative_to(item) for item in roots):
        roots.append(session)
    return [str(path) for path in roots]


def codex_command(root, session, *, batch=False, dont_ask=False, goal_mode=False,
                  codex_args=(), instructions='', model_provider=CHATGPT_SIGN_IN,
                  model=None, settings=None, sandbox='workspace-write'):
    command = ['codex', 'exec'] if batch else ['codex']
    command += ['--cd', str(root), '--sandbox', sandbox, '-c', 'approval_policy="never"']
    options = {'sandbox_workspace_write.network_access': True,
               'sandbox_workspace_write.writable_roots': writable_roots(root, session),
               'features.default_mode_request_user_input': not (dont_ask or batch),
               'features.goals': goal_mode}
    options.update(settings or {})
    if model_provider and 'model_provider' not in forwarded_settings(codex_args)[0]:
        options['model_provider'] = model_provider
    for key, value in options.items():
        command += ['-c', f'{key}={toml_literal(value)}']
    if model and not forwarded_settings(codex_args)[1]:
        command += ['--model', model]
    if instructions:
        command += ['-c', f'developer_instructions={json.dumps(instructions)}']
    if batch:
        return command + list(codex_args) + ['--json', '--output-last-message', str(session / 'final-message.md'), '-']
    return command + list(codex_args)


def claude_command(session, *, batch=False, dont_ask=False, goal_mode=False,
                   codex_args=(), model=None, allowed_tools='mcp__hp_sim5,mcp__rerun', **_):
    command = ['claude', '--mcp-config', str(session / 'attachment.json'), '--strict-mcp-config',
               '--append-system-prompt-file', str(session / 'instructions.md'),
               '--allowedTools', allowed_tools,
               '--settings', json.dumps({'modelProposedGoals': 'auto' if goal_mode else 'disabled'})]
    if dont_ask or batch:
        command += ['--disallowedTools', 'AskUserQuestion']
    if batch:
        command += ['--print', '--output-format', 'stream-json', '--verbose']
    if model:
        command += ['--model', model]
    return command + list(codex_args)
