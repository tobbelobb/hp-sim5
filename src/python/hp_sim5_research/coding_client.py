"""Coding-client argv and existing authentication shared by workers and reviewers.

No model API, credential provisioning or physics runtime is introduced here.
"""
import json
import os
from pathlib import Path
import sys
import tomllib

CHATGPT_SIGN_IN = 'openai'
STRIPPED_KEYS = ('OPENAI_API_KEY', 'CODEX_API_KEY')


def research_client():
    """Cloud chats use their subagents; local terminals use installed clients."""
    selected = os.environ.get('HP_SIM5_RESEARCH_CLIENT', 'auto')
    if selected not in ('auto', 'local', 'cloud'):
        raise ValueError('HP_SIM5_RESEARCH_CLIENT must be local, cloud or auto')
    cloud = (os.environ.get('CODEX_CLOUD') == '1' or
             os.environ.get('CODEX_HOME', '').startswith('/run/codex-environment/'))
    if cloud and selected == 'local':
        raise ValueError('Standalone research clients are disabled in cloud chats; use the current chat and --cloud')
    return 'cloud' if cloud or selected == 'cloud' else 'local'


def cloud_command(root, session, sandbox):
    return [sys.executable, str(root / 'scripts/cloud/cloud_client_bridge.py'), 'client',
            '--cd', str(root), '--output-last-message', str(session / 'final-message.md'),
            '--sandbox', sandbox]


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
    if research_client() == 'cloud':
        return environment
    variable = provider_env_key(provider, root=root)
    if variable and not environment.get(variable):
        value = dotenv_values(env_file).get(variable)
        if value:
            environment[variable] = value
    return environment


def claude_environment(*, env_file):
    environment = dict(os.environ)
    if not environment.get('ANTHROPIC_API_KEY'):
        key = dotenv_values(env_file).get('ANTHROPIC_API_KEY')
        if key:
            environment['ANTHROPIC_API_KEY'] = key
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
                                            'args': [str(root / 'scripts/research_program.py')]}}


def codex_command(root, session, *, batch=False, dont_ask=False, goal_mode=False,
                  codex_args=(), instructions='', model_provider=CHATGPT_SIGN_IN,
                  model=None, settings=None, sandbox='workspace-write'):
    if research_client() == 'cloud':
        if not batch:
            raise ValueError('The current cloud chat is the manager. Use --cloud --question TEXT or --cloud --program ID, without --codex')
        if model or model_provider not in (None, CHATGPT_SIGN_IN) or codex_args:
            raise ValueError('Cloud subagents inherit the parent client configuration; do not supply CLI model/provider arguments')
        return cloud_command(root, session, sandbox)
    command = ['codex'] + (['exec'] if batch else [])
    command += ['--cd', str(root), '--sandbox', sandbox, '-c', 'approval_policy="never"']
    if batch:
        command += ['--json', '--output-last-message', str(session / 'final-message.md')]
    options = {'sandbox_workspace_write.network_access': True,
               'sandbox_workspace_write.writable_roots': [str((root / 'output').resolve())],
               'features.default_mode_request_user_input': not (dont_ask or batch),
               'features.goals': goal_mode}
    options.update(settings or {})
    if model_provider:
        options['model_provider'] = model_provider
    for key, value in options.items():
        command += ['-c', f'{key}={toml_literal(value)}']
    if model:
        command += ['--model', model]
    if instructions:
        command += ['-c', f'developer_instructions={json.dumps(instructions)}']
    return command + list(codex_args) + (['-'] if batch else [])


def claude_command(session, *, batch=False, dont_ask=False, goal_mode=False,
                   codex_args=(), model=None, allowed_tools='mcp__hp_sim5,mcp__rerun', **_):
    if research_client() == 'cloud':
        raise ValueError('Cloud research uses authenticated subagents with backend codex, without another coding CLI')
    command = ['claude', '--mcp-config', str(session / 'attachment.json'), '--strict-mcp-config',
               '--append-system-prompt-file', str(session / 'instructions.md'),
               '--allowedTools', allowed_tools,
               '--settings', json.dumps({'modelProposedGoals': 'auto' if goal_mode else 'disabled'})]
    if dont_ask:
        command += ['--disallowedTools', 'AskUserQuestion']
    if batch:
        command += ['--print', '--output-format', 'stream-json', '--verbose']
    if model:
        command += ['--model', model]
    return command + list(codex_args)
