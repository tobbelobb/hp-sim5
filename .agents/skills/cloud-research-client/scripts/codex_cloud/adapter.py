"""Codex Cloud client policy, commands and startup workarounds.

The existing launcher/controller still own execution. The active authenticated
chat brokers actual model actors through .agents/skills/cloud-research-client/scripts/cloud_client_bridge.py.
"""
import os
import sys

from .environment import activate
from program.store import ProgramError, _json


def codex_command(root, session, *, batch=False, codex_args=(), model=None,
                  model_provider='openai', sandbox='workspace-write', **_):
    if not batch:
        raise ValueError('The current cloud chat is the manager. Use --cloud --question TEXT or --cloud --program ID, without --codex')
    if model or model_provider not in (None, 'openai') or codex_args:
        raise ValueError('Cloud subagents inherit the parent client configuration; do not supply CLI model/provider arguments')
    return [sys.executable, str(root / '.agents/skills/cloud-research-client/scripts/cloud_client_bridge.py'), 'client',
            '--cd', str(root), '--session-dir', str(session),
            '--output-last-message', str(session / 'final-message.md'), '--sandbox', sandbox]


def codex_environment(**_):
    return {key: value for key, value in os.environ.items()
            if key not in ('OPENAI_API_KEY', 'CODEX_API_KEY')}


def claude_command(*_, **__):
    raise ValueError('Cloud research uses authenticated subagents with backend codex, without another coding CLI')


def validate_manager(*, model=None, model_provider='openai', local_client=False):
    if local_client:
        raise ValueError('The current cloud chat is the manager. Use --cloud without --codex or --claude; read .agents/skills/cloud-research-client/SKILL.md')
    if model or model_provider != 'openai':
        raise ValueError('The current cloud chat is the manager. Use --cloud without a coding CLI, model or provider override')


def validate_worker_settings(settings):
    if (settings['backend'] != 'codex' or settings.get('model') or
            settings.get('model_provider', 'openai') != 'openai' or settings['client_args']):
        raise ProgramError('Cloud workers use parent-authenticated subagents; omit CLI model/provider arguments and use backend codex')


def agent_uses_external_client(args, parser):
    if not (args.batch or args.serve or args.preflight or args.doctor):
        parser.error('Use the current cloud chat as manager with hp-sim5-research-program --cloud; standalone interactive clients are disabled here')
    return not args.serve


def runtime_check_command(root, argv):
    # Reload even when the retained host predates installation.
    return ['bash', '-c', 'source "$1/.agents/skills/cloud-research-client/scripts/env.sh" && shift && exec "$@"',
            'research-runtime-check', str(root), *argv]


def runtime_check_result(result):
    if not result['ok']:
        result['next_step'] = (
            'Repair blocked checks before dispatch. For missing dependencies/artifacts run '
            'bash .agents/skills/cloud-research-client/scripts/install.sh; socket permissions require a permitted launcher context.')
    return result


def ensure_runtime_ready(root, settings, check):
    checked = check(root, physics_backend=settings['physics_backend'])
    if not checked['ok']:
        raise ProgramError('Cloud runtime is not ready; no worker attempt was charged. '
                           'Call runtime_check for diagnostics. ' + _json(checked))


def worker_instructions():
    return ('\nCloud worker setup (cloud only): in every shell, run '
            '`source .agents/skills/cloud-research-client/scripts/env.sh` before Python, Node, '
            'native, recorder or physics commands. '
            'Use its Node toolchain. Run simulation jobs one at a time; do not launch concurrent trials.\n')


def check_capacity(workers, reviews):
    if workers or reviews:
        raise ProgramError('Cloud research runs sequentially; wait for the active worker or review to finish')


def check_jobs(operation, active):
    if active:
        raise ValueError(f'Cloud research runs one job at a time; {operation} is blocked by an active '
                         f'{", ".join(active)} job')
