"""Codex Cloud client policy, commands and startup workarounds.

The existing launcher/controller still own execution. The active authenticated
chat brokers actual model actors through scripts/cloud/cloud_client_bridge.py.
"""
import os
import sys

from .environment import activate
from ..program import ProgramError, _json


def codex_command(root, session, *, batch=False, codex_args=(), model=None,
                  model_provider='openai', sandbox='workspace-write', **_):
    if not batch:
        raise ValueError('The current cloud chat is the manager. Use --cloud --question TEXT or --cloud --program ID, without --codex')
    if model or model_provider not in (None, 'openai') or codex_args:
        raise ValueError('Cloud subagents inherit the parent client configuration; do not supply CLI model/provider arguments')
    return [sys.executable, str(root / 'scripts/cloud/cloud_client_bridge.py'), 'client',
            '--cd', str(root), '--session-dir', str(session),
            '--output-last-message', str(session / 'final-message.md'), '--sandbox', sandbox]


def codex_environment(**_):
    return {key: value for key, value in os.environ.items()
            if key not in ('OPENAI_API_KEY', 'CODEX_API_KEY')}


def claude_command(*_, **__):
    raise ValueError('Cloud research uses authenticated subagents with backend codex, without another coding CLI')


def validate_manager(*, model=None, model_provider='openai', local_client=False):
    if local_client:
        raise ValueError('The current cloud chat is the manager. Use --cloud without --codex or --claude; read scripts/cloud/RESEARCH.md')
    if model or model_provider != 'openai':
        raise ValueError('The current cloud chat is the manager. Use --cloud without a coding CLI, model or provider override')


def validate_worker_settings(settings):
    if (settings['backend'] != 'codex' or settings.get('model') or
            settings.get('model_provider', 'openai') != 'openai' or settings['client_args']):
        raise ProgramError('Cloud workers use parent-authenticated subagents; omit CLI model/provider arguments and use backend codex')


def validate_machine(options):
    if 'machine' not in options:
        raise ProgramError('Cloud workers require execution.machine at commission or an explicit machine in worker settings (hp3/hp4)')


def agent_uses_external_client(args, parser):
    if not (args.batch or args.serve or args.preflight or args.doctor):
        parser.error('Use the current cloud chat as manager with hp-sim5-research-program --cloud; standalone interactive clients are disabled here')
    return not args.serve


def runtime_check_command(root, argv):
    # Reload even when the retained host predates installation.
    return ['bash', '-c', 'source "$1/scripts/cloud/env.sh" && shift && exec "$@"',
            'research-runtime-check', str(root), *argv]


def runtime_check_result(result):
    if not result['ok']:
        result['next_step'] = (
            'Repair blocked checks before dispatch. For missing dependencies/artifacts run '
            'bash scripts/cloud/install_research.sh; socket permissions require a permitted launcher context.')
    return result


def ensure_runtime_ready(root, settings, check):
    checked = check(root, physics_backend=settings['physics_backend'])
    if not checked['ok']:
        raise ProgramError('Cloud runtime is not ready; no worker attempt was charged. '
                           'Call runtime_check for diagnostics. ' + _json(checked))
