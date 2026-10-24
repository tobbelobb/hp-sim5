#!/usr/bin/env python3
"""Install the opt-in cloud client wrapper without touching model credentials."""
import os
from pathlib import Path
import shlex


def install(root, tools):
    python = root / '.venv/bin/python'
    if not python.is_file():
        raise SystemExit('Create the repository .venv first with scripts/cloud/install.sh')
    wrapper = tools / 'cloud-research-client/bin/codex'
    command = [str(python), str(root / 'scripts/cloud/cloud_client_bridge.py'), 'client']
    content = '#!/usr/bin/env bash\nexec ' + shlex.join(command) + ' "$@"\n'
    # Upgrade only the exact generated wrapper used by the original live pilot.
    old_command = [str(python), str(root / 'output/cloud-research/cloud_client_bridge.py'), 'client']
    legacy = '#!/usr/bin/env bash\nexec ' + shlex.join(old_command) + ' "$@"\n'
    if wrapper.is_symlink():
        raise SystemExit(f'Preserving existing symlink: {wrapper}')
    if wrapper.exists() and wrapper.read_text() not in (content, legacy):
        raise SystemExit(f'Preserving modified cloud-client wrapper: {wrapper}')
    wrapper.parent.mkdir(parents=True, exist_ok=True)
    temporary = wrapper.with_name(f'.codex-{os.getpid()}.tmp')
    temporary.write_text(content)
    temporary.chmod(0o755)
    temporary.replace(wrapper)
    print(f'Installed cloud-session transport: {wrapper}')
    print('Source scripts/cloud/research-env.sh in the authenticated parent chat.')


if __name__ == '__main__':
    install(Path(__file__).resolve().parents[2],
            Path(os.environ.get('HP_SIM5_CLOUD_TOOLS', '/workspace/.tools')).resolve())
