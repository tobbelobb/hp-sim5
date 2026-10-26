"""Reload tool paths at cloud process boundaries, including direct Python launches."""
import os
from pathlib import Path
import subprocess


# Import only setup-owned settings. Never serialize credentials or replace the
# inherited client transport, proxy, or authentication configuration.
SETTINGS = (
    'HP_SIM5_ROOT', 'HP_SIM5_CLOUD_TOOLS', 'HP_SIM5_CLOUD_CACHE',
    'HP_SIM5_NODE_VERSION', 'HP_SIM5_RUST_VERSION', 'CARGO_HOME', 'RUSTUP_HOME',
    'PATH', 'XDG_CACHE_HOME', 'MPLCONFIGDIR', 'WARP_CACHE_PATH', 'PIP_CACHE_DIR',
    'npm_config_cache', 'PUPPETEER_EXECUTABLE_PATH', 'PUPPETEER_SKIP_DOWNLOAD',
    'PLAYWRIGHT_MCP_CONFIG', 'OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS',
)


def activate(root):
    if os.environ.get('HP_SIM5_RESEARCH_CLIENT', 'local') != 'cloud':
        return
    script = Path(root) / 'scripts/cloud/env.sh'
    result = subprocess.run(
        ['bash', '-c', 'set -e; source "$1"; shift; '
         'for name in "$@"; do printf "%s\\0%s\\0" "$name" "${!name}"; done',
         'hp-sim5-cloud-env', str(script), *SETTINGS],
        check=True, capture_output=True, timeout=10)
    fields = result.stdout.decode().split('\0')
    if fields[-1] != '' or fields[::2][:-1] != list(SETTINGS):
        raise RuntimeError('Unexpected cloud environment setup output')
    os.environ.update(zip(fields[:-1:2], fields[1::2]))
