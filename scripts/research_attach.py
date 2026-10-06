"""Attach one Codex chat to an explicitly selected, live research supervisor."""
import fcntl
import json
import os
import signal
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/python'))
from hp_sim5_research.services import request, stop_process


def interrupted(_signum, _frame):
    raise KeyboardInterrupt


def main():
    signal.signal(signal.SIGTERM, interrupted)
    if len(sys.argv) < 2:
        raise ValueError('Expected DESCRIPTOR [--rerun]')
    descriptor = Path(sys.argv[1]).resolve()
    rerun = sys.argv[2:] == ['--rerun']
    if sys.argv[2:] and not rerun:
        raise ValueError('Expected DESCRIPTOR [--rerun]')
    if descriptor.stat().st_mode & 0o077:
        raise ValueError('Session descriptor must be private (chmod 600)')
    config = json.loads(descriptor.read_text())
    if Path(config['repo']).resolve() != ROOT:
        raise ValueError('Descriptor belongs to a different worktree')
    # One mutable world per chat. Explicitly refuse a second MCP connection.
    with descriptor.with_suffix('.lease').open('w') as lease:
        if not rerun:
            try:
                fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError('Session is attached to another chat; start a separate supervisor') from None
        status = request(config['runtime_endpoint'], 'status', token=config['runtime_token'], timeout=5)
        if status['session_id'] != config['session_id']:
            # Coordinated reset changes world identity, but keeps the same supervisor endpoint/token.
            print('Research world was reset; attaching to its fresh session.', file=sys.stderr)
        env = {**os.environ, 'HP_SIM5_RUNTIME_URL': config['runtime_endpoint'],
               'HP_SIM5_RUNTIME_TOKEN': config['runtime_token'],
               'HP_SIM5_SESSION_DIR': str(descriptor.parent)}
        command = ([str(ROOT / '.venv/bin/rerun'), 'viewer-mcp', '--endpoint', config['viewer_endpoint']]
                   if rerun else [sys.executable, str(ROOT / 'scripts/hp_sim5_mcp.py')])
        if rerun and not config['viewer_endpoint']:
            raise ValueError('Session has no Viewer; start the supervisor with --viewer headless or window')
        child = subprocess.Popen(command, env=env)
        try:
            return child.wait()
        finally:
            stop_process(child)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except (OSError, ValueError, RuntimeError) as error:
        sys.exit(f'hp-sim5 attachment: {error}; restart --serve if the supervisor ended')
