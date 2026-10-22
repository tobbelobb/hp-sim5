"""Real acceptance via MCP from a socket-denied client; not an offline replay."""
import asyncio
import ctypes
import errno
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
DIRECTORY = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'src/python'))


def deny_client_sockets():
    lib = ctypes.CDLL('libseccomp.so.2', use_errno=True)
    lib.seccomp_init.argtypes = [ctypes.c_uint32]
    lib.seccomp_init.restype = ctypes.c_void_p
    lib.seccomp_syscall_resolve_name.argtypes = [ctypes.c_char_p]
    lib.seccomp_rule_add.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_int, ctypes.c_uint]
    lib.seccomp_load.argtypes = [ctypes.c_void_p]
    lib.seccomp_release.argtypes = [ctypes.c_void_p]
    ctx = lib.seccomp_init(0x7fff0000)  # allow except explicitly denied syscalls
    assert ctx
    try:
        for name in (b'socket', b'connect'):
            assert lib.seccomp_rule_add(ctx, 0x00050000 | errno.EPERM,
                                       lib.seccomp_syscall_resolve_name(name), 0) == 0
        assert lib.seccomp_load(ctx) == 0
    finally:
        lib.seccomp_release(ctx)
    try:
        socket.socket()
    except PermissionError:
        print('Client sockets denied by seccomp; MCP process remains outside that restriction.', flush=True)
    else:
        raise AssertionError('Socket denial was not effective')


async def client_run():
    from mcp import Client, StdioServerParameters
    from rerun.chunk import RrdReader
    params = StdioServerParameters(command=str(ROOT / '.venv/bin/python'),
                                  args=[str(ROOT / 'scripts/hp_sim5_mcp.py')],
                                  env={key: os.environ[key] for key in ('PATH', 'HP_SIM5_RUNTIME_URL', 'HP_SIM5_RUNTIME_TOKEN')})
    async with Client(params, read_timeout_seconds=100) as client:
        deny_client_sockets()
        async def call(tool, args):
            result = await client.call_tool(tool, args)
            assert not result.is_error, result.content
            return result.structured_content
        before = await call('runtime_status', {})
        started = time.monotonic()
        job = await call('start_autocal', {'machine': 'hp3', 'base_radii_mm': [30],
                         'buildup_factor': 0, 'layering': False,
                         'collection': {'force_low_n': .01, 'force_mid_n': .23, 'force_max_n': 9.2},
                         'budgets': {'wall_timeout_s': 3600}})
        latency = time.monotonic() - started
        (DIRECTORY / f'mcp-start-{job["job_id"]}.json').write_text(json.dumps({'latency_s': latency, 'job': job}, indent=2))
        print(f'Job {job["job_id"]} returned in {latency:.3f}s', flush=True)
        prior = None
        while True:
            state = await call('autocal_status', {'job_id': job['job_id']})
            progress = state['progress']
            changed = (state['status'], progress.get('iteration'), progress.get('decision'), progress.get('collected_points'))
            if changed != prior:
                print(json.dumps({'job_id': job['job_id'], 'status': state['status'], 'progress': progress}), flush=True)
                prior = changed
            (DIRECTORY / f'mcp-status-{job["job_id"]}.json').write_text(json.dumps(state, indent=2))
            if state['status'] not in ('running', 'cancelling'):
                break
            await asyncio.sleep(10)
        after = await call('runtime_status', {})
        assert before['session_id'] == after['session_id'] and before['step'] == after['step'] == 0
        assert after['machine_design'] == 'hp4'
        assert state['status'] == 'accepted', state
        assert state['exit_code'] == 0 and state['stop_reason'] == 'patience-or-threshold'
        headless = state['headless']
        assert headless['normal_completion'] and headless['recording_drained']
        assert headless['simulated_s'] > 0 and headless['settings'].get('enableLayering', False) is False
        recording, = headless['recordings']
        assert recording['status'] == 'finalized' and recording['physics_samples'] > 0
        assert recording['autocal_complete'] and not recording['rejected_messages']
        assert RrdReader(recording['rrd']).store()
        rows = [json.loads(line) for line in Path(state['artifacts']['sweeps.full_auto_log.jsonl']).read_text().splitlines()]
        decisions = [row['decision'] for row in rows if 'decision' in row]
        assert decisions[-1] == 'accept'
        data = json.loads(Path(state['artifacts']['sweeps.json']).read_text())
        summary = {'job_id': job['job_id'], 'latency_s': latency, 'status': state['status'],
                   'decisions': decisions, 'sweeps': len(data['sweeps']),
                   'points': sum(len(s['data_points']) for s in data['sweeps']),
                   'simulated_s': headless['simulated_s'], 'wall_s': state['wall_s'],
                   'recording': recording, 'attached_hp4_unchanged': True,
                   'restricted_client': 'Linux seccomp EPERM on socket and connect; MCP/supervisor unrestricted',
                   'codex_chat_attachment': 'No research MCP connector is attached to this chat; SDK stdio integration exercised'}
        (DIRECTORY / 'acceptance.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    if '--client' in sys.argv:
        asyncio.run(client_run())
    else:
        from hp_sim5_research.services import RuntimeService
        service = RuntimeService(ROOT, DIRECTORY / 'native', record=False, machine='hp4').start()
        try:
            result = subprocess.run([sys.executable, str(Path(__file__)), '--client'], cwd=ROOT,
                                    env={**os.environ, 'HP_SIM5_RUNTIME_URL': service.endpoint,
                                         'HP_SIM5_RUNTIME_TOKEN': service.token}, timeout=3700)
        finally:
            service.close()
        raise SystemExit(result.returncode)
