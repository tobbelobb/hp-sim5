"""Default-force production autocal through MCP with client sockets denied."""
import asyncio
import ctypes
import errno
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
DIRECTORY = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'src/python'))


def deny_sockets():
    lib = ctypes.CDLL('libseccomp.so.2', use_errno=True)
    lib.seccomp_init.argtypes = [ctypes.c_uint32]
    lib.seccomp_init.restype = ctypes.c_void_p
    lib.seccomp_syscall_resolve_name.argtypes = [ctypes.c_char_p]
    lib.seccomp_rule_add.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_int, ctypes.c_uint]
    lib.seccomp_load.argtypes = [ctypes.c_void_p]
    lib.seccomp_release.argtypes = [ctypes.c_void_p]
    ctx = lib.seccomp_init(0x7fff0000)
    assert ctx
    try:
        for name in (b'socket', b'connect'):
            assert lib.seccomp_rule_add(ctx, 0x00050000 | errno.EPERM, lib.seccomp_syscall_resolve_name(name), 0) == 0
        assert lib.seccomp_load(ctx) == 0
    finally:
        lib.seccomp_release(ctx)
    try:
        socket.socket()
    except PermissionError:
        return
    raise AssertionError('Client socket denial ineffective')


def anchors(reply):
    return [[float(value) for value in match] for match in
            re.findall(r'^[ABCD]:\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)', reply, re.M)]


async def client_run():
    from mcp import Client, StdioServerParameters
    from rerun.chunk import RrdReader
    params = StdioServerParameters(command=str(ROOT / '.venv/bin/python'),
        args=[str(ROOT / 'scripts/hp_sim5_mcp.py')],
        env={key: os.environ[key] for key in ('PATH', 'HP_SIM5_RUNTIME_URL', 'HP_SIM5_RUNTIME_TOKEN')})
    async with Client(params, read_timeout_seconds=180) as client:
        deny_sockets()
        async def call(tool, args):
            result = await client.call_tool(tool, args)
            assert not result.is_error, result.content
            return result.structured_content
        before = await call('runtime_status', {})
        started = time.monotonic()
        job = await call('start_autocal', {})
        latency = time.monotonic() - started
        (DIRECTORY / 'E12-mcp-start.json').write_text(json.dumps({'latency_s': latency, 'job': job}, indent=2))
        print(f'Job {job["job_id"]}; start latency {latency:.6f}s', flush=True)
        prior = None
        while True:
            state = await call('autocal_status', {'job_id': job['job_id']})
            progress = state['progress']
            changed = (state['status'], progress.get('iteration'), progress.get('decision'), progress.get('collected_points'))
            if changed != prior:
                print(json.dumps({'status': state['status'], 'progress': progress}), flush=True)
                prior = changed
            (DIRECTORY / 'E12-mcp-status.json').write_text(json.dumps(state, indent=2))
            if state['status'] not in ('running', 'cancelling'):
                break
            await asyncio.sleep(10)
        after = await call('runtime_status', {})
        assert before['session_id'] == after['session_id'] and before['step'] == after['step'] == 0
        assert before['machine_design'] == after['machine_design'] == 'hp4'
        assert state['status'] == 'accepted', state
        assert state['exit_code'] == 0 and state['stop_reason'] == 'patience-or-threshold'
        headless = state['headless']
        assert headless['normal_completion'] and headless['recording_drained']
        assert headless['settings']['enableLayering'] is False
        recording, = headless['recordings']
        assert recording['status'] == 'finalized' and recording['physics_samples'] > 0
        assert recording['autocal_complete'] and not recording['rejected_messages']
        assert RrdReader(recording['rrd']).store()
        baked = Path(state['directory']) / 'sweeps.headless' / headless['baked_scene_file']
        assert hashlib.sha256(baked.read_bytes()).hexdigest() == headless['baked_scene_sha256']
        data = json.loads(Path(state['artifacts']['sweeps.json']).read_text())
        points = [point for sweep in data['sweeps'] for point in sweep['data_points']]
        truth = anchors(headless['firmware_initial']['M669'])
        estimated = anchors(headless['firmware_final']['M669'])
        assert len(truth) == len(estimated) == 4
        error = [sum((x-y)**2 for x, y in zip(a,b))**.5 for a,b in zip(truth, estimated)]
        rmse = (sum(value**2 for value in error)/4)**.5
        rows = [json.loads(line) for line in Path(state['artifacts']['sweeps.full_auto_log.jsonl']).read_text().splitlines()]
        summary = {'job_id': job['job_id'], 'latency_s': latency, 'status': state['status'],
            'decisions': [row['decision'] for row in rows if 'decision' in row],
            'sweeps': len(data['sweeps']), 'points': len(points), 'simulated_s': headless['simulated_s'],
            'wall_s': state['wall_s'], 'sample_counts': sorted(set(point['sample_count'] for point in points)),
            'max_collection_attempt': max(point.get('collection_attempt', 0) for point in points),
            'max_fixed_error_mu_deg': max(abs(value) for point in points for value in point.get('fixed_error_angles_mu_deg', []) if value is not None),
            'true_anchors_mm': truth, 'accepted_anchors_mm': estimated, 'anchor_errors_mm': error, 'anchor_rmse_mm': rmse,
            'force_tuning': data['config']['force_tuning'], 'recording': recording,
            'attached_hp4_unchanged': True, 'restricted_client': 'seccomp EPERM on socket/connect; stdio MCP and supervisor unrestricted',
            'codex_attachment': 'SDK stdio verification; no research connector attached to this chat'}
        (DIRECTORY / 'E12-acceptance.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(json.dumps(summary), flush=True)
        assert rmse < 5, f'Accepted geometry RMSE {rmse:.3f} mm exceeds 5 mm research criterion'
        for child in headless['owned_services']:
            try:
                os.kill(child['pid'], 0)
            except ProcessLookupError:
                continue
            raise AssertionError(f'Owned child still exists: {child}')


if __name__ == '__main__':
    if '--client' in sys.argv:
        asyncio.run(client_run())
    else:
        from hp_sim5_research.services import RuntimeService
        service = RuntimeService(ROOT, DIRECTORY / 'E12-native', record=False, machine='hp4').start()
        try:
            result = subprocess.run([sys.executable, str(Path(__file__)), '--client'], cwd=ROOT,
                env={**os.environ, 'HP_SIM5_RUNTIME_URL': service.endpoint, 'HP_SIM5_RUNTIME_TOKEN': service.token}, timeout=3900)
        finally:
            service.close()
        raise SystemExit(result.returncode)
