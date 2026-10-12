"""Non-moving diagnostics for an attached supervisor, separate from doctor."""
import asyncio
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import time
import uuid
from urllib.request import urlopen

from .configuration import world_configuration
from .experiments import write_json
from .services import request, register_test_process
from .test_jobs import MAX_TIMEOUT_S, SUITES, job_environment, retire_group

PREFLIGHT_TIMEOUT_S = 15
COMMAND_NETWORK_UNKNOWN = {
    'outcome': 'unknown', 'context': 'agent_command', 'reason': 'Not tested by the supervisor',
    'next_step': 'Run .venv/bin/python scripts/research_socket_probe.py in the agent command context',
}


def blocked(error, next_step):
    return {'outcome': 'blocked', 'error': str(error), 'next_step': next_step}


async def recorder_probe(root, directory):
    process = None
    try:
        with (directory / 'stdout.log').open('w') as stdout, (directory / 'stderr.log').open('w') as stderr:
            process = subprocess.Popen([str(root / '.venv/bin/python'), 'scripts/research_recorder_probe.py',
                str(directory)], cwd=root, env=job_environment(directory), stdin=subprocess.DEVNULL,
                stdout=stdout, stderr=stderr, start_new_session=True)
        register_test_process(process)
        started = time.monotonic()
        while process.poll() is None:
            if time.monotonic() - started >= 10:
                return blocked('Recorder probe exceeded 10 seconds', 'Inspect isolated recorder.log; check recorder dependencies')
            await asyncio.sleep(.05)
        path = directory / 'result.json'
        if not path.exists():
            return blocked(f'Recorder probe exit={process.returncode}', 'Inspect isolated stdout.log/stderr.log')
        result = json.loads(path.read_text())
        if process.returncode != 0 and result['outcome'] == 'passed':
            return blocked('Recorder probe exited unsuccessfully', 'Inspect isolated recorder logs')
        return result
    finally:
        await retire_group(process)


async def preflight(runtime, root, expected_configuration=None):
    root = Path(root)
    started = time.monotonic()
    directory = runtime.directory / 'preflight' / uuid.uuid4().hex
    directory.mkdir(parents=True, exist_ok=False)
    result = {'kind': 'tool_check', 'context': 'supervisor', 'command_network': dict(COMMAND_NETWORK_UNKNOWN),
              'timeout_s': PREFLIGHT_TIMEOUT_S, 'checks': {}, 'directory': str(directory)}
    checks = result['checks']

    async def check(name, action, next_step):
        try:
            checks[name] = await action()
        except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
            checks[name] = blocked(error, next_step)

    async def services():
        state = runtime.status()
        result['world'] = {key: state[key] for key in ('world_id', 'generation', 'step', 'accepted_settings',
                          'active_job', 'active_autocal_job', 'reset_required')}
        failed = [name for name, service in state['services'].items() if service['exit_code'] is not None]
        if state['reset_required'] or not state['connected'] or failed:
            return blocked(f'World disconnected or unhealthy; exited services: {failed}',
                           'Inspect runtime_status service logs; explicitly cancel active jobs before reset_session')
        return {'outcome': 'passed', 'connected': True, 'services': state['services']}

    async def settings():
        config = runtime.configuration
        # Revalidate sources and intended complete fresh-world configuration, without preparing a world.
        validated = world_configuration(root, json.loads(config.requested_json), backend=config.backend)
        state = runtime.status()
        changes = [key for key, changed in state['source_changed'].items() if changed]
        metadata = state['world_configuration']
        effective = json.loads(json.dumps(validated.effective()))
        if changes or json.loads(json.dumps(metadata.get('effective_settings'))) != effective:
            return blocked(f'World settings/source mismatch: {changes}',
                           'Inspect runtime_status; restart launcher for Python changes, otherwise explicitly reset/configure')
        if expected_configuration is not None:
            expected = world_configuration(root, expected_configuration, backend=config.backend)
            if expected.effective() != validated.effective():
                return blocked('Attached world does not match expected_configuration',
                               'Use configure_session after explicitly cancelling active collection/autocal jobs')
        return {'outcome': 'passed', 'effective_settings': effective}

    async def collector():
        if not getattr(runtime, 'collector_url', None):
            return blocked('Collector is unavailable', 'Inspect runtime_status and collector.log')
        state = await asyncio.to_thread(request, runtime.collector_url, 'status', token=runtime.token, timeout=2)
        if not state.get('connected'):
            return blocked('Collector bridge has no ready physics client', 'Inspect collector/physics logs and runtime_status')
        firmware = state.get('firmware_initial', {})
        if ('RepRapFirmware' not in firmware.get('M115', '') or
                'Kinematics is Hangprinter' not in firmware.get('M669', '') or
                not firmware.get('M666', '').startswith('M666 A1')):
            return blocked('Collector did not identify matched Hangprinter firmware', 'Inspect world manifest and collector.log')
        return {'outcome': 'passed', 'connected': True, 'busy': state.get('busy'), 'firmware_initial': firmware}

    async def firmware():
        if not getattr(runtime, 'rrf_url', None):
            return blocked('Firmware is unavailable', 'Inspect runtime_status and rrf.log')
        def read_state(key):
            with urlopen(runtime.rrf_url + f'/rr_model?key={key}&flags=d99', timeout=2) as response:
                return json.load(response)
        state = await asyncio.to_thread(read_state, 'state')
        if not isinstance(state, dict) or not isinstance(state.get('result'), dict):
            return blocked('Unexpected firmware object-model reply', 'Inspect rrf.log and matched RRF build')
        if state['result'].get('status') == 'halted':
            return blocked('Firmware is halted', 'Inspect rrf.log before explicitly recovering the world')
        kinematics = await asyncio.to_thread(read_state, 'move.kinematics')
        current = kinematics.get('result') if isinstance(kinematics, dict) else None
        if not isinstance(current, dict) or str(current.get('name')).lower() != 'hangprinter':
            return blocked('Live firmware kinematics is not Hangprinter',
                           'Inspect live firmware settings; explicitly recover the matched world before collection')
        initial = checks.get('collector', {}).get('firmware_initial')
        if initial is not None:
            expected = re.findall(r'^[ABCD]:\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\s*$',
                                  initial['M669'], re.MULTILINE)
            anchors = current.get('anchors')
            if (len(expected) != 4 or not isinstance(anchors, list) or len(anchors) < 4 or any(
                    not isinstance(actual, list) or len(actual) != 3 or any(
                        type(value) not in (int, float) or not math.isfinite(value) or abs(float(wanted) - value) > .1
                        for wanted, value in zip(target, actual))
                    for target, actual in zip(expected, anchors[:4]))):
                return blocked('Live firmware anchors differ from the attached world configuration',
                               'Inspect live M669/settings; explicitly recover matching anchors before collection')
        return {'outcome': 'passed', 'http_reachable': True, 'kinematics': current}

    async def capabilities():
        missing = [str(path.relative_to(root)) for path in
                   [root / '.venv/bin/python', root / 'RRF/build/rrf_simulator', root / 'autocal/autocal.py',
                    *(root / path for paths in SUITES.values() for path in paths)] if not path.is_file()]
        if shutil.which('node') is None:
            missing.append('node (PATH)')
        tools = {'full_auto': ['start_autocal', 'autocal_status', 'cancel_autocal'],
                 'configuration': ['configure_session', 'reset_session'],
                 'tests': ['start_test', 'test_status', 'cancel_test'], 'test_suites': SUITES,
                 'test_max_timeout_s': MAX_TIMEOUT_S}
        return {**tools, **(blocked(f'Missing prerequisites: {missing}', 'Restore the repository build/interpreter/suite files')
                           if missing else {'outcome': 'passed'})}

    actions = [('services', services), ('settings', settings), ('collector', collector),
               ('firmware', firmware), ('capabilities', capabilities),
               ('recorder', lambda: recorder_probe(root, directory))]
    try:
        async with asyncio.timeout(PREFLIGHT_TIMEOUT_S):
            for name, action in actions:
                await check(name, action, f'Inspect runtime_status and preflight logs for {name}')
    except TimeoutError:
        result['timeout'] = True
    for name, _ in actions:
        checks.setdefault(name, blocked('Preflight time budget exhausted', 'Inspect preflight logs and retry after fixing connectivity'))
    result.update(outcome='passed' if all(item['outcome'] == 'passed' for item in checks.values()) else 'blocked',
                  wall_s=time.monotonic() - started,
                  artifacts={str(path.relative_to(directory)): str(path) for path in directory.rglob('*') if path.is_file()},
                  manifest=str(directory / 'preflight.json'))
    write_json(directory / 'preflight.json', result)
    return result
