"""Launcher-owned native session, RRF and collector; never spawned by agent commands."""
import asyncio
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time
import uuid
from urllib.parse import urlparse
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/python'))
sys.path.insert(0, str(ROOT))

from hp_sim5_research.experiments import encode, write_json
from hp_sim5_research.services import free_port, request, stop_process, wait_ready
from hp_sim5_research.session import NativeSession


class Runtime:
    def __init__(self, endpoint, directory):
        self.endpoint, self.directory = endpoint, Path(directory)
        self.token = os.environ['HP_SIM5_RUNTIME_TOKEN']
        self.processes = {}
        self.session = self.socket_task = None
        self.stop = asyncio.Event()
        self.busy = False
        self.handlers = set()
        self.browser_url = None
        self.python_source_hash = self.hash_sources([ROOT / 'src/python', ROOT / 'scripts'], '*.py')

    @staticmethod
    def hash_sources(directories, pattern):
        source = hashlib.sha256()
        for path in sorted(path for directory in directories for path in directory.rglob(pattern)):
            source.update(str(path.relative_to(ROOT)).encode() + b'\0' + path.read_bytes())
        return source.hexdigest()

    def spawn(self, name, command):
        log_path = self.directory / f'{name}.log'
        with log_path.open('a') as log:
            process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=log)
        self.processes[name] = process
        return process, log_path

    async def start(self):
        self.session_id = uuid.uuid4().hex
        self.session_dir = self.directory / self.session_id
        self.session = NativeSession(ROOT, self.session_dir)
        self.bridge_source_hash = self.hash_sources([ROOT / 'integrations/rrf', ROOT / 'integrations/shared',
                                                    ROOT / 'autocal/control', ROOT / 'scripts'], '*.*js')
        self.rrf_hash = hashlib.sha256((ROOT / 'RRF/build/rrf_simulator').read_bytes()).hexdigest()
        self.config_hash = hashlib.sha256((ROOT / 'RRF/run/vsd/sys/config_hp4.g').read_bytes()).hexdigest()
        shutil.copyfile(ROOT / 'RRF/run/vsd/sys/config_hp4.g', self.session_dir / 'firmware-config.g')
        rrf_port, ws_port, collector_port = [free_port() for _ in range(3)]
        self.rrf_url = f'http://127.0.0.1:{rrf_port}'
        self.collector_url = f'http://127.0.0.1:{collector_port}'
        self.spawn('rrf', [str(ROOT / 'RRF/build/rrf_simulator'), '--vsd', str(ROOT / 'RRF/run/vsd'),
                           '-c', 'sys/config_hp4.g', '--server', '-p', str(rrf_port)])
        process, log = self.spawn('collector', ['node', 'scripts/research_collector.mjs', self.rrf_url,
                                               str(ws_port), str(collector_port), self.endpoint])
        await asyncio.to_thread(wait_ready, process, self.collector_url, 'status', log, token=self.token, timeout=40)
        self.socket_task = asyncio.create_task(self.session.run_socket(f'ws://127.0.0.1:{ws_port}'))
        await asyncio.wait_for(self.session.connected.wait(), timeout=5)
        self.session.event('session_started', session_id=self.session_id, rrf_url=self.rrf_url)

    def status(self):
        return {**self.session.status(), 'session_id': self.session_id, 'busy': self.busy, 'browser_url': self.browser_url,
                'source_changed': {
                    'python_requires_launcher_restart': self.python_source_hash != self.hash_sources([ROOT / 'src/python', ROOT / 'scripts'], '*.py'),
                    'bridge_requires_session_reset': self.bridge_source_hash != self.hash_sources(
                        [ROOT / 'integrations/rrf', ROOT / 'integrations/shared', ROOT / 'autocal/control', ROOT / 'scripts'], '*.*js')},
                'services': {name: {'pid': process.pid, 'exit_code': process.poll(),
                                    'log': str(self.directory / f'{name}.log')}
                             for name, process in self.processes.items()},
                'artifacts': {'rrd': str(self.session.recording_path),
                              'events': str(self.session_dir / 'events.jsonl'), 'scene': str(self.session_dir / 'scene.usda')}}

    async def collect(self, args):
        configs = args.get('configs', [{'fixed': [2, 3], 'drive': 0, 'sensor': 1}])
        if not isinstance(configs, list) or not 1 <= len(configs) <= 12:
            raise ValueError('Supply 1–12 HP4 sweep configurations')
        for cfg in configs:
            roles = cfg.get('fixed', []) + [cfg.get('drive'), cfg.get('sensor')]
            if len(roles) != 4 or any(type(i) is not int for i in roles) or set(roles) != set(range(4)) or 3 not in cfg['fixed']:
                raise ValueError('HP4 requires two distinct fixed anchors including 3, plus distinct drive and sensor')
        options = args.get('options', {})
        allowed = {'sweepPoints', 'fixedTargets', 'feed', 'forceLow', 'forceMid', 'forceMax',
                   'sensorCollectionForce', 'noiseSamples', 'returnToOrigin', 'projectZeroTension', 'preserveBuildupFactor'}
        if not isinstance(options, dict) or set(options) - allowed:
            raise ValueError(f'Collector options must be among {sorted(allowed)}')
        points = options.get('sweepPoints', 6)
        if type(points) is not int or not 3 <= points <= 100:
            raise ValueError('sweepPoints must be an integer in [3, 100]')
        settling = args.get('settling_timeout_s', 30)
        if not isinstance(settling, (int, float)) or not 1 <= settling <= 120:
            raise ValueError('settling_timeout_s must be in [1, 120]')
        run_id = uuid.uuid4().hex
        directory = self.session_dir / run_id
        directory.mkdir()
        artifacts = {'dataset': str(directory / 'sweeps.json'), 'manifest': str(directory / 'manifest.json'),
                     'events': str(directory / 'events.jsonl'), 'rrd': str(directory / 'recording.rrd'),
                     'scene': str(self.session_dir / 'scene.usda'),
                     'firmware_config': str(self.session_dir / 'firmware-config.g')}
        manifest = {'schema_version': 1, 'kind': 'native_collection',
                    'run_id': run_id, 'session_id': self.session_id, 'status': 'running',
                    'configs': configs, 'options': {**options, 'sweepPoints': points},
                    'start_step': self.session.step, 'dt_s': self.session.dt,
                    'events_start_byte': self.session.events.tell(),
                    'clock': self.session.status()['clock'], 'artifacts': artifacts,
                    'scene_sha256': hashlib.sha256(self.session.frozen_scene.encode()).hexdigest(),
                    'rrf_sha256': self.rrf_hash, 'firmware_config_sha256': self.config_hash,
                    'python_source_sha256': self.python_source_hash, 'bridge_source_sha256': self.bridge_source_hash,
                    'python': sys.executable,
                    'packages': {name: version(name) for name in ('numpy', 'usd-core', 'rerun-sdk', 'websockets')},
                    'git_revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()}
        write_json(directory / 'manifest.json', manifest)
        started = time.monotonic()
        wall_timeout = min(14400, max(900, 900 * len(configs) * points / 3))
        manifest['wall_timeout_s'] = wall_timeout
        self.session.deadline = started + wall_timeout
        try:
            self.session.start_recording(artifacts['rrd'])
            collected = await asyncio.to_thread(request, self.collector_url, 'collect',
                                    {'configs': configs, 'options': manifest['options'],
                                     'settlingTimeoutMs': settling * 1000,
                                     'sweepConfigFile': str(directory / 'configs.txt'), 'outputFile': artifacts['dataset']},
                                    token=self.token, timeout=wall_timeout + 5)
            manifest['collector_options'] = collected['collectionOptions']
            from autocal.json_schema import load_json_file
            from autocal.spool_model import validate_dataset_has_raw_angles
            from autocal.calibrate import _validate_sweep_roles
            data = load_json_file(Path(artifacts['dataset']), schema='sweep_dataset')
            validate_dataset_has_raw_angles(data)
            _validate_sweep_roles(data)
            from hp_sim5_research.validation import validate_collection
            manifest['validation'] = validate_collection(artifacts['dataset'])
            count = sum(len(sweep['data_points']) for sweep in data['sweeps'])
            if not count:
                raise ValueError('Collector returned no measurement points')
            manifest.update(status='complete', sweep_count=len(data['sweeps']), point_count=count,
                            dataset_sha256=hashlib.sha256(Path(artifacts['dataset']).read_bytes()).hexdigest())
        except Exception as error:
            manifest.update(status='failed', error=str(error))
            self.session.error = str(error)
            await asyncio.to_thread(stop_process, self.processes['collector'])
            raise RuntimeError(f'{error}; collection evidence: {artifacts["manifest"]}') from None
        finally:
            manifest.update(end_step=self.session.step, wall_s=time.monotonic() - started)
            self.session.deadline = None
            if manifest['status'] == 'complete':
                self.session.start_recording(self.session_dir / f'live-{uuid.uuid4().hex}.rrd')
            else:
                self.session.recording.flush(timeout_sec=5)
                self.session.recording.disconnect()
                self.session.recording = None
            self.session.events.flush()
            shutil.copyfile(self.session_dir / 'events.jsonl', artifacts['events'])
            manifest['events_end_byte'] = self.session.events.tell()
            manifest['events_sha256'] = hashlib.sha256(Path(artifacts['events']).read_bytes()).hexdigest()
            write_json(directory / 'manifest.json', manifest)
        return manifest

    async def dispatch(self, operation, args):
        if operation == 'status':
            return self.status()
        if operation == 'clock':
            return self.session.status()
        if operation == 'advance':
            seconds = args.get('seconds', 0)
            if isinstance(seconds, bool) or not isinstance(seconds, (int, float)):
                raise ValueError('seconds must be a number')
            return await self.session.advance(seconds * self.session.speed)
        if operation == 'shutdown':
            self.stop.set()
            return {'stopping': True}
        if self.busy:
            raise ValueError('Session has an active operation')
        if operation != 'reset' and (self.session.error or any(p.poll() is not None for p in self.processes.values())):
            raise RuntimeError(f'Session failed; inspect status and reset_session: {self.session.error}')
        self.busy = True
        try:
            if operation == 'collect':
                return await self.collect(args)
            if operation == 'gcode':
                line = args.get('line')
                if not isinstance(line, str) or not line.strip() or len(line) > 1000 or '\n' in line or '\r' in line:
                    raise ValueError('Supply one G-code line of at most 1000 characters')
                return await asyncio.to_thread(request, self.collector_url, 'gcode', {'line': line}, token=self.token)
            if operation == 'step':
                steps = args.get('steps', 1)
                if type(steps) is not int or not 1 <= steps <= 10_000:
                    raise ValueError('steps must be an integer in [1, 10000]')
                result = await self.session.advance(steps * self.session.dt)
                self.session.observe()
                self.session.recording.flush(timeout_sec=5)
                return result
            if operation == 'browser':
                if self.browser_url is None:
                    port = free_port()
                    process, log = self.spawn('vite', ['node', 'node_modules/vite/bin/vite.js',
                                                       '--host', '127.0.0.1', '--port', str(port), '--strictPort'])
                    self.browser_url = f'http://127.0.0.1:{port}/hp-sim5/hp-sim-3d/'
                    def check_vite():
                        deadline = time.monotonic() + 15
                        while time.monotonic() < deadline and process.poll() is None:
                            try:
                                with urlopen(self.browser_url, timeout=1) as response:
                                    if response.status == 200:
                                        return
                            except OSError:
                                time.sleep(.1)
                        raise RuntimeError(f'Vite failed readiness; read {log}')
                    await asyncio.to_thread(check_vite)
                return {'url': self.browser_url, 'log': str(self.directory / 'vite.log')}
            if operation == 'reset':
                await self.close_session()
                await self.start()
                return self.status()
            raise ValueError('Unknown runtime operation')
        finally:
            self.busy = False

    async def connection(self, reader, writer):
        task = asyncio.current_task()
        self.handlers.add(task)
        try:
            await self.handle_http(reader, writer)
        finally:
            self.handlers.discard(task)
            writer.close()

    async def handle_http(self, reader, writer):
        try:
            header = await asyncio.wait_for(reader.readuntil(b'\r\n\r\n'), timeout=5)
            lines = header.decode().split('\r\n')
            headers = {k.lower(): v for k, v in (line.split(': ', 1) for line in lines[1:] if ': ' in line)}
            if headers.get('authorization') != f'Bearer {self.token}':
                raise ValueError('Unauthorized')
            size = int(headers.get('content-length', 0))
            if not 0 <= size <= 1_000_000:
                raise ValueError('Request body too large')
            body = await reader.readexactly(size)
            result = await self.dispatch(lines[0].split()[1].strip('/'), json.loads(body or '{}'))
            status = '200 OK'
        except Exception as error:
            result, status = {'error': str(error)}, '400 Bad Request'
        data = encode(result).encode()
        writer.write(f'HTTP/1.1 {status}\r\nContent-Type: application/json\r\nContent-Length: {len(data)}\r\nConnection: close\r\n\r\n'.encode() + data)
        await writer.drain()
        writer.close()
        await writer.wait_closed()

    async def close_session(self):
        if self.socket_task is not None:
            self.socket_task.cancel()
            await asyncio.gather(self.socket_task, return_exceptions=True)
        for process in reversed(list(self.processes.values())):
            await asyncio.to_thread(stop_process, process)
        self.processes.clear()
        self.browser_url = None
        if self.session is not None:
            self.session.close()
            self.session = None


async def main():
    runtime = Runtime(*sys.argv[1:])
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, runtime.stop.set)
    server = None
    starting = stopping = None
    try:
        starting = asyncio.create_task(runtime.start())
        stopping = asyncio.create_task(runtime.stop.wait())
        done, _ = await asyncio.wait([starting, stopping], return_when=asyncio.FIRST_COMPLETED)
        if stopping in done:
            return
        await starting
        server = await asyncio.start_server(runtime.connection, '127.0.0.1', urlparse(runtime.endpoint).port)
        async with server:
            await runtime.stop.wait()
    finally:
        if server is not None:
            server.close()
        pending = list(runtime.handlers) + [task for task in (starting, stopping) if task is not None]
        for task in pending:
            task.cancel()
        await asyncio.gather(*pending, return_exceptions=True)
        await runtime.close_session()


if __name__ == '__main__':
    asyncio.run(main())
