"""Parent-owned services and the narrow HTTP client used by sandboxed MCP."""
import json
import os
from pathlib import Path
import secrets
import signal
import socket
import subprocess
import sys
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen


def free_port():
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        return listener.getsockname()[1]


def stop_process(process):
    if process is not None and process.poll() is None:
        # Detached services own their process group, including CLI wrapper children.
        try:
            group = os.getpgid(process.pid) == process.pid
            if group:
                os.killpg(process.pid, signal.SIGTERM)
            else:
                process.terminate()
        except ProcessLookupError:
            process.wait()
            return
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            if group:
                os.killpg(process.pid, signal.SIGKILL)
            else:
                process.kill()
            process.wait()


def request(endpoint, operation, args=None, *, token='', timeout=14430):
    data = json.dumps(args or {}).encode()
    query = Request(endpoint + '/' + operation, data=data,
                    headers={'Authorization': f'Bearer {token}', 'Content-Type': 'application/json'})
    try:
        with urlopen(query, timeout=timeout) as response:
            return json.load(response)
    except HTTPError as error:
        payload = json.load(error)
        raise RuntimeError(payload.get('error', str(error))) from None


def wait_ready(process, endpoint, operation, log, *, token='', timeout=30):
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline and process.poll() is None:
        try:
            return request(endpoint, operation, token=token, timeout=1)
        except (OSError, RuntimeError, ValueError) as error:
            last_error = str(error)
            time.sleep(.1)
    raise RuntimeError(f'Service failed readiness ({last_error}); exit={process.poll()}; read {log}')


class RuntimeService:
    def __init__(self, root, directory, viewer_endpoint=None, *, backend='headless-js', record=True, machine='hp4'):
        self.root = Path(root)
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.token = secrets.token_hex(32)
        self.endpoint = f'http://127.0.0.1:{free_port()}'
        self.process = None
        self.viewer_endpoint = viewer_endpoint
        if backend not in ('native-python', 'headless-js', 'native-warp', 'native-warp-cuda'):
            raise ValueError('Unknown physics backend')
        self.backend = backend
        self.record = record
        if machine not in ('hp3', 'hp4'):
            raise ValueError('Machine must be hp3 or hp4')
        self.machine = machine

    def start(self):
        log_path = self.directory / 'runtime.log'
        with log_path.open('w') as log:
            self.process = subprocess.Popen(
                [sys.executable, str(self.root / 'scripts/research_runtime.py'), self.endpoint, str(self.directory)],
                cwd=self.root, env={**os.environ, 'HP_SIM5_RUNTIME_TOKEN': self.token,
                                    'HP_SIM5_PHYSICS_BACKEND': self.backend,
                                    'HP_SIM5_RECORD': '1' if self.record else '0',
                                    'HP_SIM5_MACHINE': self.machine,
                                    'HP_SIM5_VIEWER_URL': self.viewer_endpoint or '',
                                    'MPLCONFIGDIR': str(self.directory / 'matplotlib')},
                stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
        try:
            wait_ready(self.process, self.endpoint, 'status', log_path, token=self.token, timeout=45)
        except Exception:
            self.close()
            raise
        return self

    def call(self, operation, **args):
        return request(self.endpoint, operation, args, token=self.token)

    def close(self):
        if self.process is not None and self.process.poll() is None:
            try:
                request(self.endpoint, 'shutdown', token=self.token, timeout=10)
                self.process.wait(timeout=10)
            except (OSError, RuntimeError, subprocess.TimeoutExpired):
                stop_process(self.process)


def supervised_request(operation, **args):
    endpoint = os.environ.get('HP_SIM5_RUNTIME_URL')
    if not endpoint:
        raise RuntimeError('Start with hp-sim5-research-agent; the launcher supervises native collection services')
    return request(endpoint, operation, args, token=os.environ['HP_SIM5_RUNTIME_TOKEN'])
