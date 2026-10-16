"""Own a flight recorder for a browser autocal run."""
from contextlib import contextmanager
import os
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

from autocal import extended_reference
from autocal._autocal_common import REPO_ROOT
from autocal.headless_sim import free_port


@contextmanager
def browser_recording(dataset):
    from websockets.sync.client import connect

    directory = Path(dataset or 'autocal/data/default_dataset.json').resolve().with_suffix('.recording')
    base = directory
    index = 1
    while directory.exists():
        directory = base.with_name(f'{base.name}_{index}')
        index += 1
    directory.mkdir(parents=True)
    port = free_port()
    url = f'ws://127.0.0.1:{port}'
    previous_url = os.environ.get('AUTOCAL_REFERENCE_WS')
    handlers = {}
    process = None

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    try:
        with (directory / 'recorder.log').open('w') as log:
            process = subprocess.Popen([
                sys.executable, 'scripts/hangprinter_flight_recorder.py',
                '--extended-reference', '--no-viewer', '--port', str(port),
                '--output', str(directory), '--sample-stride', '10', '--geometry-detail', 'compact',
            ], cwd=REPO_ROOT, stdout=log, stderr=log, start_new_session=True)
        for sig in (signal.SIGTERM, signal.SIGHUP):
            handlers[sig] = signal.signal(sig, interrupt)
        deadline = time.monotonic() + 30
        while True:
            try:
                with connect(url, open_timeout=1, close_timeout=1):
                    break
            except OSError:
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(f'Flight recorder did not start; see {directory / "recorder.log"}') from None
                time.sleep(.1)
        os.environ['AUTOCAL_REFERENCE_WS'] = url
        print(f'; extended recording: {directory}', flush=True)
        yield directory
    finally:
        extended_reference.close()
        if previous_url is None:
            os.environ.pop('AUTOCAL_REFERENCE_WS', None)
        else:
            os.environ['AUTOCAL_REFERENCE_WS'] = previous_url
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        if process is not None:
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise RuntimeError(f'Flight recorder did not finalize; see {directory / "recorder.log"}') from None
            if process.returncode:
                raise RuntimeError(f'Flight recorder exited with status {process.returncode}; see {directory / "recorder.log"}')
            manifests = [json.loads(path.read_text()) for path in directory.glob('hangprinter-*.json')]
            if not manifests or any(item.get('status') != 'finalized' or item.get('rejected_messages') for item in manifests):
                raise RuntimeError(f'Extended recording is incomplete; see {directory / "recorder.log"}')
