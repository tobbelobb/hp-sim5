"""An isolated, zero-motion flight-recorder protocol check, owned by preflight."""
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/python'))

from hp_sim5_research.services import free_port, stop_process
from rerun.chunk import RrdReader
from websockets.sync.client import connect


def probe(directory):
    directory = Path(directory)
    process = None
    try:
        port = free_port()
        with (directory / 'recorder.log').open('w') as output:
            # Shares the probe owner's group so supervisor timeout retires both.
            process = subprocess.Popen([str(ROOT / '.venv/bin/python'), 'scripts/hangprinter_flight_recorder.py',
                '--no-viewer', '--port', str(port), '--output', str(directory)], cwd=ROOT,
                stdin=subprocess.DEVNULL, stdout=output, stderr=output)
        deadline = time.monotonic() + 5
        while True:
            try:
                connection = connect(f'ws://127.0.0.1:{port}', open_timeout=.5, close_timeout=.5, proxy=None)
                break
            except OSError:
                if process.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError('Isolated recorder unavailable; inspect recorder.log') from None
                time.sleep(.05)
        with connection:
            sample = dict(version=1, source='headless', session=f'tool-check-preflight-{uuid.uuid4().hex}',
                          generation=0, step=0, time=0, dt=0, frames=[], cables=[],
                          wall_time_ms=time.time_ns() // 1_000_000)
            connection.send(json.dumps(sample))
            config = json.loads(connection.recv(timeout=2))
            ack = json.loads(connection.recv(timeout=2))
            if config.get('type') != 'recording_config' or ack != {'type': 'ack', 'step': 0}:
                raise RuntimeError('Recorder handshake/acknowledgment failed')
        process.terminate()
        process.wait(timeout=3)
        manifest_path, = directory.glob('hangprinter-*.json')
        manifest = json.loads(manifest_path.read_text())
        if process.returncode != 0 or manifest['status'] != 'finalized':
            raise RuntimeError('Recorder did not finalize the tool-check recording')
        chunks = RrdReader(manifest['rrd']).stream().filter(content='/clocks/headless/flight_recorder').to_chunks()
        if not chunks or any(chunk.num_rows != 1 for chunk in chunks):
            raise RuntimeError('Finalized tool-check RRD is not readable')
        return {'outcome': 'passed', 'handshake': True, 'finalized': True,
                'readable': True, 'recording_manifest': str(manifest_path), 'rrd': manifest['rrd']}
    finally:
        stop_process(process)


if __name__ == '__main__':
    directory = Path(sys.argv[1])
    try:
        result = probe(directory)
    except Exception as error:
        result = {'outcome': 'blocked', 'error': str(error),
                  'next_step': 'Inspect isolated recorder logs and Rerun/websockets dependencies'}
    result['kind'] = 'tool_check'
    (directory / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    raise SystemExit(0 if result['outcome'] == 'passed' else 1)
