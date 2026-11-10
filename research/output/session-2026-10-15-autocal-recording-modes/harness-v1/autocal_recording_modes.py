"""Compare recorder modes on one frozen real autocal schedule, gating on equality.

Run with .venv/bin/python; localhost sockets are needed for the real recorder.
See research/autocal-recording-performance.md for scope and measured results.
"""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import socket
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from autocal.headless_sim import source_hash
from rerun.chunk import RrdReader
from websockets.sync.client import connect

MODES = ['none', 'events', 'sampled', 'full']


def write(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def canonical(value):
    if isinstance(value, dict):
        return {key: canonical(item) for key, item in value.items()
                if key not in ('wall_time_ms', 'monotonic_ms', 'session')}
    if isinstance(value, list):
        return [canonical(item) for item in value]
    return value


def read_events(path):
    events = []
    for chunk in RrdReader(path).stream().filter(content='/autocal/headless', components='event_json').to_chunks():
        batch = chunk.to_record_batch()
        events.extend(zip(batch.column('event_order').to_pylist(), batch.column('event_json').to_pylist()))
    normalized = [canonical(json.loads(value[0])) for _, value in sorted(events)]
    return {'event_count_readback': len(normalized),
            'event_kinds': dict(Counter(event['kind'] for event in normalized)),
            'event_sha256': hashlib.sha256(json.dumps(normalized, sort_keys=True).encode()).hexdigest()}


def freeze(source, output, steps):
    output.mkdir()
    manifest = json.loads((source / 'manifest.json').read_text())
    if manifest['machine_type'] != 'hangprinter_3' or manifest['firmware_config'] != 'sys/config_hp3.g':
        raise ValueError('This benchmark requires the unlayered HP3 configuration')
    if steps > manifest['total_steps'] or steps < 1:
        raise ValueError('steps must select a positive prefix of the actual workload')
    shutil.copyfile(source / 'scene.usda', output / 'scene.usda')
    shutil.copyfile(source / 'manifest.json', output / 'source-manifest.json')
    counts = Counter()
    with (source / 'events.jsonl').open() as stream, (output / 'events.jsonl').open('w') as target:
        for line in stream:
            event = json.loads(line)
            if event['step'] > steps:
                break
            if event['epoch'] != 0:
                raise ValueError('Select a prefix in one generation')
            target.write(line)
            counts[event['type']] += 1
    contract = {'kind': 'frozen_production_autocal_prefix', 'source': str(source.resolve()),
                'source_trace_sha256': digest(source / 'events.jsonl'),
                'trace_sha256': digest(output / 'events.jsonl'), 'scene_sha256': digest(output / 'scene.usda'),
                'steps': steps, 'warmup_steps': 5000, 'checkpoint_every': 1000, 'trace_rows': dict(counts),
                'machine': 'hp3', 'layering': False, 'backend': 'native Rust',
                'scope': 'Frozen physics/command/encoder replay; no new planning, collection decisions or fitting. '
                         'Headless events only; Python/collector logs and stage artifacts are excluded.'}
    write(output / 'contract.json', contract)
    return contract


def trial(inputs, directory, mode):
    directory.mkdir()
    recorder = None
    url = '-'
    started = time.monotonic()
    try:
        with (directory / 'recorder.log').open('w') as log:
            if mode != 'none':
                with socket.socket() as probe:
                    probe.bind(('127.0.0.1', 0))
                    port = probe.getsockname()[1]
                url = f'ws://127.0.0.1:{port}'
                recorder = subprocess.Popen([sys.executable, 'scripts/hangprinter_flight_recorder.py',
                    '--extended-reference', '--no-viewer', '--port', str(port), '--output', str(directory),
                    '--sample-stride', '100' if mode == 'sampled' else '10', '--geometry-detail', 'compact'],
                    cwd=ROOT, stdout=log, stderr=log, start_new_session=True)
                deadline = time.monotonic() + 30
                while True:
                    try:
                        with connect(url, open_timeout=1, close_timeout=1):
                            break
                    except OSError:
                        if recorder.poll() is not None or time.monotonic() > deadline:
                            raise RuntimeError(f'Recorder failed; see {directory / "recorder.log"}') from None
                        time.sleep(.05)
            with (directory / 'replay.log').open('w') as output:
                env = {key: value for key, value in os.environ.items()
                       if not key.startswith(('AUTOCAL_', 'HP_SIM5_'))}
                subprocess.run(['node', 'tests/benchmark/autocal_recording_modes.mjs',
                    str(inputs), str(directory), mode, url], cwd=ROOT, env=env,
                    stdout=output, stderr=output, check=True, timeout=300)
            finalize_started = time.monotonic()
            if recorder:
                recorder.send_signal(signal.SIGTERM)
                if recorder.wait(timeout=60) != 0:
                    raise RuntimeError('Recorder did not finalize successfully')
            finalize_s = time.monotonic() - finalize_started
            total_s = time.monotonic() - started
    finally:
        if recorder and recorder.poll() is None:
            recorder.terminate()
            try:
                recorder.wait(timeout=10)
            except subprocess.TimeoutExpired:
                recorder.kill()
                recorder.wait()
    result = json.loads((directory / 'result.json').read_text())
    result.update(finalize_s=finalize_s, total_s=total_s, directory=str(directory),
                  workload_s=result['replayAndDrainMs'] / 1000 + finalize_s,
                  rrd_bytes=sum(path.stat().st_size for path in directory.glob('*.rrd')),
                  journal_bytes=(directory / 'events.jsonl').stat().st_size)
    if mode != 'none':
        recording, = [json.loads(path.read_text()) for path in directory.glob('hangprinter-*.json')]
        if recording['status'] != 'finalized' or recording['rejected_messages']:
            raise RuntimeError('Recording is incomplete')
        result.update(physics_samples=recording['physics_samples'], event_count=recording['event_count'])
        result.update(read_events(Path(recording['rrd'])))  # outside every measured interval
        if result['event_count_readback'] != result['event_count']:
            raise RuntimeError('Recorded event count does not match readback')
    else:
        result.update(physics_samples=0, event_count=0)
    write(directory / 'trial.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-headless', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--steps', type=int, default=100000)
    parser.add_argument('--repeats', type=int, default=4)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('repeats must be positive')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    contract = freeze(args.source_headless, args.output / 'input', args.steps)
    result = {'contract': contract, 'git_revision': subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'source_sha256': source_hash(), 'autocal_sha256': source_hash(('autocal',), ('.py',)),
        'harness_sha256': {path: digest(ROOT / path) for path in
            ('tests/benchmark/autocal_recording_modes.py', 'tests/benchmark/autocal_recording_modes.mjs')},
        'platform': platform.platform(), 'python': sys.version,
        'cpu': next(line.split(':', 1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines()
                    if line.startswith('model name')),
        'node': subprocess.check_output(['node', '--version'], text=True).strip(), 'trials': []}
    (args.output / 'working.diff').write_text(subprocess.check_output(['git', 'diff', 'HEAD'], cwd=ROOT, text=True))
    for repeat in range(args.repeats):
        # Rotate positions across four repetitions; trials never run concurrently.
        for mode in MODES[repeat % 4:] + MODES[:repeat % 4]:
            row = trial(args.output / 'input', args.output / f'R{repeat + 1}-{mode}', mode)
            baseline = result['trials'][0] if result['trials'] else row
            for field in ('checkpointSha256', 'scheduleSha256', 'bakedSceneSha256',
                          'steps', 'commands', 'encoderReads', 'physicsEngine', 'physicsArtifact', 'journal_bytes'):
                if row[field] != baseline[field]:
                    raise RuntimeError(f'Non-equivalent trial: {mode} {field}; preserve trials, do not compare timings')
            if mode != 'none':
                recorded = next((trial for trial in result['trials'] if trial['mode'] != 'none'), row)
                if row['event_sha256'] != recorded['event_sha256']:
                    raise RuntimeError(f'Event evidence differs for {mode}; do not compare timings')
            result['trials'].append(row)
            write(args.output / 'results.json', result)
            print(f'R{repeat + 1} {mode}: {row["workload_s"]:.3f}s workload+finalize, '
                  f'{row["total_s"]:.3f}s total, {row["rrd_bytes"] / 1e6:.3f} MB RRD; equivalent', flush=True)
    result['equivalent'] = True
    result['summary'] = {mode: {key: statistics.median(row[key] for row in result['trials'] if row['mode'] == mode)
        for key in ('workload_s', 'total_s', 'rrd_bytes', 'journal_bytes', 'physics_samples', 'event_count')}
        for mode in MODES}
    write(args.output / 'results.json', result)


if __name__ == '__main__':
    main()
