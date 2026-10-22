"""Independent inspection of the finalized default-force job recording."""
from collections import Counter
import hashlib
import json
from pathlib import Path

from rerun.chunk import RrdReader

directory = Path(__file__).resolve().parent
summary = json.loads((directory / 'E12-acceptance.json').read_text())
recording = Path(summary['recording']['rrd'])
reader = RrdReader(recording)
assert reader.store()


def events(source):
    for chunk in reader.stream().filter(content=f'/autocal/{source}', components='event_json').to_chunks():
        for value in chunk.to_record_batch().column('event_json').to_pylist():
            yield json.loads(value[0])


counts = Counter(event['kind'] for event in events('collector'))
assert counts['measurement'] == summary['points']
assert counts['collection_complete'] == summary['sweeps'] - 2
physics = Counter()
stopped = []
contexts = []
for event in events('headless'):
    physics[event['kind']] += 1
    if event['kind'] == 'service_stopped':
        stopped.append(event)
    if event['kind'] == 'scene_context':
        contexts.append(event['payload'])
assert len(stopped) == 1 and physics['encoder_response_sent'] > 0
artifacts = {}
python_kinds = Counter()
for event in events('python'):
    python_kinds[event['kind']] += 1
    if event['kind'] == 'artifact':
        payload = event['payload']
        artifacts[payload['path']] = payload['content']
job_dir = recording.parent.parent
for name in ('sweeps.json', 'sweeps.full_auto.log', 'sweeps.full_auto_log.jsonl'):
    path = job_dir / name
    assert artifacts[str(path)] == path.read_text()
stages = list((job_dir / 'sweeps.stages').glob('*.json'))
assert stages and all(artifacts[str(path)] == path.read_text() for path in stages)
assert python_kinds['run_complete'] == 1
headless = json.loads((recording.parent / 'manifest.json').read_text())
for name, key in [('scene.usda', 'scene_sha256'), (headless['baked_scene_file'], 'baked_scene_sha256'),
                  ('firmware-config.g', 'firmware_config_sha256')]:
    assert hashlib.sha256((recording.parent / name).read_bytes()).hexdigest() == headless[key]
assert headless['settings']['enableLayering'] is False
assert 'Q0.0000' in headless['firmware_initial']['M666']
assert 'R30.00:30.00:30.00:30.00' in headless['firmware_initial']['M666']
assert contexts and all(context['settings']['enableLayering'] is False for context in contexts)
assert all(hashlib.sha256(context['bakedSourceText'].encode()).hexdigest() == context['baked_scene_sha256'] for context in contexts)
assert contexts[-1]['baked_scene_sha256'] == headless['baked_scene_sha256']
assert stopped[0]['payload']['step'] == headless['step']
clocks = reader.stream().filter(content='/clocks/headless/flight_recorder', components='Scalars:scalars').to_chunks()
samples = sum(len(chunk.to_record_batch().column('sim_step')) for chunk in clocks)
assert samples == summary['recording']['physics_samples']
checks = {'rrd_readable': True, 'recorded_measurements': counts['measurement'],
          'collection_completions': counts['collection_complete'], 'headless_events': dict(physics),
          'physical_samples': samples, 'physical_stop_step': headless['step'],
          'embedded_stage_artifacts': len(stages), 'final_dataset_and_logs_match_rrd': True,
          'frozen_scene_and_firmware_hashes_match': True, 'recorded_bakes_match_hashes': True,
          'layering_disabled': True, 'known_initial_radii_mm': [30] * 4, 'buildup_zero': True,
          'rrd_bytes': recording.stat().st_size}
(directory / 'E12-rrd-verification.json').write_text(json.dumps(checks, indent=2)+'\n')
print(json.dumps(checks))
