"""Inspect finalized physical events and embedded production stage artifacts."""
from collections import Counter
import hashlib
import json
from pathlib import Path

from rerun.chunk import RrdReader

directory = Path(__file__).resolve().parent
summary = json.loads((directory / 'acceptance.json').read_text())
recording = Path(summary['recording']['rrd'])
reader = RrdReader(recording)
assert reader.store()


def events(source):
    for chunk in reader.stream().filter(content=f'/autocal/{source}', components='event_json').to_chunks():
        for value in chunk.to_record_batch().column('event_json').to_pylist():
            yield json.loads(value[0])


counts = Counter(event['kind'] for event in events('collector'))
assert counts['measurement'] == summary['points']
assert counts['collection_complete'] == summary['sweeps'] - 2  # one bootstrap + five planned sweeps
physics = Counter(event['kind'] for event in events('headless'))
assert physics['service_stopped'] == 1 and physics['encoder_response_sent'] > 0
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
for name, key in [('scene.usda', 'scene_sha256'), ('baked-scene.usda', 'baked_scene_sha256'),
                  ('firmware-config.g', 'firmware_config_sha256')]:
    assert hashlib.sha256((recording.parent / name).read_bytes()).hexdigest() == headless[key]
assert headless['settings']['enableLayering'] is False
assert 'Q0.0000' in headless['firmware_initial']['M666']
assert 'R30.00:30.00:30.00:30.00' in headless['firmware_initial']['M666']
checks = {'rrd_readable': True, 'recorded_measurements': counts['measurement'],
          'collection_completions': counts['collection_complete'], 'headless_events': dict(physics),
          'embedded_stage_artifacts': len(stages), 'final_dataset_and_logs_match_rrd': True,
          'frozen_scene_and_firmware_hashes_match': True, 'layering_disabled': True,
          'known_initial_radii_mm': [30] * 4, 'buildup_zero': True,
          'rrd_bytes': recording.stat().st_size}
(directory / 'rrd-verification.json').write_text(json.dumps(checks, indent=2))
print(json.dumps(checks))
