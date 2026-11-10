import json
import sys
from argparse import Namespace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from autocal.headless_sim import HeadlessSimulation
from autocal import extended_reference
from rerun.chunk import RrdReader

root = Path(__file__).resolve().parent
args = Namespace(machine_type='hangprinter_3', find_radii='off', find_buildup_factor='off',
                 rrf_config=None, config=None, dataset=root / 'sweeps.json',
                 keep_sim_alive=False, extended_reference=True)
with HeadlessSimulation(args, []) as service:
    extended_reference.emit('run_start', test='unexpected-physics-death')
    service.request('advance', seconds=.05)
    # Stand in for the final optimization's completion markers; this probe
    # evaluates lifecycle validation, not calibration correctness.
    service.manifest.update(stop_reason='patience-or-threshold', applied_parameters=['test-marker'])
    extended_reference.emit('run_complete', returncode=0)
    physics = service.processes[-1]
    physics.kill()
    physics.wait(timeout=5)

recording, = service.manifest['recordings']
events = []
for chunk in RrdReader(recording['rrd']).stream().filter(content='/autocal/headless', components='event_json').to_chunks():
    events.extend(json.loads(row[0]) for row in chunk.to_record_batch().column('event_json').to_pylist())
result = dict(physics_returncode=physics.returncode, context_exit='success',
              status=service.manifest['status'], normal_completion=service.manifest['normal_completion'],
              recording_error=service.manifest.get('recording_error'),
              recorder_status=recording['status'], autocal_complete=recording['autocal_complete'],
              service_stopped_events=sum(event['kind'] == 'service_stopped' for event in events),
              manifest=str(service.directory / 'manifest.json'))
(root / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
