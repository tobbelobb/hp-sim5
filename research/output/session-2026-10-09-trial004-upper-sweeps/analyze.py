import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent

def metrics(path):
    dataset = json.loads(path.read_text())
    result = []
    for sweep in dataset['sweeps']:
        points = sweep['data_points']
        result.append({
            'id': sweep['id'], 'points': len(points),
            'fixed_anchors': sweep['fixed_anchors'],
            'fixed_lengths_mm': sweep['fixed_lengths'],
            'fixed_encoder_span_deg': {
                str(axis): max(p['raw_angles_deg'][axis] for p in points)
                - min(p['raw_angles_deg'][axis] for p in points)
                for axis in sweep['fixed_anchors']
            },
            'sensor_force_n': sweep.get('metadata', {}).get('sensor_force_n'),
        })
    return result

results = {}
for label in ['baseline', 'patched-489-20x', 'patched-464-20x', 'patched-bootstrap-20x']:
    results[label] = metrics(HERE / label / 'sweeps.json')
results['original_trial004'] = metrics(HERE.parent.parent / 'extended-autocal/trial-004/sweeps.json')
events = [json.loads(line) for line in (HERE / 'overload-guard-20x/events.jsonl').open()]
results['overload_cleanup_tail'] = [e for e in events if e['type'] == 'bridge_payload'][-6:]
(HERE / 'sweep-metrics.json').write_text(json.dumps(results, indent=2))

fig = plt.figure(figsize=(11, 4.5))
trajectory = fig.add_subplot(121, projection='3d')
encoder = fig.add_subplot(122)
for label, color, title in [('baseline', 'tab:red', 'Original forces'), ('patched-489-20x', 'tab:blue', 'Patched forces')]:
    sweep = json.loads((HERE / label / 'sweeps.json').read_text())['sweeps'][0]
    first_direction = sweep['data_points'][:10]
    start = first_direction[0]['timestamp_ms'] / 1000 - 5
    end = first_direction[-1]['timestamp_ms'] / 1000
    rows = [json.loads(line) for line in (HERE / label / 'physics.jsonl').open()]
    rows = [r for r in rows if start <= r['sim_s'] <= end]
    xyz = list(zip(*[r['effector'] for r in rows]))
    trajectory.plot(*xyz, color=color, label=title, linewidth=1)
    encoder.plot([r['sim_s'] - start for r in rows], [r['motors'][1]['angle_deg'] for r in rows], color=color, label=title, linewidth=1)
trajectory.set(xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)', title='First upper sweep: one direction')
encoder.axhline(0, color='black', linestyle='--', linewidth=.7, label='Fixed B target')
encoder.set(xlabel='Elapsed simulation time (s)', ylabel='B encoder angle (degrees)', title='Held B slips with the original forces')
encoder.legend()
fig.tight_layout()
fig.savefig(HERE / 'upper-sweep-comparison.png', dpi=160)
fig.savefig(HERE / 'upper-sweep-comparison.svg')
print(json.dumps({k: v for k, v in results.items() if k != 'overload_cleanup_tail'}, indent=2))
