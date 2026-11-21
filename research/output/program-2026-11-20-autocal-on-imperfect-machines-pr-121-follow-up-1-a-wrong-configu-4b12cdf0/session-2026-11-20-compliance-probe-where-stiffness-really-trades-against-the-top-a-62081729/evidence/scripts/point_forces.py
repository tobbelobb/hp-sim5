"""Realised versus commanded force at every sweep data point (sensor line and HP5 extra force lines), from an exact replay.

usage: point_forces.py RUN_DIR REPLAY.jsonl

A point is matched to replay reads within 0.005 deg (max over axes) of its mean encoder angles.
Prints per-line realised/commanded ratio and offset, and the probe levels' return-to-1x offset.
"""
import json, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from probe_replay_attrib import line_paths
from autocal.dataset_roles import normalize_dataset_point_roles


def main():
    run, replay = Path(sys.argv[1]), sys.argv[2]
    imp = json.loads((run / 'imperfection.json').read_text())
    ds = json.loads((run / 'sweeps.json').read_text()); ds.setdefault('machine_type', imp['machine']); normalize_dataset_point_roles(ds)
    letters = list(imp['truth_mm'])
    reads = [json.loads(line) for line in open(replay)]
    angles = np.array([r['angles_deg'] for r in reads])
    _, long = line_paths(reads[0]['joints'])
    force = lambda R, letter: np.mean([sum(r['joints'][n][0] for n in long[letter]) for r in R])
    rows = []
    for sweep in ds['sweeps']:
        for p in sweep['data_points']:
            mu = np.asarray(p.get('mu') or p['raw_angles_deg'], float)
            hit = np.flatnonzero(np.max(np.abs(angles - mu), axis=1) < 0.005)
            if not len(hit):
                continue
            R = [reads[j] for j in hit]
            pairs = [(int(p.get('source_sensor_anchor', sweep['sensor_anchor'])), p['sensor_force_n'])]
            pairs += [(int(e['anchor']), e['force_n']) for e in p.get('extra_sensor_ranges', []) if 'force_n' in e]
            for line, cmd in pairs:
                rows.append((line, cmd, force(R, letters[line])))
    r = np.array(rows)
    print(f'{run.name}: {len(r)} line-points matched')
    for line in sorted(set(r[:, 0].astype(int))):
        m = r[r[:, 0] == line]
        print(f'  line {letters[line]}: n {len(m)}, commanded {m[:, 1].mean():.3f} N, realised/commanded median {np.median(m[:, 2] / m[:, 1]):.4f}, '
              f'offset mean {np.mean(m[:, 2] - m[:, 1]):+.3f} sd {np.std(m[:, 2] - m[:, 1]):.3f} N')
    print(f'  all: realised/commanded median {np.median(r[:, 2] / r[:, 1]):.4f}, mean offset {np.mean(r[:, 2] - r[:, 1]):+.3f} N')


if __name__ == '__main__':
    main()
