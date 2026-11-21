"""How the parallel paths of each line and the strands within a path share tension at the probe levels.
usage: strand_share.py REPLAY.jsonl ; prints per line the max relative spread of path tensions and of
segment tensions within a path, over all full encoder reads with tension > 0.5 N."""
import json, sys
from collections import defaultdict
import numpy as np
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from probe_replay_attrib import line_paths
reads = [json.loads(line) for line in open(sys.argv[1])]
paths, _ = line_paths(reads[0]['joints'])
for letter, by_path in sorted(paths.items()):
    between, within = [], []
    for r in reads[::5]:
        per_path = [np.array([r['joints'][n][0] for n in names]) for names in by_path.values()]
        means = np.array([p.mean() for p in per_path])
        if means.min() < 0.5:
            continue
        between.append(np.ptp(means) / means.mean())
        within.append(max(np.ptp(p) / p.mean() for p in per_path))
    print(f'line {letter}: {len(by_path)} paths; path-to-path spread median {np.median(between):.2e} max {np.max(between):.2e}; '
          f'segment spread within a path max {np.max(within):.2e} (n {len(between)})')
