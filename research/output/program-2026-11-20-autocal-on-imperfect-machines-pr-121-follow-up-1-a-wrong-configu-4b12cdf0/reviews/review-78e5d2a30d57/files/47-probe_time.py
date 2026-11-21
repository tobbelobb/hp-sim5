"""Simulated time spent in compliance probes, from every sweep file of a run (refinement + bootstrap)."""
import json, sys, glob
from pathlib import Path


def probe_seconds(run):
    seen, total = set(), 0.0
    for path in sorted(glob.glob(f'{run}/sweeps*.json')):
        try:
            data = json.load(open(path))
        except (json.JSONDecodeError, UnicodeDecodeError):
            continue
        for sweep in data.get('sweeps', []) if isinstance(data, dict) else []:
            for probe in (sweep.get('metadata') or {}).get('compliance_probes', []):
                end = probe['levels'][-1]['clock_ms']
                if end in seen:
                    continue
                seen.add(end)
                # The probe starts right after the direction's last point.
                last = max(p['noise_clock_end_ms'] for p in sweep['data_points'] if p.get('noise_clock_end_ms', 0) < end)
                total += (end - last) / 1000
    return total, len(seen)


if __name__ == '__main__':
    for run in sys.argv[1:]:
        total, n = probe_seconds(run)
        manifest = json.load(open(f'{run}/sweeps.headless/manifest.json'))
        print(f'{Path(run).name}: {n} probes, {total:.1f} s simulated ({total / n if n else 0:.1f} s each) of {manifest["simulated_s"]:.0f} s total')
