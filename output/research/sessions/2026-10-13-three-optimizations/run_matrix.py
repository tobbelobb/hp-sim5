"""Sequential, unprofiled full-workload comparisons; run from the repo root."""
import json
from pathlib import Path
import subprocess
import sys

OUT = Path(__file__).resolve().parent
ORIGINAL = Path('/home/torbjorn/repos/hp-sim5/output/research/sessions/2026-10-13-headless-js-profiling')
root, stage = Path(sys.argv[1]), sys.argv[2]
comparison = Path(sys.argv[3]) if len(sys.argv) > 3 else ORIGINAL
results = []
for machine in ('hp3', 'hp4'):
    for workload in ('logo', 'autocal'):
        for layering in ('on', 'off'):
            name = f'{machine}-{workload}-{layering}'
            baseline = comparison / f'{name}-plain.json'
            directory = OUT / stage
            directory.mkdir(exist_ok=True)
            result = directory / f'{name}-plain.json'
            flag, extension = ('--commands', 'json') if workload == 'logo' else ('--events', 'jsonl')
            command = ['node', 'scripts/profile_headless.mjs', '--machine', machine,
                       flag, str(ORIGINAL / 'inputs' / f'{machine}-{workload}.{extension}'),
                       '--layering', layering, '--repeats', '3', '--compare', str(baseline),
                       '--output', str(result)]
            print('START', stage, name, flush=True)
            with (directory / f'{name}.log').open('w') as log:
                subprocess.run(command, cwd=root, stdout=log, stderr=subprocess.STDOUT, check=True)
            data = json.loads(result.read_text())
            results.append(data)
            print('PASS', stage, name, data['medianPhysicsMs'], flush=True)
(OUT / f'{stage}-matrix.json').write_text(json.dumps(results, indent=2) + '\n')
