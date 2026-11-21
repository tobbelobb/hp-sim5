"""Per-run table for the rough-bootstrap sweep.  usage: table.py RUN_DIR... (writes markdown to stdout)

Columns: machine, seed, true EA scale, stop reason, print-frame max error, simulated time,
tuned edge force, size-tune travel, whether the pre-fix edge rule (replayed) differs.
"""
import json, re, subprocess, sys
from pathlib import Path

def log_value(run, pattern):
    log = Path(run, 'sweeps.full_auto.log')
    if not log.exists():
        return None
    match = re.search(pattern, log.read_text())
    return match[1] if match else None

rows = []
for run in [r for r in sys.argv[1:] if Path(r, 'result.json').exists()]:
    result = json.loads(Path(run, 'result.json').read_text())
    tuning = Path(run, 'sweeps.json.force-tuning.json')
    differs = None
    if tuning.exists():
        out = subprocess.run(['node', str(Path(__file__).with_name('edge_replay.mjs')),
                              'autocal/control/behaviors/_before.tmp.mjs', 'autocal/control/behaviors/force_tuning.mjs', str(tuning)],
                             capture_output=True, text=True).stdout
        differs = None if not out else not json.loads(out)['same']
    error = result.get('max_abs_frame_error_mm')
    rows.append((result['machine'].replace('hangprinter_', 'hp').replace('_pulley_reduced', 'r'), result['seed'],
                 result['imperfection']['stiffness_scale'], result.get('stop_reason'),
                 '—' if error is None else f'{error:.3f}', result.get('simulated_s') or result.get('elapsed_s'),
                 log_value(run, r'edge=([\d.]+)'), log_value(run, r'size-tune max travel raw=([\d.]+)mm'),
                 {None: '?', True: 'yes', False: 'no'}[differs], Path(run).name))
print('| run | machine | seed | EA scale | stop | max err mm | sim s | edge N | size-tune mm | old rule differs |')
print('| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |')
for r in sorted(rows, key=lambda r: (r[0], str(r[1]), r[9])):
    sim = f'{float(r[5]):.0f}' if r[5] else '?'
    print(f'| {r[9]} | {r[0]} | {r[1]} | {r[2]:.3f} | {r[3]} | {r[4]} | {sim} | {r[6]} | {r[7]} | {r[8]} |')
