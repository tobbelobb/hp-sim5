#!/usr/bin/env python
"""Reproducer: Python/native length telemetry aliases `commanded` to `actual`.

The JS world (hp-sim-3d/app/flightRecorderSnapshot.js) computes
`commanded` from the position-mode motor target and reports
`error = actual - commanded`. The Python mirror
(src/python/hp_sim5_research/mirror.py, `capture_snapshot`) hard-codes
`commanded = actual` and `error = actual - actual = 0` for every cable, and
never suppresses either value in torque mode.

This script runs one production headless world (checked-in Wasm solver), feeds
the same state to the production JS recorder and to the production Python
mirror, and asserts that the two agree. It exits nonzero when the defect is
present.

Run from anywhere:
    .venv/bin/python repro_length_telemetry.py
"""
from __future__ import annotations

import json
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve()
ROOT = HERE.parents[4]
sys.path.insert(0, str(ROOT / 'src/python'))

from hp_sim5_research import mirror  # noqa: E402


def main() -> int:
    probe = subprocess.run(
        ['node', str(HERE.parent / 'probe_length_telemetry.mjs')],
        cwd=ROOT, capture_output=True, text=True, timeout=60)
    if probe.returncode != 0:
        print('probe failed:\n' + probe.stderr[-4000:])
        return 2
    data = json.loads(probe.stdout)
    print(f"scene: public/usd_scenes/slideprinter.usda, axes {data['axes']}, dt {data['dt']} s")

    failures: list[str] = []
    for sample in data['samples']:
        world = mirror.World()
        mirror.apply_state(world, sample['state'])
        row = mirror.sample(world, sample['state']['step'], sample['state']['dt_s'])
        js = {cable['name']: cable['lengths'] for cable in sample['flight']}

        for cable in row['cables']:
            base = cable['name'].split('/', 1)[-1]
            key = next(k for k in js if k == base or k.startswith(base + '_'))
            js_lengths, py_lengths = js[key], cable['lengths_m']
            tag = f"{sample['label']}/{base}"

            # Values both sides can read agree: actual/geometric/stretch are fine.
            for field in ('actual', 'geometric', 'stretch'):
                delta = abs(js_lengths[field] - py_lengths[field])
                if delta > 1e-12:
                    failures.append(f'{tag}: {field} disagrees by {delta:g} m')

            if js_lengths['commanded'] is None or py_lengths['commanded'] is None:
                # Torque mode: the JS contract is that both series are omitted.
                if js_lengths['commanded'] is None and py_lengths['commanded'] is not None:
                    failures.append(
                        f'{tag}: torque mode, JS commanded=None, '
                        f'Python mirror reports commanded={py_lengths["commanded"]!r}')
                if js_lengths['error'] is None and py_lengths['error'] is not None:
                    failures.append(
                        f'{tag}: torque mode, JS error=None, '
                        f'Python mirror reports error={py_lengths["error"]!r}')
                continue

            commanded_delta = abs(js_lengths['commanded'] - py_lengths['commanded'])
            error_delta = abs(js_lengths['error'] - py_lengths['error'])
            print(f"  {tag}: JS commanded={js_lengths['commanded']:.9f} "
                  f"error={js_lengths['error']:+.9f} m | "
                  f"Python commanded={py_lengths['commanded']:.9f} "
                  f"error={py_lengths['error']:+.9f} m")
            # error must equal actual - commanded on both sides, and both sides
            # must describe the same world.
            if commanded_delta > 1e-9:
                failures.append(
                    f'{tag}: commanded disagrees by {commanded_delta:g} m '
                    f'(JS {js_lengths["commanded"]:.9f}, Python {py_lengths["commanded"]:.9f})')
            if error_delta > 1e-9:
                failures.append(
                    f'{tag}: error disagrees by {error_delta:g} m '
                    f'(JS {js_lengths["error"]:+.9f}, Python {py_lengths["error"]:+.9f})')

    if failures:
        print('\nBUG REPRODUCED: Python/native length telemetry does not match the JS world.')
        for failure in failures:
            print('  - ' + failure)
        print('\nExpected contract (FLIGHT_RECORDER.md): commanded = predicted paid-out length at '
              'the position-mode motor target; error = actual - commanded; both omitted in torque mode.')
        return 1
    print('OK: JS and Python length telemetry agree.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
