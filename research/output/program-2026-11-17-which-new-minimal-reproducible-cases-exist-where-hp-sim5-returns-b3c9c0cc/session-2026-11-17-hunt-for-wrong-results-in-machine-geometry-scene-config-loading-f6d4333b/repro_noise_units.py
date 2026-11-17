"""The forward-transform comparison harness labels motor-degree perturbations as
millimetres of line, but converts with a 1:1, zero-mechanical-advantage spool.

For the shipped datasets (spool radius 75 mm, 255:20 gearing = 12.75,
mechanical advantage 2 on A-D and 4 on I) one millimetre of line at the origin
is 12.75*2*360/(2*pi*75) = 19.48 motor degrees, not 360/(2*pi*75) = 0.764.
The noise and bias sweeps are therefore 25.5x weaker than their labels.
"""
from pathlib import Path
import json
import math
import random
import sys

ROOT = Path('/home/torbjorn/repos/hp-sim5')
sys.path.insert(0, str(ROOT / 'forward-transform-dev/hangprinter-forward-transform'))
import run_tests as rt  # noqa: E402

DATA = ROOT / 'forward-transform-dev/hangprinter-forward-transform/datasets/clean_baseline_HP5.jsonl'
TOL = 0.01  # relative tolerance on the deg/mm conversion


def main():
    entry = json.loads(DATA.read_text().splitlines()[0])
    cfg = entry['config_used']
    radius = cfg['spool_r_in_origin'][0]
    gearing = cfg['spool_gear_teeth'] / cfg['motor_gear_teeth']
    ma = cfg['mechanical_advantage']
    harness = rt.MM_TO_DEG
    exact = [gearing * m * 360.0 / (2.0 * math.pi * radius) for m in ma]
    print(f'harness deg/mm = {harness:.6f}')
    print(f'correct deg/mm A-D = {exact[0]:.6f} (ratio {exact[0]/harness:.2f}x)')
    print(f'correct deg/mm I   = {exact[4]:.6f} (ratio {exact[4]/harness:.2f}x)')
    for label in (1.0, 3.0, 5.0):
        injected = label * harness
        implied_mm = injected / exact[0]
        print(f'noise labelled {label:g} mm -> {injected:.3f} deg -> {implied_mm:.4f} mm of line (A-D)')
    for label in (5.0, 2.5):
        injected = label * harness
        implied_mm = injected / exact[0]
        print(f'bias labelled +/-{label:g} mm -> +/-{injected:.3f} deg -> +/-{implied_mm:.4f} mm of line (A-D)')

    # Confirm the harness applies those amplitudes to real dataset samples.
    samples = rt.load_samples('clean_baseline_HP5.jsonl')[:200]
    noisy = rt.add_gaussian_noise(samples, 5.0, random.Random(0))
    deg_sigma = math.sqrt(sum((n.motor_deg[i] - s.motor_deg[i]) ** 2
                              for n, s in zip(noisy, samples) for i in range(5))
                          / (5 * len(samples)))
    biased = rt.add_bias(samples, [5.0, -5.0, 2.5, -2.5, 0.0])
    deg_bias = [sum(n.motor_deg[i] - s.motor_deg[i] for n, s in zip(biased, samples)) / len(samples)
                for i in range(5)]
    print(f'observed add_gaussian_noise(5 mm) motor-degree sigma: {deg_sigma:.3f} deg '
          f'= {deg_sigma / exact[0]:.4f} mm of line (label: 5 mm)')
    print(f'observed add_bias(5/-5/2.5/-2.5/0 mm) motor-degree offsets: '
          f'{", ".join(f"{v:.3f}" for v in deg_bias)} deg = '
          f'{", ".join(f"{v / exact[0]:.4f}" for v in deg_bias)} mm of line')

    ratio = exact[0] / harness
    if abs(ratio - 1.0) > TOL:
        print(f'FAIL: noise/bias amplitudes are off by {ratio:.2f}x for the shipped dataset config')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
