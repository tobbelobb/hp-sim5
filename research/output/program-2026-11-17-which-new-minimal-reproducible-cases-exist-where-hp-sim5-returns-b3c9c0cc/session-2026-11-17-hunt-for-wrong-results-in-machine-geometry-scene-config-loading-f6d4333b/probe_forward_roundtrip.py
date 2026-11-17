"""Randomized inverse/forward round-trip probe for the production forward transform.

For each probe we compute line positions independently (numpy) from a true pose,
feed them to build/test_forward_driver, and compare the recovered pose.  The
control also compiles the firmware's hp_reachable() and checks that every
non-converged probe is genuinely outside the configured workspace, so a
non-convergence cannot be mistaken for a solver failure.

Exits nonzero if any converged probe is worse than the 0.02 mm clean-case
tolerance, or if any non-converged probe is inside the workspace.
"""
from pathlib import Path
import ctypes
import os
import subprocess
import sys
import tempfile
import numpy as np

ROOT = Path(__file__).resolve().parent
REPO = Path('/home/torbjorn/repos/hp-sim5')
sys.path.insert(0, str(REPO / 'flex-compensation-dev/hangprinter-flex-compensation'))
from test_forces import HP3, HP5, case  # noqa: E402

CUBE = np.array([[1000, -1000, 1900], [1000, 1000, 1900], [-1000, 1000, 1900], [-1000, -1000, 1900],
                 [1000, -1000, -280], [1000, 1000, -280], [-1000, 1000, -280], [-1000, -1000, -280]], float)
DRIVER = os.environ.get('FORWARD_TEST_DRIVER', str(REPO / 'forward-transform-dev/hangprinter-forward-transform/build/test_forward_driver'))
HEADER_DIR = REPO / 'forward-transform-dev/hangprinter-forward-transform'
TOL = 0.02  # mm, same as test_forward.py clean-case bound


def lengths(c):
    return np.linalg.norm(c['anchors'] - c['p'], axis=1) - np.linalg.norm(c['anchors'], axis=1)


def run(rows):
    lines = []
    for c, enabled, reduced, y, seed in rows:
        values = [len(c['anchors']), enabled, reduced, 0, c.get('anchor_mode', 0),
                  *np.asarray(c['anchors'], float).ravel(), *np.asarray(y, float), *np.asarray(seed, float)]
        lines.append(' '.join(map(str, values)))
    proc = subprocess.run([DRIVER], input='\n'.join(lines) + '\n', capture_output=True, text=True, check=True)
    return np.array([list(map(float, line.split())) for line in proc.stdout.splitlines()])


def compile_workspace_probe():
    os.makedirs('/tmp/opencode', exist_ok=True)
    directory = tempfile.mkdtemp(dir='/tmp/opencode')
    source = Path(directory) / 'probe.c'
    source.write_text(
        '#include "forward_workspace.h"\n'
        'int reachable(const double *a,int n,int mode,const double *p) {'
        'return hp_reachable((const double (*)[3])a,n,mode,p);}\n'
    )
    library = Path(directory) / 'probe.so'
    subprocess.run(['gcc', '-std=c11', '-shared', '-fPIC', f'-I{HEADER_DIR}',
                    str(source), '-lm', '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    fn = lib.reachable
    array = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
    fn.argtypes = [array, ctypes.c_int, ctypes.c_int, array]
    fn.restype = ctypes.c_int

    def reachable(anchors, mode, p):
        return bool(fn(np.ascontiguousarray(anchors, float), len(anchors), mode, np.array(p, float)))
    return reachable


def main():
    rng = np.random.default_rng(20261117)
    probes = []
    names = []
    for anchors, name, mode in ((HP5, 'hp5', 1), (HP3, 'hp3', 1), (CUBE, 'cubecorners', 3)):
        for _ in range(400):
            p = rng.uniform([-900, -900, -270], [900, 900, 1500])
            c = case(p, anchors=anchors)
            c['anchor_mode'] = mode
            probes.append((c, lengths(c), p))
            names.append(name)
    results = run([(c, 0, 0, y, np.zeros(3)) for c, y, _ in probes])
    assert len(results) == len(probes)
    err = np.linalg.norm(results[:, 1:4] - np.array([p for _, _, p in probes]), axis=1)
    ok = results[:, 0] == 1

    reachable = compile_workspace_probe()
    outside = 0
    false_negatives = []
    for i, (c, _, p) in enumerate(probes):
        if ok[i]:
            continue
        if reachable(c['anchors'], c['anchor_mode'], p):
            false_negatives.append((names[i], c['anchor_mode'], p, err[i]))
        else:
            outside += 1

    print(f'{len(probes)} probes: converged {ok.sum()}, non-converged {len(probes) - ok.sum()} '
          f'({outside} outside hp_reachable, {len(false_negatives)} inside)')
    print(f'converged error: max {err[ok].max():.6g} mm, median {np.median(err[ok]):.3g} mm '
          f'(tolerance {TOL} mm)')
    worst = np.argsort(err)[-5:][::-1]
    for i in worst:
        print(f'  worst {names[i]} mode{probes[i][0]["anchor_mode"]} ok={int(results[i,0])} '
              f'err={err[i]:.6g} mm truth={probes[i][2]} got={results[i,1:4]}')
    for name, mode, p, e in false_negatives[:10]:
        print(f'  FALSE NEGATIVE {name} mode{mode} p={p} err={e:.6g} mm')

    if np.any(err[ok] > TOL):
        print(f'FAIL: {int((err[ok] > TOL).sum())} converged probes exceed {TOL} mm')
        return 1
    if false_negatives:
        print(f'FAIL: {len(false_negatives)} reachable probes were reported non-converged')
        return 1
    print(f'PASS: all converged probes within {TOL} mm and all non-converged probes are outside hp_reachable')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
