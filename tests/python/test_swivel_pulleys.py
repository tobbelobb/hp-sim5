"""Swivel pulley anchors in RRF and Klipper against an independent wrap reference.

COPacabana's anchors are swivel pulleys: the line enters each one along a fixed
axis, wraps the wheel and leaves from a tangent point that moves with the mover.
"""
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'flex-compensation-dev/hangprinter-flex-compensation'))
from test_swivel import reference  # noqa: E402

# COPacabana.xml entry points relative to PlaneA P0, and platform attachments (mm)
ENTRIES = np.array([[-2427.578, 1959.987, 2157.96], [2411.243, 1944.128, 2170.735],
                    [2408.63, -1976.237, 2149.285], [-2433.437, -1965.415, 2158.603],
                    [-2298.856, 1609.864, -925.174], [2315.327, 1615.126, -848.102],
                    [2308.631, -1613.281, -925.031], [-2385.985, -1614.363, -846.768]])
ATTACHMENTS = np.array([[-150, 236.5, 22.5], [150, 236.5, 22.5], [150, -236.5, 22.5], [-150, -236.5, 22.5],
                        [-200, 150, 575], [200, 150, 575], [200, -150, 575], [-200, -150, 575]], float)
ANCHORS = ENTRIES - ATTACHMENTS
AXES = np.array([[-1., 0, 0], [1, 0, 0], [1, 0, 0], [-1, 0, 0]] * 2)
RADII = np.array([37.] * 4 + [33.] * 4)
HOME = np.array([90.51, 84.99, 90.51, 84.15, 40.16, 34.93, 35.64, 30.83])
EA, GUY = 118500., 200.


def swivel_lengths(p):
    return np.array([reference(a, u, r, np.asarray(p, float))[0] for a, u, r in zip(ANCHORS, AXES, RADII)])


def test_firmware_copies_of_the_geometry_header_are_identical():
    dev = (ROOT / 'flex-compensation-dev/hangprinter-flex-compensation/swivel_pulley.h').read_bytes()
    assert (ROOT / 'RRF/ReprapFirmware/src/Movement/Kinematics/SwivelPulley.h').read_bytes() == dev
    assert (ROOT / 'klipper/klippy/chelper/swivel_pulley.h').read_bytes() == dev


@pytest.fixture(scope='module')
def klipper():
    sys.path.insert(0, str(ROOT / 'klipper/klippy'))
    import chelper
    ffi, lib = chelper.get_ffi()
    wf = ffi.gc(lib.winch_flex_alloc(), lib.winch_flex_free)
    n = len(ANCHORS)
    d = lambda values: ffi.new('double[]', [float(v) for v in np.ravel(values)])
    lib.winch_flex_configure(wf, n, d(ANCHORS), d(ATTACHMENTS), 0., 13.9, EA, d([50.] * n), d([700.] * n),
                             d([GUY] * n), 1, 0, 0, ffi.new('int[]', [1] * n), ffi.new('int[]', [1] * n))
    lib.winch_flex_set_swivels(wf, d(RADII), d(AXES), d(np.zeros(n)))
    lib.winch_flex_set_anchor_mode(wf, 3)

    def arrays(p):
        distances, flex = ffi.new('double[]', n), ffi.new('double[]', n)
        lib.winch_flex_calc_arrays(wf, *p, distances, flex)
        return np.array(list(distances)), np.array(list(flex))
    return ffi, lib, wf, arrays, d


def test_klipper_line_lengths_and_forward_round_trip(klipper):
    ffi, lib, wf, arrays, d = klipper
    rng = np.random.default_rng(4614)
    for p in rng.uniform([-700, -700, -300], [700, 700, 800], (20, 3)):
        np.testing.assert_allclose(arrays(p)[0], swivel_lengths(p), atol=1e-6)
    for enabled in (0, 1):
        lib.winch_flex_set_enabled(wf, enabled)
        origin = arrays((0., 0., 0.))[0]
        for p in rng.uniform([-700, -700, -300], [700, 700, 800], (10, 3)):
            distances, flex = arrays(p)
            out = ffi.new('double[3]')
            assert lib.winch_forward_solve(wf, d(distances - origin + flex), d([0, 0, 0]), 1e-3, 1e-3, 3, 30,
                                           out, ffi.NULL, ffi.NULL)
            np.testing.assert_allclose(list(out), p, atol=1e-3)
    lib.winch_flex_set_enabled(wf, 0)


def test_klipper_home_tension_is_the_stretch_already_in_the_lines(klipper):
    ffi, lib, wf, arrays, d = klipper
    lib.winch_flex_set_enabled(wf, 1)
    p = (300., -200., 100.)
    plain = arrays(p)[1]
    lib.winch_flex_set_swivels(wf, d(RADII), d(AXES), d(HOME))
    homed = arrays(p)[1]
    lib.winch_flex_set_swivels(wf, d(RADII), d(AXES), d(np.zeros(len(HOME))))
    lib.winch_flex_set_enabled(wf, 0)
    np.testing.assert_allclose(homed - plain, HOME * (swivel_lengths((0, 0, 0)) + GUY) / EA, atol=1e-9)


POSITION = re.compile(r'^X:([-\d.]+) Y:([-\d.]+) Z:([-\d.]+).* Count ([-\d ]+) Machine', re.M)


def rrf(code, tmp_path):
    shutil.copytree(ROOT / 'RRF/run/vsd/sys', tmp_path / 'sys')
    (tmp_path / 'gcodes').mkdir()
    (tmp_path / 'logs').mkdir()
    # The host file-input reader needs trailing data to execute short files.
    (tmp_path / 'gcodes/test.gcode').write_text(code + '\n' + '; padding\n' * 1000)
    result = subprocess.run([str(ROOT / 'RRF/build/rrf_simulator'), '--vsd', str(tmp_path), '--gcode',
                             'gcodes/test.gcode', '-c', 'sys/config_copacabana.g', '--can-log', 'logs/test.csv'],
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout + result.stderr


@pytest.mark.skipif(not (ROOT / 'RRF/build/rrf_simulator').exists(), reason='build RRF/build/rrf_simulator first')
def test_rrf_motor_steps_follow_swivel_lengths(tmp_path):
    target = np.array([600., 600., 0.])
    zero_axes = 'M669 V' + ':'.join(['0'] * 24)
    output = rrf(f'G28\nG1 X{target[0]} Y{target[1]} Z{target[2]} F1200\nM400\nM114\n{zero_axes}\n', tmp_path)
    (*position, counts), = POSITION.findall(output)
    np.testing.assert_allclose(np.array(position, float), target, atol=.05)
    counts = np.array(counts.split(), float)[:8]
    swivel = swivel_lengths(target) - swivel_lengths((0, 0, 0))
    pinhole_exits = np.array([reference(a, u, r, np.zeros(3))[1] for a, u, r in zip(ANCHORS, AXES, RADII)])
    pinhole = np.linalg.norm(pinhole_exits - target, axis=1) - np.linalg.norm(pinhole_exits, axis=1)
    steps_per_mm = 200 * 16 / (2 * np.pi * 49)  # J200, 16 microsteps, 1:1 gears, 49 mm drum
    assert np.max(abs(counts - steps_per_mm * swivel)) <= 1
    # The old fixed tangent points would be several motor steps off.
    assert np.max(abs(counts - steps_per_mm * pinhole)) > 5
    assert 'Swivel pulley A needs a nonzero V axis' in output
