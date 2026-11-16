"""Compare the actual firmware QP kernels with an independent bounded solve."""
import ctypes
from pathlib import Path
import re
import subprocess

import numpy as np
import pytest
from scipy.optimize import lsq_linear

ROOT = Path(__file__).resolve().parents[2]
ANCHORS = np.array([[0., -1900., -280.], [1900., 0., -280.], [0., 1900., -280.],
                    [-1900., 0., -280.], [0., 0., 1900.]])


def function(source, name):
    match = re.search(r'static\s+(?:inline\s+)?(?:void|bool|int)\s+' + name + r'\([^;]*?\)\s*\{', source)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


@pytest.fixture(scope='module', params=['rrf', 'klipper'])
def kernel(request, tmp_path_factory):
    rrf = request.param == 'rrf'
    path = ('RRF/ReprapFirmware/src/Movement/Kinematics/HangprinterKinematics.cpp'
            if rrf else 'klipper/klippy/chelper/kin_winch.c')
    source = (ROOT / path).read_text()
    names = ['chol_decompose', 'chol_solve', 'factor_gram', 'solve_box_min_norm'] if rrf else [
        'chol_decompose_spd', 'chol_solve_spd', 'factor_gram', 'solve_box_min_norm']
    scalar = 'float' if rrf else 'double'
    code = '''#include <cmath>
#include <algorithm>
#define HANGPRINTER_MAX_ANCHORS 8
#define HANGPRINTER_MAX_ROWS 6
#define WINCH_MAX_ANCHORS 26
#define WINCH_MAX_ROWS 6
'''
    code += '\n'.join(function(source, name) for name in names)
    code += f'''
extern "C" void probe(int rows, int n, const double *matrix, const double *load,
                      const double *lower, const double *upper, double *result) {{
    {scalar} a[6*26], f[6], l[26], u[26], t[26];
    for (int i=0; i<rows*n; ++i) a[i]=matrix[i];
    for (int i=0; i<rows; ++i) f[i]=load[i];
    for (int i=0; i<n; ++i) {{ l[i]=lower[i]; u[i]=upper[i]; }}
    solve_box_min_norm(a,rows,n,f,.001,l,u,100,.001,t);
    for (int i=0; i<n; ++i) result[i]=t[i];
}}
'''
    directory = tmp_path_factory.mktemp('firmware-force')
    cpp, library = directory / 'probe.cpp', directory / 'probe.so'
    cpp.write_text(code)
    subprocess.run(['g++', '-std=c++17', '-O2', '-shared', '-fPIC', str(cpp), '-o', str(library)], check=True)
    solve = ctypes.CDLL(str(library)).probe
    array = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
    solve.argtypes = [ctypes.c_int, ctypes.c_int, *([array]*5)]
    return solve, rrf


def test_qp_continues_when_a_released_cable_reaches_its_opposite_bound(kernel):
    solve, rrf = kernel
    cases = [(np.array([359.94156306,146.46334715,365.59173919]), 1.4498027524,
        np.array([14.3142437,2.13110311,10.74169376,4.82548273,10.33616582]),
        np.array([39.94422401,58.98935271,67.59582286,47.95212782,19.38067176]))]
    rng = np.random.default_rng(5505)
    cases += [(rng.uniform([-1400,-1400,-200], [1400,1400,1700]), rng.uniform(0,3),
               rng.uniform(0,15,5), rng.uniform(16,80,5)) for _ in range(60)]
    for position, mass, lower, upper in cases:
        vectors = ANCHORS - position
        matrix = np.ascontiguousarray((vectors / np.linalg.norm(vectors, axis=1)[:,None]).T)
        load = np.array([0., 0., mass*9.81])
        attachments = np.array([[0.,-200.,0.], [200.,0.,0.], [0.,200.,0.], [-200.,0.,0.], [0.,0.,100.]])
        moment = np.cross(attachments, matrix.T).T / 200.
        for matrix, load in ((matrix, load), (np.ascontiguousarray(np.vstack([matrix, moment])), np.r_[load, np.zeros(3)])):
            if rrf:
                matrix, load, lower, upper = [v.astype(np.float32).astype(np.float64) for v in (matrix, load, lower, upper)]
            result = np.zeros(5)
            solve(len(load), 5, matrix, load, lower, upper, result)
            reference = lsq_linear(np.vstack([matrix, np.sqrt(.001)*np.eye(5)]),
                np.r_[load, np.zeros(5)], bounds=(lower, upper), tol=1e-12).x
            np.testing.assert_allclose(result, reference, atol=.002, rtol=0)
            assert np.all(result >= lower-1e-5) and np.all(result <= upper+1e-5)
