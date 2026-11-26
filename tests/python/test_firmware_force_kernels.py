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


@pytest.fixture(scope='module')
def rrf_tikhonov(tmp_path_factory):
    """Compile the real RRF Tikhonov body against the standalone flex harness."""
    directory = tmp_path_factory.mktemp('rrf-tikhonov')
    dev = ROOT / 'flex-compensation-dev/hangprinter-flex-compensation'
    source = (ROOT / 'RRF/ReprapFirmware/src/Movement/Kinematics/HangprinterKinematics.cpp').read_text()
    start = source.index('void HangprinterKinematics::StaticForcesTikhonov(')
    end = source.index('void HangprinterKinematics::StaticForcesQp(', start)
    body = source[start:end].replace('HangprinterKinematics::', 'Flex::').replace('HANGPRINTER_MAX_ROWS', '6')
    (directory / 'tikhonov.cpp').write_text('#include "force_solver.hpp"\n' + body)
    executable = directory / 'probe'
    subprocess.run(['g++', '-O2', '-std=c++20', f'-I{dev}', str(dev/'flex.cpp'),
        str(dev/'flex_qp.cpp'), str(directory/'tikhonov.cpp'), str(dev/'test_driver.cpp'),
        '-o', str(executable)], check=True)
    return executable


def test_rrf_tikhonov_keeps_soft_minimum_and_hard_physical_limits(rrf_tikhonov):
    def solve(position, mass=.081, upper=170., ignore_pretension=0):
        values = [5, 1, 0, ignore_pretension, mass, .001, 0., 0., 0, *position,
                  *ANCHORS.ravel(), *([3.]*5), *([upper]*5),
                  *([1.]*5), *([1.]*5), *([0.]*5), *([0.]*15)]
        result = subprocess.run([str(rrf_tikhonov)], input=' '.join(map(str, values))+'\n',
                                text=True, capture_output=True, check=True)
        return np.array(list(map(float, result.stdout.split())))[:5]
    tensions = solve([0., 0., 0.])
    assert .2 < max(3.-tensions) < .3
    for ignore in (0, 1):
        tensions = solve([1800., 300., 1800.], mass=10., upper=.1, ignore_pretension=ignore)
        assert np.all(tensions >= 0.) and np.all(tensions <= .100001)


@pytest.mark.parametrize('implementation', ['dev', 'rrf', 'klipper'])
def test_workspace_candidates_match_in_all_implementations(implementation, tmp_path):
    paths = {'dev':'forward-transform-dev/hangprinter-forward-transform/forward_workspace.h',
             'rrf':'RRF/ReprapFirmware/src/Movement/Kinematics/ForwardWorkspace.h',
             'klipper':'klipper/klippy/chelper/forward_workspace.h'}
    contents = [(ROOT / path).read_bytes() for path in paths.values()]
    assert contents[0] == contents[1] == contents[2]
    code = '#include "' + str(ROOT / paths[implementation]) + '"\n'
    code += r'''
int probe(const double *input, const double *ranges, double *output) {
    const double (*anchors)[3]=(const double (*)[3])input;
    double candidates[3][3], normal[3]; int planar=0;
    int count=hp_range_candidates(anchors,ranges,5,candidates,normal,&planar);
    double best=INFINITY;
    for (int i=0;i<count;++i) {
        if (!hp_forward_allowed(anchors,5,0,normal,planar,candidates[i])) continue;
        double cost=0;
        for (int k=0;k<5;++k) {
            double d[3];hp_difference(candidates[i],anchors[k],d);
            double residual=sqrt(hp_dot(d,d))-ranges[k];cost+=residual*residual;
        }
        if (cost<best) {best=cost;for(int j=0;j<3;++j)output[j]=candidates[i][j];}
    }
    return isfinite(best);
}
'''
    source, library = tmp_path/'seed.c', tmp_path/'seed.so'
    source.write_text(code)
    subprocess.run(['gcc', '-std=c11', '-shared', '-fPIC', str(source), '-lm', '-o', str(library)], check=True)
    solve = ctypes.CDLL(str(library)).probe
    array = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
    solve.argtypes = [array]*3
    position = np.array([10.,20.,-100.])
    for height in (1900., .01, 0.):
        anchors = ANCHORS.copy(); anchors[:,2]=0.; anchors[-1,2]=height
        output = np.zeros(3)
        assert solve(anchors.ravel(), np.linalg.norm(anchors-position, axis=1), output)
        np.testing.assert_allclose(output, position, atol=.001, rtol=0)
