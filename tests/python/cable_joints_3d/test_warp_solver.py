"""Differential coverage of the optional compiled solver's full ECS behavior."""
import json
from contextlib import nullcontext
from unittest.mock import patch

import pytest

wp = pytest.importorskip('warp')
from cable_joints_3d.pbd_cable_constraint_solver import PBDCableConstraintSolver
from cable_joints_3d.warp_solver import WarpCableConstraintSolver
from parity_harness import FIXTURES, assert_equivalent, run_python


@pytest.mark.parametrize('fixture_path', sorted(FIXTURES.glob('*.json')), ids=lambda p: p.stem)
def test_compiled_solver_matches_reference(fixture_path):
    fixture = json.loads(fixture_path.read_text())
    warning = lambda: pytest.warns(UserWarning, match='Insufficient available rest length') if fixture_path.stem == 'topology_split_abort' else nullcontext()
    with warning():
        reference = run_python(fixture)
    compiled = WarpCableConstraintSolver('cpu')
    with patch.object(PBDCableConstraintSolver, 'update', lambda self, world, dt: compiled.update(world, dt)):
        with warning():
            actual = run_python(fixture)
    assert_equivalent(actual, reference, **fixture['tolerance'], path=fixture_path.stem)


def test_requested_cuda_requires_a_device():
    if wp.is_cuda_available():
        pytest.skip('Unavailable-device check needs a CPU-only host')
    with pytest.raises(ValueError, match='no CUDA device'):
        WarpCableConstraintSolver('cuda:0')


def test_zero_cable_world_clears_previous_torque_loads():
    from cable_joints_3d.ecs import World
    world = World()
    world.set_resource('dt', .002)
    world.set_resource('torqueModeCableLoadTorques', {42: 1.})
    WarpCableConstraintSolver('cpu').update(world, .002)
    assert world.get_resource('torqueModeCableLoadTorques') == {}


def test_compiler_diagnostics_do_not_enter_stdout(capsys):
    from cable_joints_3d.ecs import World
    world = World()
    world.set_resource('dt', .002)
    compiled = WarpCableConstraintSolver('cpu')
    compiled.update(world, .002)
    assert capsys.readouterr().out == ''


@pytest.mark.parametrize('name', ['cable_solver_bodies', 'cable_solver_spools', 'cable_solver_pinhole',
                                 'machine_pipeline_hp3_rigid_body', 'machine_pipeline_hp4_loaded_torque'])
def test_cuda_solver_matches_reference_when_available(name):
    if not wp.is_cuda_available():
        pytest.skip('CUDA hardware unavailable; GPU parity remains unmeasured')
    fixture = json.loads((FIXTURES / f'{name}.json').read_text())
    reference = run_python(fixture)
    compiled = WarpCableConstraintSolver('cuda:0')
    with patch.object(PBDCableConstraintSolver, 'update', lambda self, world, dt: compiled.update(world, dt)):
        actual = run_python(fixture)
    assert_equivalent(actual, reference, **fixture['tolerance'], path=name)
