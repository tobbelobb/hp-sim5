"""Keep unsupported runtimes and reused outputs from spending collection budgets."""
from pathlib import Path
import pytest
from autocal.tools import collect_native_parameter_pair as pair


def test_unsupported_node_fails_before_output_or_simulation(tmp_path, monkeypatch):
    monkeypatch.setattr(pair.shutil, 'which', lambda _: '/node24')
    monkeypatch.setattr(pair.subprocess, 'check_output', lambda *a, **k: 'v24.19.0\n')
    monkeypatch.setattr(pair, 'HeadlessSimulation', lambda *a, **k: pytest.fail('must not start physics'))
    destination=tmp_path/'unused'
    with pytest.raises(SystemExit) as error:
        pair.main(['--output',str(destination)])
    assert error.value.code == 2
    assert not destination.exists()


def test_existing_output_fails_before_simulation(tmp_path, monkeypatch):
    monkeypatch.setattr(pair, 'node_identity', lambda: ('/node23','v23.11.1'))
    monkeypatch.setattr(pair, 'HeadlessSimulation', lambda *a, **k: pytest.fail('must not start physics'))
    marker=tmp_path/'keep';marker.write_text('prior evidence')
    with pytest.raises(FileExistsError):
        pair.main(['--output',str(tmp_path)])
    assert marker.read_text() == 'prior evidence'


def test_node_identity_records_resolved_command(monkeypatch):
    monkeypatch.setattr(pair.shutil, 'which', lambda _: '/pinned/node')
    def command(argv, **kwargs):
        assert argv == ['/pinned/node','--version']
        return 'v23.11.1\n'
    monkeypatch.setattr(pair.subprocess,'check_output',command)
    assert pair.node_identity() == ('/pinned/node','v23.11.1')


def test_scene_field_readback_uses_all_cable_paths():
    source=(Path(__file__).resolve().parents[2]/'public/usd_scenes/hp3_rigid_body.usda').read_text()
    assert pair.fields(source,'stiffness') == [20000.]*9
    assert pair.fields(source,'halfWidth') == [.001]*9


def test_nonfinite_or_missing_measurements_never_count_as_complete():
    point={'l_drive':1.,'l_sensor':2.,'raw_angles_deg':[0.,1.,2.,3.]}
    data={'sweeps':[{'data_points':[point]}]}
    assert pair.finite_measurements(data)
    point['raw_angles_deg'][0]=float('nan')
    assert not pair.finite_measurements(data)
    assert not pair.finite_measurements({'sweeps':[]})
    assert not pair.finite_measurements({'sweeps':[{'data_points':[{}]}]})


def test_keyboard_interrupt_does_not_launch_second_world(tmp_path, monkeypatch):
    monkeypatch.setattr(pair, 'node_identity', lambda: ('/node23','v23.11.1'))
    worlds=[]
    def simulation(*args):
        worlds.append(args)
        raise KeyboardInterrupt
    monkeypatch.setattr(pair,'HeadlessSimulation',simulation)
    monkeypatch.setenv('HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS', 'prior-limit')
    with pytest.raises(KeyboardInterrupt):
        pair.main(['--output',str(tmp_path/'interrupted')])
    assert len(worlds) == 1
    assert pair.os.environ['HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS'] == 'prior-limit'
