"""Offline counterexamples for terminal validation at a2f47d14; no physics."""
import copy
import json
from pathlib import Path
import subprocess

import numpy as np
import pytest

import autocal.autocal as ac
import autocal.json_schema as schema
import autocal.terminal_validation as terminal
from autocal.stage_artifacts import read_stage_artifact
from autocal.stopping import StopDecision
from autocal.tests.test_autocal_no_collect import _fake_plan, _write_dataset
from autocal.tests.test_predictive_validation import _dataset
from autocal.tests.test_home_compensation import ANCHORS, evidence_for


def loop(tmp_path, monkeypatch, *, extra_args=(), action=None, machine='slideprinter', home=False, headless=False):
    path = tmp_path / 'training.json'
    if home:
        offsets, ev = evidence_for(np.array([0., 0., -0.3]))
        data = {'version': '1.0', 'machine_type': 'hangprinter_3', 'num_anchors': 4, 'dimensions': 3,
                'config': {'home_extension_evidence': {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in ev.items()}},
                'sweeps': [{'id': f'sweep_{i:03d}', 'fixed_anchors': [0, 3], 'fixed_lengths': [10., 10.],
                            'drive_anchor': 1, 'sensor_anchor': 2, 'data_points': [{'l_drive': 1., 'l_sensor': 2.}],
                            'metadata': {'sensor_force_profile': True}} for i in range(1, 4)]}
        ac._write_json(path, data)
    else:
        _write_dataset(path, sweeps=3)
    before = path.read_bytes()
    applied, seen = [], []
    def fit(data_path, **kwargs):
        plan = _fake_plan()
        if action == 'stop_during_fit':
            ac._full_auto_stop_path(path).touch()
        plan['dataset'] = ac._load_json(data_path)
        plan['machine_type'] = machine
        if not home:
            from autocal.spool_model import build_spool_model_params
            model_data = copy.deepcopy(plan['dataset'])
            for sweep in model_data['sweeps']:
                for point in sweep['data_points']:
                    point['raw_angles_deg'] = [0., 0., 0.]
            plan['spool_model_params'] = build_spool_model_params(
                model_data, base_radii_mm=[30.]*3, modeled_radii_mm=[41.]*3,
                modeled_buildup_factor=[0.]*3, spool_to_motor_gearing_factor=[1.]*3,
                mechanical_advantage=[1.]*3, lines_per_spool=[1.]*3)
        if home:
            plan['anchors'] = ANCHORS.copy()
            plan['home_offsets_mm'] = offsets.tolist()
            plan['home_offsets_fitted'] = True
        plan['length_model'] = {'effective_radii_mm': [41.] * (4 if home else 3), 'modeled_buildup_factor': [0.]}
        plan['collect_command'] += ['--output-file', str(tmp_path / 'planned.json'), *kwargs['collector_args']]
        return plan
    real_run = subprocess.run
    def collect(command, **kwargs):
        if command[0] != 'node':
            return real_run(command, **kwargs)
        seen.append(command.copy())
        # The real parseBridgeArgs last-wins behavior is checked separately below.
        dest = None
        for i, arg in enumerate(command[:-1]):
            if arg in ('--output-file', '--output', '--out'):
                dest = command[i + 1]
        if action == 'stop_during_collect':
            ac._full_auto_stop_path(path).touch()
        if home:
            one = copy.deepcopy(data)
            one['sweeps'] = one['sweeps'][:1]
            ac._write_json(Path(dest), one)
        else:
            _write_dataset(Path(dest), sweeps=1)
    def send(_self, _server, cmd):
        applied.append(cmd)
        if action == 'interrupt_send' and len(applied) == 1:
            raise KeyboardInterrupt()
        return ''
    monkeypatch.setattr(ac, 'plan_next_ellipse_sweep', fit)
    monkeypatch.setattr(ac, '_print_ellipse_plan', lambda *a, **kw: None)
    monkeypatch.setattr(ac, '_plan_hits_underconstrained_penalty', lambda *a: False)
    monkeypatch.setattr(ac.StoppingPolicy, 'update', lambda *a, **kw: StopDecision(1, True, False, None, None, 'accept'))
    monkeypatch.setattr(ac.subprocess, 'run', collect)
    monkeypatch.setattr(ac.RRFFirmwareProvider, 'send_gcode', send)
    def score(*a, **kw):
        if action == 'interrupt_score':
            raise KeyboardInterrupt()
        return (float('inf'), 1) if action == 'failed_score' else (.1, 1)
    monkeypatch.setattr(terminal, 'score_future_sweeps', score)
    argv = ['--sim', '--machine-type', machine, '--dataset', str(path), '--terminal-validation', '--max-steps', '3',
            '--collector-args', '--server', 'http://127.0.0.1:12345', '--no-ws', *extra_args]
    if not home:
        index = argv.index('--collector-args')
        argv[index:index] = ['--no-fit-home-offsets', '--find-radii', 'global']
    if headless:
        argv.insert(argv.index('--collector-args'), '--headless-sim')
    return path, before, applied, seen, lambda: ac.main(argv)


def test_collector_parser_last_wins_without_loading_deps():
    # Evaluate the exact parser function source; do not import bridge dependencies or start a server.
    code = r'''const fs = require('fs');
const source = fs.readFileSync('autocal/control/cli/collect_sweep_data.mjs', 'utf8');
const start = source.indexOf('export function parseBridgeArgs(');
const end = source.indexOf('\nfunction printHelp()', start);
eval(source.slice(start, end).replace('export function', 'function') + `\nconsole.log(JSON.stringify(parseBridgeArgs(['--output-file','fresh.json','--out','training.json','--sweep-config-file','planned.cfg','--sweepFile','other.cfg','--fixed-targets','0','--fixedTargets','100','--auto-tune-force','--hp-sim-reset'])));`);'''
    result = subprocess.run(['node', '-e', code], check=True, text=True, capture_output=True)
    parsed = json.loads(result.stdout)
    assert (parsed['outputFile'], parsed['sweepConfigFile'], parsed['fixedTargets']) == ('training.json', 'other.cfg', '100')
    assert parsed['autoTuneForce'] and parsed['hpSimReset']


def test_alias_can_overwrite_training_before_missing_terminal_is_reported(tmp_path, monkeypatch):
    training = tmp_path / 'training.json'
    path, before, applied, seen, run = loop(tmp_path, monkeypatch, extra_args=['--out', str(training)])
    with pytest.raises(FileNotFoundError):
        run()
    assert path.read_bytes() != before
    assert len(ac._load_json(path)['sweeps']) == 1
    assert len(seen) == 1 and not applied


def test_stop_file_during_collection_still_applies(tmp_path, monkeypatch):
    path, before, applied, seen, run = loop(tmp_path, monkeypatch, action='stop_during_collect')
    assert run() == 0
    assert ac._full_auto_stop_path(path).exists() and len(applied) == 1
    assert path.read_bytes() == before
    assert len(seen) == 1


def test_interrupt_on_applying_terminal_model_retries_old_selection(tmp_path, monkeypatch):
    _, _, applied, _, run = loop(tmp_path, monkeypatch, action='interrupt_send')
    assert run() == 0
    assert len(applied) == 2 and applied[0] == applied[1]


def test_non_headless_does_not_send_scored_spool_model(tmp_path, monkeypatch):
    path, _, applied, _, run = loop(tmp_path, monkeypatch)
    assert run() == 0
    artifact = read_stage_artifact(path.with_suffix('.terminal_validation.stage.json'))['payload']
    saved_plan = artifact['candidates'][0]['plan']
    assert saved_plan['length_model']['effective_radii_mm'] == [41.] * 3
    assert saved_plan['spool_model_params'].modeled_models[0].base_radius == 41.
    assert len(applied) == 1 and applied[0].startswith('M669 ')
    assert not any(cmd.startswith('M666 ') for cmd in applied)


def test_independent_home_evidence_translates_after_validation_and_counterfactual(tmp_path, monkeypatch):
    path, _, applied, _, run = loop(tmp_path, monkeypatch, machine='hangprinter_3', home=True)
    assert run() == 0
    artifact = read_stage_artifact(path.with_suffix('.terminal_validation.stage.json'))['payload']
    scored = artifact['candidates'][0]['plan']['anchors']
    original = artifact['original_selection']['m669']
    assert original == ac._format_m669_command(scored, 'hangprinter_3')
    actual = np.array([[float(n) for n in part[1:].split(':')] for part in applied[0].split()[1:]])
    np.testing.assert_allclose(actual, scored + [0., 0., 0.3], atol=0.0051)
    assert applied[0] != original


def test_terminal_rewrite_of_collected_data_is_not_atomic(tmp_path, monkeypatch):
    path = tmp_path / 'terminal.json'
    _write_dataset(path, sweeps=1)
    payload = ac._load_json(path)
    def fail(obj, handle, **kwargs):
        handle.write('{"partial":')
        raise OSError('mock interrupted write')
    monkeypatch.setattr(schema.json, 'dump', fail)
    with pytest.raises(OSError):
        ac._write_json(path, payload)
    assert path.read_text() == '{"partial":'


def test_projected_training_can_be_scored_against_nonprojected_validation():
    data, anchors = _dataset()
    training = copy.deepcopy(data)
    for p in training['sweeps'][0]['data_points']:
        p['raw_angles_zero_tension_deg'] = [0., 0., 0.]
    candidate = {'iteration': 1, 'rank_score': 1., 'training_sweep_ids': ['train'],
                 'plan': {'anchors': anchors, 'dataset': training}, 'summary_meta': {}}
    data['sweeps'][0]['id'] = 'terminal_001'
    scored = terminal.evaluate_terminal_candidates([candidate], data)
    assert np.isfinite(scored[0][2]['prediction_score'])


def test_common_scorer_rejects_matching_ids_but_not_renamed_copied_content():
    data, anchors = _dataset()
    training = copy.deepcopy(data)
    candidate = {'iteration': 1, 'rank_score': 1., 'training_sweep_ids': ['heldout'],
                 'plan': {'anchors': anchors, 'dataset': training}, 'summary_meta': {}}
    with pytest.raises(ValueError, match='overlap'):
        terminal.evaluate_terminal_candidates([candidate], data)
    data['sweeps'][0]['id'] = 'terminal_001'
    assert np.isfinite(terminal.evaluate_terminal_candidates([candidate], data)[0][2]['prediction_score'])


def test_explicit_force_overrides_cause_retuning_despite_saved_force_values():
    data = {"config": {"force_tuning": {"force_low_n": .1, "force_mid_n": .2, "force_max_n": .3}}}
    for flags in (["--auto-tune-force"], ["--force-low", "0.1"]):
        args, tuning, applied = ac._inject_force_args(data, flags)
        assert args == flags and tuning is not None and not applied
    # Extract the exact auto-tune predicate from collector source.
    code = r'''const fs = require('fs');
const s = fs.readFileSync('autocal/control/behaviors/sweep_data_collection.mjs', 'utf8');
const line = s.match(/const autoTuneForce = input\.autoTuneForce[^;]+;/)[0];
const fn = new Function('input', line + ' return autoTuneForce;');
console.log(JSON.stringify([fn({autoTuneForce:true,noAutoTuneForce:false,forceArgsProvided:false}), fn({autoTuneForce:false,noAutoTuneForce:false,forceArgsProvided:false})]));'''
    result = subprocess.run(['node', '-e', code], check=True, text=True, capture_output=True)
    assert json.loads(result.stdout) == [True, True]
    args, _, applied = ac._inject_force_args(data, ['--torque-min', '9'])
    assert applied and args[-1] == '--no-auto-tune-force' and '--force-mid' in args


@pytest.mark.parametrize('action,expected', [('interrupt_score', 130), ('failed_score', 2)])
def test_interrupted_or_failed_scoring_does_not_send_candidate(tmp_path, monkeypatch, action, expected):
    path, before, applied, _, run = loop(tmp_path, monkeypatch, action=action)
    assert run() == expected and not applied and path.read_bytes() == before
    if action == 'failed_score':
        payload = read_stage_artifact(path.with_suffix('.terminal_validation.stage.json'))['payload']
        assert payload['selected'] is None


def test_cli_terminal_flag_is_default_off():
    parser = ac.build_semi_auto_parser()
    assert parser.parse_args(['--machine-type', 'slideprinter']).terminal_validation is False
    assert parser.parse_args(['--machine-type', 'slideprinter', '--terminal-validation']).terminal_validation is True


def test_spool_collection_options_issue_firmware_adjustment_before_scoring():
    args, changed = ac._inject_spool_collection_args([], find_radii_mode='global', find_buildup_mode='off',
                                                      base_radii=[30., 30., 30.], buildup_factor=0.)
    assert changed and '--force-base-radii' in args and '--force-buildup-factor' in args
    code = r'''const fs = require('fs');
const s = fs.readFileSync('autocal/control/behaviors/sweep_data_collection.mjs', 'utf8');
const a = s.indexOf('function formatM666Number(');
const b = s.indexOf('// In case spool was fully wound out', a);
eval(s.slice(a,b).replaceAll('export function', 'function') + `\nconsole.log(buildM666AdjustmentCommand({forcedBaseRadii:[30,30,30],forcedBuildupFactor:0}));`);'''
    result = subprocess.run(['node', '-e', code], check=True, text=True, capture_output=True)
    assert result.stdout.strip() == 'M666 R30:30:30 Q0'


@pytest.mark.parametrize('projected', [False, True])
def test_terminal_spool_models_are_frozen_and_roundtrip_replay(tmp_path, monkeypatch, projected):
    from autocal.spool_model import WinchSpoolModel, build_spool_model_params
    from autocal.stage_artifacts import _encode, write_stage_artifact
    import autocal.tools.replay_stage as replay
    data, anchors = _dataset()
    data['sweeps'][0]['id'] = 'terminal_001'
    spool = WinchSpoolModel.from_firmware(base_radius=40., buildup_factor=.636619,
                                           spool_to_motor_gearing_factor=1., mechanical_advantage=1.,
                                           lines_per_spool=1.)
    for point in data['sweeps'][0]['data_points']:
        angles = [spool.linepos_mm_to_theta_deg(x) for x in (200., point['l_drive'], point['l_sensor'])]
        point['raw_angles_deg'] = angles
        if projected:
            point['raw_angles_zero_tension_deg'] = angles.copy()
            point['l_drive_mu'] = point['l_drive'] + 50.
            point['l_sensor_mu'] = point['l_sensor'] - 50.
    candidates = []
    for iteration, radius in enumerate((35., 40.), 1):
        params = build_spool_model_params(data, base_radii_mm=[30.]*3, modeled_radii_mm=[radius]*3,
                                         modeled_buildup_factor=[.636619]*3,
                                         spool_to_motor_gearing_factor=[1.]*3, mechanical_advantage=[1.]*3,
                                         lines_per_spool=[1.]*3)
        candidates.append({'iteration': iteration, 'run_id': 'default', 'rank_score': float(iteration),
                           'training_sweep_ids': ['training'], 'summary_meta': {},
                           'plan': {'anchors': anchors.copy(), 'spool_model_params': params,
                                    'home_offsets_mm': [0., 0., 0.]}})
    before = json.dumps(_encode([data, candidates]), sort_keys=True)
    scored = terminal.evaluate_terminal_candidates(candidates, data)
    assert scored[1][2]['prediction_score'] < scored[0][2]['prediction_score']
    assert terminal.select_terminal_plan(scored)[1]['iteration'] == 2
    assert json.dumps(_encode([data, candidates]), sort_keys=True) == before
    artifact, output = tmp_path / 'stage.json', tmp_path / 'replay.json'
    write_stage_artifact(artifact, 'terminal_validation', {'candidates': candidates, 'validation_dataset': data,
                                                          'validation_options': {}, 'original_selection': {'m669': 'original'}})
    monkeypatch.setattr(replay, 'fit_ellipse_dataset', lambda *a, **kw: pytest.fail('fit called during replay'))
    assert replay.main(['terminal', str(artifact), '--output', str(output)]) == 0
    payload = read_stage_artifact(output)['payload']
    assert payload['selected']['iteration'] == 2
    assert payload['original_selection']['m669'] == 'original'
    assert [x[2]['prediction_score'] for x in payload['evaluated']] == pytest.approx([x[2]['prediction_score'] for x in scored])


@pytest.mark.parametrize('code,status', [(2, 'failed'), (130, 'interrupted')])
def test_headless_terminal_failure_manifest_is_not_normal(tmp_path, code, status):
    from argparse import Namespace
    from autocal.headless_sim import HeadlessSimulation
    args = Namespace(machine_type='hangprinter_3', find_radii='off', find_buildup_factor='off',
                     rrf_config=None, config=None, dataset=tmp_path / 'training.json', keep_sim_alive=False)
    service = HeadlessSimulation(args, [])
    service.manifest.update(stop_reason='terminal-validation-failed', exit_code=code)
    service.__exit__(None, None, None)
    assert service.manifest['status'] == status and not service.manifest['normal_completion']
    assert not service.manifest.get('applied_parameters')


def test_stop_file_appearing_during_fit_is_recorded_as_normal_patience(tmp_path, monkeypatch):
    import autocal.headless_sim as headless_module
    services = []
    class FakeHeadless:
        def __init__(self, args, collector_args):
            self.manifest, self.directory, self.applied = {}, tmp_path, []
            services.append(self)
        def __enter__(self):
            return self
        def __exit__(self, *args):
            return False
        def apply_parameters(self, commands):
            self.applied.extend(commands)
    monkeypatch.setattr(headless_module, 'HeadlessSimulation', FakeHeadless)
    path, _, _, seen, run = loop(tmp_path, monkeypatch, action='stop_during_fit', headless=True)
    assert run() == 0
    assert ac._full_auto_stop_path(path).exists() and not seen
    assert services[0].manifest['stop_reason'] == 'patience'
    assert services[0].applied[0].startswith('M669 ')
