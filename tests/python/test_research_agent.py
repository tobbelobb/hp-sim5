import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest
from rerun.chunk import RrdReader

from hp_sim5_research.experiments import compare_runs, read_run, run_experiment

ROOT = Path(__file__).resolve().parents[2]
SCENE = 'public/usd_scenes/hp4_rigid_body.usda'


@pytest.fixture
def experiment_root(tmp_path):
    target = tmp_path / SCENE
    target.parent.mkdir(parents=True)
    shutil.copyfile(ROOT / SCENE, target)
    (tmp_path / 'src').mkdir()
    (tmp_path / 'src/python').symlink_to(ROOT / 'src/python', target_is_directory=True)
    return tmp_path


def test_real_physics_measurements_recording_and_reproducible_inputs(experiment_root):
    root = experiment_root
    commands = [{'type': 'Move', 'A': i * .0003} for i in range(30)]
    first = run_experiment(root, steps=30, commands=commands)
    second = run_experiment(root, steps=30, commands=commands, record=False)
    assert first['status'] == second['status'] == 'complete'
    assert first['metrics'] == second['metrics']
    assert first['metrics']['peak_cable_force_n'] > 0
    assert max(first['metrics']['effector_displacement_m'].values()) > 0
    assert first['scene_sha256'] == second['scene_sha256']
    assert first['python_source_sha256'] == second['python_source_sha256']
    assert read_run(root, first['run_id']) == first
    rows = read_run(root, first['run_id'], 0, 100)['samples']
    assert [row['step'] for row in rows] == list(range(31))
    assert rows[-1]['sim_time_s'] == .06
    assert rows[-1]['motors'] and rows[-1]['cables']
    assert any(motor['target_angle_rad'] == commands[-1]['A'] for motor in rows[-1]['motors'])
    rrd = Path(first['artifacts']['rrd'])
    assert RrdReader(rrd).blueprints()
    cable = rows[-1]['cables'][0]
    chunks = RrdReader(rrd).stream().filter(content='/line_lengths/' + cable['name'] + '/actual',
                                           components='Scalars:scalars').to_chunks()
    recorded_steps = [step for chunk in chunks for step in chunk.to_record_batch().column('sim_step').to_pylist()]
    assert sorted(recorded_steps) == list(range(31))
    comparison = compare_runs(root, first['run_id'], second['run_id'])
    assert comparison['changed_inputs'] == ['record']
    assert all(delta == 0 for delta in comparison['metric_delta_candidate_minus_baseline'].values())
    # Frozen scenes can reproduce physics after the original authored file changes.
    (root / SCENE).write_text('invalid replacement')
    frozen = run_experiment(root, first['artifacts']['scene'], steps=30, commands=commands, record=False)
    assert frozen['metrics'] == first['metrics']


def test_movement_and_torque_changes_are_visible_and_comparison_is_honest(experiment_root):
    root = experiment_root
    baseline = run_experiment(root, steps=200, record=False)
    trial = run_experiment(root, steps=200, commands=[{'type': 'Move', 'A': .01}], record=False)
    difference = compare_runs(root, baseline['run_id'], trial['run_id'])
    assert difference['changed_inputs'] == ['commands_sha256']
    assert max(difference['final_effector_distance_m'].values()) > 1e-6
    torque = run_experiment(root, steps=3, commands=[{'type': 'SetTorqueMode', 'axis': 'A', 'torqueNm': .003},
                                                   {}, {'type': 'SetPositionMode', 'axis': 'A'}], record=False)
    rows = read_run(root, torque['run_id'], 0, 4)['samples']
    axis = lambda row: next(motor for motor in row['motors'] if motor['axis'] == 'A')
    assert axis(rows[1])['mode'] == 'torque'
    assert axis(rows[1])['tracking_error_rad'] is None
    assert axis(rows[3])['mode'] == 'position'
    with pytest.raises(ValueError, match='identical steps and dt'):
        compare_runs(root, baseline['run_id'], torque['run_id'])


@pytest.mark.parametrize('options, message', [
    ({'steps': 0}, 'steps must'), ({'steps': True}, 'steps must'),
    ({'steps': 10_001}, 'steps must'), ({'dt': float('nan')}, 'dt must'),
    ({'commands': [{'type': 'Move', 'Q': 1}]}, 'Unknown command'),
    ({'commands': [{'type': 'Move', 'A': float('inf')}]}, 'finite'),
    ({'commands': [{'type': 'SetTorqueMode', 'axis': 'Q'}]}, 'known axis'),
    ({'commands': [{'A': .1}]}, 'require type'),
    ({'commands': [{'type': 'G1', 'A': .1}]}, 'Unsupported command'),
    ({'commands': [None]}, 'must be an object'),
])
def test_invalid_experiments_do_not_write_success_artifacts(experiment_root, options, message):
    with pytest.raises(ValueError, match=message):
        run_experiment(experiment_root, record=False, **options)
    assert not (experiment_root / 'output').exists()


def test_input_and_artifact_path_boundaries(experiment_root):
    with pytest.raises(ValueError, match='inside the repository'):
        run_experiment(experiment_root, str(ROOT / SCENE), record=False)
    with pytest.raises(ValueError, match='run_id'):
        read_run(experiment_root, '../outside')


@pytest.mark.asyncio
async def test_stdio_mcp_discovery_execution_readback_and_error():
    from mcp import Client, StdioServerParameters
    parameters = StdioServerParameters(command=sys.executable, args=[str(ROOT / 'scripts/hp_sim5_mcp.py')])
    async with Client(parameters, read_timeout_seconds=30) as client:
        tools = (await client.list_tools()).tools
        assert {tool.name for tool in tools} == {'capabilities', 'run_experiment', 'read_experiment', 'compare_experiments',
                                               'runtime_status', 'send_gcode', 'capture_native_context', 'reset_session',
                                               'step_physics', 'start_browser_service', 'start_collection', 'collection_status', 'cancel_collection',
                                               'browser_status', 'browser_action'}
        result = await client.call_tool('capabilities', {})
        assert not result.is_error
        assert SCENE in result.structured_content['scenes']
        result = await client.call_tool('run_experiment', {'steps': 3, 'record': False})
        assert not result.is_error, result.content
        run = result.structured_content
        assert run['status'] == 'complete'
        result = await client.call_tool('read_experiment', {'run_id': run['run_id'], 'start_step': 2, 'limit': 2})
        assert [row['step'] for row in result.structured_content['samples']] == [2, 3]
        result = await client.call_tool('compare_experiments', {'baseline_id': run['run_id'], 'candidate_id': run['run_id']})
        assert result.structured_content['changed_inputs'] == []
        captured = await client.call_tool('capture_native_context', {'message': 'Investigate this selected step',
            'run_id': run['run_id'], 'step': 1, 'selected_entity': 'effector'})
        assert not captured.is_error, captured.content
        assert captured.structured_content['sim_step'] == 1
        assert captured.structured_content['run_id'] == run['run_id']
        assert captured.structured_content['backend'] == 'native-python'
        result = await client.call_tool('run_experiment', {'commands': [{'type': 'Move', 'A': 'bad'}]})
        assert result.is_error


def test_launcher_dry_run_works_outside_repo_and_preserves_prompt():
    prompt = 'Compare $targets and `angles`, with "quoted" names.\nRetain failed trials.'
    result = subprocess.run([str(ROOT / 'hp-sim5-research-agent'), '--dry-run', '--viewer', 'none', '--prompt', prompt],
                            cwd='/tmp', capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    launch = json.loads(result.stdout)
    assert prompt in launch['prompt']
    assert 'workspace-write' in launch['argv']
    assert '--dangerously-bypass-approvals-and-sandbox' not in launch['argv']
    assert 'mcp_servers.rerun.enabled=false' in launch['argv']
    assert 'model_provider="openai"' in launch['argv']
    assert f'mcp_servers.hp_sim5.command="{ROOT / ".venv/bin/python"}"' in launch['argv']


@pytest.mark.parametrize('options, enabled', [
    ([], True), (['--dont-ask'], False), (['--batch'], False),
    (['--batch', '--dont-ask'], False), (['--serve'], True),
    (['--serve', '--dont-ask'], False),
])
def test_launcher_clarification_setting(options, enabled):
    command = [str(ROOT / 'hp-sim5-research-agent'), '--dry-run', '--viewer', 'none', *options]
    if '--serve' not in options:
        command += ['--prompt', 'Research check']
    result = subprocess.run(command, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    argv = json.loads(result.stdout)['argv']
    assert f'features.default_mode_request_user_input={str(enabled).lower()}' in argv
    assert '--dont-ask' not in argv


@pytest.mark.parametrize('batch', [False, True])
def test_sidebar_hook_uses_lifecycle_identity_without_model_instructions(batch):
    import tomllib
    section = '6f8f42eb-e2ae-4882-a538-5bf0202ec59b'
    result = subprocess.run([str(ROOT / 'hp-sim5-research-agent'), '--dry-run', '--viewer', 'none',
                             '--sidebar-section', section, '--prompt', 'Research check'] + (['--batch'] if batch else []),
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    argv = json.loads(result.stdout)['argv']
    config = next(value for value in argv if value.startswith('hooks.UserPromptSubmit='))
    hook = tomllib.loads(config)['hooks']['UserPromptSubmit'][0]['hooks'][0]
    assert hook['type'] == 'mcp_tool' and hook['server'] == 'codex_app'
    assert hook['tool'] == 'move_thread_to_sidebar_section'
    assert hook['input'] == {'threadId': '${session_id}', 'sectionId': section}
    instructions = next(value for value in argv if value.startswith('developer_instructions='))
    assert 'move_thread_to_sidebar_section' not in instructions
    assert section not in instructions
    disabled = subprocess.run([str(ROOT / 'hp-sim5-research-agent'), '--dry-run', '--sidebar-section', 'none'],
                              capture_output=True, text=True, timeout=15)
    assert disabled.returncode == 0, disabled.stderr
    assert not any(value.startswith('hooks.') for value in json.loads(disabled.stdout)['argv'])


@pytest.mark.parametrize('real_runtime', [False, pytest.param(True, marks=pytest.mark.slow)])
@pytest.mark.parametrize('batch', [False, True])
def test_launcher_preserves_written_report_and_cleans_api_key_environment(tmp_path, monkeypatch, real_runtime, batch):
    codex = tmp_path / 'codex'
    codex.write_text(f'#!{sys.executable}\n' + '''
import json
from pathlib import Path
import os
import sys
assert 'OPENAI_API_KEY' not in os.environ and 'CODEX_API_KEY' not in os.environ
if sys.argv[1:] == ['login', 'status']:
    print('Logged in using ChatGPT')
else:
    batch = 'exec' in sys.argv
    prompt = sys.stdin.read() if batch else sys.argv[-1]
    assert 'preservation check' in prompt
    settings = [value for value in sys.argv if value.startswith('developer_instructions=')]
    instructions = json.loads(settings[0].split('=', 1)[1])
    session = Path(instructions.split('Session artifacts: ')[1].splitlines()[0])
    final = session / 'final-message.md'
    if batch:
        assert sys.argv[sys.argv.index('--output-last-message') + 1] == str(final)
    else:
        assert '--json' not in sys.argv and sys.stdin.fileno() == 0
    (final.parent / 'report.md').write_text('Full research evidence.\\n')
    final.write_text('Short final response.\\n')
    print(json.dumps({'type': 'item.completed', 'item': {'type': 'agent_message', 'text': 'Done'}}))
''')
    codex.chmod(0o755)
    import importlib.util
    spec = importlib.util.spec_from_file_location('research_agent', ROOT / 'scripts/research_agent.py')
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    services = []

    class Service:
        endpoint = 'http://127.0.0.1:12345'
        token = 'test-runtime-secret'

        def __init__(self, root, directory, viewer_endpoint):
            self.directory = directory
            self.closed = False
            services.append(self)

        def start(self):
            return self

        def call(self, operation):
            assert operation == 'status', 'Startup must leave experiment selection to the agent'
            return {'connected': True, 'step': 0, 'queue_length': 0, 'session_id': 'test-session'}

        def close(self):
            self.closed = True

    if real_runtime:
        runtime_service = launcher.RuntimeService

        def start_real_service(root, directory, viewer_endpoint):
            service = runtime_service(root, directory, viewer_endpoint)
            services.append(service)
            return service

        monkeypatch.setattr(launcher, 'RuntimeService', start_real_service)
    else:
        monkeypatch.setattr(launcher, 'RuntimeService', Service)

    def forbidden_doctor():
        pytest.fail('Normal research startup must not run doctor or collect sweeps')

    monkeypatch.setattr(launcher, 'doctor', forbidden_doctor)
    monkeypatch.setenv('PATH', str(tmp_path) + os.pathsep + os.environ['PATH'])
    monkeypatch.setenv('OPENAI_API_KEY', 'test-sentinel')
    monkeypatch.setenv('CODEX_API_KEY', 'test-sentinel')
    monkeypatch.setattr(sys, 'argv', ['research_agent.py', '--viewer', 'none', '--prompt', 'preservation check',
                                    '--sidebar-section', '6f8f42eb-e2ae-4882-a538-5bf0202ec59b'] + (['--batch'] if batch else []))
    assert launcher.main() == 0
    if real_runtime:
        assert services[0].process.poll() == 0
    else:
        assert services[0].closed
    session = services[0].directory.parent
    assert (session / 'report.md').read_text() == 'Full research evidence.\n'
    assert (session / 'final-message.md').read_text() == 'Short final response.\n'
    assert json.loads((session / 'exit.json').read_text()) == {'returncode': 0}
    if batch:
        assert json.loads((session / 'events.jsonl').read_text())['item']['text'] == 'Done'
    else:
        assert not (session / 'events.jsonl').exists()
    assert (session / 'connection.json').stat().st_mode & 0o077 == 0
    assert 'test-runtime-secret' not in (session / 'attachment.toml').read_text()
    import tomllib
    attachment = tomllib.loads((session / 'attachment.toml').read_text())
    assert attachment['features']['default_mode_request_user_input'] == (not batch)
    assert attachment['hooks']['UserPromptSubmit'][0]['hooks'][0]['input']['threadId'] == '${session_id}'
    launch = (session / 'launch.json').read_text()
    assert 'test-runtime-secret' not in launch
    assert 'research_attach.py' in launch
    metadata = json.loads(launch)
    assert 'preflight' not in metadata
    assert metadata['runtime_status']['connected']
    assert metadata['runtime_status']['step'] == 0
    assert metadata['runtime_status']['queue_length'] == 0

def test_launcher_refuses_existing_session_without_overwriting_evidence(tmp_path):
    (tmp_path / 'exit.json').write_text('{"returncode": 42}')
    result = subprocess.run([str(ROOT / 'hp-sim5-research-agent'), '--viewer', 'none',
                             '--session-dir', str(tmp_path)], capture_output=True, text=True, timeout=10)
    assert result.returncode == 1
    assert (tmp_path / 'exit.json').read_text() == '{"returncode": 42}'


@pytest.mark.slow
def test_service_only_launcher_exits_cleanly_on_supervisor_shutdown(tmp_path):
    import time
    from hp_sim5_research.services import request
    directory = tmp_path / 'desktop-session'
    process = subprocess.Popen([str(ROOT / 'hp-sim5-research-agent'), '--serve', '--viewer', 'none',
                                '--session-dir', str(directory)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        deadline = time.monotonic() + 20
        while not (directory / 'connection.json').exists():
            assert time.monotonic() < deadline and process.poll() is None
            time.sleep(.1)
        config = json.loads((directory / 'connection.json').read_text())
        request(config['runtime_endpoint'], 'shutdown', token=config['runtime_token'], timeout=10)
        stdout, stderr = process.communicate(timeout=10)
        assert process.returncode == 0, stderr
        assert config['runtime_token'] not in stdout + stderr
        assert json.loads((directory / 'exit.json').read_text()) == {'returncode': 0}
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=10)
