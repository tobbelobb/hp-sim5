import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest
from rerun.chunk import RrdReader

from hp_sim5_research.experiments import compare_runs, read_run, run_experiment
from hp_sim5_research.archive import session_name

ROOT = Path(__file__).resolve().parents[2]
SCENE = 'public/usd_scenes/hp4_rigid_body.usda'


@pytest.fixture
def experiment_root(tmp_path):
    target = tmp_path / SCENE
    target.parent.mkdir(parents=True)
    shutil.copyfile(ROOT / SCENE, target)
    (tmp_path / 'src').mkdir()
    (tmp_path / 'src/python').symlink_to(ROOT / 'src/python', target_is_directory=True)
    (tmp_path / 'scripts').symlink_to(ROOT / 'scripts', target_is_directory=True)
    return tmp_path


def test_real_physics_measurements_recording_and_reproducible_inputs(experiment_root):
    root = experiment_root
    commands = [{'type': 'Move', 'A': i * .0003} for i in range(30)]
    first = run_experiment(root, steps=30, commands=commands)
    second = run_experiment(root, steps=30, commands=commands, record=False)
    assert first['status'] == second['status'] == 'complete'
    assert first['physics_engine'] == second['physics_engine'] == 'rust-native'
    assert first['physics_artifact']['sha256']
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


def test_compiled_fresh_experiment_records_backend(experiment_root):
    pytest.importorskip('warp')
    trial = run_experiment(experiment_root, steps=5, record=False, cable_solver_device='cpu')
    assert trial['status'] == 'complete'
    assert trial['backend'] == 'native-warp' and trial['cable_solver_device'] == 'cpu'
    assert trial['packages']['warp-lang']
    assert read_run(experiment_root, trial['run_id']) == trial
    baseline = run_experiment(experiment_root, steps=5, record=False, physics_backend='native-python')
    comparison = compare_runs(experiment_root, baseline['run_id'], trial['run_id'])
    assert comparison['changed_inputs'] == ['packages', 'backend', 'cable_solver_device', 'physics_engine']
    with pytest.raises(ValueError, match='cable_solver_device'):
        run_experiment(experiment_root, cable_solver_device='automatic')


@pytest.mark.asyncio
async def test_stdio_mcp_discovery_execution_readback_and_error():
    from mcp import Client, StdioServerParameters
    parameters = StdioServerParameters(command=sys.executable, args=[str(ROOT / 'scripts/hp_sim5_mcp.py')])
    async with Client(parameters, read_timeout_seconds=30) as client:
        tools = (await client.list_tools()).tools
        assert {tool.name for tool in tools} == {'capabilities', 'run_experiment', 'read_experiment', 'compare_experiments',
                                               'runtime_status', 'send_gcode', 'capture_native_context', 'reset_session',
                                               'step_physics', 'configure_session', 'start_browser_service', 'start_collection', 'collection_status', 'cancel_collection',
                                               'browser_status', 'browser_action', 'start_autocal', 'autocal_status', 'cancel_autocal',
                                               'research_preflight', 'start_test', 'test_status', 'cancel_test'}
        config_tool = next(tool for tool in tools if tool.name == 'configure_session')
        assert config_tool.input_schema['properties']['configuration']['$ref'] == '#/$defs/WorldSettings'
        settings_schema = config_tool.input_schema['$defs']['WorldSettings']
        assert settings_schema['additionalProperties'] is False
        assert settings_schema['properties']['machine']['enum'] == ['hp3', 'hp4']
        autocal_tool = next(tool for tool in tools if tool.name == 'start_autocal')
        recording = autocal_tool.input_schema['properties']['recording_mode']
        assert recording['enum'] == ['none', 'events', 'sampled', 'full']
        assert recording['default'] == 'events'
        result = await client.call_tool('capabilities', {})
        assert not result.is_error
        assert SCENE in result.structured_content['scenes']
        assert result.structured_content['full_autocal']['default_recording_mode'] == 'events'
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
        assert captured.structured_content['backend'] == 'headless-js'
        assert captured.structured_content['physics_engine'] == 'rust-native'
        result = await client.call_tool('run_experiment', {'commands': [{'type': 'Move', 'A': 'bad'}]})
        assert result.is_error


@pytest.mark.parametrize('network_override', [None, False])
def test_launcher_dry_run_works_outside_repo_and_preserves_prompt(network_override):
    prompt = 'Compare $targets and `angles`, with "quoted" names.\nRetain failed trials.'
    command = [str(ROOT / 'hp-sim5-research-agent'), '--dry-run', '--viewer', 'none', '--prompt', prompt]
    if network_override is not None:
        command += ['--', '-c', 'sandbox_workspace_write.network_access=false']
    result = subprocess.run(command,
                            cwd='/tmp', capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    launch = json.loads(result.stdout)
    assert prompt in launch['prompt']
    assert 'workspace-write' in launch['argv']
    network_settings = [arg for arg in launch['argv'] if arg.startswith('sandbox_workspace_write.network_access=')]
    assert network_settings[0] == 'sandbox_workspace_write.network_access=true'
    assert network_settings[-1] == f'sandbox_workspace_write.network_access={str(network_override is None).lower()}'
    assert '--dangerously-bypass-approvals-and-sandbox' not in launch['argv']
    assert 'mcp_servers.rerun.enabled=false' in launch['argv']
    assert 'model_provider="openai"' in launch['argv']
    assert f'mcp_servers.hp_sim5.command="{ROOT / ".venv/bin/python"}"' in launch['argv']


def test_launcher_selects_model_provider_and_model_by_option_or_forwarding():
    base = [str(ROOT / 'hp-sim5-research-agent'), '--dry-run', '--viewer', 'none', '--sidebar-section', 'none']
    result = subprocess.run(base + ['--model-provider', 'opencode_go', '--model', 'deepseek-v4.1-flash',
                                    '--prompt', 'Research check'], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    launch = json.loads(result.stdout)
    assert launch['model_provider'] == 'opencode_go'
    assert launch['model'] == 'deepseek-v4.1-flash'
    assert launch['argv'].count('model_provider="opencode_go"') == 1
    assert launch['argv'].count('--model') == 1
    assert launch['argv'][launch['argv'].index('--model') + 1] == 'deepseek-v4.1-flash'
    # The same selection forwarded after -- is detected for credential lookup and not duplicated.
    forwarded = subprocess.run(base + ['--prompt', 'Research check', '--', '-c', 'model_provider="opencode_go"',
                                       '-m', 'deepseek-v4.1-flash'], capture_output=True, text=True, timeout=15)
    assert forwarded.returncode == 0, forwarded.stderr
    launch = json.loads(forwarded.stdout)
    assert launch['model_provider'] == 'opencode_go'
    assert launch['model'] == 'deepseek-v4.1-flash'
    assert launch['argv'].count('model_provider="opencode_go"') == 1
    assert launch['argv'].count('--model') == 0 and launch['argv'].count('-m') == 1
    # A forwarded value wins over the option without producing a repeated Codex option.
    combined = subprocess.run(base + ['--model-provider', 'opencode_go', '--model', 'gpt-6-luna',
                                      '--prompt', 'Research check', '--', '-m', 'deepseek-v4.1-flash'],
                              capture_output=True, text=True, timeout=15)
    assert combined.returncode == 0, combined.stderr
    launch = json.loads(combined.stdout)
    assert launch['model'] == 'deepseek-v4.1-flash'
    assert launch['argv'].count('--model') == 0 and launch['argv'].count('-m') == 1
    assert launch['argv'][launch['argv'].index('-m') + 1] == 'deepseek-v4.1-flash'


def test_launcher_worktree_dry_run_plans_without_creating_anything(tmp_path):
    target = tmp_path / 'worktrees'
    result = subprocess.run([str(ROOT / 'hp-sim5-research-agent'), '--dry-run', '--worktree', '--worktree-dir',
                             str(target), '--viewer', 'none', '--prompt', 'Optimize anchor layout'],
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    plan = json.loads(result.stdout)
    assert plan['mode'] == 'worktree'
    assert plan['worktree']['path'].startswith(str(target))
    assert plan['worktree']['branch'].startswith('research/optimize-anchor-layout-')
    assert set(plan['worktree']['links']) == {'.venv', '.env', 'node_modules', 'RRF/build'}
    assert isinstance(plan['worktree']['includes'], list)
    assert not target.exists()
    missing = subprocess.run([str(ROOT / 'hp-sim5-research-agent'), '--dry-run', '--worktree-dir', str(target),
                              '--viewer', 'none', '--prompt', 'x'], capture_output=True, text=True, timeout=15)
    assert missing.returncode != 0 and '--worktree-dir requires --worktree' in missing.stderr


def test_create_worktree_links_ignored_runtime_pieces(tmp_path, monkeypatch):
    import importlib.util
    repo = tmp_path / 'repo'
    (repo / 'scripts').mkdir(parents=True)
    (repo / 'scripts/research_agent.py').write_text('')
    (repo / '.gitignore').write_text('.venv\n.env\nnode_modules/\nRRF/build\n.worktreeinclude\nlocal/\n')
    (repo / 'tracked.txt').write_text('tracked\n')
    (repo / 'local').mkdir()
    (repo / 'local/notes.md').write_text('prior research\n')
    (repo / '.worktreeinclude').write_text('# local research to copy\nlocal/notes.md\n')
    subprocess.run(['git', '-C', str(repo), 'init', '-q', '-b', 'main'], check=True, capture_output=True)
    subprocess.run(['git', '-C', str(repo), 'add', '-A'], check=True, capture_output=True)
    subprocess.run(['git', '-C', str(repo), '-c', 'user.email=t@t', '-c', 'user.name=t', 'commit', '-qm', 'init'],
                   check=True, capture_output=True)
    (repo / '.venv/bin').mkdir(parents=True)
    (repo / '.venv/bin/python').write_text('')
    (repo / '.env').write_text('K=v\n')
    (repo / 'node_modules').mkdir()
    (repo / 'RRF/build').mkdir(parents=True)
    spec = importlib.util.spec_from_file_location('research_agent', ROOT / 'scripts/research_agent.py')
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    monkeypatch.setattr(launcher, 'ROOT', repo)
    plan = launcher.create_worktree('My Topic', tmp_path / 'worktrees')
    assert plan['path'].is_dir() and plan['base'] == 'main'
    assert (plan['path'] / 'tracked.txt').read_text() == 'tracked\n'
    for entry in launcher.WORKTREE_LINKS:
        link = plan['path'] / entry
        assert link.is_symlink() and link.resolve() == (repo / entry).resolve()
    assert plan['includes'] == ['local/notes.md']
    copied = plan['path'] / 'local/notes.md'
    assert copied.read_text() == 'prior research\n' and not copied.is_symlink()
    assert (repo / 'local/notes.md').read_text() == 'prior research\n'
    assert plan['branch'] in subprocess.run(['git', '-C', str(repo), 'worktree', 'list'],
                                            capture_output=True, text=True).stdout
    assert launcher.strip_worktree_options(['--viewer', 'none', '--worktree', '--worktree-dir', '/x',
                                            '--prompt', 'p']) == ['--viewer', 'none', '--prompt', 'p']


def test_launcher_loads_provider_key_from_env_file_without_leaking_it(tmp_path, monkeypatch):
    import importlib.util
    secret = 'opencode-go-test-secret'
    observed = tmp_path / 'child-environment.json'
    codex = tmp_path / 'codex'
    codex.write_text(f'#!{sys.executable}\n'
                     'import json, os\n'
                     'values = {"provider": os.environ.get("OPENCODE_GO_API_KEY"),\n'
                     '          "openai": os.environ.get("OPENAI_API_KEY"), "codex": os.environ.get("CODEX_API_KEY")}\n'
                     f'open({str(observed)!r}, "w").write(json.dumps(values))\n')
    codex.chmod(0o755)
    codex_home = tmp_path / 'codex-home'
    codex_home.mkdir()
    (codex_home / 'config.toml').write_text(
        '[model_providers.opencode_go]\nname = "OpenCode Go"\nbase_url = "https://example.invalid/v1"\n'
        'wire_api = "responses"\nenv_key = "OPENCODE_GO_API_KEY"\n')
    env_file = tmp_path / '.env'
    env_file.write_text(f'# provider credentials\nOPENCODE_GO_API_KEY="{secret}"\nOTHER_KEY=unused\n')
    monkeypatch.setenv('CODEX_HOME', str(codex_home))
    monkeypatch.setenv('OPENAI_API_KEY', 'openai-sentinel')
    monkeypatch.setenv('CODEX_API_KEY', 'codex-sentinel')

    spec = importlib.util.spec_from_file_location('research_agent', ROOT / 'scripts/research_agent.py')
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    monkeypatch.setattr(launcher, 'ENV_FILE', env_file)
    monkeypatch.setattr(launcher, 'RuntimeService', _stub_runtime_service())
    monkeypatch.setenv('PATH', str(tmp_path) + os.pathsep + os.environ['PATH'])
    session = tmp_path / 'provider session'
    monkeypatch.setattr(sys, 'argv', ['research_agent.py', '--viewer', 'none', '--prompt', 'provider key check',
                                      '--model-provider', 'opencode_go', '--model', 'deepseek-v4.1-flash',
                                      '--session-dir', str(session), '--sidebar-section', 'none'])
    assert launcher.main() == 0
    assert json.loads(observed.read_text()) == {'provider': secret, 'openai': None, 'codex': None}
    artifacts = (session / 'launch.json').read_text() + (session / 'attachment.toml').read_text()
    assert secret not in artifacts
    metadata = json.loads((session / 'launch.json').read_text())
    assert metadata['model_provider'] == 'opencode_go' and metadata['model'] == 'deepseek-v4.1-flash'
    assert secret not in session.joinpath('prompt.txt').read_text()


def _stub_runtime_service():
    class Service:
        endpoint = 'http://127.0.0.1:12345'
        token = 'test-runtime-secret'

        def __init__(self, root, directory, viewer_endpoint, *, backend, record, machine):
            self.directory = directory

        def start(self):
            return self

        def call(self, operation):
            assert operation == 'status'
            return {'connected': True, 'step': 0, 'queue_length': 0, 'session_id': 'test-session'}

        def close(self):
            pass
    return Service


def test_launcher_named_session_keeps_explicit_path_and_summary_instructions(tmp_path):
    session = tmp_path / 'HP3 radius validation'
    result = subprocess.run([str(ROOT / 'hp-sim5-research-agent'), '--dry-run', '--viewer', 'none',
                             '--sidebar-section', 'none', '--name', 'HP3 radius validation',
                             '--session-dir', str(session), '--prompt', 'Test candidate ranking'],
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    argv = json.loads(result.stdout)['argv']
    instructions = json.loads(next(arg.split('=', 1)[1] for arg in argv
                                   if arg.startswith('developer_instructions=')))
    assert f'Session artifacts: {session}' in instructions
    assert str(session / 'abstract.md') in instructions
    assert 'actually tested hypothesis' in instructions
    assert not session.exists()


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
    assert 'sandbox_workspace_write.network_access=true' in argv
    assert '--dont-ask' not in argv


@pytest.mark.parametrize('mode', [[], ['--batch'], ['--serve']])
@pytest.mark.parametrize('servers', [[], [{'name': 'codex_app', 'enabled': False}],
                                    [{'name': 'codex_app', 'enabled': True}]])
def test_sidebar_hook_uses_lifecycle_identity_without_model_instructions(tmp_path, monkeypatch, mode, servers):
    import tomllib
    codex = tmp_path / 'codex'
    codex.write_text(f'#!{sys.executable}\nimport sys\n'
                     'assert sys.argv[1:] == ["-c", "features.apps=true", "mcp", "list", "--json"]\n'
                     f'print({json.dumps(json.dumps(servers))})\n')
    codex.chmod(0o755)
    monkeypatch.setenv('PATH', str(tmp_path) + os.pathsep + os.environ['PATH'])
    section = '6f8f42eb-e2ae-4882-a538-5bf0202ec59b'
    command = [str(ROOT / 'hp-sim5-research-agent'), '--dry-run', '--viewer', 'none',
               '--sidebar-section', section, *mode]
    if '--serve' not in mode:
        command += ['--prompt', 'Research check', '--', '-c', 'features.apps=true']
    result = subprocess.run(command, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    argv = json.loads(result.stdout)['argv']
    config = next((value for value in argv if value.startswith('hooks.UserPromptSubmit=')), None)
    enabled = '--serve' in mode or any(server['enabled'] for server in servers)
    assert (config is not None) == enabled
    assert ('Sidebar categorization skipped' in result.stderr) == (not enabled)
    if enabled:
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
    assert not disabled.stderr
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
elif sys.argv[1:] == ['mcp', 'list', '--json']:
    print(json.dumps([{'name': 'codex_app', 'enabled': True}]))
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

        def __init__(self, root, directory, viewer_endpoint, *, backend, record, machine):
            assert backend == 'headless-js'
            assert record
            assert machine == 'hp4'
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

        def start_real_service(root, directory, viewer_endpoint, *, backend, record, machine):
            service = runtime_service(root, directory, viewer_endpoint, backend=backend, record=record, machine=machine)
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
    session = tmp_path / session_name('HP3 held-out validation')
    monkeypatch.setattr(sys, 'argv', ['research_agent.py', '--viewer', 'none', '--prompt', 'preservation check',
                                    '--session-dir', str(session),
                                    '--sidebar-section', '6f8f42eb-e2ae-4882-a538-5bf0202ec59b'] + (['--batch'] if batch else []))
    assert launcher.main() == 0
    if real_runtime:
        assert services[0].process.poll() == 0
    else:
        assert services[0].closed
    assert services[0].directory.parent == session
    assert 'Status: not yet summarized' in (session / 'abstract.md').read_text()
    catalog_root = ROOT / 'output/research' if session.is_relative_to(ROOT / 'output/research') else tmp_path
    assert (catalog_root / 'index.md').is_file()
    # The catalog deliberately skips nested test/session artifacts.
    if catalog_root == tmp_path:
        assert session.name in (catalog_root / 'index.md').read_text()
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
    assert attachment['sandbox_workspace_write']['network_access'] is True
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
