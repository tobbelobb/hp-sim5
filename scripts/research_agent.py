"""Launch a chosen coding agent with local physics tools and a separately supervised Rerun Viewer."""
import argparse
import signal
from importlib import import_module
from importlib.metadata import version
import json
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/python'))
sys.path.insert(0, str(ROOT))
from hp_sim5_research import coding_client
from hp_sim5_research.coding_client import toml_literal
from hp_sim5_research.services import RuntimeService, free_port, stop_process, register_test_process
from hp_sim5_research.archive import initial_abstract, session_name, write_index


ENV_FILE = ROOT / '.env'
CHATGPT_SIGN_IN = 'openai'
STRIPPED_KEYS = ('OPENAI_API_KEY', 'CODEX_API_KEY')
# Shared runtime pieces; output can also contain tracked research files.
WORKTREE_LINKS = ('.venv', '.env', 'node_modules', 'RRF/build', 'output',
                  'src/wasm/cable_solver/target/native/physics.node')


def ensure_native_physics():
    # Reuse the worker's freshness check, including custom artifact overrides.
    probe = subprocess.run(
        ['node', '--input-type=module', '-e',
         "import { loadNativePhysicsAddon } from './src/js/cable_joints_3d/native_physics_engine.js'; "
         'loadNativePhysicsAddon();'],
        cwd=ROOT, capture_output=True, text=True)
    if probe.returncode == 0:
        return
    if ('Native Rust physics build is stale.' not in probe.stderr and
            'Native Rust physics is missing:' not in probe.stderr):
        raise RuntimeError(f'Native physics check failed: {probe.stderr.strip()}')
    print('Native Rust physics build is stale, recompiling', file=sys.stderr, flush=True)
    if subprocess.run(['npm', 'run', 'build:native'], cwd=ROOT).returncode != 0:
        raise RuntimeError('Native Rust physics recompilation failed (npm run build:native)')


def provider_env_key(provider):
    return coding_client.provider_env_key(provider, root=ROOT)


def dotenv_values(path=None):
    return coding_client.dotenv_values(path or ENV_FILE)


def codex_environment(provider=CHATGPT_SIGN_IN):
    return coding_client.codex_environment(provider, root=ROOT, env_file=ENV_FILE)


def forwarded_settings(codex_args):
    # Effective `-c` overrides and `--model` from arguments forwarded to Codex; later wins.
    settings = {}
    model = None
    arguments = iter(codex_args)
    for argument in arguments:
        if argument in ('-c', '--config'):
            override = next(arguments, None)
        elif argument.startswith('--config='):
            override = argument.split('=', 1)[1]
        else:
            override = None
        if override and '=' in override:
            key, value = override.split('=', 1)
            settings[key.strip()] = value.strip().strip('"')
        elif argument in ('-m', '--model'):
            model = next(arguments, None)
        elif argument.startswith('--model='):
            model = argument.split('=', 1)[1].strip().strip('"')
    return settings, model


def worktree_parent(directory=None):
    return (directory or ROOT.parent / f'{ROOT.name}-worktrees').expanduser().resolve()


def worktree_slug(topic):
    slug = re.sub(r'[^a-z0-9]+', '-', topic.lower()).strip('-')[:40].strip('-')
    return slug or 'research'


def worktree_plan(slug, directory=None):
    name = f'{slug}-{uuid.uuid4().hex[:8]}'
    return {'path': worktree_parent(directory) / name, 'branch': f'research/{name}', 'links': list(WORKTREE_LINKS)}


def worktree_include_sources():
    """The existing repository paths listed in `.worktreeinclude`, as Codex's worktree includes."""
    try:
        lines = (ROOT / '.worktreeinclude').read_text().splitlines()
    except OSError:
        return []
    sources = []
    for line in lines:
        pattern = line.strip()
        if not pattern or pattern.startswith('#'):
            continue
        matches = ROOT.glob(pattern) if any(char in pattern for char in '*?[') else [ROOT / pattern]
        sources += [match for match in matches if match.exists() and match.is_relative_to(ROOT)]
    return sources


def copy_worktree_includes(path):
    copied = []
    for source in worktree_include_sources():
        target = path / source.relative_to(ROOT)
        if target.exists():
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        if source.is_dir():
            shutil.copytree(source, target)
        else:
            shutil.copy2(source, target)
        copied.append(str(source.relative_to(ROOT)))
    return copied


def create_worktree(topic, directory=None):
    """Create a raw Git worktree and link the ignored runtime pieces the launcher needs."""
    if not (ROOT / '.venv/bin/python').exists():
        raise RuntimeError('--worktree requires the repository .venv; create it before sharing it into a worktree')
    head = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', '--abbrev-ref', 'HEAD'],
                          capture_output=True, text=True)
    if head.returncode != 0:
        raise RuntimeError(f'--worktree requires a Git checkout: {head.stderr.strip()}')
    plan = worktree_plan(worktree_slug(topic), directory)
    plan['base'] = head.stdout.strip()
    plan['path'].parent.mkdir(parents=True, exist_ok=True)
    add = subprocess.run(['git', '-C', str(ROOT), 'worktree', 'add', '-b', plan['branch'], str(plan['path'])],
                         capture_output=True, text=True)
    if add.returncode != 0:
        raise RuntimeError(f'git worktree add failed: {(add.stderr or add.stdout).strip()}')
    common_git = subprocess.run(
        ['git', '-C', str(ROOT), 'rev-parse', '--path-format=absolute', '--git-common-dir'],
        check=True, capture_output=True, text=True)
    shared_output = Path(common_git.stdout.strip()).parent / 'output'
    shared_output.mkdir(exist_ok=True)
    for entry in WORKTREE_LINKS:
        source, target = ROOT / entry, plan['path'] / entry
        if entry == 'output':
            source = shared_output.resolve()
            # This is a fresh checkout: discard only Git's checked-out copy,
            # before any includes or session artifacts can be written there.
            if target.is_symlink():
                target.unlink()
            elif target.exists():
                shutil.rmtree(target)
        if source.exists() and not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            target.symlink_to(source, target_is_directory=source.is_dir())
    # Share the immutable starting addon, but keep build metadata and future
    # outputs local. The native build atomically replaces physics.node, which
    # turns this worktree's symlink into its own file on rebuild.
    source_metadata = ROOT / 'src/wasm/cable_solver/target/native/physics.node.json'
    target_metadata = plan['path'] / 'src/wasm/cable_solver/target/native/physics.node.json'
    if source_metadata.is_file() and not target_metadata.exists():
        shutil.copyfile(source_metadata, target_metadata)
    plan['includes'] = copy_worktree_includes(plan['path'])
    return plan


def strip_worktree_options(arguments):
    # The worktree launcher must not create another worktree from the same invocation.
    remaining = []
    arguments = iter(arguments)
    for argument in arguments:
        if argument == '--worktree':
            continue
        if argument == '--worktree-dir':
            next(arguments, None)
            continue
        if argument.startswith('--worktree-dir='):
            continue
        remaining.append(argument)
    return remaining


def worktree_launch(args, launcher_args, codex_args, provider, model):
    prompt = args.prompt or (args.prompt_file.read_text() if args.prompt_file else None)
    topic = args.name or (prompt.strip().splitlines()[0] if prompt and prompt.strip() else 'research')
    if args.dry_run:
        plan = worktree_plan(worktree_slug(topic), args.worktree_dir)
        print(json.dumps({'mode': 'worktree', 'backend': args.backend, 'model_provider': provider, 'model': model,
                          'worktree': {'path': str(plan['path']), 'branch': plan['branch'], 'links': plan['links'],
                                       'includes': [str(source.relative_to(ROOT))
                                                    for source in worktree_include_sources()]}}, indent=2))
        return 0
    worktree = create_worktree(topic, args.worktree_dir)
    print(f'Research worktree: {worktree["path"]} (branch {worktree["branch"]}, base {worktree["base"]})',
          file=sys.stderr, flush=True)
    if worktree['includes']:
        print(f'Copied {len(worktree["includes"])} .worktreeinclude path(s) into the worktree.',
              file=sys.stderr, flush=True)
    remaining = strip_worktree_options(launcher_args)
    if codex_args:
        remaining += ['--'] + codex_args
    # Run the worktree's own launcher so ROOT, MCP cwd, sandbox and artifacts all follow it.
    python = worktree['path'] / '.venv/bin/python'
    script = worktree['path'] / 'scripts/research_agent.py'
    os.execv(str(python), [str(python), str(script), *remaining])


def doctor(provider=CHATGPT_SIGN_IN, model=None, *, backend='codex'):
    checks = {}
    for module, package in [('numpy', 'numpy'), ('pxr', 'usd-core'), ('rerun', 'rerun-sdk'), ('mcp', 'mcp')]:
        try:
            import_module(module)
            checks[package] = {'ok': True, 'version': version(package)}
        except ImportError as error:
            checks[package] = {'ok': False, 'error': str(error)}
    executable = shutil.which(backend)
    checks[backend] = {'ok': executable is not None, 'path': executable}
    if backend == 'claude' and executable is not None:
        result = subprocess.run([executable, 'auth', 'status'], capture_output=True, text=True,
                                env=claude_environment(), timeout=15)
        checks['claude_login'] = {'ok': result.returncode == 0,
                                   'next_step': None if result.returncode == 0 else 'claude auth login'}
    elif executable is not None and provider == CHATGPT_SIGN_IN:
        result = subprocess.run([executable, 'login', 'status'], capture_output=True, text=True,
                                env=codex_environment(provider), timeout=15)
        signed_in = result.returncode == 0 and 'ChatGPT' in result.stdout + result.stderr
        checks['chatgpt_login'] = {'ok': signed_in,
                                   'next_step': None if signed_in else 'codex login (choose ChatGPT)'}
    elif backend == 'codex' and provider != CHATGPT_SIGN_IN:
        variable = provider_env_key(provider)
        available = bool(variable and codex_environment(provider).get(variable))
        checks['provider_credentials'] = {'ok': available or variable is None, 'variable': variable,
                                          'next_step': None if available or variable is None
                                          else f'export {variable} or add it to {ENV_FILE.name}'}
    rerun = ROOT / '.venv/bin/rerun'
    checks['viewer_mcp'] = {'ok': False, 'path': str(rerun)}
    if rerun.is_file():
        result = subprocess.run([str(rerun), 'viewer-mcp', '--help'], capture_output=True, timeout=15)
        checks['viewer_mcp']['ok'] = result.returncode == 0
    if all(checks[name]['ok'] for name in ('numpy', 'usd-core', 'rerun-sdk')):
        try:
            from hp_sim5_research.experiments import DEFAULT_SCENE, encode, sample
            from cable_joints_3d.machine_simulation import load_machine_world
            world = load_machine_world(ROOT / DEFAULT_SCENE)
            dt = world.get_resource('dt')
            world.update(dt)
            observed = sample(world, 1, dt)
            encode(observed)
            checks['physics'] = {'ok': bool(observed['cables'] and observed['effectors'] and observed['motors']),
                                 'dt_s': dt, 'cable_paths': len(observed['cables'])}
        except Exception as error:
            checks['physics'] = {'ok': False, 'error': str(error)}
    runtime = None
    if checks.get('physics', {}).get('ok'):
        try:
            runtime = RuntimeService(ROOT, ROOT / 'output/research/preflight' / uuid.uuid4().hex).start()
            collection = runtime.call('collect', options={'sweepPoints': 3, 'noiseSamples': 4})
            from hp_sim5_research.validation import validate_collection
            validation = validate_collection(collection['artifacts']['dataset'])
            checks['native_collection'] = {'ok': True, 'run_id': collection['run_id'],
                                           'artifacts': collection['artifacts'], 'validation': validation}
        except Exception as error:
            checks['native_collection'] = {'ok': False, 'error': str(error)}
        finally:
            if runtime is not None:
                runtime.close()
    return {'ok': all(check['ok'] for check in checks.values()), 'repo': str(ROOT),
            'backend': backend, 'model_provider': provider, 'model': model, 'checks': checks}


def sidebar_hooks(section_id):
    return {'UserPromptSubmit': [{'hooks': [{
        'type': 'mcp_tool', 'server': 'codex_app', 'tool': 'move_thread_to_sidebar_section',
        'input': {'threadId': '${session_id}', 'sectionId': section_id}, 'timeout': 10,
    }]}]}


def sidebar_server_enabled(codex_args, provider=CHATGPT_SIGN_IN):
    # Ask Codex for effective configuration, including plugin servers and CLI overrides.
    overrides = []
    arguments = iter(codex_args)
    for argument in arguments:
        if argument in ('-c', '--config', '-p', '--profile', '--enable', '--disable'):
            value = next(arguments, None)
            if value is None:
                return False  # Let the Codex invocation report its invalid arguments.
            overrides.extend((argument, value))
        elif argument.startswith(('--config=', '--profile=', '--enable=', '--disable=')):
            overrides.append(argument)
    try:
        result = subprocess.run(['codex', *overrides, 'mcp', 'list', '--json'],
                                capture_output=True, text=True, env=codex_environment(provider), timeout=10)
        if result.returncode == 0:
            return any(server['name'] == 'codex_app' and server['enabled']
                       for server in json.loads(result.stdout))
    except (OSError, subprocess.TimeoutExpired, ValueError):
        pass
    return False


def codex_command(session, viewer_endpoint, runtime=None, *, batch=False, dont_ask=False, goal_mode, codex_args=(), instructions='', sidebar_section=None,
                  model_provider=CHATGPT_SIGN_IN, model=None):
    settings = {'mcp_servers.hp_sim5.command': sys.executable,
                'mcp_servers.hp_sim5.args': [str(ROOT / 'scripts/hp_sim5_mcp.py')],
                'mcp_servers.hp_sim5.cwd': str(ROOT), 'mcp_servers.hp_sim5.required': True,
                'mcp_servers.hp_sim5.startup_timeout_sec': 30, 'mcp_servers.hp_sim5.tool_timeout_sec': 14500,
                'mcp_servers.hp_sim5.default_tools_approval_mode': 'approve',
                'mcp_servers.rerun.enabled': viewer_endpoint is not None,
                'mcp_servers.rerun.command': str(ROOT / '.venv/bin/rerun'),
                'mcp_servers.rerun.args': ['viewer-mcp']}
    if runtime is not None:
        settings['mcp_servers.hp_sim5.args'] = [str(ROOT / 'scripts/research_attach.py'), str(session / 'connection.json')]
    if viewer_endpoint is not None:
        settings.update({'mcp_servers.rerun.command': str(ROOT / '.venv/bin/rerun'),
                         'mcp_servers.rerun.args': ['viewer-mcp', '--endpoint', viewer_endpoint],
                         'mcp_servers.rerun.required': True,
                         'mcp_servers.rerun.startup_timeout_sec': 30,
                         'mcp_servers.rerun.default_tools_approval_mode': 'approve'})
    if sidebar_section:
        settings['hooks.UserPromptSubmit'] = sidebar_hooks(sidebar_section)['UserPromptSubmit']
    return coding_client.codex_command(ROOT, session, batch=batch, dont_ask=dont_ask,
                                       goal_mode=goal_mode, codex_args=codex_args,
                                       instructions=instructions, model_provider=model_provider,
                                       model=model, settings=settings)


def claude_mcp_config(session, viewer_endpoint):
    servers = {'hp_sim5': {'command': sys.executable,
                           'args': [str(ROOT / 'scripts/research_attach.py'), str(session / 'connection.json')]}}
    if viewer_endpoint is not None:
        servers['rerun'] = {'command': sys.executable,
                            'args': [str(ROOT / 'scripts/research_attach.py'),
                                     str(session / 'connection.json'), '--rerun']}
    return {'mcpServers': servers}


def claude_command(session, viewer_endpoint=None, runtime=None, *, batch=False, dont_ask=False, goal_mode, codex_args=(), model=None, **_):
    return coding_client.claude_command(session, batch=batch, dont_ask=dont_ask,
                                        goal_mode=goal_mode, codex_args=codex_args, model=model)


def claude_environment():
    return coding_client.claude_environment(env_file=ENV_FILE)


def start_viewer(mode, session):
    port = free_port()
    command = [str(ROOT / '.venv/bin/rerun'), '--bind', '127.0.0.1', '--port', str(port), '--memory-limit', '1GB']
    if mode == 'headless':
        command.append('--headless')
    environment = {**os.environ, 'XDG_DATA_HOME': str(session / 'viewer-data'),
                   'XDG_CONFIG_HOME': str(session / 'viewer-config'), 'XDG_CACHE_HOME': str(session / 'viewer-cache')}
    with (session / 'viewer.log').open('w') as log:
        # Only the agent may own terminal input and receive terminal interrupt signals.
        viewer = subprocess.Popen(command, cwd=ROOT, env=environment, stdin=subprocess.DEVNULL,
                                  stdout=log, stderr=log, start_new_session=True)
    try:
        register_test_process(viewer)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and viewer.poll() is None:
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=.2):
                    return viewer, f'http://127.0.0.1:{port}'
            except OSError:
                time.sleep(.1)
        raise RuntimeError(f'Rerun did not become ready. Read {session / "viewer.log"}; use --viewer none for numeric experiments.')
    except BaseException:
        stop_process(viewer)
        raise


def interrupted(_signum, _frame):
    raise KeyboardInterrupt


def session_manifest(session, backend, machine):
    """Non-secret paths survive worktree re-exec; written before runtime startup."""
    revision = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'],
                              capture_output=True, text=True, check=True).stdout.strip()
    branch = subprocess.run(['git', '-C', str(ROOT), 'branch', '--show-current'],
                            capture_output=True, text=True, check=True).stdout.strip()
    return {'launcher_session_id': uuid.uuid4().hex, 'session_dir': str(session),
            'repo': str(ROOT), 'source_revision': revision, 'branch': branch,
            'pid': os.getpid(), 'backend': backend, 'machine': machine}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    backend = parser.add_mutually_exclusive_group(required=True)
    backend.add_argument('--codex', action='store_const', const='codex', dest='backend', help='Use Codex CLI')
    backend.add_argument('--claude', action='store_const', const='claude', dest='backend', help='Use Claude Code CLI')
    task = parser.add_mutually_exclusive_group()
    task.add_argument('--prompt', help='Research or robot design task')
    task.add_argument('--prompt-file', type=Path, help='UTF-8 task file')
    parser.add_argument('--doctor', action='store_true', help='Explicitly run diagnostics, including real RRF/native collection and autocal loading')
    parser.add_argument('--preflight', action='store_true', help='Run bounded non-moving supervised checks and exit; command networking remains untested')
    parser.add_argument('--dry-run', action='store_true', help='Show the agent command and task without starting processes')
    parser.add_argument('--viewer', choices=['headless', 'window', 'none'], default='headless')
    parser.add_argument('--physics-backend', choices=['headless-js', 'native-python', 'native-warp', 'native-warp-cuda'], default='headless-js',
                        help='Continuing physics: native Rust by default; native-warp compiles the Python cable solver (CUDA experimental)')
    parser.add_argument('--no-record', action='store_true', help='Keep numerical collection evidence without Rerun logging')
    parser.add_argument('--machine', choices=['hp3', 'hp4'], default='hp4', help='Matched scene and RRF configuration')
    parser.add_argument('--model-provider', default=CHATGPT_SIGN_IN,
                        help='Codex model provider; opencode_go reads OPENCODE_GO_API_KEY, falling back to the .env file')
    parser.add_argument('--model', help='Model name for the selected backend')
    parser.add_argument('--worktree', action='store_true',
                        help='Create a fresh Git worktree and run the whole session from it')
    parser.add_argument('--worktree-dir', type=Path,
                        help='Parent directory for --worktree (default: a sibling of the repository)')
    parser.add_argument('--dont-ask', action='store_true', help='Disable structured clarification questions in Default mode')
    parser.add_argument('--goal-mode', action=argparse.BooleanOptionalAction, default=True,
                        help='Goal mode for autonomous continuation: Codex goals or Claude /goal (default: on; --no-goal-mode disables)')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--batch', action='store_true', help='Run the selected backend non-interactively and save JSONL events')
    mode.add_argument('--serve', action='store_true', help='Supervise services for desktop MCP attachment until interrupted')
    parser.add_argument('--session-dir', type=Path, help='New session artifact directory (must not exist)')
    parser.add_argument('--name', help='Short research topic used in the default session directory name')
    parser.add_argument('--sidebar-section', help='Desktop sidebar section UUID, or none; defaults to output/research/sidebar.json')
    # Everything after -- belongs to the selected backend, including resume and its normal options.
    argv = sys.argv[1:]
    split = argv.index('--') if '--' in argv else len(argv)
    args = parser.parse_args(argv[:split])
    codex_args = argv[split + 1:]
    # Forwarded `-c model_provider=...` and `--model` win, matching Codex's own last-wins order.
    forwarded, forwarded_model = forwarded_settings(codex_args)
    if args.backend == 'claude':
        forwarded = {}
        forwarded_model = None
        arguments = iter(codex_args)
        for argument in arguments:
            if argument == '--model':
                forwarded_model = next(arguments, None)
            elif argument.startswith('--model='):
                forwarded_model = argument.split('=', 1)[1]
        if '--model-provider' in argv[:split] or any(a.startswith('--model-provider=') for a in argv[:split]):
            parser.error('--model-provider is only supported with --codex')
    command_builder = claude_command if args.backend == 'claude' else codex_command
    provider = forwarded.get('model_provider', args.model_provider) if args.backend == 'codex' else None
    model = forwarded_model or args.model
    # Codex rejects a repeated --model, so only add options the caller did not forward.
    injected_provider = None if 'model_provider' in forwarded else args.model_provider
    injected_model = None if forwarded_model else args.model
    if args.worktree_dir and not args.worktree:
        parser.error('--worktree-dir requires --worktree')
    previous_sigterm = signal.signal(signal.SIGTERM, interrupted)
    viewer = agent = session = runtime = None
    session_created = False
    returncode = 1
    try:
        if args.worktree:
            return worktree_launch(args, argv[:split], codex_args, provider, model)
        if args.doctor:
            if args.preflight:
                parser.error('--doctor and --preflight are separate diagnostics; select one')
            result = doctor(provider, model, backend=args.backend)
            print(json.dumps(result, indent=2))
            return 0 if result['ok'] else 1
        prompt = args.prompt_file.read_text() if args.prompt_file else args.prompt
        if args.batch and not args.preflight and (not prompt or not prompt.strip()):
            parser.error('Batch mode requires --prompt or --prompt-file')
        if args.serve and (prompt or codex_args):
            parser.error('--serve accepts neither a task nor agent arguments')
        sidebar_section = args.sidebar_section if args.backend == 'codex' else None
        sidebar_config = ROOT / 'output/research/sidebar.json'
        if args.backend == 'codex' and sidebar_section is None and sidebar_config.exists():
            sidebar_section = json.loads(sidebar_config.read_text())['section_id']
        if sidebar_section == 'none':
            sidebar_section = None
        if sidebar_section:
            sidebar_section = str(uuid.UUID(sidebar_section))
            if not args.serve and not sidebar_server_enabled(codex_args, provider):
                print('Sidebar categorization skipped: codex_app MCP server is not enabled in this Codex CLI configuration.',
                      file=sys.stderr)
                sidebar_section = None
        topic = args.name or (prompt.strip().splitlines()[0] if prompt and prompt.strip() else
                              'service research' if args.serve else 'interactive research')
        session = (args.session_dir.resolve() if args.session_dir else
                   ROOT / 'output/research/sessions' / ('dry-run' if args.dry_run else session_name(topic)))
        catalog_root = (ROOT / 'output/research' if session.is_relative_to(ROOT / 'output/research')
                        else session.parent)
        instructions = (ROOT / 'research/AGENTS.md').read_text()
        if args.backend == 'claude':
            instructions = (ROOT / 'AGENTS.md').read_text() + '\n' + instructions
        instructions += (f'\nSession artifacts: {session}\n'
                         f'Write the research record to {session / "research.md"} and final report to {session / "report.md"}.\n'
                         f'Keep the searchable abstract at {session / "abstract.md"} current too.\n'
                         'This launcher started a fresh native world. Conversation resume does not restore physics checkpoints.\n')
        full_prompt = prompt or ''
        if args.dry_run:
            endpoint = None if args.viewer == 'none' else 'http://127.0.0.1:PORT'
            print(json.dumps({'mode': 'serve' if args.serve else 'batch' if args.batch else 'interactive',
                              'backend': args.backend, 'model_provider': provider, 'model': model,
                              'mcp_config': claude_mcp_config(session, endpoint) if args.backend == 'claude' else None,
                              'argv': command_builder(session, endpoint, batch=args.batch, dont_ask=args.dont_ask, goal_mode=args.goal_mode, codex_args=codex_args, instructions=instructions, sidebar_section=sidebar_section, model_provider=injected_provider, model=injected_model) + ([] if args.batch or not prompt else [prompt]),
                              'prompt': full_prompt}, indent=2))
            return 0
        session.mkdir(parents=True)
        session_created = True
        (session / 'session.json').write_text(json.dumps(session_manifest(session, args.backend, args.machine), indent=2) + '\n')
        (session / 'prompt.txt').write_text(full_prompt)
        (session / 'research.md').write_text(f'# Research record\n\nObjective: {prompt or "Set in the agent conversation"}\n\nCurrent hypothesis: pending.\nConstraints and budgets: set before experiments.\nExperiment IDs: none yet.\nAccepted steering: none yet.\nNext decision: establish a measurable baseline.\n')
        (session / 'abstract.md').write_text(initial_abstract(topic))
        write_index(catalog_root)
        if args.physics_backend == 'headless-js':
            ensure_native_physics()
        endpoint = None
        if args.viewer != 'none' and not args.preflight:
            viewer, endpoint = start_viewer(args.viewer, session)
        runtime = RuntimeService(ROOT, session / 'native', endpoint,
                                 backend=args.physics_backend, record=not args.no_record, machine=args.machine)
        runtime.start()
        runtime_status = runtime.call('status')
        if args.preflight:
            result = runtime.call('preflight')
            print(json.dumps(result, indent=2))
            returncode = 0 if result['outcome'] == 'passed' else 1
            return returncode
        descriptor = session / 'connection.json'
        descriptor.touch(mode=0o600)
        descriptor.write_text(json.dumps({'repo': str(ROOT), 'session_id': runtime_status['session_id'],
                                          'runtime_endpoint': runtime.endpoint, 'runtime_token': runtime.token,
                                          'viewer_endpoint': endpoint}) + '\n')
        attachment = ('[sandbox_workspace_write]\nnetwork_access = true\n'
                      f'writable_roots = {toml_literal([str((ROOT / "output").resolve())])}\n\n'
                      '[features]\n'
                      f'default_mode_request_user_input = {toml_literal(not (args.dont_ask or args.batch))}\n\n'
                      f'[mcp_servers.hp_sim5]\ncommand = {json.dumps(sys.executable)}\n'
                      f'args = {json.dumps([str(ROOT / "scripts/research_attach.py"), str(descriptor)])}\n'
                      'required = true\ntool_timeout_sec = 14500\n')
        if endpoint:
            attachment += (f'\n[mcp_servers.rerun]\ncommand = {json.dumps(sys.executable)}\n'
                           f'args = {json.dumps([str(ROOT / "scripts/research_attach.py"), str(descriptor), "--rerun"])}\n'
                           'required = true\n')
        if sidebar_section:
            attachment += '\n[hooks]\n' + '\n'.join(f'{key} = {toml_literal(value)}' for key, value in sidebar_hooks(sidebar_section).items()) + '\n'
        (session / 'attachment.toml').write_text(attachment)
        (session / 'attachment.json').write_text(json.dumps(claude_mcp_config(session, endpoint), indent=2) + '\n')
        (session / 'instructions.md').write_text(instructions)
        command = command_builder(session, endpoint, runtime, batch=args.batch, dont_ask=args.dont_ask, goal_mode=args.goal_mode, codex_args=codex_args, instructions=instructions, sidebar_section=sidebar_section,
                                model_provider=injected_provider, model=injected_model)
        if not args.batch and prompt:
            command += [prompt]
        # Keep bearer credentials out of launch artifacts.
        saved_command = [arg.replace(runtime.token, '<runtime token>') for arg in command]
        (session / 'launch.json').write_text(json.dumps({'argv': saved_command, 'runtime_status': runtime_status,
                                                       'backend': args.backend, 'model_provider': provider, 'model': model,
                                                       'viewer_endpoint': endpoint, 'runtime_endpoint': runtime.endpoint}, indent=2) + '\n')
        print(f'Research session: {session}', file=sys.stderr, flush=True)
        if args.serve:
            print(f'MCP configuration: {session / ("attachment.json" if args.backend == "claude" else "attachment.toml")}', file=sys.stderr, flush=True)
            while runtime.process.poll() is None:
                time.sleep(.2)
            if runtime.process.returncode == 0:
                returncode = 0
                return 0
            raise RuntimeError(f'Runtime stopped; read {session / "native/runtime.log"}')
        environment = claude_environment() if args.backend == 'claude' else codex_environment(provider)
        if not args.batch:
            # The agent owns the terminal and conversation across all research turns.
            agent = subprocess.Popen(command, cwd=ROOT, env=environment)
            # Ctrl-C belongs to the agent's turn interruption UI, not supervisor shutdown.
            previous_sigint = signal.signal(signal.SIGINT, lambda *_: None)
            try:
                returncode = agent.wait()
            finally:
                signal.signal(signal.SIGINT, previous_sigint)
            return returncode
        agent = subprocess.Popen(command, cwd=ROOT, env=environment,
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        agent.stdin.write(full_prompt)
        agent.stdin.close()
        batch_failed = args.backend == 'claude'
        with (session / 'events.jsonl').open('w') as output:
            for line in agent.stdout:
                output.write(line)
                output.flush()
                event = json.loads(line)
                if args.backend == 'claude' and event.get('type') == 'result':
                    message = event.get('result', '')
                    (session / 'final-message.md').write_text(message + '\n')
                    print(message or json.dumps(event), flush=True)
                    batch_failed = bool(event.get('is_error'))
                elif event.get('type') == 'item.completed' and event.get('item', {}).get('type') == 'agent_message':
                    print(event['item']['text'], flush=True)
                elif event.get('type') in ('error', 'turn.failed'):
                    print(line.strip(), file=sys.stderr, flush=True)
        returncode = agent.wait() or int(batch_failed)
        return returncode
    except KeyboardInterrupt:
        returncode = 130
        return 130
    except (OSError, ValueError, RuntimeError, ImportError, subprocess.TimeoutExpired) as error:
        parser.exit(1, f'hp-sim5-research-agent: {error}\n')
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)
        cleanup_errors = []
        for name, close in (('agent', lambda: stop_process(agent)),
                            ('runtime', lambda: runtime.close() if runtime is not None else None),
                            ('viewer', lambda: stop_process(viewer))):
            try:
                close()
            except Exception as error:
                cleanup_errors.append(f'{name}: {error}')
                print(f'Research cleanup failed: {name}: {error}', file=sys.stderr, flush=True)
        if session_created:
            exit_observation = {'returncode': returncode}
            if cleanup_errors:
                exit_observation['cleanup_errors'] = cleanup_errors
            (session / 'exit.json').write_text(json.dumps(exit_observation) + '\n')
            try:
                write_index(catalog_root)
            except OSError as error:
                print(f'Research catalog refresh failed: {error}; run scripts/research_index.py.', file=sys.stderr)


if __name__ == '__main__':
    sys.exit(main())
