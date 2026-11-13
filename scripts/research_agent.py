"""Prepare or resume an investigation, optionally opening a coding client or private physics tools."""
import argparse
import signal
from importlib import import_module
from importlib.metadata import version
import json
import math
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
from hp_sim5_research.services import RuntimeService, free_port, stop_process, register_test_process
from hp_sim5_research.archive import write_index
from hp_sim5_research.workspace import Sessions, lock, read_json, researcher_handoff, role, write_json
from hp_sim5_research.services import process_identity, process_live


ENV_FILE = ROOT / '.env'
CHATGPT_SIGN_IN = 'openai'
# Only ignored runtime pieces are shared; tracked research stays in each checkout.
WORKTREE_LINKS = ('.venv', '.env', 'node_modules', 'RRF/build', 'output',
                  'src/wasm/cable_solver/target/native/physics.node')


def provider_env_key(provider):
    return coding_client.provider_env_key(provider, root=ROOT)


def codex_environment(provider=CHATGPT_SIGN_IN):
    return coding_client.codex_environment(provider, root=ROOT, env_file=ENV_FILE)


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
        import tempfile
        try:
            from hp_sim5_research.experiments import DEFAULT_SCENE, encode
            from hp_sim5_research.js_physics import JSPhysics
            from hp_sim5_research import mirror
            with tempfile.TemporaryDirectory() as directory:
                physics = JSPhysics(ROOT, Path(directory), ROOT / DEFAULT_SCENE)
                try:
                    state = physics.request(steps=1)
                    world = mirror.World()
                    mirror.apply_state(world, state)
                    observed = mirror.sample(world, 1, state['dt_s'])
                    encode(observed)
                    checks['physics'] = {'ok': bool(observed['cables'] and observed['effectors'] and observed['motors']),
                                         'dt_s': state['dt_s'], 'cable_paths': len(observed['cables'])}
                finally:
                    physics.close()
        except Exception as error:
            checks['physics'] = {'ok': False, 'error': str(error)}
    runtime = None
    if checks.get('physics', {}).get('ok'):
        try:
            runtime = RuntimeService(ROOT, ROOT / 'research/output/scratch/preflight' / uuid.uuid4().hex).start()
            runtime.call('configure', machine='hp3')
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
    return coding_client.claude_environment()


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


def normalize_resume(arguments):
    """Legacy client resume must reopen artifacts rather than silently allocate a new investigation."""
    arguments = list(arguments)
    for index, value in enumerate(arguments):
        if value.startswith('--resume='):
            return arguments[:index] + arguments[index + 1:], value.split('=', 1)[1]
    for flag in ('--resume', 'resume'):
        if flag in arguments:
            index = arguments.index(flag)
            value = arguments[index + 1] if index + 1 < len(arguments) else None
            if value and not value.startswith('-'):
                return arguments[:index] + arguments[index + 2:], value
            return [v for v in arguments[:index] + arguments[index + 1:] if v != '--last'], 'latest'
    return arguments, None


def running_runtime(sessions, session):
    path = sessions.state / 'runtime' / (session['id'] + '.json')
    live = read_json(path) if path.is_file() else None
    return live if live and (process_live(live.get('owner')) or process_live(live.get('runtime_owner'))) else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    backend = parser.add_mutually_exclusive_group()
    backend.add_argument('--codex', help='Open a local Codex conversation', action='store_const', const='codex', dest='backend')
    backend.add_argument('--claude', help='Open a local Claude Code conversation', action='store_const', const='claude', dest='backend')
    task = parser.add_mutually_exclusive_group()
    task.add_argument('--prompt', help='Research question or continuation instruction')
    task.add_argument('--prompt-file', type=Path, help='Read the question from a file')
    parser.add_argument('--resume', nargs='?', const='latest', help='Reopen investigation ID or artifact directory; omit value for latest')
    parser.add_argument('--conversation', help='Bind an exact backend conversation ID when reopening')
    parser.add_argument('--workspace', action='store_true', help='Bind a prepared handoff to the platform-provided current checkout')
    parser.add_argument('--session-dir', type=Path, help='Artifact path for a new investigation; created only when evidence is written')
    parser.add_argument('--name', help='Short investigation label')
    parser.add_argument('--doctor', action='store_true', help='Check runtime dependencies and physics')
    parser.add_argument('--preflight', action='store_true', help='Run client dependency and collection diagnostics')
    parser.add_argument('--dry-run', action='store_true', help='Describe launch without writing state or starting services')
    parser.add_argument('--viewer', choices=['none', 'headless', 'window'], default='none', help='Viewer mode (default: none)')
    parser.add_argument('--physics-backend', choices=['headless-js'], default='headless-js', help='Physics execution backend')
    parser.add_argument('--no-record', action='store_true', help='Disable Rerun recording')
    parser.add_argument('--wall-deadline', type=float, help='Stop runtime at this Unix timestamp')
    parser.add_argument('--model-provider', default=CHATGPT_SIGN_IN, help='Codex provider from config.toml')
    parser.add_argument('--model', help='Backend model override')
    parser.add_argument('--worktree', action='store_true', help='Create an isolated Git worktree')
    parser.add_argument('--worktree-dir', type=Path, help='Parent directory for launcher worktrees')
    parser.add_argument('--dont-ask', action='store_true', help='Disable backend clarification tools')
    parser.add_argument('--goal-mode', action=argparse.BooleanOptionalAction, default=True, help='Enable backend goal tracking')
    parser.add_argument('--sidebar-section', help='Optional Codex sidebar section UUID')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--batch', action='store_true', help='Run a local client noninteractively with the supplied prompt')
    mode.add_argument('--serve', action='store_true', help='Keep private runtime and file tools alive in foreground')
    mode.add_argument('--start-runtime', action='store_true', help='Start a detached runtime; no world until a tool configures one')
    mode.add_argument('--stop-runtime', action='store_true', help='Stop the investigation runtime and finalize partial evidence')
    mode.add_argument('--runtime-status', action='store_true', help='Inspect runtime ownership without sockets')
    argv = sys.argv[1:]
    split = argv.index('--') if '--' in argv else len(argv)
    args = parser.parse_args(argv[:split])
    client_args, legacy_resume = normalize_resume(argv[split + 1:])
    if args.wall_deadline is not None and (not math.isfinite(args.wall_deadline) or args.wall_deadline <= 0):
        parser.error('--wall-deadline must be a positive finite Unix timestamp')
    if args.worktree_dir and not args.worktree:
        parser.error('--worktree-dir requires --worktree')
    if args.worktree and args.resume:
        parser.error('--resume reuses the saved worktree; omit --worktree')
    if args.backend == 'claude' and any(v == '--model-provider' or v.startswith('--model-provider=') for v in argv[:split]):
        parser.error('--model-provider is only supported with --codex')
    if args.batch and not args.backend:
        parser.error('--batch requires --codex or --claude')
    try:
        sessions = Sessions(ROOT)
        if args.doctor:
            result = doctor(args.model_provider, args.model, backend=args.backend or 'codex')
            print(json.dumps(result, indent=2))
            return 0 if result['ok'] else 1
        if legacy_resume:
            if not args.resume:
                matches = []
                for path in (sessions.state / 'sessions').glob('*.json'):
                    saved = read_json(path)
                    if legacy_resume in saved.get('conversations', {}).values():
                        matches.append(saved['id'])
                if len(matches) == 1:
                    args.resume = matches[0]
                elif legacy_resume == 'latest':
                    args.resume = 'latest'
                else:
                    raise ValueError('Client resume cannot identify its artifact directory. Use --resume ID_OR_DIRECTORY --conversation CLIENT_ID')
            if legacy_resume != 'latest':
                args.conversation = legacy_resume
        prompt = args.prompt_file.read_text() if args.prompt_file else args.prompt
        if args.batch and not args.preflight and not (prompt and prompt.strip()):
            parser.error('Batch mode requires --prompt or --prompt-file')
        topic = args.name or (prompt.strip().splitlines()[0] if prompt and prompt.strip() else 'interactive research')
        session = sessions.open(resume=args.resume, directory=args.session_dir, topic=topic,
                                backend=args.backend, conversation=args.conversation, dry_run=args.dry_run)
        if args.workspace:
            old_root = Path(session['repo'])
            old_directory = Path(session['directory'])
            if old_root != ROOT:
                if old_directory.exists():
                    raise ValueError('Existing evidence belongs to another workspace; reopen there or explicitly copy it first')
                session['directory'] = str(ROOT / old_directory.relative_to(old_root))
                session['repo'] = str(ROOT)
                if not args.dry_run:
                    sessions.save(session)
        if args.worktree:
            if args.dry_run:
                print(json.dumps({'worktree': worktree_plan(worktree_slug(topic), args.worktree_dir),
                                  'session_dir': session['directory']}, default=str, indent=2))
                return 0
            worktree = create_worktree(topic, args.worktree_dir)
            # New evidence belongs to the isolated checkout; tracked corpus stays intact.
            if args.session_dir is None:
                session['directory'] = str(worktree['path'] / 'research/output' / session['id'])
            session['repo'] = str(worktree['path'])
            session['launcher_worktree'] = True
            sessions.save(session)
            remaining = strip_worktree_options(argv[:split])
            if args.prompt_file:
                index = next((i for i, value in enumerate(remaining) if value == '--prompt-file'), None)
                if index is not None:
                    remaining[index + 1] = str(args.prompt_file.resolve())
                else:
                    remaining = [('--prompt-file=' + str(args.prompt_file.resolve())) if value.startswith('--prompt-file=') else value for value in remaining]
            if args.session_dir:
                remaining = [('--session-dir=' + session['directory']) if value.startswith('--session-dir=') else value for value in remaining]
                if '--session-dir' in remaining:
                    remaining[remaining.index('--session-dir') + 1] = session['directory']
            remaining += ['--resume', session['id']]
            if client_args:
                remaining += ['--', *client_args]
            os.execv(str(worktree['path'] / '.venv/bin/python'),
                     [str(worktree['path'] / '.venv/bin/python'), str(worktree['path'] / 'scripts/research_agent.py'), *remaining])
        saved_root = Path(session['repo'])
        if saved_root != ROOT and not session.get('launcher_worktree'):
            raise ValueError(f'Investigation is bound to {saved_root}; use --resume {session["id"]} --workspace first')
        if saved_root != ROOT and not args.dry_run:
            if not (saved_root / 'scripts/research_agent.py').is_file():
                raise ValueError(f'Saved worktree is unavailable: {saved_root}; reopen the evidence directory on this host')
            remaining = argv[:split]
            if not args.resume:
                remaining += ['--resume', session['id']]
            if client_args:
                remaining += ['--', *client_args]
            os.execv(str(saved_root / '.venv/bin/python'),
                     [str(saved_root / '.venv/bin/python'), str(saved_root / 'scripts/research_agent.py'), *remaining])
        live = running_runtime(sessions, session)
        if args.runtime_status or args.stop_runtime:
            if args.stop_runtime and live:
                Path(live['run_dir'], 'stop').touch()
                deadline = time.monotonic() + 30
                while running_runtime(sessions, session) and time.monotonic() < deadline:
                    time.sleep(.1)
                if running_runtime(sessions, session):
                    raise RuntimeError('Runtime is still finalizing; inspect --runtime-status before retrying')
            print(json.dumps({'id': session['id'], 'running': bool(running_runtime(sessions, session)),
                              'runtime': live}, indent=2))
            return 0
        if args.start_runtime:
            if live:
                print(json.dumps(live, indent=2))
                return 0
            if args.dry_run:
                print(json.dumps(researcher_handoff(ROOT, session), indent=2))
                return 0
            startup = sessions.run_directory(session)
            command = [sys.executable, str(ROOT / 'scripts/research_agent.py'), '--serve', '--resume', session['id'],
                       '--viewer', args.viewer]
            if args.no_record:
                command.append('--no-record')
            if args.wall_deadline:
                command += ['--wall-deadline', str(args.wall_deadline)]
            with (startup / 'supervisor.log').open('w') as log:
                process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                           start_new_session=True)
            register_test_process(process)
            deadline = time.monotonic() + 50
            while process.poll() is None and time.monotonic() < deadline:
                live = running_runtime(sessions, session)
                if live:
                    print(json.dumps(live, indent=2))
                    return 0
                time.sleep(.05)
            stop_process(process)
            raise RuntimeError(f'Runtime startup failed; read {startup / "supervisor.log"}')
        backend_name = args.backend or (session.get('backend') if args.resume and not args.serve else None)
        if not (backend_name or args.serve or args.preflight):
            print(json.dumps(researcher_handoff(ROOT, session), indent=2, ensure_ascii=False))
            return 0
        if live:
            raise ValueError('Runtime is already open. Use its file tools, or --stop-runtime before reopening a client')
        instructions = role(ROOT, 'researcher') + f'\nInvestigation artifacts: {session["directory"]}\n'
        instructions += 'The runtime has no physics world until configure_session builds one.\n'
        if args.resume:
            instructions += 'Continue the existing report and evidence; the previous live world is not restored.\n'
        command_builder = claude_command if backend_name == 'claude' else codex_command
        if args.dry_run:
            run = sessions.state / 'runs' / session['id'] / 'dry-run'
            command = command_builder(run, None, batch=args.batch, dont_ask=args.dont_ask, goal_mode=args.goal_mode,
                                      instructions=instructions, model=args.model, model_provider=args.model_provider,
                                      codex_args=client_args) if backend_name else []
            print(json.dumps({'id': session['id'], 'session_dir': session['directory'], 'argv': command,
                              'instructions': instructions, 'prompt': prompt or ''}, indent=2))
            return 0
        with lock(sessions.state / 'session-locks' / (session['id'] + '.lock'), blocking=False):
            return run_session(args, sessions, session, backend_name, client_args, prompt, instructions)
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, KeyError, RuntimeError, ImportError, subprocess.SubprocessError) as error:
        parser.exit(1, f'hp-sim5-research-agent: {error}\n')


def run_session(args, sessions, session, backend, client_args, prompt, instructions):
    run = sessions.run_directory(session)
    artifact = Path(session['directory'])
    runtime = viewer = agent = relay = None
    returncode = 1
    old_term = signal.signal(signal.SIGTERM, interrupted)
    old_alarm = None
    try:
        if args.wall_deadline:
            old_alarm = signal.signal(signal.SIGALRM, interrupted)
            if args.wall_deadline <= time.time():
                raise KeyboardInterrupt
            signal.setitimer(signal.ITIMER_REAL, args.wall_deadline - time.time())
        endpoint = None
        if args.viewer != 'none' and not args.preflight:
            viewer, endpoint = start_viewer(args.viewer, run)
        runtime = RuntimeService(ROOT, artifact / 'native', endpoint, record=not args.no_record, log_directory=run)
        runtime.start()
        status = runtime.call('status')
        if args.preflight:
            result = runtime.call('preflight')
            print(json.dumps(result, indent=2))
            returncode = 0 if result['outcome'] == 'passed' else 1
            return returncode
        descriptor = run / 'connection.json'
        descriptor.touch(mode=0o600)
        descriptor.write_text(json.dumps({'repo': str(ROOT), 'artifact_dir': str(artifact),
            'session_id': status['session_id'], 'runtime_endpoint': runtime.endpoint,
            'runtime_token': runtime.token, 'viewer_endpoint': endpoint}) + '\n')
        attachment = {'mcp_servers': claude_mcp_config(run, endpoint)['mcpServers']}
        (run / 'attachment.json').write_text(json.dumps({'mcpServers': attachment['mcp_servers']}) + '\n')
        (run / 'instructions.md').write_text(instructions)
        (run / 'prompt.txt').write_text(prompt or '')
        live = {'id': session['id'], 'session_dir': str(artifact), 'run_dir': str(run),
                'owner': process_identity(os.getpid()),
                'runtime_owner': process_identity(runtime.process.pid) if getattr(runtime, 'process', None) else None,
                'connection': str(descriptor)}
        if args.serve:
            import research_tools
            relay, connection = research_tools.start(ROOT, run)
            live['tool_connection'] = str(connection)
        write_json(sessions.state / 'runtime' / (session['id'] + '.json'), live)
        print(json.dumps(live), flush=True, file=sys.stdout if args.serve else sys.stderr)
        if args.serve:
            while runtime.process.poll() is None and not (run / 'stop').exists():
                if relay.poll() is not None:
                    raise RuntimeError(f'Tool relay stopped; read {run / "tools.log"}')
                time.sleep(.1)
            returncode = 0 if runtime.process.poll() in (None, 0) else 1
            return returncode
        conversation = session['conversations'].get(backend)
        resume_client = bool(args.resume and (conversation or session.get('clients_started', {}).get(backend)))
        extras = list(client_args)
        if backend == 'codex' and resume_client:
            if args.batch and not conversation:
                raise ValueError('Batch resume requires --conversation CLIENT_ID from the original run')
            extras = ['resume', *([conversation] if conversation else []), *extras]
            if not conversation:
                print('Choose the saved Codex conversation in the resume picker. The artifact path is already restored.', file=sys.stderr)
        if backend == 'claude':
            if resume_client and conversation:
                extras += ['--resume', conversation]
            elif not any(x in extras for x in ('--session-id', '--resume', '--continue')):
                conversation = str(uuid.uuid4())
                extras += ['--session-id', conversation]
                session['conversations'][backend] = conversation
                sessions.save(session)
        builder = claude_command if backend == 'claude' else codex_command
        command = builder(run, endpoint, runtime, batch=args.batch, dont_ask=args.dont_ask, goal_mode=args.goal_mode,
                          codex_args=extras, instructions=instructions, model=args.model, model_provider=args.model_provider,
                          sidebar_section=args.sidebar_section if args.sidebar_section != 'none' else None)
        write_json(run / 'launch.json', {'argv': command, 'backend': backend, 'session_dir': str(artifact)})
        environment = claude_environment() if backend == 'claude' else codex_environment(coding_client.forwarded_settings(client_args)[0].get('model_provider', args.model_provider))
        if not args.batch:
            if prompt and (not resume_client or conversation):
                command.append(prompt)
            agent = subprocess.Popen(command, cwd=ROOT, env=environment)
            session.setdefault('clients_started', {})[backend] = True
            sessions.save(session)
            previous = signal.signal(signal.SIGINT, lambda *_: None)
            try:
                returncode = agent.wait()
                return returncode
            finally:
                signal.signal(signal.SIGINT, previous)
        agent = subprocess.Popen(command, cwd=ROOT, env=environment, stdin=subprocess.PIPE,
                                 stdout=subprocess.PIPE, text=True)
        session.setdefault('clients_started', {})[backend] = True
        sessions.save(session)
        agent.stdin.write(prompt)
        agent.stdin.close()
        batch_failed = backend == 'claude'
        with (run / 'events.jsonl').open('w') as output:
            for line in agent.stdout:
                output.write(line)
                output.flush()
                event = json.loads(line)
                conversation = event.get('thread_id') or (event.get('session_id') if backend == 'claude' else None)
                if conversation:
                    session['conversations'][backend] = conversation
                    sessions.save(session)
                if backend == 'claude' and event.get('type') == 'result':
                    (run / 'final-message.md').write_text(event.get('result', '') + '\n')
                    batch_failed = bool(event.get('is_error'))
                    print(event.get('result', ''), flush=True)
                elif event.get('type') == 'item.completed' and event.get('item', {}).get('type') == 'agent_message':
                    print(event['item']['text'], flush=True)
        returncode = agent.wait() or int(batch_failed)
        return returncode
    finally:
        if old_alarm is not None:
            signal.setitimer(signal.ITIMER_REAL, 0)
            signal.signal(signal.SIGALRM, old_alarm)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        errors = []
        for name, close in [('agent', lambda: stop_process(agent)), ('relay', lambda: stop_process(relay)),
                            ('runtime', lambda: runtime.close() if runtime else None), ('viewer', lambda: stop_process(viewer))]:
            try:
                close()
            except Exception as error:
                errors.append(f'{name}: {error}')
        write_json(run / 'exit.json', {'returncode': returncode, 'cleanup_errors': errors})
        current = sessions.state / 'runtime' / (session['id'] + '.json')
        if current.is_file() and not errors:
            saved = read_json(current)
            if saved['run_dir'] == str(run):
                saved.update(owner=None, runtime_owner=None)
                write_json(current, saved)
        if artifact.exists():
            write_index(artifact.parent.parent if artifact.parent.name.startswith('program-') else artifact.parent)
        signal.signal(signal.SIGTERM, old_term)
        if errors:
            print('Research cleanup: ' + '; '.join(errors), file=sys.stderr)


if __name__ == '__main__':
    sys.exit(main())
