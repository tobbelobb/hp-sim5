"""Launch Codex with local physics tools and a separately supervised Rerun Viewer."""
import argparse
from importlib import import_module
from importlib.metadata import version
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/python'))
sys.path.insert(0, str(ROOT))
from hp_sim5_research.services import RuntimeService


def codex_environment():
    # This launcher uses the user's Codex ChatGPT sign-in, never a project API key.
    return {key: value for key, value in os.environ.items() if key not in ('OPENAI_API_KEY', 'CODEX_API_KEY')}


def doctor():
    checks = {}
    for module, package in [('numpy', 'numpy'), ('pxr', 'usd-core'), ('rerun', 'rerun-sdk'), ('mcp', 'mcp')]:
        try:
            import_module(module)
            checks[package] = {'ok': True, 'version': version(package)}
        except ImportError as error:
            checks[package] = {'ok': False, 'error': str(error)}
    codex = shutil.which('codex')
    checks['codex'] = {'ok': codex is not None, 'path': codex}
    if codex is not None:
        result = subprocess.run([codex, 'login', 'status'], capture_output=True, text=True,
                                env=codex_environment(), timeout=15)
        signed_in = result.returncode == 0 and 'ChatGPT' in result.stdout + result.stderr
        checks['chatgpt_login'] = {'ok': signed_in,
                                   'next_step': None if signed_in else 'codex login (choose ChatGPT)'}
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
    return {'ok': all(check['ok'] for check in checks.values()), 'repo': str(ROOT), 'checks': checks}


def codex_command(session, model, viewer_endpoint, runtime=None):
    command = ['codex', 'exec', '--cd', str(ROOT), '--sandbox', 'workspace-write', '--json',
               '--output-last-message', str(session / 'final-message.md'), '-c', 'approval_policy="never"']
    settings = {'mcp_servers.hp_sim5.command': sys.executable,
                'model_provider': 'openai',
                'mcp_servers.hp_sim5.args': [str(ROOT / 'scripts/hp_sim5_mcp.py')],
                'mcp_servers.hp_sim5.cwd': str(ROOT), 'mcp_servers.hp_sim5.required': True,
                'mcp_servers.hp_sim5.startup_timeout_sec': 30, 'mcp_servers.hp_sim5.tool_timeout_sec': 14500,
                'mcp_servers.hp_sim5.default_tools_approval_mode': 'approve',
                'mcp_servers.rerun.enabled': viewer_endpoint is not None}
    if runtime is not None:
        settings['mcp_servers.hp_sim5.env'] = {'HP_SIM5_RUNTIME_URL': runtime.endpoint,
                                             'HP_SIM5_RUNTIME_TOKEN': runtime.token}
    if viewer_endpoint is not None:
        settings.update({'mcp_servers.rerun.command': str(ROOT / '.venv/bin/rerun'),
                         'mcp_servers.rerun.args': ['viewer-mcp', '--endpoint', viewer_endpoint],
                         'mcp_servers.rerun.required': True,
                         'mcp_servers.rerun.startup_timeout_sec': 30,
                         'mcp_servers.rerun.default_tools_approval_mode': 'approve'})
    for key, value in settings.items():
        command += ['-c', f'{key}={json.dumps(value)}']
    if model:
        command += ['--model', model]
    return command + ['-']


def start_viewer(mode, session):
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        port = listener.getsockname()[1]
    command = [str(ROOT / '.venv/bin/rerun'), '--bind', '127.0.0.1', '--port', str(port), '--memory-limit', '1GB']
    if mode == 'headless':
        command.append('--headless')
    environment = {**os.environ, 'XDG_DATA_HOME': str(session / 'viewer-data'),
                   'XDG_CONFIG_HOME': str(session / 'viewer-config'), 'XDG_CACHE_HOME': str(session / 'viewer-cache')}
    with (session / 'viewer.log').open('w') as log:
        viewer = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=log, stderr=log)
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline and viewer.poll() is None:
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=.2):
                return viewer, f'http://127.0.0.1:{port}'
        except OSError:
            time.sleep(.1)
    stop_process(viewer)
    raise RuntimeError(f'Rerun did not become ready. Read {session / "viewer.log"}; use --viewer none for numeric experiments.')


def stop_process(process):
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    task = parser.add_mutually_exclusive_group()
    task.add_argument('--prompt', help='Research or robot design task')
    task.add_argument('--prompt-file', type=Path, help='UTF-8 task file')
    parser.add_argument('--doctor', action='store_true', help='Explicitly run diagnostics, including real RRF/native collection and autocal loading')
    parser.add_argument('--dry-run', action='store_true', help='Show the Codex command and task without starting processes')
    parser.add_argument('--viewer', choices=['headless', 'window', 'none'], default='headless')
    parser.add_argument('--model', help='Optional Codex model; otherwise use your existing Codex setting')
    args = parser.parse_args()
    viewer = agent = session = runtime = None
    returncode = 1
    try:
        if args.doctor:
            result = doctor()
            print(json.dumps(result, indent=2))
            return 0 if result['ok'] else 1
        prompt = args.prompt_file.read_text() if args.prompt_file else args.prompt
        if not prompt or not prompt.strip():
            parser.error('Supply --prompt or --prompt-file, or use --doctor')
        session = ROOT / 'output/research/sessions' / ('dry-run' if args.dry_run else uuid.uuid4().hex)
        instructions = (ROOT / 'research/AGENTS.md').read_text()
        full_prompt = (f'{instructions}\n\nSession artifacts: {session}\n'
                       f'Write the final research report to {session / "report.md"}.\n'
                       f'User task:\n{prompt}\n')
        if args.dry_run:
            endpoint = None if args.viewer == 'none' else 'http://127.0.0.1:PORT'
            print(json.dumps({'argv': codex_command(session, args.model, endpoint), 'prompt': full_prompt}, indent=2))
            return 0
        session.mkdir(parents=True)
        endpoint = None
        if args.viewer != 'none':
            viewer, endpoint = start_viewer(args.viewer, session)
        runtime = RuntimeService(ROOT, session / 'native', endpoint).start()
        runtime_status = runtime.call('status')
        (session / 'prompt.txt').write_text(full_prompt)
        command = codex_command(session, args.model, endpoint, runtime)
        # Keep bearer credentials out of launch artifacts.
        saved_command = [arg.replace(runtime.token, '<runtime token>') for arg in command]
        (session / 'launch.json').write_text(json.dumps({'argv': saved_command, 'runtime_status': runtime_status,
                                                       'viewer_endpoint': endpoint, 'runtime_endpoint': runtime.endpoint}, indent=2) + '\n')
        print(f'Research session: {session}', file=sys.stderr, flush=True)
        agent = subprocess.Popen(command, cwd=ROOT, env=codex_environment(),
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        agent.stdin.write(full_prompt)
        agent.stdin.close()
        with (session / 'events.jsonl').open('w') as output:
            for line in agent.stdout:
                output.write(line)
                output.flush()
                event = json.loads(line)
                if event.get('type') == 'item.completed' and event.get('item', {}).get('type') == 'agent_message':
                    print(event['item']['text'], flush=True)
                elif event.get('type') in ('error', 'turn.failed'):
                    print(line.strip(), file=sys.stderr, flush=True)
        returncode = agent.wait()
        return returncode
    except KeyboardInterrupt:
        returncode = 130
        return 130
    except (OSError, ValueError, RuntimeError, ImportError, subprocess.TimeoutExpired) as error:
        parser.exit(1, f'hp-sim5-research-agent: {error}\n')
    finally:
        stop_process(agent)
        if runtime is not None:
            runtime.close()
        stop_process(viewer)
        if session is not None and session.is_dir() and not args.dry_run:
            (session / 'exit.json').write_text(json.dumps({'returncode': returncode}) + '\n')


if __name__ == '__main__':
    sys.exit(main())
