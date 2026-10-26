"""Check launcher prerequisites without building, starting a world or moving it."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from research_socket_probe import probe


def check(backend='headless-js'):
    checks = {}

    def command(name, argv):
        try:
            result = subprocess.run(argv, cwd=ROOT, capture_output=True, text=True, timeout=8)
            checks[name] = {'ok': result.returncode == 0}
            if result.returncode:
                checks[name]['error'] = (result.stderr or result.stdout)[-2000:].strip()
            return result.stdout.strip() if result.returncode == 0 else None
        except (OSError, subprocess.TimeoutExpired) as error:
            checks[name] = {'ok': False, 'error': str(error)}

    python = ROOT / '.venv/bin/python'
    modules = ['numpy', 'scipy', 'pxr', 'rerun', 'websockets', 'mcp', 'jax']
    if backend.startswith('native-warp'):
        modules.append('warp')
    command('python_dependencies', [str(python), '-c',
            'import importlib; ' + '; '.join(f'importlib.import_module({name!r})' for name in modules)])
    command('node_dependencies', ['node', '--input-type=module', '-e',
            "await import('./src/js/usd/usda_parser.js'); await import('ws');"])
    command('cargo', ['cargo', '--version'])
    libdir = command('rustc', ['rustc', '--print', 'target-libdir'])
    llvm = Path(libdir).parent / 'bin/llvm-profdata' if libdir else None
    checks['llvm_tools'] = {'ok': bool(llvm and os.access(llvm, os.X_OK))}
    if backend == 'headless-js':
        command('native_physics', ['node', '--input-type=module', '-e',
                "import { loadNativePhysicsAddon } from './src/js/cable_joints_3d/native_physics_engine.js'; "
                'loadNativePhysicsAddon();'])
    checks['firmware_simulator'] = {'ok': os.access(ROOT / 'RRF/build/rrf_simulator', os.X_OK)}
    sockets = probe()
    sockets['context'] = 'worker_launcher'
    checks['launcher_sockets'] = {'ok': sockets['outcome'] == 'passed', 'probe': sockets}
    if not checks['launcher_sockets']['ok']:
        # The command-context probe's usual advice assumes a supervisor already
        # exists outside that sandbox. Here we are testing the supervisor host.
        for item in sockets['checks'].values():
            item.pop('next_step', None)
        checks['launcher_sockets']['next_step'] = (
            'The launcher itself cannot provide local socket services here. Allow local sockets '
            'in this command context or run the launcher in an allowed context. Switching to '
            'supervised jobs or spawning another child in this same context cannot grant permission.')
    ready = all(item['ok'] for item in checks.values())
    return {'ok': ready, 'kind': 'runtime_setup_check', 'physics_started': False, 'checks': checks,
            'next_step': None if ready else
            'Repair blocked checks before dispatch. For missing dependencies/artifacts run '
            'bash scripts/cloud/install_research.sh; socket permissions require a permitted launcher context.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--physics-backend', default='headless-js',
                        choices=('headless-js', 'native-python', 'native-warp', 'native-warp-cuda'))
    result = check(parser.parse_args().physics_backend)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['ok'] else 1)
