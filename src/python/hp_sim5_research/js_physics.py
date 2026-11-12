"""Native Rust stepping hosted in Node; Python only mirrors observations."""
import json
import hashlib
import subprocess
import sys
from pathlib import Path


def ensure_native_physics(root):
    """Check/build native physics only when a world is actually requested."""
    probe = subprocess.run(
        ['node', '--input-type=module', '-e',
         "import { loadNativePhysicsAddon } from './src/js/cable_joints_3d/native_physics_engine.js'; "
         'loadNativePhysicsAddon();'],
        cwd=root, capture_output=True, text=True)
    if probe.returncode == 0:
        return
    if ('Native Rust physics build is stale.' not in probe.stderr and
            'Native Rust physics is missing:' not in probe.stderr):
        raise RuntimeError(f'Native physics check failed: {probe.stderr.strip()}')
    print('Native Rust physics build is stale, recompiling', file=sys.stderr, flush=True)
    # This can run inside an MCP server; reserve stdout for its protocol.
    if subprocess.run(['npm', 'run', 'build:native'], cwd=root, stdout=sys.stderr).returncode != 0:
        raise RuntimeError('Native Rust physics recompilation failed (npm run build:native)')


class JSPhysics:
    def __init__(self, root, directory, scene_path, *, layering=True, dt=None, scene_prim=None):
        self.directory = Path(directory)
        self.physics_wall_s = 0.
        self.queue_length = 0
        worker = (root / 'scripts/research_physics.mjs').resolve()
        source_root = worker.parents[1]
        ensure_native_physics(source_root)
        sources = sorted([worker, *(path for directory in ('src/js', 'hp-sim-3d/app')
                                    for path in (source_root / directory).rglob('*.js'))])
        digest = hashlib.sha256()
        for path in sources:
            digest.update(str(path.relative_to(source_root)).encode() + b'\0' + path.read_bytes())
        self.physics_host_sha256 = digest.hexdigest()
        self.log = (self.directory / 'physics-js.log').open('a')
        self.process = subprocess.Popen(['node', str(worker), str(scene_path), str(layering).lower(), scene_prim or ''],
                                        cwd=root, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.log, text=True, bufsize=1)
        from .services import register_test_process
        register_test_process(self.process)
        try:
            state = self.request(dt=dt)
            self.physics_engine = state['physics_engine']
            self.physics_artifact = state['physics_artifact']
        except Exception:
            self.close()
            raise

    def request(self, *, steps=0, commands=(), clear_commands=False, dt=None):
        self.process.stdin.write(json.dumps({'steps': steps, 'commands': commands,
                                            'clear_commands': clear_commands, 'dt_s': dt}) + '\n')
        self.process.stdin.flush()
        line = self.process.stdout.readline()
        if not line:
            self.log.flush()
            detail = next((line for line in Path(self.log.name).read_text().splitlines()
                           if line.startswith(('Error:', 'RangeError:', 'TypeError:'))), 'no error reply')
            raise RuntimeError(f'Physics worker exited: {detail}; see {self.log.name}')
        state = json.loads(line)
        if 'error' in state:
            raise RuntimeError(state['error'])
        self.queue_length = state.get('queue_length', 0)
        return state

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.close()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.process.stdout.close()
        self.log.close()
