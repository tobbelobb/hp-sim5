"""Native Rust stepping hosted in Node; Python only mirrors observations."""
import json
import hashlib
import subprocess
from pathlib import Path

import numpy as np

from cable_joints_3d.quaternion import Quaternion
from .services import register_test_process


class JSPhysics:
    def __init__(self, root, directory, world, *, layering=True, dt=None, scene_prim=None):
        self.world = world
        self.physics_wall_s = 0.
        worker = (root / 'scripts/research_physics.mjs').resolve()
        source_root = worker.parents[1]
        sources = sorted([worker, *(path for directory in ('src/js', 'hp-sim-3d/app')
                                    for path in (source_root / directory).rglob('*.js'))])
        digest = hashlib.sha256()
        for path in sources:
            digest.update(str(path.relative_to(source_root)).encode() + b'\0' + path.read_bytes())
        self.physics_host_sha256 = digest.hexdigest()
        self.log = (directory / 'physics-js.log').open('a')
        self.process = subprocess.Popen(['node', str(root / 'scripts/research_physics.mjs'),
                                         str(directory / 'scene.usda'), str(layering).lower(), scene_prim or ''], cwd=root,
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.log, text=True, bufsize=1)
        register_test_process(self.process)
        try:
            state = self.request(dt=dt)
            if state['dt_s'] != world.get_resource('dt'):
                raise ValueError('Rust/Python observation timesteps differ')
            self.physics_engine = state['physics_engine']
            self.physics_artifact = state['physics_artifact']
            from cable_joints_3d.ecs import SceneEntityInfoComponent
            expected = [[entity, world.get_component(entity, SceneEntityInfoComponent).name]
                        for entity in world.query([SceneEntityInfoComponent])]
            if state['identities'] != expected:
                raise ValueError('JS/Python scene entity identities differ; cannot mirror telemetry')
            self.apply(state)
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
        return state

    def apply(self, state):
        # Only detached observation fields are copied. Python never advances this world.
        stores = {component.__name__: store for component, store in self.world.components.items()}
        for name, rows in state['components']:
            for entity, values in rows:
                component = stores[name][entity]
                for key, value in values.items():
                    previous = getattr(component, key)
                    if isinstance(previous, np.ndarray):
                        value = np.asarray(value, dtype=float)
                    elif isinstance(previous, Quaternion):
                        value = Quaternion(*value)
                    elif key.startswith('machine_'):
                        value = {machine: np.asarray(position) for machine, position in value.items()}
                    elif key == 'extrusions':
                        value = [{**record, 'pos': np.asarray(record['pos'])} for record in value]
                    setattr(component, key, value)

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
