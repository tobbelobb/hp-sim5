"""Production JS stepping; the Python world is a read-only telemetry mirror."""
import json
import subprocess

import numpy as np

from cable_joints_3d.quaternion import Quaternion


class JSPhysics:
    def __init__(self, root, directory, world):
        self.world = world
        self.physics_wall_s = 0.
        self.log = (directory / 'physics-js.log').open('a')
        self.process = subprocess.Popen(['node', str(root / 'scripts/research_physics.mjs'),
                                         str(directory / 'scene.usda')], cwd=root,
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.log, text=True, bufsize=1)
        try:
            state = self.request()
            from cable_joints_3d.ecs import SceneEntityInfoComponent
            expected = [[entity, world.get_component(entity, SceneEntityInfoComponent).name]
                        for entity in world.query([SceneEntityInfoComponent])]
            if state['identities'] != expected:
                raise ValueError('JS/Python scene entity identities differ; cannot mirror telemetry')
            self.apply(state)
        except Exception:
            self.close()
            raise

    def request(self, *, steps=0, commands=(), clear_commands=False):
        self.process.stdin.write(json.dumps({'steps': steps, 'commands': commands,
                                            'clear_commands': clear_commands}) + '\n')
        self.process.stdin.flush()
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError('JS physics worker exited; inspect physics-js.log')
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
