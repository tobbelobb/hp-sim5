"""Persistent fixed-step native world implementing the browser's bridge protocol."""
import asyncio
import json
import math
import os
import time
import uuid
from pathlib import Path

import rerun as rr
from websockets.asyncio.client import connect

from cable_joints_3d.__main__ import _blueprint
from cable_joints_3d.ecs import EncoderComponent
from cable_joints_3d.machine_simulation import load_machine_world
from cable_joints_3d.remote_spool_system import RemoteSpoolSystem
from cable_joints_3d.rerun_system import RerunSystem
from usd.cable_scene_loader import open_cable_scene
from .experiments import DEFAULT_SCENE, encode, repo_path, sample, validate_commands


class NativeSession:
    def __init__(self, root, directory, scene=DEFAULT_SCENE, *, backend='headless-js', record=True, frozen_scene=None, layering=True):
        if backend not in ('native-python', 'headless-js', 'native-warp', 'native-warp-cuda'):
            raise ValueError('Unknown physics backend')
        self.backend = backend
        self.cable_solver_device = {'native-warp': 'cpu', 'native-warp-cuda': 'cuda:0'}.get(backend)
        self.recording_enabled = record
        self.root = Path(root)
        self.js_physics = None
        self.worker_lock = asyncio.Lock()
        self.physics_wall_s = 0.
        self.worker_wall_s = 0.
        self.observation_wall_s = 0.
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.layering = layering
        if frozen_scene is None:
            stage = open_cable_scene(repo_path(root, scene), **({} if layering else
                {'derive_all': True, 'cable_path_half_width_override': 0.}))
            layer = stage.Flatten()
            # Flatten appends a multiline provenance doc unsupported by the JS serializer.
            layer.documentation = stage.GetMetadata('doc') or ''
            frozen_scene = layer.ExportToString()
        self.frozen_scene = frozen_scene
        (self.directory / 'scene.usda').write_text(self.frozen_scene)
        self.world = self.create_world()
        self.dt = self.world.get_resource('dt')
        if not math.isclose(self.dt, .002, abs_tol=1e-12):
            raise ValueError('RRF bridge requires a 0.002 s physics timestep')
        self.remote = self.world.get_system(RemoteSpoolSystem)
        self.remote._ensure_axis_mapping(self.world)
        if set(self.remote.axis_to_entity) != set('ABCD'):
            raise ValueError('Collection requires one Hangprinter with axes A/B/C/D')
        self.step = 0
        self.epoch = 0
        self.collector_time_s = 0.
        self.speed = 1.
        self.connected = asyncio.Event()
        self.lock = asyncio.Lock()
        self.error = None
        self.deadline = None
        self.cancel_requested = False
        self.recording_id = uuid.uuid4().hex
        self.events = (self.directory / 'events.jsonl').open('w')
        self.recording = None
        self.recorder = None
        try:
            if backend == 'headless-js':
                from .js_physics import JSPhysics
                self.js_physics = JSPhysics(Path(root), self.directory, self.world, layering=self.layering)
            self.start_recording(self.directory / 'recording.rrd')
        except Exception:
            self.close()
            raise

    def create_world(self):
        world = load_machine_world(self.frozen_scene, cable_solver_device=self.cable_solver_device)
        for key in ('enableLayering', 'layeringFrictionEffectiveRadius', 'layeringRenderWraps'):
            world.set_resource(key, self.layering)
        return world

    def start_recording(self, path):
        if self.recording is not None:
            self.recorder.flush()
            self.recording.flush(timeout_sec=5)
            self.recording.disconnect()  # finalize this file's footer before returning it
        self.recording_path = Path(path)
        self.last_observed = None
        if not self.recording_enabled:
            self.recorder = None
            self.observe()
            return
        self.recording = rr.RecordingStream('hp-sim5 collection', recording_id=self.recording_id,
            batcher_config=rr.ChunkBatcherConfig(flush_tick=5., flush_num_bytes=16 * 1024 * 1024))
        sinks = [rr.FileSink(self.recording_path)]
        viewer = os.environ.get('HP_SIM5_VIEWER_URL')
        self.live_viewer = bool(viewer)
        self.next_live_flush = time.monotonic() + 5
        if viewer:
            sinks.append(rr.GrpcSink(viewer.replace('http://', 'rerun+http://') + '/proxy'))
        self.recording.set_sinks(*sinks, default_blueprint=_blueprint())
        self.recorder = RerunSystem(self.recording)
        self.last_observed = None
        self.observe()
        if self.live_viewer:
            self.recorder.flush()

    def event(self, kind, **values):
        self.events.write(encode({'type': kind, 'epoch': self.epoch, 'step': self.step,
                                 'sim_time_s': self.step * self.dt, 'scene_generation': self.world.get_resource('sceneGeneration'), **values}) + '\n')
        self.events.flush()

    def observe(self):
        # Record at 10 simulated Hz and at every encoder observation. Physics remains 500 Hz.
        if self.last_observed == (self.epoch, self.step):
            return
        started = time.monotonic()
        self.last_observed = (self.epoch, self.step)
        snapshot = None
        if self.recorder is not None:
            self.recorder.step = self.step
            self.recorder.elapsed = self.step * self.dt
            snapshot = self.recorder.update(self.world, 0.)
        self.event('observation', sample=sample(self.world, self.step, self.dt, snapshot=snapshot),
                   recording=str(self.recording_path) if self.recording is not None else None)
        if self.recording is not None and self.live_viewer and time.monotonic() >= self.next_live_flush:
            # Explicit delivery also updates the Viewer during long collector advances.
            self.recorder.flush()
            self.recording.flush(timeout_sec=5)
            self.next_live_flush = time.monotonic() + 5
        self.observation_wall_s += time.monotonic() - started

    def encoder_angles(self, axes):
        # Match externalCommandSocket: first mapped EncoderComponent, raw degrees.
        result = []
        for axis in axes:
            angle = None
            for entity in self.remote.axis_to_entity.get(axis, []):
                encoder = self.world.get_component(entity, EncoderComponent)
                if encoder is not None and math.isfinite(encoder.angle):
                    angle = math.degrees(encoder.angle)
                    break
            result.append(angle)
        return result

    def stop_execution(self):
        # Called on the event loop between fixed steps. No later payload may mutate this world.
        self.cancel_requested = True
        queued = self.queue_length()
        self.remote.clear_command_queue()
        if self.js_physics:
            self.js_physics.request(clear_commands=True)
        self.js_queue_length = 0
        self.error = 'Collection cancelled; reset_session is required'
        self.observe()
        self.event('execution_cancelled', discarded_commands=queued)
        return {'step': self.step, 'sim_time_s': self.step * self.dt,
                'discarded_commands': queued, 'boundary': 'before next fixed physics step',
                'reset_required': True}

    async def stop_at_boundary(self):
        self.cancel_requested = True
        # JS yields between bounded batches; report the observed stopped step.
        # No subsequent batch is submitted. Python yields between fixed steps.
        async with self.worker_lock:
            return self.stop_execution()

    async def advance_js(self, count):
        remaining = count
        while remaining:
            if self.cancel_requested:
                raise RuntimeError(self.error or 'Collection cancelled; reset_session is required')
            if self.deadline is not None and time.monotonic() > self.deadline:
                raise TimeoutError('Collection exceeded its wall-time budget')
            batch = min(remaining, 50 - self.step % 50)
            async with self.worker_lock:
                if self.cancel_requested:
                    raise RuntimeError(self.error or 'Collection cancelled; reset_session is required')
                commands = list(self.remote.commands)
                self.remote.clear_command_queue()
                started = time.monotonic()
                # One bounded batch takes milliseconds; yield between batches so
                # cancellation and bridge delivery do not wait for an entire move.
                state = self.js_physics.request(steps=batch, commands=commands)
                self.worker_wall_s += time.monotonic() - started
                self.physics_wall_s += state['physics_wall_s'] - self.js_physics.physics_wall_s
                self.js_physics.physics_wall_s = state['physics_wall_s']
                if state['step'] != self.step + batch:
                    raise RuntimeError('JS worker did not execute the requested fixed steps')
                self.js_physics.apply(state)
                self.step = state['step']
                self.collector_time_s += batch * self.dt / self.speed
                self.js_queue_length = state['queue_length']
                if self.step % 50 == 0:
                    self.observe()
            remaining -= batch
            await asyncio.sleep(0)
        if self.cancel_requested:
            raise RuntimeError(self.error or 'Collection cancelled; reset_session is required')
        if self.recording is not None and self.live_viewer:
            self.recorder.flush()
        return self.status()

    def queue_length(self):
        return self.remote.get_queue_length() + (getattr(self, 'js_queue_length', 0) if self.js_physics else 0)

    async def advance(self, seconds=0., *, drain=False):
        if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not math.isfinite(seconds) or not 0 <= seconds <= 120:
            raise ValueError('Advance seconds must be finite and in [0, 120]')
        async with self.lock:
            count = max(math.ceil(seconds / self.dt - 1e-9), self.queue_length() if drain else 0)
            if count > 60_000:
                raise ValueError('Motion exceeds the 60,000-step completion bound')
            if self.js_physics:
                return await self.advance_js(count)
            for index in range(count):
                if self.cancel_requested:
                    raise RuntimeError(self.error)
                if self.deadline is not None and time.monotonic() > self.deadline:
                    raise TimeoutError('Collection exceeded its wall-time budget')
                started = time.monotonic()
                self.world.update(self.dt)
                self.physics_wall_s += time.monotonic() - started
                self.step += 1
                self.collector_time_s += self.dt / self.speed
                if self.step % 50 == 0:
                    self.observe()  # JSON encoding also rejects non-finite physical state
                if index % 10 == 0:
                    await asyncio.sleep(0)
            if self.recording is not None and self.live_viewer:
                self.recorder.flush()
            return self.status()

    def status(self):
        from cable_joints_3d.cable_joints_components import CablePathComponent
        return {'connected': self.connected.is_set(), 'epoch': self.epoch, 'step': self.step,
                'sim_time_s': self.step * self.dt, 'collector_time_s': self.collector_time_s,
                'dt_s': self.dt, 'speed_scale': self.speed, 'queue_length': self.queue_length(),
                'layering_flags': {key: self.world.get_resource(key) for key in
                    ('enableLayering', 'layeringFrictionEffectiveRadius', 'layeringRenderWraps')},
                'cable_half_widths_m': sorted({self.world.get_component(entity, CablePathComponent).cable_half_width
                                              for entity in self.world.query([CablePathComponent])}),
                'backend': self.backend, 'physics_engine': self.js_physics.physics_engine if self.js_physics else self.backend,
                'physics_artifact': self.js_physics.physics_artifact if self.js_physics else None,
                'physics_wall_s': self.physics_wall_s,
                'worker_wall_s': self.worker_wall_s,
                'observation_wall_s': self.observation_wall_s,
                'clock': 'fixed-step; collector delays advance simulation time', 'error': self.error}

    async def handle(self, payload):
        if self.cancel_requested:
            raise RuntimeError(self.error)
        kind = payload.get('type')
        self.event('bridge_payload', payload=payload)
        if kind == 'encoder_request':
            if payload['axes']:
                await self.advance(drain=True)
            angles = self.encoder_angles(payload['axes'])
            self.observe()
            if self.recording is not None and self.live_viewer:
                self.recorder.flush()
            self.event('encoder_response', request_id=payload['requestId'], axes=payload['axes'], angles_deg=angles)
            return {'type': 'encoder_response', 'requestId': payload['requestId'],
                    'axes': payload['axes'], 'anglesDeg': angles}
        if kind == 'reset':
            async with self.lock:
                self.world = self.create_world()
                self.epoch += 1
                self.world.set_resource('sceneGeneration', self.epoch + 1)
                self.remote = self.world.get_system(RemoteSpoolSystem)
                self.remote._ensure_axis_mapping(self.world)
                if self.js_physics:
                    self.js_physics.close()
                    from .js_physics import JSPhysics
                    self.js_physics = JSPhysics(self.root, self.directory, self.world, layering=self.layering)
                    self.js_queue_length = 0
                self.step = 0
                self.observe()
            return None
        if kind == 'set_speed_scale':
            value = payload['value']
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('Speed scale must be positive and finite')
            self.speed = value
        if kind and kind.startswith('klipper_'):
            raise ValueError('Klipper streamed motion is not supported by this RRF session')
        commands = ([payload['command']] if kind == 'command' else []) + payload.get('commands', [])
        # Bridge records carry firmware provenance which the motor API does not consume.
        validate_commands([{k: v for k, v in command.items() if k not in ('driver', 'timestamp')}
                           for command in commands], set(self.remote.axis_to_entity), 60_000)
        if self.queue_length() + len(commands) > 60_000:
            raise ValueError('Native command queue exceeds 60,000 steps')
        for command in commands:
            self.remote.add_command(command)

    async def run_socket(self, url):
        try:
            async with connect(url, max_size=16 * 1024 * 1024) as socket:
                self.connected.set()
                async for message in socket:
                    response = await self.handle(json.loads(message))
                    if response is not None:
                        await socket.send(encode(response))
        except asyncio.CancelledError:
            raise
        except Exception as error:
            self.error = str(error)
            self.event('failure', error=self.error)
        finally:
            self.connected.clear()

    def close(self):
        if self.js_physics:
            self.js_physics.close()
        if self.recording is not None:
            if self.recorder is not None:
                self.recorder.flush()
            self.recording.flush(timeout_sec=5)
            self.recording.disconnect()
        self.events.close()
