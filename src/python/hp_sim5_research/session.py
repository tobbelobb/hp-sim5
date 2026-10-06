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
    def __init__(self, root, directory, scene=DEFAULT_SCENE):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.frozen_scene = open_cable_scene(repo_path(root, scene)).Flatten().ExportToString()
        (self.directory / 'scene.usda').write_text(self.frozen_scene)
        self.world = load_machine_world(self.frozen_scene)
        self.dt = self.world.get_resource('dt')
        if not math.isclose(self.dt, .002, abs_tol=1e-12):
            raise ValueError('RRF bridge requires a 0.002 s physics timestep')
        self.remote = self.world.get_system(RemoteSpoolSystem)
        self.remote._ensure_axis_mapping(self.world)
        if set(self.remote.axis_to_entity) != set('ABCD'):
            raise ValueError('Native collection currently supports one HP4 with axes A/B/C/D')
        self.step = 0
        self.epoch = 0
        self.collector_time_s = 0.
        self.speed = 1.
        self.connected = asyncio.Event()
        self.lock = asyncio.Lock()
        self.error = None
        self.deadline = None
        self.recording_id = uuid.uuid4().hex
        self.events = (self.directory / 'events.jsonl').open('w')
        self.recording = None
        self.start_recording(self.directory / 'recording.rrd')

    def start_recording(self, path):
        if self.recording is not None:
            self.recording.flush(timeout_sec=5)
            self.recording.disconnect()  # finalize this file's footer before returning it
        self.recording_path = Path(path)
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

    def event(self, kind, **values):
        self.events.write(encode({'type': kind, 'epoch': self.epoch, 'step': self.step,
                                 'sim_time_s': self.step * self.dt, **values}) + '\n')
        self.events.flush()

    def observe(self):
        # Record at 10 simulated Hz and at every encoder observation. Physics remains 500 Hz.
        if self.last_observed == (self.epoch, self.step):
            return
        self.last_observed = (self.epoch, self.step)
        self.recorder.step = self.step
        self.recorder.elapsed = self.step * self.dt
        self.recorder.update(self.world, 0.)
        self.event('observation', sample=sample(self.world, self.step, self.dt))
        if self.live_viewer and time.monotonic() >= self.next_live_flush:
            # Explicit delivery also updates the Viewer during long collector advances.
            self.recording.flush(timeout_sec=5)
            self.next_live_flush = time.monotonic() + 5

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

    async def advance(self, seconds=0., *, drain=False):
        if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not math.isfinite(seconds) or not 0 <= seconds <= 120:
            raise ValueError('Advance seconds must be finite and in [0, 120]')
        async with self.lock:
            count = max(math.ceil(seconds / self.dt - 1e-9), self.remote.get_queue_length() if drain else 0)
            if count > 60_000:
                raise ValueError('Motion exceeds the 60,000-step completion bound')
            for index in range(count):
                if self.deadline is not None and time.monotonic() > self.deadline:
                    raise TimeoutError('Collection exceeded its wall-time budget')
                self.world.update(self.dt)
                self.step += 1
                self.collector_time_s += self.dt / self.speed
                if self.step % 50 == 0:
                    self.observe()  # JSON encoding also rejects non-finite physical state
                if index % 10 == 0:
                    await asyncio.sleep(0)
            return self.status()

    def status(self):
        return {'connected': self.connected.is_set(), 'epoch': self.epoch, 'step': self.step,
                'sim_time_s': self.step * self.dt, 'collector_time_s': self.collector_time_s,
                'dt_s': self.dt, 'speed_scale': self.speed, 'queue_length': self.remote.get_queue_length(),
                'clock': 'fixed-step; collector delays advance simulation time', 'error': self.error}

    async def handle(self, payload):
        kind = payload.get('type')
        self.event('bridge_payload', payload=payload)
        if kind == 'encoder_request':
            if payload['axes']:
                await self.advance(drain=True)
            angles = self.encoder_angles(payload['axes'])
            self.observe()
            self.event('encoder_response', request_id=payload['requestId'], axes=payload['axes'], angles_deg=angles)
            return {'type': 'encoder_response', 'requestId': payload['requestId'],
                    'axes': payload['axes'], 'anglesDeg': angles}
        if kind == 'reset':
            async with self.lock:
                self.world = load_machine_world(self.frozen_scene)
                self.epoch += 1
                self.world.set_resource('sceneGeneration', self.epoch + 1)
                self.remote = self.world.get_system(RemoteSpoolSystem)
                self.remote._ensure_axis_mapping(self.world)
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
        if self.remote.get_queue_length() + len(commands) > 60_000:
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
        if self.recording is not None:
            self.recording.flush(timeout_sec=5)
            self.recording.disconnect()
        self.events.close()
