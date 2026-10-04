"""Native Rerun frames, cable telemetry and motor/extrusion diagnostics."""
from dataclasses import dataclass, field
import math

import numpy as np

from .ecs import AngularVelocityComponent, EncoderComponent, OrientationComponent, PositionComponent, RadiusComponent, RenderableComponent, VelocityComponent
from .extruder import ExtruderComponent
from .machine_snapshot import capture_machine_snapshot, entity_name, frame_path, machine_id, path_part
from .stepper_motor import StepperMotorComponent


def _rgb(color):
    value = color.lstrip('#')
    if len(value) == 3:
        value = ''.join(character * 2 for character in value)
    return [int(value[index:index + 2], 16) for index in (0, 2, 4)]


@dataclass
class RerunSystem:
    recording: object
    root: str = 'world'
    timeline: str = 'sim_time'
    elapsed: float = 0.
    force_scale: float = .01
    step: int = 0
    _generation: int | None = field(default=None, init=False)
    _active_paths: set[str] = field(default_factory=set, init=False)
    _shape_styles: dict = field(default_factory=dict, init=False)
    _series_names: dict = field(default_factory=dict, init=False)
    _entity_tokens: dict = field(default_factory=dict, init=False)
    _axes_paths: set[str] = field(default_factory=set, init=False)
    _coordinates_logged: bool = field(default=False, init=False)
    run_in_pause = True

    def _clear_path(self, rr, path):
        # Temporal clears cannot shadow static archetypes in latest-at queries.
        self.recording.log(path, rr.Clear(recursive=True), static=True)
        self.recording.log(path, rr.Clear(recursive=True))

    def _clear_paths(self, rr, paths):
        for path in sorted(paths):
            parts = path.split('/')
            if not any('/'.join(parts[:end]) in paths for end in range(1, len(parts))):
                self._clear_path(rr, path)

    def update(self, world, dt):
        import rerun as rr

        if not math.isfinite(dt):
            raise ValueError('Recording timestep must be finite')
        if not self._coordinates_logged:
            self.recording.log(self.root, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
            self._coordinates_logged = True
        snapshot = capture_machine_snapshot(world)
        entity_paths = {frame_path(world, entity).replace('world', self.root, 1): entity
                        for entity in world.query([PositionComponent])}
        generation = world.get_resource('sceneGeneration') or 0
        reused = any(path in self._entity_tokens and self._entity_tokens[path] is not
                     world.get_component(entity, PositionComponent) for path, entity in entity_paths.items())
        if self._generation is not None and (generation != self._generation or reused):
            self.elapsed, self.step = 0., 0
            self._clear_paths(rr, self._active_paths)
            self._active_paths.clear()
            self._shape_styles.clear()
            self._series_names.clear()
            self._entity_tokens.clear()
            self._axes_paths.clear()
        self._generation = generation
        pause = world.get_resource('pauseState')
        if dt > 0 and not (pause is not None and getattr(pause, 'paused', False)):
            self.elapsed += dt
            self.step += 1
        self.recording.set_time('scene_generation', sequence=generation)
        self.recording.set_time('sim_step', sequence=self.step)
        self.recording.set_time(self.timeline, duration=self.elapsed)
        active = set()

        def log(path, value, **options):
            active.add(path)
            self.recording.log(path, value, **options)

        def series(path, names, values):
            # Each quantity has a stable path. Omitting a torque-mode target or
            # changing topology must not reinterpret an earlier series index.
            for name, value in zip(names, values):
                metric = path + '/' + path_part(name)
                if metric not in self._series_names:
                    log(metric, rr.SeriesLines(names=[f'{path}: {name}']))
                    self._series_names[metric] = name
                log(metric, rr.Scalars([value]))

        for frame in snapshot['frames']:
            path = frame['path'].replace('world', self.root, 1)
            entity = entity_paths.get(path)
            if entity is not None:
                self._entity_tokens[path] = world.get_component(entity, PositionComponent)
            log(path, rr.Transform3D(translation=frame['position'], rotation=rr.Quaternion(xyzw=frame['quaternion'])))
            if entity is None or world.has_component(entity, OrientationComponent):
                active.add(path + '/axes')
                if path not in self._axes_paths:
                    log(path + '/axes', rr.TransformAxes3D(.15 if frame['kind'] == 'effector' else .025), static=True)
                    self._axes_paths.add(path)
            render = world.get_component(entity, RenderableComponent) if entity is not None else None
            radius = world.get_component(entity, RadiusComponent) if entity is not None else None
            style = (render.shape, render.color, radius.radius) if render and radius else None
            if entity is None:
                style = ('point', '#ff8228', .008)
            if style is not None:
                active.add(path + '/shape')
                if self._shape_styles.get(path) != style:
                    log(path + '/shape', rr.Points3D([[0, 0, 0]], colors=_rgb(style[1]), radii=style[2]), static=True)
                    self._shape_styles[path] = style
            if entity is not None:
                names, values = [], []
                for component, attribute, prefix in [(VelocityComponent, 'vel', 'linear'),
                                                     (AngularVelocityComponent, 'omega', 'angular')]:
                    state = world.get_component(entity, component)
                    if state is not None:
                        names.extend(prefix + '_' + axis for axis in 'xyz')
                        values.extend(getattr(state, attribute))
                if names:
                    key = f'{path_part(machine_id(world, entity))}/{entity_name(world, entity)}'
                    series('velocities/' + key, names, values)

        for cable in snapshot['cables']:
            key = f"{cable['machine']}/{cable['name']}"
            visual = f"{self.root}/machines/{cable['machine']}/cables/{cable['name']}"
            segments = cable['segments']
            if segments:
                log(visual + '/segments', rr.LineStrips3D([segment['points'] for segment in segments],
                    colors=_rgb(cable['color']), radii=rr.Radius.ui_points(1.5)))
                log(visual + '/forces', rr.Arrows3D(origins=[segment['origin'] for segment in segments],
                    vectors=np.array([segment['force_vector_n'] for segment in segments]) * self.force_scale,
                    colors=[255, 90, 90], radii=.002))
                series('cable_forces/' + key, [segment['name'] for segment in segments],
                       [segment['force_n'] for segment in segments])
            lengths = cable['lengths']
            for prefix, candidates in [('line_lengths', ['commanded', 'actual', 'geometric']),
                                       ('line_errors', ['error', 'stretch'])]:
                names = [name for name in candidates if lengths[name] is not None]
                series(prefix + '/' + key, names, [lengths[name] for name in names])

        for entity in world.query([StepperMotorComponent]):
            motor = world.get_component(entity, StepperMotorComponent)
            key = f'{path_part(machine_id(world, entity))}/{entity_name(world, entity)}'
            names = ['commanded_angle', 'reference_angle', 'target_torque_nm', 'torque_mode',
                     'current_missed_steps', 'peak_missed_steps']
            values = [motor.commanded_angle, motor.delta_angle, motor.target_torque, float(motor.torque_mode),
                      motor.current_missed_steps, motor.missed_steps]
            series('motors/' + key, names, values)

        for entity in world.query([EncoderComponent]):
            encoder = world.get_component(entity, EncoderComponent)
            key = f'{path_part(machine_id(world, entity))}/{entity_name(world, entity)}'
            series('encoders/' + key, ['angle', 'axis_x', 'axis_y', 'axis_z'], [encoder.angle, *encoder.axis])

        for entity in world.query([ExtruderComponent]):
            extruder = world.get_component(entity, ExtruderComponent)
            maps = [('effector_center', extruder.machine_effector_centers), ('root', extruder.machine_centers),
                    ('tip', extruder.machine_tips), ('cold_end', extruder.machine_cold_ends)]
            machines = dict.fromkeys(machine for _, positions in maps for machine in positions)
            for machine in machines:
                points = [(name, positions[machine]) for name, positions in maps if positions.get(machine) is not None]
                log(f'{self.root}/machines/{path_part(machine)}/tool_points/{entity}',
                    rr.Points3D([point for _, point in points], labels=[name for name, _ in points],
                                colors=[255, 180, 40], radii=.003))
            deposits = extruder.extrusions
            machines = dict.fromkeys(record.get('machineId') or 'default' for record in deposits)
            for machine in machines:
                records = [record for record in deposits if (record.get('machineId') or 'default') == machine]
                path = f'{self.root}/machines/{path_part(machine)}/extrusions/{entity}'
                log(path, rr.Points3D([record['pos'] for record in records],
                    colors=[_rgb(record.get('color') or '#800080') for record in records], radii=.001))
                series(f'extrusion_lengths/{path_part(machine)}/{entity}', ['deposited_length'],
                       [sum(record['length'] for record in records)])

        removed = self._active_paths - active
        self._clear_paths(rr, removed)
        for path in removed:
            self._series_names.pop(path, None)
            if path.endswith('/shape'):
                self._shape_styles.pop(path[:-6], None)
            if path.endswith('/axes'):
                self._axes_paths.discard(path[:-5])
            self._entity_tokens.pop(path, None)
        self._active_paths = active
