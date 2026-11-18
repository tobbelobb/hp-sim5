"""Rerun recording for detached native-world observations."""
import math

import rerun as rr

from . import mirror
from .columnar_recording import ColumnarRecording


def _rgb(color):
    value = color.lstrip('#')
    if len(value) == 3:
        value = ''.join(character * 2 for character in value)
    return [int(value[index:index + 2], 16) for index in (0, 2, 4)]


def blueprint(*, timeline='sim_time'):
    import rerun.blueprint as rrb
    metrics = [
        rrb.TimeSeriesView(origin='/line_lengths', name='Line lengths (m)'),
        rrb.TimeSeriesView(origin='/line_errors', name='Length error / stretch (m)'),
        rrb.TimeSeriesView(origin='/cable_forces', name='Cable forces (N)'),
    ]
    drive = rrb.Vertical(
        rrb.TimeSeriesView(origin='/motors', name='Motors'),
        rrb.TimeSeriesView(origin='/encoders', name='Encoders'),
        name='Drive',
    )
    return rrb.Blueprint(
        rrb.Horizontal(rrb.Spatial3DView(origin='/world', name='Hangprinter'),
                       rrb.Tabs(rrb.Vertical(*metrics, name='Cables'), drive), column_shares=[2, 1]),
        rrb.TimePanel(timeline=timeline, play_state=rrb.components.PlayState.Following),
    )


class RerunSystem:
    """Log poses, cable spans and scalar telemetry from a detached world."""

    def __init__(self, recording):
        self.recording = recording
        self.root = 'world'
        self.timeline = 'sim_time'
        self.elapsed = 0.
        self.step = 0
        self._coordinates_logged = False
        self._frame_paths = set()
        self._series = set()
        self._columns = ColumnarRecording(recording)

    def flush(self):
        self._columns.flush()
        self.recording.flush(timeout_sec=5)

    def update(self, world, dt, *, recording_step=None):
        if not math.isfinite(dt):
            raise ValueError('Recording timestep must be finite')
        if not self._coordinates_logged:
            self.recording.log(self.root, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
            self._coordinates_logged = True
        pause = world.get_resource('pauseState')
        if dt > 0 and not (pause is not None and getattr(pause, 'paused', False)):
            self.elapsed += dt
            self.step += 1
        timelines = {
            'sim_step': ('sequence', self.step if recording_step is None else recording_step),
            self.timeline: ('duration', self.elapsed),
        }
        if recording_step is not None:
            timelines['scene_generation'] = ('sequence', world.get_resource('sceneGeneration'))
            timelines['sim_step_in_generation'] = ('sequence', self.step)
        for name, (kind, value) in timelines.items():
            self.recording.set_time(name, **{kind: value})
        self._columns.begin(**timelines)
        columns = self._columns.append
        snapshot = mirror.capture_snapshot(world)
        for frame in snapshot['frames']:
            path = frame['path'].replace('world', self.root, 1)
            columns(path, rr.Transform3D, translation=[frame['position']], quaternion=[frame['quaternion']])
            if path not in self._frame_paths:
                effector = frame['kind'] == 'effector'
                color = [255, 130, 40] if effector else [100, 150, 220]
                self.recording.log(path + '/shape', rr.Points3D([[0, 0, 0]], colors=color,
                                                               radii=.008 if effector else .006), static=True)
                self.recording.log(path + '/axes', rr.TransformAxes3D(.15 if effector else .025), static=True)
                self._frame_paths.add(path)
        for cable in snapshot['cables']:
            key = f"{cable['machine']}/{cable['name']}"
            visual = f"{self.root}/machines/{cable['machine']}/cables/{cable['name']}"
            segments = cable['segments']
            if segments:
                self.recording.log(visual + '/segments', rr.LineStrips3D(
                    [segment['points'] for segment in segments], colors=[_rgb(cable['color'])] * len(segments)))
                self.recording.log(visual + '/forces', rr.Arrows3D(
                    origins=[segment['origin'] for segment in segments],
                    vectors=[[value * .01 for value in segment['force_vector_n']] for segment in segments],
                    colors=[[255, 90, 90]] * len(segments)))
            for name, value in cable['lengths'].items():
                namespace = 'line_errors' if name in ('error', 'stretch') else 'line_lengths'
                path = f'{namespace}/{key}/{name}'
                if value is not None:
                    self.recording.log(path, rr.Scalars(value))
                    self._series.add(path)
                elif path in self._series:
                    self.recording.log(path, rr.Clear.flat())
                    self._series.remove(path)
            for segment in segments:
                self.recording.log(f"cable_forces/{key}/{segment['name']}", rr.Scalars(segment['force_n']))

        self._record_telemetry(world)
        self._columns.end()
        return snapshot

    def _record_telemetry(self, world):
        columns = self._columns.append

        def scalars(path, **values):
            for name, value in values.items():
                columns(f'{path}/{name}', rr.Scalars, scalars=[value])

        for entity in world.query([mirror.StepperMotorComponent]):
            motor = world.get_component(entity, mirror.StepperMotorComponent)
            path = 'motors/' + mirror.entity_key(world, entity)
            scalars(path, commanded_angle=motor.commanded_angle, reference_angle=motor.delta_angle,
                    target_torque_nm=motor.target_torque, torque_mode=float(motor.torque_mode),
                    missed_steps=motor.missed_steps, current_missed_steps=motor.current_missed_steps)
            if not motor.torque_mode:
                scalars(path, target_angle_rad=motor.commanded_angle - motor.delta_angle)
        for entity in world.query([mirror.EncoderComponent]):
            encoder = world.get_component(entity, mirror.EncoderComponent)
            x, y, z = encoder.axis
            scalars('encoders/' + mirror.entity_key(world, entity), angle_rad=encoder.angle, axis_x=x, axis_y=y, axis_z=z)

        for component, attribute, prefix in ((mirror.VelocityComponent, 'vel', 'linear'),
                                              (mirror.AngularVelocityComponent, 'omega', 'angular')):
            for entity in world.query([component]):
                velocity = getattr(world.get_component(entity, component), attribute)
                scalars('velocities/' + mirror.entity_key(world, entity),
                        **{f'{prefix}_{axis}': value for axis, value in zip('xyz', velocity)})

        deposits = {}
        for entity in world.query([mirror.ExtruderComponent]):
            extruder = world.get_component(entity, mirror.ExtruderComponent)
            for label, positions in (('effector_center', extruder.machine_effector_centers),
                                     ('root', extruder.machine_centers), ('tip', extruder.machine_tips),
                                     ('cold_end', extruder.machine_cold_ends)):
                for machine, point in positions.items():
                    path = f'{self.root}/machines/{mirror._path_part(machine)}/tool_points/{label}'
                    columns(path, rr.Points3D, positions=[point.tolist()], colors=[[255, 180, 40]], radii=[.003])
            for record in extruder.extrusions:
                deposits.setdefault(record.get('machineId') or 'default', []).append(record)
        for machine, records in deposits.items():
            key = mirror._path_part(machine)
            columns(f'{self.root}/machines/{key}/extrusions/deposits', rr.Points3D,
                    positions=[record['pos'].tolist() for record in records],
                    colors=[_rgb(record.get('color') or '#800080') for record in records], radii=[.001] * len(records))
            columns(f'extrusion_lengths/{key}', rr.Scalars, scalars=[sum(record['length'] for record in records)])
