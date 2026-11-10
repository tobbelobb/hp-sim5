"""Rerun recording for detached native-world observations."""
import math

from . import mirror


def _rgb(color):
    value = color.lstrip('#')
    if len(value) == 3:
        value = ''.join(character * 2 for character in value)
    return [int(value[index:index + 2], 16) for index in (0, 2, 4)]


def blueprint():
    import rerun.blueprint as rrb
    metrics = [
        rrb.TimeSeriesView(origin='/line_lengths', name='Line lengths (m)'),
        rrb.TimeSeriesView(origin='/line_errors', name='Length error / stretch (m)'),
        rrb.TimeSeriesView(origin='/cable_forces', name='Cable forces (N)'),
    ]
    return rrb.Blueprint(
        rrb.Horizontal(rrb.Spatial3DView(origin='/world', name='Hangprinter'),
                       rrb.Vertical(*metrics), column_shares=[2, 1]),
        rrb.TimePanel(timeline='sim_time', play_state=rrb.components.PlayState.Following),
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
        self._series = set()

    def flush(self):
        self.recording.flush(timeout_sec=5)

    def update(self, world, dt):
        import rerun as rr
        if not math.isfinite(dt):
            raise ValueError('Recording timestep must be finite')
        if not self._coordinates_logged:
            self.recording.log(self.root, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
            self._coordinates_logged = True
        pause = world.get_resource('pauseState')
        if dt > 0 and not (pause is not None and getattr(pause, 'paused', False)):
            self.elapsed += dt
            self.step += 1
        self.recording.set_time('sim_step', sequence=self.step)
        self.recording.set_time(self.timeline, duration=self.elapsed)
        snapshot = mirror.capture_snapshot(world)
        for frame in snapshot['frames']:
            path = frame['path'].replace('world', self.root, 1)
            self.recording.log(path, rr.Transform3D(translation=frame['position'],
                                                    quaternion=frame['quaternion']))
            self.recording.log(path + '/point', rr.Points3D([frame['position']],
                colors=[255, 130, 40] if frame['kind'] == 'effector' else [100, 150, 220], radii=[.006]))
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
                if value is None:
                    continue
                if name in ('actual', 'commanded', 'geometric'):
                    self.recording.log(f'line_lengths/{key}/{name}', rr.Scalars(value))
                if name in ('error', 'stretch'):
                    self.recording.log(f'line_errors/{key}/{name}', rr.Scalars(value))
            for segment in segments:
                self.recording.log(f"cable_forces/{key}/{segment['name']}", rr.Scalars(segment['force_n']))
        return snapshot
