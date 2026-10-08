"""Run an authored 3D Hangprinter headlessly and record every step in Rerun."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from .machine_simulation import load_machine_world, register_machine_systems
from .machine_snapshot import capture_machine_snapshot
from .remote_spool_system import RemoteSpoolSystem


def _blueprint():
    import rerun.blueprint as rrb
    return rrb.Blueprint(rrb.Horizontal(
        rrb.Spatial3DView(origin='/world', name='Hangprinter'),
        rrb.Vertical(*[rrb.TimeSeriesView(origin='/' + path, name=name) for path, name in [
            ('line_lengths', 'Line lengths (m)'), ('line_errors', 'Length error / stretch (m)'),
            ('cable_forces', 'Cable forces (N)'), ('motors', 'Motor state'), ('encoders', 'Encoder state'),
        ]]), column_shares=[2, 1],
    ), rrb.TimePanel(timeline='sim_time', play_state=rrb.components.PlayState.Following))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene', type=Path)
    parser.add_argument('--scene-prim', help='Override the authored machine root')
    parser.add_argument('--steps', type=int, default=200)
    parser.add_argument('--dt', type=float, help='Override both authored cable and update timestep (seconds)')
    parser.add_argument('--commands', type=Path, help='JSON array of headless command records')
    parser.add_argument('--output', type=Path, help='RRD file; defaults to a timestamped file in output/rerun')
    parser.add_argument('--snapshot', type=Path, help='Also save final frames and cable telemetry as JSON')
    parser.add_argument('--connect', help='Optional Rerun gRPC sink URI for live viewing')
    parser.add_argument('--cable-solver-device', choices=['cpu', 'cuda:0'],
                        help='Opt-in compiled Warp solver; CUDA is experimental')
    args = parser.parse_args()
    if args.steps < 0 or (args.dt is not None and (not math.isfinite(args.dt) or args.dt <= 0)):
        parser.error('steps must be nonnegative and dt must be positive and finite')
    recording = None
    try:
        commands = json.loads(args.commands.read_text()) if args.commands is not None else None
        if commands is not None and not isinstance(commands, list):
            parser.error('commands must be a JSON array')
        world = load_machine_world(args.scene, args.scene_prim, cable_solver_device=args.cable_solver_device)
        if args.dt is not None:
            world.set_resource('dt', args.dt)
        dt = world.get_resource('dt')
        if not isinstance(dt, (int, float)) or not math.isfinite(dt) or dt <= 0:
            parser.error('authored dt must be positive and finite')
        import rerun as rr
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        output = args.output or Path('output/rerun') / f'hangprinter-python-{stamp}.rrd'
        output.parent.mkdir(parents=True, exist_ok=True)
        recording = rr.RecordingStream('hp-sim5 native Hangprinter')
        sinks = [rr.FileSink(output)]
        if args.connect is not None:
            sinks.append(rr.GrpcSink(args.connect))
        recording.set_sinks(*sinks, default_blueprint=_blueprint())
        recording.log('recording_info', rr.TextDocument(
            'Native Hangprinter simulation. Positions/lengths: metres; forces: newtons; rotations: XYZW.\n'
            'Cable geometry uses straight constraint spans. Stored intermediate wraps remain in length traces.\n'
            'Member transforms are local to live body frames; cable endpoints retain solver sample time.\n'
            'Velocity traces are ECS state. sim_time/sim_step advance on positive unpaused updates.\n'
            f'Source: {args.scene}; dt: {dt:g} s.'), static=True)
        register_machine_systems(world, recording)
        if commands is not None:
            world.get_system(RemoteSpoolSystem).commands = commands
        for _ in range(args.steps):
            world.update(dt)
        if args.snapshot is not None:
            args.snapshot.parent.mkdir(parents=True, exist_ok=True)
            args.snapshot.write_text(json.dumps(capture_machine_snapshot(world), allow_nan=False, indent=2) + '\n')
        recording.flush(timeout_sec=5)
        print(f'Recorded {args.steps} steps ({args.steps * dt:g} s): {output}')
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(1, f'Hangprinter: {error}\n')
    finally:
        if recording is not None:
            recording.disconnect()


if __name__ == '__main__':
    main()
