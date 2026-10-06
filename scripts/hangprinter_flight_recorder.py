#!/usr/bin/env python3
"""Record hp-sim-3d timesteps in Rerun, with a live viewer and durable RRD files."""

import argparse
import asyncio
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import uuid
from urllib.parse import urlencode

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed


def blueprint():
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/world", name="Hangprinter"),
            rrb.Vertical(
                rrb.TimeSeriesView(origin="/line_lengths", name="Line lengths (m)"),
                rrb.TimeSeriesView(origin="/line_errors", name="Length error / stretch (m)"),
                rrb.TimeSeriesView(origin="/cable_forces", name="Cable forces (N)"),
            ),
            column_shares=[2, 1],
        ),
        rrb.TimePanel(timeline="sim_time", play_state=rrb.components.PlayState.Following),
    )


def rgb(color):
    value = color.lstrip("#")
    if len(value) == 3:
        value = "".join(char * 2 for char in value)
    return [int(value[index:index + 2], 16) for index in (0, 2, 4)]


class FlightRecording:
    def __init__(self, output_dir, viewer_sink, sample, force_scale):
        self.key = (sample["session"], sample["generation"])
        self.force_scale = force_scale
        self.paths = set()
        self.styles = {}
        self.last_step = None
        self.last_time = None
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        self.path = output_dir / f"hangprinter-{stamp}-{uuid.uuid4().hex[:8]}.rrd"
        self.manifest = {"backend": "browser-js", "browser_session": sample["session"],
                         "scene_generation": sample["generation"], "recording_id": uuid.uuid4().hex,
                         "rrd": str(self.path.resolve()), "status": "recording", "start_step": sample["step"]}
        self.stream = rr.RecordingStream("hp-sim5 Hangprinter flight recorder", recording_id=self.manifest["recording_id"])
        self.write_manifest()
        sinks = [rr.FileSink(self.path)]
        if viewer_sink is not None:
            sinks.append(viewer_sink)
        self.stream.set_sinks(*sinks, default_blueprint=blueprint())
        self.stream.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        self.stream.log(
            "recording_info",
            rr.TextDocument(
                "Lengths are in metres, forces in newtons, quaternions in XYZW order.\n"
                "actual = paid-out rest length, including intermediate guide wraps.\n"
                "commanded = paid-out length predicted from motor targets and the solver winding model.\n"
                "geometric = straight attachment spans plus guide wraps.\n"
                "error = actual - commanded; stretch = geometric - actual (negative indicates slack).\n"
                "Cable force includes force transferred through guides; arrows show force on endpoint A.\n"
                f"Force arrows use {force_scale:g} metres per newton.\n"
                "Torque-controlled cables have no commanded-length series.\n"
                f"Browser session: {sample['session']}; scene generation: {sample['generation']}."
            ),
            static=True,
        )
        print(f"Recording: {self.path}", flush=True)

    def write_manifest(self):
        target = self.path.with_suffix('.json')
        temporary = target.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(self.manifest, indent=2) + '\n')
        temporary.replace(target)

    def log_sample(self, sample):
        step, time = sample["step"], sample["time"]
        if self.last_step is not None and step != self.last_step + 1:
            raise ValueError(f"Nonconsecutive timestep: expected {self.last_step + 1}, received {step}")
        if self.last_time is not None and not np.isclose(time - self.last_time, sample["dt"], rtol=1e-8, atol=1e-10):
            raise ValueError("Simulation clock did not advance by dt")
        self.stream.set_time("sim_step", sequence=step)
        self.stream.set_time("sim_time", duration=time)
        current_paths = set()

        def log(path, *archetypes):
            current_paths.add(path)
            self.stream.log(path, *archetypes, strict=True)

        def series(path, names, values, colors=None):
            if self.styles.get(path) != names:
                log(path, rr.SeriesLines(names=names, colors=colors))
                self.styles[path] = names
            log(path, rr.Scalars(values))

        for frame in sample["frames"]:
            path = frame["path"]
            if path not in self.paths:
                kind = frame["kind"]
                color = {"anchor": [180, 180, 180], "effector": [255, 130, 40]}.get(kind, [90, 180, 230])
                log(
                    path,
                    rr.Points3D([[0, 0, 0]], colors=color, radii=0.02 if kind == "anchor" else 0.008),
                    rr.TransformAxes3D(0.15 if kind == "effector" else 0.025),
                )
            log(path, rr.Transform3D(translation=frame["position"], rotation=rr.Quaternion(xyzw=frame["quaternion"])))

        for cable in sample["cables"]:
            key = f"{cable['machine']}/{cable['name']}"
            visual = f"world/machines/{cable['machine']}/cables/{cable['name']}"
            segments = cable["segments"]
            color = rgb(cable["color"])
            if segments:
                log(
                    f"{visual}/segments",
                    rr.LineStrips3D([segment["points"] for segment in segments], colors=color, radii=rr.Radius.ui_points(1.5)),
                )
                log(
                    f"{visual}/forces",
                    rr.Arrows3D(
                        origins=[segment["origin"] for segment in segments],
                        vectors=np.asarray([segment["force_vector_n"] for segment in segments]) * self.force_scale,
                        colors=[255, 90, 90], radii=0.002,
                    ),
                )
            if cable["wraps"]:
                log(f"{visual}/wraps", rr.LineStrips3D(cable["wraps"], colors=color, radii=rr.Radius.ui_points(1.5)))

            lengths = cable["lengths"]
            length_names = [name for name in ("commanded", "actual", "geometric") if lengths[name] is not None]
            palette = {"commanded": [255, 180, 40], "actual": [70, 180, 255], "geometric": [100, 220, 130]}
            series(
                f"line_lengths/{key}", [f"{key}: {name}" for name in length_names],
                [lengths[name] for name in length_names], [palette[name] for name in length_names],
            )
            error_names = [name for name in ("error", "stretch") if lengths[name] is not None]
            series(f"line_errors/{key}", [f"{key}: {name}" for name in error_names], [lengths[name] for name in error_names])
            if segments:
                series(
                    f"cable_forces/{key}", [f"{key}: {segment['name']}" for segment in segments],
                    [segment["force_n"] for segment in segments],
                )

        # Merging/splitting segments and removing machines must remove old geometry at this time.
        for path in self.paths - current_paths:
            self.stream.log(path, rr.Clear(recursive=True))
            self.styles.pop(path, None)
        self.paths = current_paths
        self.last_step, self.last_time = step, time

    def close(self):
        try:
            self.stream.flush(timeout_sec=5)
        except RuntimeError as error:
            print(f"Recorder flush: {error}", flush=True)
        finally:
            self.stream.disconnect()
        self.manifest.update(status="finalized", end_step=self.last_step, end_time_s=self.last_time)
        self.write_manifest()
        print(f"Saved: {self.path} (last timestep {self.last_step})", flush=True)


async def run(args):
    output_dir = args.output.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    viewer_sink = None
    viewer_server = None
    if args.viewer_endpoint:
        viewer_sink = rr.GrpcSink(args.viewer_endpoint.replace('http://', 'rerun+http://') + '/proxy')
    elif not args.no_viewer:
        viewer_uri = f"rerun+http://127.0.0.1:{args.grpc_port}/proxy"
        # Keep the server alive across recordings. A GrpcServerSink configuration
        # on each recording would start a separate listener on the same port.
        viewer_server = rr.RecordingStream("hp-sim5 flight recorder server", send_properties=False)
        viewer_server.set_sinks(rr.GrpcServerSink(bind_ip="127.0.0.1", port=args.grpc_port))
        viewer_sink = rr.GrpcSink(viewer_uri)
        rr.serve_web_viewer(
            web_port=args.web_port, open_browser=False,
            connect_to=viewer_uri,
        )
        print(f"Rerun viewer: http://localhost:{args.web_port}/?{urlencode({'url': viewer_uri})}", flush=True)

    async def receive(socket):
        recording = None
        try:
            async for message in socket:
                sample = json.loads(message)
                if sample.get("version") != 1:
                    raise ValueError("Unsupported flight recorder protocol version")
                key = (sample["session"], sample["generation"])
                if recording is None or recording.key != key:
                    if recording is not None:
                        recording.close()
                    recording = FlightRecording(output_dir, viewer_sink, sample, args.force_scale)
                recording.log_sample(sample)
                # Acknowledge only after logging: the browser bounds its queue and waits here.
                await socket.send(json.dumps({"type": "ack", "step": sample["step"]}))
        except ConnectionClosed:
            pass
        except (ValueError, KeyError, TypeError) as error:
            print(f"Recorder rejected a sample: {error}", flush=True)
            await socket.close(code=1008, reason=str(error)[:100])
        finally:
            if recording is not None:
                recording.close()

    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)
    try:
        async with serve(receive, "127.0.0.1", args.port, max_size=16 * 1024 * 1024, compression=None):
            print(f"Flight recorder: ws://127.0.0.1:{args.port}", flush=True)
            print("Click Rerun in the simulator controls, then start playback. Ctrl-C stops recording.", flush=True)
            await stop.wait()
    finally:
        if viewer_server is not None:
            viewer_server.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("output/rerun"), help="Directory for per-scene RRD recordings")
    parser.add_argument("--port", type=int, default=9877, help="Browser telemetry WebSocket port")
    parser.add_argument("--web-port", type=int, default=9090, help="Rerun web viewer port")
    parser.add_argument("--grpc-port", type=int, default=9876, help="Rerun gRPC port")
    parser.add_argument("--force-scale", type=float, default=0.01, help="3D force arrow length in metres per newton")
    parser.add_argument("--viewer-endpoint", help="Use an existing supervisor-owned native Viewer")
    parser.add_argument("--no-viewer", action="store_true", help="Do not start a separate Viewer; retain disk and any supplied Viewer endpoint")
    args = parser.parse_args()
    try:
        asyncio.run(run(args))
    except (OSError, RuntimeError) as error:
        parser.exit(1, f"Flight recorder: {error}\n")


if __name__ == "__main__":
    main()
