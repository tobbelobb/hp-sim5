#!/usr/bin/env python3
"""Record hp-sim-3d timesteps in Rerun, with a live viewer and durable RRD files."""

import argparse
import asyncio
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import sys
import time as wall_clock
import subprocess
import uuid
from urllib.parse import urlencode

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed


def blueprint(extended=False):
    metrics = [
        rrb.TimeSeriesView(origin="/line_lengths", name="Line lengths (m)"),
        rrb.TimeSeriesView(origin="/line_errors", name="Length error / stretch (m)"),
        rrb.TimeSeriesView(origin="/cable_forces", name="Cable forces (N)"),
    ]
    details = rrb.Vertical(*metrics)
    if extended:
        details = rrb.Vertical(
            rrb.TextLogView(origin="/autocal", name="Autocal events"),
            rrb.Tabs(rrb.TimeSeriesView(origin="/clocks", name="Simulation clocks (s)"), *metrics),
        )
    return rrb.Blueprint(
        rrb.Horizontal(rrb.Spatial3DView(origin="/world", name="Hangprinter"), details, column_shares=[2, 1]),
        rrb.TimePanel(timeline="wall_time" if extended else "sim_time", play_state=rrb.components.PlayState.Following),
    )


def rgb(color):
    value = color.lstrip("#")
    if len(value) == 3:
        value = "".join(char * 2 for char in value)
    return [int(value[index:index + 2], 16) for index in (0, 2, 4)]


class FlightRecording:
    def __init__(self, output_dir, viewer_sink, sample, force_scale, extended=False):
        self.extended = extended
        self.event_count = 0
        self.physics_samples = 0
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
        self.stream.set_sinks(*sinks, default_blueprint=blueprint(extended))
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
                + ("Shared reference: browser sessions and generations are recorded with samples."
                 if extended else f"Browser session: {sample['session']}; scene generation: {sample['generation']}.")
            ),
            static=True,
        )
        if extended:
            self.manifest.update(format_version=1, extended_reference=True, autocal_complete=False,
                                 browser_session=None, scene_generation=None, browser_segments=[], rejected_messages=[])
            root = Path(__file__).resolve().parents[1]
            for label, command in (("revision", ["git", "rev-parse", "HEAD"]),
                                   ("working_diff", ["git", "diff", "HEAD"]),
                                   ("untracked_files", ["git", "ls-files", "--others", "--exclude-standard"])):
                result = subprocess.run(command, cwd=root, capture_output=True, text=True, check=True)
                self.stream.log(f"provenance/{label}", rr.TextDocument(result.stdout), static=True)
                if label == "untracked_files":
                    for name in result.stdout.splitlines():
                        path = root / name
                        if path.suffix in (".py", ".js", ".mjs", ".g", ".usda"):
                            self.stream.log(f"provenance/untracked/{name}", rr.TextDocument(path.read_text()), static=True)
            self.stream.log("provenance/runtime", rr.TextDocument(
                json.dumps({"python": sys.version, "rerun": rr.__version__})), static=True)
        self.write_manifest()
        print(f"Recording: {self.path}", flush=True)

    def write_manifest(self):
        target = self.path.with_suffix('.json')
        temporary = target.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(self.manifest, indent=2) + '\n')
        temporary.replace(target)

    def log_event(self, event):
        if not self.extended:
            raise ValueError("Events require --extended-reference")
        source = event["source"]
        if source not in ("python", "collector", "browser"):
            raise ValueError("Unknown event source")
        self.stream.reset_time()
        wall_ms = int(event["wall_time_ms"])
        self.stream.set_time("wall_time", timestamp=np.datetime64(wall_ms, "ms"))
        self.event_count += 1
        if source == "python" and event.get("kind") == "run_start":
            self.manifest["autocal_complete"] = False
        if source == "python" and event.get("kind") == "run_complete" and event.get("payload", {}).get("returncode") == 0:
            self.manifest["autocal_complete"] = True
        self.stream.set_time("event_order", sequence=self.event_count)
        payload = event.get("payload", {})
        kind = event.get("kind", "event")
        if kind == "text_log":
            body = payload["text"]
        elif kind == "artifact":
            body = f"artifact: {payload['path']} ({len(payload['content'])} characters)"
        else:
            body = f"{kind}: {json.dumps(payload, ensure_ascii=False)}"
        # Never assign a guessed physics time to an event from another clock.
        self.stream.log(f"autocal/{source}", rr.TextLog(body),
                        rr.AnyValues(event_json=json.dumps(event), received_wall_time_ms=wall_clock.time_ns() // 1_000_000), strict=True)
        if event.get("sim_time_s") is not None:
            clock_path = "clocks/browser/research_clock" if source == "browser" else f"clocks/{source}"
            self.stream.log(clock_path, rr.Scalars(event["sim_time_s"]),
                            rr.AnyValues(sim_time_source=event["sim_time_source"]), strict=True)

    def log_sample(self, sample):
        if self.extended and self.key != (sample["session"], sample["generation"]):
            self.key = (sample["session"], sample["generation"])
            self.last_step = self.last_time = None
            for path in self.paths:
                self.stream.reset_time()
                self.stream.set_time("wall_time", timestamp=np.datetime64(int(sample["wall_time_ms"]), "ms"))
                self.stream.log(path, rr.Clear(recursive=True))
            self.paths.clear()
            self.styles.clear()
        step, time = sample["step"], sample["time"]
        if self.last_step is not None and step != self.last_step + 1:
            raise ValueError(f"Nonconsecutive timestep: expected {self.last_step + 1}, received {step}")
        if self.last_time is not None and not np.isclose(time - self.last_time, sample["dt"], rtol=1e-8, atol=1e-10):
            raise ValueError("Simulation clock did not advance by dt")
        self.stream.reset_time()
        self.stream.set_time("wall_time", timestamp=np.datetime64(int(sample.get("wall_time_ms", wall_clock.time_ns() // 1_000_000)), "ms"))
        self.stream.set_time("scene_generation", sequence=sample["generation"])
        self.stream.set_time("sim_step", sequence=step)
        self.stream.set_time("sim_time", duration=time)
        self.stream.log("clocks/browser/flight_recorder", rr.Scalars(time), rr.AnyValues(
            sim_time_source="browser.flightRecorder", session=sample["session"],
            speed_scale=sample.get("speed_scale", 1),
            wall_time_source="browser.Date.now" if "wall_time_ms" in sample else "recorder.receive"))
        research_clock = sample.get("research_clock")
        if research_clock is not None:
            self.stream.log("clocks/browser/research_clock", rr.Scalars(research_clock["time"]),
                            rr.AnyValues(sim_time_source="browser.researchClock",
                                         observation_phase="world.update before runner advances researchClock"))
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
        self.physics_samples += 1
        if self.extended:
            segments = self.manifest["browser_segments"]
            if not segments or (segments[-1]["session"], segments[-1]["generation"]) != self.key:
                segments.append(dict(session=sample["session"], generation=sample["generation"],
                                     start_step=step, start_time_s=time, start_wall_time_ms=sample["wall_time_ms"]))
            segments[-1].update(end_step=step, end_time_s=time, end_wall_time_ms=sample["wall_time_ms"])
            self.manifest.update(browser_session=sample["session"], scene_generation=sample["generation"])

    def close(self):
        self.manifest.update(status="finalized", end_step=self.last_step, end_time_s=self.last_time,
                             physics_samples=self.physics_samples, event_count=self.event_count)
        self.stream.log("provenance/manifest", rr.TextDocument(json.dumps(self.manifest, indent=2)), static=True)
        try:
            self.stream.flush(timeout_sec=30)
        except RuntimeError as error:
            self.manifest["status"] = "flush_failed"
            print(f"Recorder flush: {error}", flush=True)
        finally:
            self.stream.disconnect()
        self.write_manifest()
        print(f"Recording {self.manifest['status']}: {self.path} (last timestep {self.last_step})", flush=True)


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

    extended_recording = None
    if args.extended_reference:
        extended_recording = FlightRecording(output_dir, viewer_sink,
            {"session": "extended-reference", "generation": 0, "step": 0}, args.force_scale, extended=True)

    active_browser = None

    async def receive(socket):
        nonlocal active_browser
        recording = None
        try:
            async for message in socket:
                sample = json.loads(message)
                if sample.get("version") != 1:
                    raise ValueError("Unsupported flight recorder protocol version")
                if sample.get("type") == "autocal_event":
                    if extended_recording is None:
                        raise ValueError("Events require --extended-reference")
                    if sample.get("kind") in ("run_start", "collection_start", "gcode_send") and active_browser is None:
                        raise ValueError("Connect the visual browser recorder before starting autocal")
                    extended_recording.log_event(sample)
                    await socket.send(json.dumps({"type": "event_ack"}))
                    continue
                if extended_recording is not None:
                    if active_browser is not None and active_browser is not socket:
                        raise ValueError("Extended reference already has an active browser")
                    active_browser = socket
                    if "wall_time_ms" not in sample:
                        raise ValueError("Extended recording requires browser wall_time_ms")
                    extended_recording.log_sample(sample)
                    await socket.send(json.dumps({"type": "ack", "step": sample["step"]}))
                    continue
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
            if extended_recording is not None:
                extended_recording.manifest["rejected_messages"].append(str(error))
                extended_recording.write_manifest()
            print(f"Recorder rejected a sample: {error}", flush=True)
            await socket.close(code=1008, reason=str(error)[:100])
        finally:
            if active_browser is socket:
                active_browser = None
            if recording is not None:
                recording.close()

    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)
    try:
        async with serve(receive, "127.0.0.1", args.port, max_size=None if args.extended_reference else 16 * 1024 * 1024, compression=None):
            print(f"Flight recorder: ws://127.0.0.1:{args.port}", flush=True)
            print("Click Rerun in the simulator controls, then start playback. Ctrl-C stops recording.", flush=True)
            await stop.wait()
    finally:
        if extended_recording is not None:
            extended_recording.close()
        if viewer_server is not None:
            viewer_server.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--extended-reference", action="store_true", help="One wall-clock RRD for browser physics and autocal events; stop after autocal exits")
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
