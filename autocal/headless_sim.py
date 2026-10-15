"""Own the headless physics/bridge and simulated RRF for one full-auto run."""
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import socket
import subprocess
import time
import signal
import sys
from urllib.request import Request, urlopen

from autocal._autocal_common import (
    NORMAL_STOP_REASONS,
    REPO_ROOT,
    RRF_SIM_BINARY,
    RRF_SIM_VSD_PATH,
    _stop_process,
)


MACHINES = {
    "hp3": ("hp3_rigid_body.usda", "hp3"),
    "hangprinter_3": ("hp3_rigid_body.usda", "hp3"),
    "hp4": ("hp4_rigid_body.usda", "hp4"),
    "hangprinter_4": ("hp4_rigid_body.usda", "hp4"),
    "slideprinter": ("slideprinter_rigid_body.usda", "slideprinter"),
    "cubecorners": ("cubecorners_rigid_body.usda", "cubecorners"),
    "skycam": ("four_high_anchors_rigid_body.usda", "skycam"),
}


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def source_hash(directories=("src/js", "hp-sim-3d/app", "src/wasm/cable_solver/src", "src/wasm/cable_solver/native", "integrations", "autocal/control", "scripts"), suffixes=(".js", ".mjs", ".rs", ".c")):
    digest = hashlib.sha256()
    for directory in directories:
        for path in sorted((REPO_ROOT / directory).rglob("*")):
            if path.suffix in suffixes:
                digest.update(str(path.relative_to(REPO_ROOT)).encode() + b"\0" + path.read_bytes())
    return digest.hexdigest()


class HeadlessSimulation:
    def __init__(self, args, collector_args):
        scene, machine = MACHINES[args.machine_type]
        search = args.find_radii != "off" or args.find_buildup_factor != "off"
        config = args.rrf_config or args.config or os.environ.get("AUTOCAL_RRF_SIM_CONFIG")
        self.config = config or f"sys/config_{machine}{'_w_line_layers' if search else ''}.g"
        self.scene = REPO_ROOT / "public/usd_scenes" / scene
        self.args, self.collector_args = args, collector_args
        self.processes = []
        self.physics = None
        self.recorder = None
        self.reference_url = os.environ.get('AUTOCAL_REFERENCE_WS')
        self.recording_mode = getattr(args, 'recording_mode', None)
        if self.recording_mode is None:
            self.recording_mode = 'full' if getattr(args, 'extended_reference', False) or self.reference_url else 'none'
        self.signal_handlers = {}
        self.started = time.monotonic()
        self.directory = Path(args.dataset).resolve().with_suffix(".headless")
        # Retain prior manifests/partial points when an existing dataset is continued.
        index = 1
        while self.directory.exists():
            self.directory = Path(args.dataset).resolve().with_suffix(f".headless_{index}")
            index += 1
        self.directory.mkdir(parents=True)
        self.manifest = {"backend": "headless-js", "status": "starting", "machine_type": args.machine_type,
                         "scene": str(self.scene), "firmware_config": self.config, "recording_mode": self.recording_mode,
                         "clock": "fixed steps; collector waits advance simulation time",
                         "dataset": str(Path(args.dataset).resolve()), "source_sha256": source_hash()}
        self.manifest["autocal_source_sha256"] = source_hash(("autocal",), (".py",))
        self.manifest["git_revision"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True).strip()

    def write_manifest(self):
        temporary = self.directory / 'manifest.tmp'
        temporary.write_text(json.dumps(self.manifest, indent=2) + "\n")
        temporary.replace(self.directory / 'manifest.json')

    def spawn(self, name, command):
        with (self.directory / f"{name}.log").open("a") as log:
            process = subprocess.Popen(command, cwd=REPO_ROOT, stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=log,
                                       start_new_session=not bool(os.environ.get('HP_SIM5_AUTOCAL_JOB')))
        self.processes.append(process)
        if os.environ.get('HP_SIM5_TEST_PROCESS_REGISTRY'):
            sys.path.insert(0, str(REPO_ROOT / 'src/python'))
            from hp_sim5_research.services import register_test_process
            register_test_process(process)
        self.track_process(name, process)
        return process

    def track_process(self, name, process):
        self.manifest.setdefault('owned_services', []).append({'name': name, 'pid': process.pid})
        self.write_manifest()

    def request(self, operation, **args):
        request = Request(f"{self.url}/{operation}", data=json.dumps(args).encode(),
                          headers={"Content-Type": "application/json"})
        with urlopen(request, timeout=180) as response:
            return json.load(response)

    def __enter__(self):
        try:
            for sig in (signal.SIGTERM, signal.SIGHUP):
                self.signal_handlers[sig] = signal.getsignal(sig)
                signal.signal(sig, self.interrupt)
            if self.recording_mode != 'none' and not self.reference_url:
                from websockets.sync.client import connect
                port = free_port()
                self.recorder = self.spawn('recorder', [sys.executable, 'scripts/hangprinter_flight_recorder.py',
                    '--extended-reference', '--no-viewer', '--port', str(port), '--output', str(self.directory),
                    '--sample-stride', '100' if self.recording_mode == 'sampled' else '10', '--geometry-detail', 'compact'])
                self.reference_url = f'ws://127.0.0.1:{port}'
                deadline = time.monotonic() + 30
                while True:
                    try:
                        with connect(self.reference_url, open_timeout=1, close_timeout=1):
                            break
                    except OSError:
                        if self.recorder.poll() is not None or time.monotonic() > deadline:
                            raise RuntimeError(f'Reference recorder did not start; see {self.directory / "recorder.log"}') from None
                        time.sleep(.1)
                os.environ['AUTOCAL_REFERENCE_WS'] = self.reference_url
            if self.reference_url:
                self.manifest.update(extended_reference_ws=self.reference_url, owned_recorder=self.recorder is not None)
            from autocal._autocal_common import _arg_has_flag, _arg_value, _resolve_rrf_target, _wait_for_rrf_server, _send_rrf_gcode
            rrf_url, explicit, port = _resolve_rrf_target(self.collector_args)
            if not explicit:
                port = port if _arg_value(self.collector_args, "--port") else free_port()
                rrf_url = f"http://127.0.0.1:{port}"
            cfg_path = REPO_ROOT / RRF_SIM_VSD_PATH / self.config
            for label, path in (("scene", self.scene), ("firmware_config", cfg_path), ("rrf", RRF_SIM_BINARY)):
                self.manifest[f"{label}_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
            shutil.copyfile(self.scene, self.directory / "scene.usda")
            shutil.copyfile(cfg_path, self.directory / "firmware-config.g")
            owns_rrf = not explicit and not _arg_has_flag(self.collector_args, "--no-spawn-rrf-simulator")
            if owns_rrf:
                self.spawn("rrf", [str(RRF_SIM_BINARY), "--vsd", RRF_SIM_VSD_PATH, "-c", self.config,
                                   "--server", "-p", str(port)])
            _wait_for_rrf_server(rrf_url)
            buildup = getattr(self.args, 'buildup_factor', None)
            if buildup is None:
                initial_q = re.search(r'\bQ([-+\d.eE]+)', _send_rrf_gcode(rrf_url, 'M666'))
                buildup = float(initial_q[1]) if initial_q else 0
            layering = _arg_has_flag(self.collector_args, '--line-layering') or buildup != 0
            ws_port, api_port = free_port(), free_port()
            self.url = f"http://127.0.0.1:{api_port}"
            service = self.physics = self.spawn("physics", ["node", "scripts/autocal_headless.mjs", str(self.directory / "scene.usda"),
                                            rrf_url, str(ws_port), str(api_port), str(self.directory), str(layering).lower(),
                                            self.recording_mode])
            deadline = time.monotonic() + 30
            while True:
                try:
                    status = self.request("status")
                    self.manifest.update(physics_engine=status['physics_engine'], physics_artifact=status['physics_artifact'])
                    break
                except OSError:
                    if service.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError(f"Headless service did not start; see {self.directory / 'physics.log'}") from None
                    time.sleep(.1)
            self.collector_args.extend(["--server", rrf_url, "--headless-url", self.url, "--no-spawn-rrf-simulator"])
            self.manifest["baked_scene_sha256"] = hashlib.sha256((self.directory / "baked-scene.usda").read_bytes()).hexdigest()
            self.manifest['initial_baked_scene_sha256'] = self.manifest['baked_scene_sha256']
            self.manifest.update(status="running", service_url=self.url, rrf_url=rrf_url,
                                 services=[process.pid for process in self.processes],
                                 owned_rrf=owns_rrf,
                                 firmware_initial={command: self.request("gcode", line=command)["result"]["reply"]
                                                   for command in ("M115", "M669", "M666")})
            self.write_manifest()
            if self.reference_url:
                from autocal import extended_reference
                extended_reference.emit('headless_start', **self.manifest)
                for name in ('scene.usda', 'baked-scene.usda', 'firmware-config.g'):
                    extended_reference.artifact(self.directory / name)
            return self
        except BaseException as error:
            self.__exit__(type(error), error, error.__traceback__)
            raise

    def __exit__(self, error_type, error, traceback):
        failed = bool(error or self.manifest.get("exit_code") or
                      (self.physics is not None and self.physics.poll() is not None))
        capture_error = None
        interrupted = error_type is KeyboardInterrupt or (error_type is SystemExit and error.code == 130)
        self.manifest.update(status="interrupted" if interrupted else "failed" if error else "complete",
                             wall_s=time.monotonic() - self.started)
        if error:
            self.manifest["error"] = str(error)
        if not self.args.keep_sim_alive or failed:
            for process in reversed(self.processes):
                if self.reference_url and process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=45 if process is self.recorder else 15)
                    except subprocess.TimeoutExpired:
                        capture_error = 'Reference services did not drain before shutdown'
                        process.kill()
                        process.wait()
                else:
                    _stop_process(process)
        self.manifest["kept_alive"] = bool(self.args.keep_sim_alive and not failed)
        clock = self.directory / "clock.json"
        if clock.exists():
            state = json.loads(clock.read_text())
            self.manifest["service_wall_s"] = state.pop("wall_s")
            if not state.get("error"):
                state.pop("error", None)
            elif self.reference_url:
                capture_error = state['error']
            self.manifest.update(state)
        if self.physics is not None and not self.manifest["kept_alive"]:
            self.manifest["physics_returncode"] = self.physics.poll()
            if self.reference_url:
                if self.physics.returncode != 0:
                    capture_error = capture_error or f'Headless physics exited with status {self.physics.returncode}; see physics.log'
                if not self.manifest.get("recording_drained"):
                    capture_error = capture_error or 'Headless physics did not confirm recording drain; see physics.log'
        if self.manifest.get("exit_code"):
            self.manifest["status"] = "failed"
        self.manifest["normal_completion"] = (
            self.manifest["status"] == "complete"
            and self.manifest.get("stop_reason") in NORMAL_STOP_REASONS
            and bool(self.manifest.get("applied_parameters"))
        )
        if self.manifest.get("stop_reason") == "Ctrl-C":
            self.manifest["status"] = "interrupted"
        if self.reference_url:
            from autocal import extended_reference
            extended_reference.close()
            if self.recorder:
                os.environ.pop('AUTOCAL_REFERENCE_WS', None)
                self.manifest['recordings'] = [json.loads(path.read_text()) for path in self.directory.glob('hangprinter-*.json')]
                if not self.manifest['kept_alive'] and (self.recorder.returncode != 0 or
                        any(recording['status'] != 'finalized' for recording in self.manifest['recordings'])):
                    capture_error = 'Reference recorder did not finalize; see recorder.log'
        if capture_error:
            self.manifest.update(status='failed' if not error else self.manifest['status'], recording_error=capture_error,
                                 normal_completion=False)
        for sig, handler in self.signal_handlers.items():
            signal.signal(sig, handler)
        self.write_manifest()
        if capture_error and not error:
            raise RuntimeError(capture_error)

    def interrupt(self, signum, frame):
        if os.environ.get('HP_SIM5_AUTOCAL_JOB'):
            # Interactive Ctrl-C accepts best-so-far. Supervisor cancellation must
            # bypass that branch and stop movement without applying a candidate.
            self.manifest['stop_reason'] = 'cancelled'
            raise SystemExit(130)
        raise KeyboardInterrupt(f"Signal {signum}")

    def apply_parameters(self, commands):
        for command in commands:
            if command:
                self.request("gcode", line=command)
        self.manifest["applied_parameters"] = [command for command in commands if command]
        self.manifest["firmware_final"] = {command: self.request("gcode", line=command)["result"]["reply"]
                                           for command in ("M669", "M666")}
