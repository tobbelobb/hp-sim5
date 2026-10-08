"""Own the headless physics/bridge and simulated RRF for one full-auto run."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import time
import signal
from urllib.request import Request, urlopen

from autocal._autocal_common import REPO_ROOT, RRF_SIM_BINARY, RRF_SIM_VSD_PATH, _stop_process


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


def source_hash(directories=("src/js", "hp-sim-3d/app", "integrations", "autocal/control", "scripts"), suffixes=(".js", ".mjs")):
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
                         "scene": str(self.scene), "firmware_config": self.config,
                         "clock": "fixed steps; collector waits advance simulation time",
                         "dataset": str(Path(args.dataset).resolve()), "source_sha256": source_hash()}
        self.manifest["autocal_source_sha256"] = source_hash(("autocal",), (".py",))
        self.manifest["git_revision"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True).strip()

    def write_manifest(self):
        (self.directory / "manifest.json").write_text(json.dumps(self.manifest, indent=2) + "\n")

    def spawn(self, name, command):
        with (self.directory / f"{name}.log").open("a") as log:
            process = subprocess.Popen(command, cwd=REPO_ROOT, stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=log, start_new_session=True)
        self.processes.append(process)
        return process

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
            from autocal._autocal_common import _arg_has_flag, _arg_value, _resolve_rrf_target, _wait_for_rrf_server
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
            ws_port, api_port = free_port(), free_port()
            self.url = f"http://127.0.0.1:{api_port}"
            service = self.spawn("physics", ["node", "scripts/autocal_headless.mjs", str(self.directory / "scene.usda"),
                                            rrf_url, str(ws_port), str(api_port), str(self.directory)])
            deadline = time.monotonic() + 30
            while True:
                try:
                    self.request("status")
                    break
                except OSError:
                    if service.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError(f"Headless service did not start; see {self.directory / 'physics.log'}") from None
                    time.sleep(.1)
            self.collector_args.extend(["--server", rrf_url, "--headless-url", self.url, "--no-spawn-rrf-simulator"])
            self.manifest["baked_scene_sha256"] = hashlib.sha256((self.directory / "baked-scene.usda").read_bytes()).hexdigest()
            self.manifest.update(status="running", service_url=self.url, rrf_url=rrf_url,
                                 services=[process.pid for process in self.processes],
                                 owned_rrf=owns_rrf,
                                 firmware_initial={command: self.request("gcode", line=command)["result"]["reply"]
                                                   for command in ("M115", "M669", "M666")})
            self.write_manifest()
            return self
        except BaseException as error:
            self.__exit__(type(error), error, error.__traceback__)
            raise

    def __exit__(self, error_type, error, traceback):
        failed = bool(error or self.manifest.get("exit_code"))
        self.manifest.update(status="interrupted" if error_type is KeyboardInterrupt else "failed" if error else "complete",
                             wall_s=time.monotonic() - self.started)
        if error:
            self.manifest["error"] = str(error)
        if not self.args.keep_sim_alive or failed:
            for process in reversed(self.processes):
                _stop_process(process)
        self.manifest["kept_alive"] = bool(self.args.keep_sim_alive and not failed)
        clock = self.directory / "clock.json"
        if clock.exists():
            state = json.loads(clock.read_text())
            self.manifest["service_wall_s"] = state.pop("wall_s")
            if not state.get("error"):
                state.pop("error", None)
            self.manifest.update(state)
        if self.manifest.get("exit_code"):
            self.manifest["status"] = "failed"
        self.manifest["normal_completion"] = self.manifest["status"] == "complete" and self.manifest.get("stop_reason") == "patience-or-threshold" and bool(self.manifest.get("applied_parameters"))
        if self.manifest.get("stop_reason") == "Ctrl-C":
            self.manifest["status"] = "interrupted"
        for sig, handler in self.signal_handlers.items():
            signal.signal(sig, handler)
        self.write_manifest()

    @staticmethod
    def interrupt(signum, frame):
        raise KeyboardInterrupt(f"Signal {signum}")

    def apply_parameters(self, commands):
        for command in commands:
            if command:
                self.request("gcode", line=command)
        self.manifest["applied_parameters"] = [command for command in commands if command]
        self.manifest["firmware_final"] = {command: self.request("gcode", line=command)["result"]["reply"]
                                           for command in ("M669", "M666")}
