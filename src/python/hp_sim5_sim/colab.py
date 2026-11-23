"""Google Colab compute backend using the official `google-colab-cli`.

Runs entire headless experiments (RRF firmware + physics + controller + optimizer)
remotely on Colab VMs, including parallel CPU runs across all VM cores, with zero
per-G-code internet traffic.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import textwrap
from typing import Any, Sequence
import uuid

from .interfaces import ArtifactStore, ExperimentResult, ExperimentSpec
from .storage import ScratchWorkspace


BUNDLE_TOP_LEVEL_PATHS = (
    "package.json",
    "machines",
    "autocal",
    "hp-sim-3d",
    "integrations",
    "public/usd_scenes",
    "public/wasm",
    "scripts",
    "src/js",
    "src/python",
    "src/wasm/cable_solver",
    "RRF/CMakeLists.txt",
    "RRF/host",
    "RRF/run/vsd/sys",
    "RRF/ReprapFirmware/src",
    "RRF/RRFLibraries/src",
    "RRF/CANlib/src",
)

BUNDLE_EXCLUDE_PARTS = frozenset({
    ".git",
    "__pycache__",
    ".pytest_cache",
    "node_modules",
    ".venv",
    "target",
})

BUNDLE_EXCLUDE_PREFIXES = (
    "autocal/data/references/",
    "autocal/auto-calibration-simulation-for-hangprinter/synthetic_datasets/",
    "autocal/tests/",
    "autocal/control/tests/",
)


def resolve_colab_cli(explicit: str | Path | None = None) -> list[str]:
    """Return the argv prefix to invoke google-colab-cli with ADC auth.

    Checks `explicit`, `HP_SIM5_COLAB_BIN`, `~/bin/colab` (which wraps `--auth=adc`
    even in non-login shells where `~/bin` is not in PATH), and finally `colab` on PATH.
    """
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    elif os.environ.get("HP_SIM5_COLAB_BIN"):
        candidates.append(Path(os.environ["HP_SIM5_COLAB_BIN"]).expanduser())
    home_wrapper = Path.home() / "bin/colab"
    if home_wrapper.is_file() and os.access(home_wrapper, os.X_OK):
        candidates.append(home_wrapper)
    on_path = shutil.which("colab")
    if on_path:
        candidates.append(Path(on_path))

    for binary in candidates:
        if binary.is_file() and os.access(binary, os.X_OK):
            # ~/bin/colab already injects --auth=adc; avoid duplicating it.
            try:
                head = binary.read_bytes()[:256]
                if b"--auth=adc" in head:
                    return [str(binary)]
            except OSError:
                pass
            return [str(binary), "--auth=adc"]

    raise FileNotFoundError(
        "google-colab-cli (`colab`) not found. Install via `uv tool install google-colab-cli` "
        "or set HP_SIM5_COLAB_BIN."
    )


def _portable_native_cache(repo_root: Path) -> Path:
    return repo_root / "src/wasm/cable_solver/target/native-portable/physics.node"


def _native_sources_fresh(repo_root: Path, artifact: Path) -> bool:
    meta_path = Path(f"{artifact}.json")
    if not artifact.is_file() or not meta_path.is_file():
        return False
    try:
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
        if meta.get("artifactSha256") != sha(artifact):
            return False
        crate = repo_root / "src/wasm/cable_solver"
        for rel, expected in meta.get("sources", {}).items():
            if sha(crate / rel) != expected:
                return False
        if meta.get("buildScriptSha256") != sha(repo_root / "scripts/build_native_physics.mjs"):
            return False
        return True
    except (OSError, ValueError):
        return False


def ensure_portable_native_physics(repo_root: Path) -> Path | None:
    """Build or reuse a portable x86-64 `physics.node` (GLIBC <= 2.39) for Linux Colab VMs."""
    repo_root = Path(repo_root).resolve()
    artifact = _portable_native_cache(repo_root)
    if _native_sources_fresh(repo_root, artifact):
        return artifact
    if not shutil.which("cargo") or not shutil.which("node"):
        return None
    env = dict(os.environ)
    env["RUSTFLAGS"] = "-C target-cpu=x86-64"
    env.pop("CARGO_ENCODED_RUSTFLAGS", None)
    result = subprocess.run(
        ["node", "scripts/build_native_physics.mjs", "--no-pgo", "--output", str(artifact)],
        cwd=repo_root,
        env=env,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Portable native physics build failed:\n{result.stderr[-1500:]}")
    return artifact


def bundle_workspace(
    repo_root: Path | str,
    dest_tar: Path | str,
    *,
    include_portable_native: bool = True,
) -> dict[str, Any]:
    """Create a compact `.tar.gz` containing everything needed for headless simulation on Colab."""
    root = Path(repo_root).resolve()
    dest = Path(dest_tar).resolve()
    dest.parent.mkdir(parents=True, exist_ok=True)
    file_count = 0
    rrf_hash = hashlib.sha256()

    def _skip(rel: str, parts: tuple[str, ...]) -> bool:
        if any(part in BUNDLE_EXCLUDE_PARTS for part in parts):
            return True
        if rel.endswith((".pyc", ".pyo", ".o", ".a", ".profraw", ".profdata")):
            return True
        return any(rel.startswith(prefix) for prefix in BUNDLE_EXCLUDE_PREFIXES)

    with tarfile.open(dest, "w:gz", compresslevel=6) as tar:
        for entry in BUNDLE_TOP_LEVEL_PATHS:
            full = root / entry
            if not full.exists():
                continue
            if full.is_file():
                tar.add(full, arcname=entry, recursive=False)
                file_count += 1
                if entry.startswith("RRF/"):
                    rrf_hash.update(entry.encode() + b"\0" + full.read_bytes())
                continue
            for path in sorted(full.rglob("*")):
                if not path.is_file() or path.is_symlink():
                    continue
                rel = path.relative_to(root).as_posix()
                if _skip(rel, path.relative_to(root).parts):
                    continue
                tar.add(path, arcname=rel, recursive=False)
                file_count += 1
                if rel.startswith("RRF/"):
                    rrf_hash.update(rel.encode() + b"\0" + path.read_bytes())

        # Embed git revision so portable remote checkouts preserve provenance without .git/.
        try:
            rev = subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=root, stderr=subprocess.DEVNULL, text=True
            ).strip()
        except (OSError, subprocess.CalledProcessError):
            rev = "unknown"
        rev_bytes = (rev + "\n").encode("utf-8")
        rev_info = tarfile.TarInfo(name=".hp_sim5_git_revision")
        rev_info.size = len(rev_bytes)
        tar.addfile(rev_info, io.BytesIO(rev_bytes))
        file_count += 1

        included_native = False
        if include_portable_native:
            portable = ensure_portable_native_physics(root)
            if portable is not None and portable.is_file() and Path(f"{portable}.json").is_file():
                tar.add(portable, arcname="src/wasm/cable_solver/target/native/physics.node", recursive=False)
                tar.add(
                    Path(f"{portable}.json"),
                    arcname="src/wasm/cable_solver/target/native/physics.node.json",
                    recursive=False,
                )
                included_native = True
                file_count += 2

    return {
        "tar_path": str(dest),
        "size_bytes": dest.stat().st_size,
        "file_count": file_count,
        "rrf_source_sha256": rrf_hash.hexdigest()[:20],
        "included_portable_native": included_native,
    }


@dataclass
class ColabConfig:
    """Configuration for provisioning and running experiments on Google Colab."""

    gpu: str | None = None
    tpu: str | None = None
    high_mem: bool = True
    session_name: str | None = None
    keep_session: bool = False
    remote_workers: int | None = None
    build_portable_native: bool = True
    colab_bin: str | None = None
    config_path: Path | None = None
    remote_store_uri: str | None = None
    command_timeout_s: float = 7200.0


class ColabSession:
    """Manages one Google Colab VM session via the official `colab` CLI."""

    def __init__(self, config: ColabConfig | None = None) -> None:
        self.config = config or ColabConfig()
        self.cli_prefix = resolve_colab_cli(self.config.colab_bin)
        self.session_name = self.config.session_name or f"hpsim-{uuid.uuid4().hex[:8]}"
        self._temp_config_dir: tempfile.TemporaryDirectory[str] | None = None
        if self.config.config_path is not None:
            self.config_path = Path(self.config.config_path).expanduser().resolve()
        else:
            self._temp_config_dir = tempfile.TemporaryDirectory(prefix="hp-colab-cfg-")
            self.config_path = Path(self._temp_config_dir.name) / "sessions.json"
        self._started = False

    def _cmd(self, *args: str) -> list[str]:
        return [*self.cli_prefix, "--config", str(self.config_path), *args]

    def _run_cli(
        self,
        *args: str,
        input_text: str | None = None,
        timeout_s: float = 600.0,
        check: bool = True,
    ) -> subprocess.CompletedProcess[str]:
        proc = subprocess.run(
            self._cmd(*args),
            input=input_text,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
        if check and proc.returncode != 0:
            raise RuntimeError(
                f"colab {' '.join(args[:3])} failed (exit {proc.returncode}):\n"
                f"STDOUT: {proc.stdout[-1500:]}\nSTDERR: {proc.stderr[-1500:]}"
            )
        return proc

    def start(self) -> "ColabSession":
        args = ["new", "-s", self.session_name]
        if self.config.gpu:
            args.extend(["--gpu", self.config.gpu])
        if self.config.tpu:
            args.extend(["--tpu", self.config.tpu])
        if self.config.high_mem:
            args.append("--high-mem")
        self._run_cli(*args, timeout_s=180.0)
        self._started = True
        return self

    def status(self) -> str:
        return self._run_cli("status", "-s", self.session_name, timeout_s=60.0).stdout

    def upload(self, local_path: Path | str, remote_path: str, *, timeout_s: float = 600.0) -> None:
        self._run_cli("upload", "-s", self.session_name, str(local_path), remote_path, timeout_s=timeout_s)

    def download(self, remote_path: str, local_path: Path | str, *, timeout_s: float = 600.0) -> Path:
        dest = Path(local_path)
        dest.parent.mkdir(parents=True, exist_ok=True)
        self._run_cli("download", "-s", self.session_name, remote_path, str(dest), timeout_s=timeout_s)
        return dest

    def exec_file(self, script_path: Path | str, *, timeout_s: float | None = None) -> subprocess.CompletedProcess[str]:
        return self._run_cli(
            "exec",
            "-s",
            self.session_name,
            "--timeout",
            "0",
            "-f",
            str(script_path),
            timeout_s=timeout_s or self.config.command_timeout_s,
        )

    def exec_python(self, code: str, *, timeout_s: float | None = None) -> subprocess.CompletedProcess[str]:
        return self._run_cli(
            "exec",
            "-s",
            self.session_name,
            "--timeout",
            "0",
            input_text=code,
            timeout_s=timeout_s or self.config.command_timeout_s,
        )

    def stop(self) -> None:
        if self._started:
            try:
                self._run_cli("stop", "-s", self.session_name, timeout_s=120.0, check=False)
            finally:
                self._started = False
        if self._temp_config_dir is not None:
            self._temp_config_dir.cleanup()
            self._temp_config_dir = None

    def __enter__(self) -> "ColabSession":
        return self.start()

    def __exit__(self, *_exc: object) -> None:
        if not self.config.keep_session:
            self.stop()


def _render_remote_bootstrap_script(rrf_hash: str, *, need_rerun: bool, need_usd: bool) -> str:
    return textwrap.dedent(f"""\
        import json, os, shutil, subprocess, sys, tarfile
        from pathlib import Path

        WORK = Path("/content/hp_sim5_work")
        CACHE = Path("/content/hp_sim5_cache")
        BUNDLE = Path("/content/hp_sim5_bundle.tar.gz")
        RRF_HASH = {rrf_hash!r}
        NEED_RERUN = {need_rerun!r}
        NEED_USD = {need_usd!r}

        def run(cmd, cwd=None, check=True):
            print("$", " ".join(map(str, cmd)), flush=True)
            p = subprocess.run(cmd, cwd=cwd, text=True, capture_output=True)
            if p.stdout:
                print(p.stdout[-2000:], flush=True)
            if p.stderr:
                print(p.stderr[-2000:], file=sys.stderr, flush=True)
            if check and p.returncode != 0:
                raise RuntimeError(f"Command failed ({{p.returncode}}): {{cmd}}")
            return p

        if WORK.exists():
            shutil.rmtree(WORK)
        WORK.mkdir(parents=True)
        with tarfile.open(BUNDLE, "r:gz") as tar:
            tar.extractall(WORK, filter="data")

        # 1. Minimal headless JS dependencies (cached across runs in /content/hp_sim5_cache/node_modules)
        nm_cache = CACHE / "node_modules"
        if not (nm_cache / "@kroxilon/usda-parser").is_dir() or not (nm_cache / "ws").is_dir():
            nm_stage = CACHE / "npm_stage"
            nm_stage.mkdir(parents=True, exist_ok=True)
            (nm_stage / "package.json").write_text('{{"name":"hp-sim5-colab-deps","private":true}}\\n')
            run(["npm", "install", "--no-audit", "--no-fund", "@kroxilon/usda-parser@^0.0.4", "ws@^8.18.0"], cwd=nm_stage)
            if nm_cache.exists():
                shutil.rmtree(nm_cache)
            shutil.move(str(nm_stage / "node_modules"), str(nm_cache))
        (WORK / "node_modules").symlink_to(nm_cache, target_is_directory=True)

        # 2. Build or reuse cached rrf_simulator
        rrf_bin = CACHE / "rrf" / RRF_HASH / "rrf_simulator"
        if not rrf_bin.is_file():
            rrf_build = CACHE / "rrf-build" / RRF_HASH
            rrf_build.mkdir(parents=True, exist_ok=True)
            cpus = str(os.cpu_count() or 4)
            run(["cmake", "-S", str(WORK / "RRF"), "-B", str(rrf_build)])
            run(["cmake", "--build", str(rrf_build), "--target", "rrf_simulator", f"-j{{cpus}}"])
            rrf_bin.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(rrf_build / "rrf_simulator", rrf_bin)
            shutil.rmtree(rrf_build, ignore_errors=True)
        (WORK / "RRF/build").mkdir(parents=True, exist_ok=True)
        shutil.copy2(rrf_bin, WORK / "RRF/build/rrf_simulator")

        # 3. Optional Python packages (only install what is missing and requested)
        missing_py = []
        for mod, pkg in [("jsonschema", "jsonschema"), ("websockets", "websockets")]:
            try:
                __import__(mod)
            except ImportError:
                missing_py.append(pkg)
        if NEED_RERUN:
            try:
                __import__("rerun")
            except ImportError:
                missing_py.append("rerun-sdk>=0.38.1,<0.39")
        if NEED_USD:
            try:
                __import__("pxr")
            except ImportError:
                missing_py.append("usd-core>=26.3")
        if missing_py:
            installer = ["uv", "pip", "install", "--system", *missing_py] if shutil.which("uv") else [sys.executable, "-m", "pip", "install", *missing_py]
            run(installer)

        # 4. Probe native Rust physics addon compatibility on this Colab VM
        native_ok = False
        native_err = None
        probe = subprocess.run(
            ["node", "--input-type=module", "-e",
             "import {{ loadNativePhysicsAddon }} from './src/js/cable_joints_3d/native_physics_engine.js'; loadNativePhysicsAddon(); console.log('NATIVE_OK');"],
            cwd=WORK, capture_output=True, text=True,
        )
        if probe.returncode == 0 and "NATIVE_OK" in probe.stdout:
            native_ok = True
        else:
            native_err = (probe.stderr or probe.stdout).strip()[-400:]

        info = {{
            "ok": True,
            "cpus": os.cpu_count(),
            "python": sys.executable,
            "native_physics_ok": native_ok,
            "native_physics_error": native_err,
        }}
        Path("/content/hp_sim5_bootstrap.json").write_text(json.dumps(info, indent=2) + "\\n")
        print("BOOTSTRAP_INFO:", json.dumps(info), flush=True)
    """)


def _render_remote_batch_script(*, max_workers: int | None, remote_store_uri: str | None) -> str:
    return textwrap.dedent(f"""\
        import json, os, shutil, sys, tarfile, threading, time
        from pathlib import Path

        WORK = Path("/content/hp_sim5_work")
        SCRATCH = Path("/content/hp_sim5_scratch")
        OUT_TAR = Path("/content/hp_sim5_results.tar.gz")
        SPECS_PATH = Path("/content/hp_sim5_specs.json")
        BOOTSTRAP = json.loads(Path("/content/hp_sim5_bootstrap.json").read_text())

        sys.path.insert(0, str(WORK / "src/python"))
        sys.path.insert(0, str(WORK))

        from hp_sim5_sim.interfaces import ExperimentSpec
        from hp_sim5_sim.runner import LocalScheduler
        from hp_sim5_sim.storage import LocalArtifactStore, store_from_uri

        if SCRATCH.exists():
            shutil.rmtree(SCRATCH)
        SCRATCH.mkdir(parents=True)

        raw_specs = json.loads(SPECS_PATH.read_text())
        specs = [ExperimentSpec.from_dict(item) for item in raw_specs]
        for s in specs:
            if s.solver == "native" and not BOOTSTRAP.get("native_physics_ok"):
                raise RuntimeError(
                    "Native Rust physics addon failed to load on Colab VM "
                    f"({{BOOTSTRAP.get('native_physics_error')}}). "
                    "Pass solver='wasm' or rebuild portable native physics."
                )

        remote_store_uri = {remote_store_uri!r}
        stage_store_dir = Path("/content/hp_sim5_stage_store")
        if stage_store_dir.exists():
            shutil.rmtree(stage_store_dir)
        store = store_from_uri(remote_store_uri) if remote_store_uri else LocalArtifactStore(stage_store_dir)

        cpus = os.cpu_count() or 2
        requested_workers = {max_workers!r}
        workers = requested_workers if requested_workers is not None else max(1, min(len(specs), cpus))
        print(f"COLAB_BATCH_START: specs={{len(specs)}} workers={{workers}} cpus={{cpus}}", flush=True)

        done_event = threading.Event()
        started_t = time.monotonic()
        def _heartbeat():
            while not done_event.wait(15.0):
                print(f"COLAB_BATCH_HEARTBEAT: elapsed={{time.monotonic() - started_t:.1f}}s", flush=True)
        hb = threading.Thread(target=_heartbeat, daemon=True)
        hb.start()
        try:
            scheduler = LocalScheduler(
                WORK,
                scratch_base=SCRATCH,
                python_exe=sys.executable,
                backend_label=f"colab-cpu-{{cpus}}c",
            )
            results = scheduler.run_batch(specs, max_workers=workers, store=store, keep_scratch=False)
        finally:
            done_event.set()
            hb.join(timeout=2.0)

        summary = {{
            "cpus": cpus,
            "workers": workers,
            "native_physics_ok": BOOTSTRAP.get("native_physics_ok"),
            "remote_store_uri": remote_store_uri,
            "results": [r.to_dict() for r in results],
        }}
        summary_path = Path("/content/hp_sim5_batch_summary.json")
        summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\\n")

        with tarfile.open(OUT_TAR, "w:gz", compresslevel=5) as tar:
            tar.add(summary_path, arcname="batch_summary.json")
            if not remote_store_uri and stage_store_dir.exists():
                for path in sorted(stage_store_dir.rglob("*")):
                    if path.is_file():
                        tar.add(path, arcname="store/" + path.relative_to(stage_store_dir).as_posix())

        print("COLAB_BATCH_DONE:", json.dumps({{
            "count": len(results),
            "statuses": [r.status for r in results],
            "archive_bytes": OUT_TAR.stat().st_size,
        }}), flush=True)
    """)


class ColabScheduler:
    """Executes single or parallel batches of ExperimentSpecs remotely on Google Colab."""

    def __init__(
        self,
        repo_root: Path | str,
        *,
        config: ColabConfig | None = None,
        session: ColabSession | None = None,
    ) -> None:
        self.repo_root = Path(repo_root).resolve()
        self.config = config or ColabConfig()
        self._external_session = session

    @property
    def backend_name(self) -> str:
        accel = self.config.gpu or self.config.tpu or ("cpu-highmem" if self.config.high_mem else "cpu")
        return f"colab-{accel}"

    def run(
        self,
        spec: ExperimentSpec,
        *,
        store: ArtifactStore | None = None,
        keep_scratch: bool = False,
    ) -> ExperimentResult:
        return self.run_batch([spec], max_workers=1, store=store, keep_scratch=keep_scratch)[0]

    def run_batch(
        self,
        specs: Sequence[ExperimentSpec],
        *,
        max_workers: int | None = None,
        store: ArtifactStore | None = None,
        keep_scratch: bool = False,
    ) -> list[ExperimentResult]:
        spec_list = list(specs)
        if not spec_list:
            return []

        need_rerun = any(s.recording_mode != "none" for s in spec_list)
        need_native = any(s.solver == "native" for s in spec_list)

        with ScratchWorkspace(f"colab-batch-{uuid.uuid4().hex[:8]}", keep=keep_scratch) as local_ws:
            assert local_ws.path is not None
            ws_dir = local_ws.path
            bundle_tar = ws_dir / "hp_sim5_bundle.tar.gz"
            bundle_info = bundle_workspace(
                self.repo_root,
                bundle_tar,
                include_portable_native=need_native and self.config.build_portable_native,
            )

            specs_json = ws_dir / "specs.json"
            specs_json.write_text(
                json.dumps([s.to_dict() for s in spec_list], indent=2) + "\n", encoding="utf-8"
            )
            bootstrap_py = ws_dir / "bootstrap.py"
            bootstrap_py.write_text(
                _render_remote_bootstrap_script(
                    bundle_info["rrf_source_sha256"],
                    need_rerun=need_rerun,
                    need_usd=False,
                ),
                encoding="utf-8",
            )
            batch_py = ws_dir / "run_batch.py"
            batch_py.write_text(
                _render_remote_batch_script(
                    max_workers=max_workers if max_workers is not None else self.config.remote_workers,
                    remote_store_uri=self.config.remote_store_uri,
                ),
                encoding="utf-8",
            )

            own_session = self._external_session is None
            session = self._external_session or ColabSession(self.config)
            try:
                if own_session:
                    session.start()
                session.upload(bundle_tar, "/content/hp_sim5_bundle.tar.gz")
                session.upload(specs_json, "/content/hp_sim5_specs.json")
                session.exec_file(bootstrap_py, timeout_s=900.0)
                session.exec_file(batch_py, timeout_s=self.config.command_timeout_s)

                results_tar = ws_dir / "results.tar.gz"
                session.download("/content/hp_sim5_results.tar.gz", results_tar, timeout_s=900.0)
            finally:
                if own_session and not self.config.keep_session:
                    session.stop()

            extract_dir = ws_dir / "extracted"
            extract_dir.mkdir(parents=True)
            with tarfile.open(results_tar, "r:gz") as tar:
                tar.extractall(extract_dir, filter="data")

            summary = json.loads((extract_dir / "batch_summary.json").read_text(encoding="utf-8"))
            results = [ExperimentResult.from_dict(item) for item in summary["results"]]

            # Persist downloaded artifacts into local/Drive ArtifactStore if requested.
            if store is not None and not self.config.remote_store_uri:
                for res in results:
                    exp_dir = extract_dir / "store/experiments" / res.experiment_id
                    if exp_dir.is_dir():
                        meta = dict(res.manifest.get("metadata", {})) if res.manifest else {}
                        meta.setdefault("kind", res.kind)
                        meta.setdefault("status", res.status)
                        meta.setdefault("backend", res.backend)
                        manifest = store.put_directory(res.experiment_id, exp_dir, metadata=meta)
                        res.manifest = manifest
                        res.artifact_uri = f"{store.uri}/experiments/{res.experiment_id}"

            if keep_scratch:
                for res in results:
                    exp_dir = extract_dir / "store/experiments" / res.experiment_id
                    if exp_dir.is_dir():
                        res.scratch_dir = str(exp_dir)

            return results
