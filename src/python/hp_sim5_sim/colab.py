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
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
import textwrap
from typing import Any, Sequence
import uuid

from .experiment_lookup import directory_metadata
from .interfaces import ArtifactStore, ExperimentResult, ExperimentSpec
from .provenance import git_provenance
from .storage import (
    ArtifactUploadError,
    DriveMountArtifactStore,
    GoogleDriveArtifactStore,
    LOCAL_STORE_DIR,
    LocalArtifactStore,
    ScratchWorkspace,
    UPLOAD_PENDING_DIR,
    UPLOAD_PENDING_NAME,
    retry_upload_command,
    write_upload_pending,
)


class ColabShutdownError(RuntimeError):
    """Raised when stopping a Google Colab VM fails; session config is preserved for manual cleanup."""

    def __init__(
        self,
        message: str,
        *,
        session_name: str,
        config_path: Path | str,
        cleanup_command: str,
    ) -> None:
        super().__init__(message)
        self.session_name = session_name
        self.config_path = str(config_path)
        self.cleanup_command = cleanup_command


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
    provenance = git_provenance(root)

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

        # Portable remote workspaces have no .git directory.
        for key, value in provenance.items():
            data = (str(value).lower() if isinstance(value, bool) else str(value)).encode("utf-8")
            info = tarfile.TarInfo(name=f".hp_sim5_{key}")
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
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
    direct_drive_upload: bool = False
    default_local_store_dir: Path | None = None
    command_timeout_s: float = 7200.0
    persist_by_default: bool = True


class ColabSession:
    """Manages one Google Colab VM session via the official `colab` CLI."""

    def __init__(self, config: ColabConfig | None = None) -> None:
        self.config = config or ColabConfig()
        self.cli_prefix = resolve_colab_cli(self.config.colab_bin)
        self.session_name = self.config.session_name or f"hpsim-{uuid.uuid4().hex[:8]}"
        self._temp_config_dir: Path | None = None
        if self.config.config_path is not None:
            self.config_path = Path(self.config.config_path).expanduser().resolve()
        else:
            # Use mkdtemp (not TemporaryDirectory) so sessions.json survives process exit if stop() fails.
            self._temp_config_dir = Path(tempfile.mkdtemp(prefix="hp-colab-cfg-"))
            self.config_path = self._temp_config_dir / "sessions.json"
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

    def _cleanup_temp_config(self) -> None:
        if self._temp_config_dir is not None:
            shutil.rmtree(self._temp_config_dir, ignore_errors=True)
            self._temp_config_dir = None

    def _session_still_listed(self) -> bool | None:
        """Check whether this session is still present in `colab sessions` (None if check failed)."""
        try:
            proc = self._run_cli("sessions", timeout_s=60.0, check=False)
        except (subprocess.TimeoutExpired, OSError):
            return None
        if proc.returncode != 0:
            return None
        return self.session_name in (proc.stdout or "")

    def start(self) -> "ColabSession":
        args = ["new", "-s", self.session_name]
        if self.config.gpu:
            args.extend(["--gpu", self.config.gpu])
        if self.config.tpu:
            args.extend(["--tpu", self.config.tpu])
        if self.config.high_mem:
            args.append("--high-mem")
        try:
            self._run_cli(*args, timeout_s=180.0)
        except Exception:
            if self.config_path.is_file():
                self._started = True
            raise
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
        if not self._started:
            self._cleanup_temp_config()
            return

        last_err = ""
        for _attempt in range(2):
            try:
                proc = self._run_cli("stop", "-s", self.session_name, timeout_s=120.0, check=False)
                if proc.returncode == 0:
                    self._started = False
                    self._cleanup_temp_config()
                    return
                last_err = (proc.stderr or proc.stdout or f"exit code {proc.returncode}").strip()[-1000:]
            except (subprocess.TimeoutExpired, OSError) as err:
                last_err = f"{type(err).__name__}: {err}"

        # If `stop` failed because the session is already gone/pruned, treat shutdown as complete.
        if self._session_still_listed() is False:
            self._started = False
            self._cleanup_temp_config()
            return

        cleanup_cmd = shlex.join(self._cmd("stop", "-s", self.session_name))
        msg = (
            f"Failed to stop Colab session {self.session_name!r}: {last_err}\n"
            f"Preserved session config at: {self.config_path}\n"
            f"Stop manually with: {cleanup_cmd}"
        )
        print(f"[hp_sim5_sim] ERROR: {msg}", file=sys.stderr, flush=True)
        raise ColabShutdownError(
            msg,
            session_name=self.session_name,
            config_path=self.config_path,
            cleanup_command=cleanup_cmd,
        )

    def __enter__(self) -> "ColabSession":
        return self.start()

    def __exit__(self, *_exc: object) -> None:
        if not self.config.keep_session:
            self.stop()


def _render_remote_bootstrap_script(rrf_hash: str, *, need_rerun: bool, need_usd: bool) -> str:
    return textwrap.dedent(f"""\
        import json, os, shutil, subprocess, sys, tarfile, threading, time
        from pathlib import Path

        WORK = Path("/content/hp_sim5_work")
        CACHE = Path("/content/hp_sim5_cache")
        BUNDLE = Path("/content/hp_sim5_bundle.tar.gz")
        RRF_HASH = {rrf_hash!r}
        NEED_RERUN = {need_rerun!r}
        NEED_USD = {need_usd!r}

        _hb_done = threading.Event()
        _hb_start = time.monotonic()
        def _hb():
            while not _hb_done.wait(15.0):
                print(f"BOOTSTRAP_HEARTBEAT: elapsed={{time.monotonic() - _hb_start:.1f}}s", flush=True)
        _hb_thread = threading.Thread(target=_hb, daemon=True)
        _hb_thread.start()

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

        try:
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
        finally:
            _hb_done.set()
            _hb_thread.join(timeout=2.0)
    """)


def _render_remote_batch_script(
    *,
    max_workers: int | None,
    remote_store_uri: str | None,
    direct_drive_config: dict[str, Any] | None = None,
) -> str:
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
        from hp_sim5_sim.storage import GoogleDriveArtifactStore, LocalArtifactStore, store_from_uri

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
        direct_drive_cfg = {direct_drive_config!r}
        stage_store_dir = Path("/content/hp_sim5_stage_store")
        if stage_store_dir.exists():
            shutil.rmtree(stage_store_dir)

        token_path = None
        if direct_drive_cfg is not None:
            # Read short-lived bearer access token (no refresh_token/client_secret) and delete the
            # handoff file BEFORE spawning any experiment subprocesses.
            token_path = Path(direct_drive_cfg["token_remote_path"])
            try:
                token_payload = json.loads(token_path.read_text(encoding="utf-8"))
            finally:
                token_path.unlink(missing_ok=True)
            access_token = str(token_payload["access_token"])
            quota_project = token_payload.get("quota_project")
            store = GoogleDriveArtifactStore(
                root_folder=direct_drive_cfg["root_folder"],
                root_folder_id=direct_drive_cfg.get("root_folder_id"),
                experiments_folder_id=direct_drive_cfg.get("experiments_folder_id"),
                token_provider=lambda _tok=access_token: _tok,
                quota_project=quota_project,
                api_base=direct_drive_cfg["api_base"],
                upload_base=direct_drive_cfg["upload_base"],
                chunk_bytes=direct_drive_cfg["chunk_bytes"],
            )
            direct_persist = True
        elif remote_store_uri:
            store = store_from_uri(remote_store_uri)
            direct_persist = True
        else:
            store = LocalArtifactStore(stage_store_dir)
            direct_persist = False

        cpus = os.cpu_count() or 2
        requested_workers = {max_workers!r}
        workers = requested_workers if requested_workers is not None else max(1, min(len(specs), cpus))
        print(
            f"COLAB_BATCH_START: specs={{len(specs)}} workers={{workers}} cpus={{cpus}} direct_persist={{direct_persist}}",
            flush=True,
        )

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
            results = scheduler.run_batch(
                specs,
                max_workers=workers,
                store=store,
                keep_scratch=False,
                raise_on_upload_error=False,
            )
        finally:
            done_event.set()
            hb.join(timeout=2.0)
            if token_path is not None:
                token_path.unlink(missing_ok=True)

        rescue_dirs = {{}}
        for r in results:
            if direct_persist and r.status == "upload_failed" and r.scratch_dir:
                p = Path(r.scratch_dir)
                if p.is_dir():
                    rescue_dirs[r.experiment_id] = p
                r.scratch_dir = None
            elif not direct_persist:
                # Never return ephemeral /content/... URIs back to the local host.
                r.artifact_uri = None
                r.scratch_dir = None
                if r.manifest and str(r.manifest.get("store_uri", "")).startswith("local:///content/"):
                    r.manifest["store_uri"] = None

        summary = {{
            "cpus": cpus,
            "workers": workers,
            "native_physics_ok": BOOTSTRAP.get("native_physics_ok"),
            "remote_store_uri": remote_store_uri,
            "direct_drive_upload": direct_drive_cfg is not None,
            "rescued_experiments": sorted(rescue_dirs.keys()),
            "results": [r.to_dict() for r in results],
        }}
        summary_path = Path("/content/hp_sim5_batch_summary.json")
        summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\\n")

        with tarfile.open(OUT_TAR, "w:gz", compresslevel=5) as tar:
            tar.add(summary_path, arcname="batch_summary.json")
            if not direct_persist and stage_store_dir.exists():
                for path in sorted(stage_store_dir.rglob("*")):
                    if path.is_file():
                        tar.add(path, arcname="store/" + path.relative_to(stage_store_dir).as_posix())
            for exp_id, rdir in sorted(rescue_dirs.items()):
                for path in sorted(rdir.rglob("*")):
                    if path.is_file():
                        tar.add(path, arcname=f"rescue/{{exp_id}}/" + path.relative_to(rdir).as_posix())

        print("COLAB_BATCH_DONE:", json.dumps({{
            "count": len(results),
            "statuses": [r.status for r in results],
            "rescued": sorted(rescue_dirs.keys()),
            "archive_bytes": OUT_TAR.stat().st_size,
        }}), flush=True)
    """)


def _render_remote_rescue_script() -> str:
    """Pack any completed staged experiments or remaining scratch dirs before stopping a failed VM."""
    return textwrap.dedent("""\
        import json, tarfile
        from pathlib import Path

        Path("/content/.hp_sim5_drive_token.json").unlink(missing_ok=True)
        STAGE_EXPS = Path("/content/hp_sim5_stage_store/experiments")
        SCRATCH = Path("/content/hp_sim5_scratch")
        RESCUE_TAR = Path("/content/hp_sim5_rescue.tar.gz")

        completed = []
        partial = []
        with tarfile.open(RESCUE_TAR, "w:gz", compresslevel=5) as tar:
            if STAGE_EXPS.is_dir():
                for exp_dir in sorted(STAGE_EXPS.iterdir()):
                    if not exp_dir.is_dir():
                        continue
                    files = [p for p in sorted(exp_dir.rglob("*")) if p.is_file() and not p.is_symlink()]
                    if not files:
                        continue
                    completed.append(exp_dir.name)
                    for p in files:
                        tar.add(p, arcname=f"store/experiments/{exp_dir.name}/" + p.relative_to(exp_dir).as_posix())
            if SCRATCH.is_dir():
                for sc_dir in sorted(SCRATCH.iterdir()):
                    if not sc_dir.is_dir():
                        continue
                    files = [p for p in sorted(sc_dir.rglob("*")) if p.is_file() and not p.is_symlink()]
                    if not files:
                        continue
                    partial.append(sc_dir.name)
                    for p in files:
                        tar.add(p, arcname=f"partial_scratch/{sc_dir.name}/" + p.relative_to(sc_dir).as_posix())
            index_path = Path("/content/hp_sim5_rescue_index.json")
            index_path.write_text(json.dumps({
                "completed_experiments": completed,
                "partial_scratch_dirs": partial,
            }, indent=2) + "\\n")
            tar.add(index_path, arcname="rescue_index.json")

        print("COLAB_RESCUE_DONE:", json.dumps({
            "completed_experiments": completed,
            "partial_scratch_dirs": partial,
            "archive_bytes": RESCUE_TAR.stat().st_size,
        }), flush=True)
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

    def _best_effort_rescue(
        self,
        *,
        session: ColabSession,
        ws_dir: Path,
        local_ws: ScratchWorkspace,
        effective_store: ArtifactStore | None,
    ) -> dict[str, Any]:
        """Attempt to recover completed and partial remote artifacts before stopping a failed VM."""
        recovered: dict[str, Any] = {"persisted": [], "preserved_local": [], "partial_scratch": []}
        try:
            rescue_tar = ws_dir / "rescue.tar.gz"
            rescue_py = ws_dir / "rescue.py"
            rescue_py.write_text(_render_remote_rescue_script(), encoding="utf-8")
            session.exec_file(rescue_py, timeout_s=120.0)
            session.download("/content/hp_sim5_rescue.tar.gz", rescue_tar, timeout_s=300.0)

            rescue_extract = ws_dir / "rescued"
            rescue_extract.mkdir(parents=True, exist_ok=True)
            with tarfile.open(rescue_tar, "r:gz") as tar:
                tar.extractall(rescue_extract, filter="data")

            exps_root = rescue_extract / "store/experiments"
            if exps_root.is_dir():
                for exp_dir in sorted(p for p in exps_root.iterdir() if p.is_dir()):
                    exp_id = exp_dir.name
                    meta = directory_metadata(exp_dir)
                    if effective_store is not None:
                        try:
                            effective_store.put_directory(exp_id, exp_dir, metadata=meta)
                            recovered["persisted"].append(f"{effective_store.uri}/experiments/{exp_id}")
                            continue
                        except Exception as err:
                            local_ws.preserve()
                            write_upload_pending(
                                exp_dir,
                                experiment_id=exp_id,
                                store_uri=effective_store.uri,
                                metadata=meta,
                                error=f"{type(err).__name__}: {err}",
                            )
                            recovered["preserved_local"].append(str(exp_dir))
                    else:
                        local_ws.preserve()
                        recovered["preserved_local"].append(str(exp_dir))

            partial_root = rescue_extract / "partial_scratch"
            if partial_root.is_dir():
                for sc_dir in sorted(p for p in partial_root.iterdir() if p.is_dir()):
                    # If direct upload failed on Colab after experiment finished, try persisting it now.
                    pending_file = sc_dir / UPLOAD_PENDING_NAME
                    if pending_file.is_file() and effective_store is not None:
                        try:
                            pdata = json.loads(pending_file.read_text(encoding="utf-8"))
                            exp_id = pdata.get("experiment_id")
                            meta = dict(pdata.get("metadata") or {})
                            if exp_id:
                                effective_store.put_directory(exp_id, sc_dir, metadata=meta)
                                recovered["persisted"].append(f"{effective_store.uri}/experiments/{exp_id}")
                                continue
                        except Exception:
                            pass
                    local_ws.preserve()
                    recovered["partial_scratch"].append(str(sc_dir))

            if recovered["persisted"] or recovered["preserved_local"] or recovered["partial_scratch"]:
                lines = ["[hp_sim5_sim] RECOVERY: Rescued remote artifacts before stopping Colab VM:"]
                for uri in recovered["persisted"]:
                    lines.append(f"[hp_sim5_sim]   persisted: {uri}")
                for path in recovered["preserved_local"]:
                    lines.append(f"[hp_sim5_sim]   preserved locally: {path}")
                for path in recovered["partial_scratch"]:
                    lines.append(f"[hp_sim5_sim]   partial scratch: {path}")
                print("\n".join(lines), file=sys.stderr, flush=True)
        except Exception as err:
            print(
                f"[hp_sim5_sim] WARNING: Best-effort remote artifact rescue failed: {type(err).__name__}: {err}",
                file=sys.stderr,
                flush=True,
            )
        return recovered

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

        # Default to persistent local storage when the caller hasn't chosen how to keep results.
        effective_store: ArtifactStore | None = store
        if (
            self.config.persist_by_default and effective_store is None
            and not self.config.remote_store_uri and not keep_scratch
        ):
            default_dir = (
                Path(self.config.default_local_store_dir).expanduser().resolve()
                if self.config.default_local_store_dir is not None
                else (self.repo_root / LOCAL_STORE_DIR).resolve()
            )
            effective_store = LocalArtifactStore(default_dir)
            print(
                f"[hp_sim5_sim] No artifact store specified for Colab run; "
                f"persisting artifacts to {effective_store.uri}",
                file=sys.stderr,
                flush=True,
            )

        # Opt-in direct Colab -> Google Drive upload (bypassing this host) using ONLY a short-lived
        # OAuth access token minted locally (never copying refresh_token/client_secret to Colab).
        direct_drive_config: dict[str, Any] | None = None
        direct_drive_token_payload: dict[str, Any] | None = None
        drive = effective_store if isinstance(effective_store, GoogleDriveArtifactStore) and not self.config.remote_store_uri else None
        if drive is not None:
            try:
                exp_folder_id = drive.ensure_experiments_folder()
            except (OSError, RuntimeError) as err:
                raise ArtifactUploadError(
                    f"Could not open {drive.uri} before starting Colab: {err}",
                    store_uri=drive.uri,
                ) from err
        if drive is not None and self.config.direct_drive_upload:
            access_token, quota_project = drive.mint_access_token()
            direct_drive_token_payload = {
                "access_token": access_token,
                "quota_project": quota_project,
            }
            direct_drive_config = {
                "root_folder": drive.root_folder,
                "root_folder_id": drive._explicit_root_id,
                "experiments_folder_id": exp_folder_id,
                "api_base": drive.api_base,
                "upload_base": drive.upload_base,
                "chunk_bytes": drive.chunk_bytes,
                "token_remote_path": "/content/.hp_sim5_drive_token.json",
            }

        scratch_base = (
            self.repo_root / LOCAL_STORE_DIR / UPLOAD_PENDING_DIR
            if isinstance(effective_store, (GoogleDriveArtifactStore, DriveMountArtifactStore))
            or self.config.remote_store_uri else None
        )
        with ScratchWorkspace(f"colab-batch-{uuid.uuid4().hex[:8]}", base_dir=scratch_base, keep=keep_scratch) as local_ws:
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
                    direct_drive_config=direct_drive_config,
                ),
                encoding="utf-8",
            )

            own_session = self._external_session is None
            session = self._external_session or ColabSession(self.config)
            results_tar = ws_dir / "results.tar.gz"
            batch_started = False
            body_failed = False
            shutdown_error: ColabShutdownError | None = None
            try:
                if own_session:
                    session.start()
                session.upload(bundle_tar, "/content/hp_sim5_bundle.tar.gz")
                session.upload(specs_json, "/content/hp_sim5_specs.json")
                if direct_drive_config is not None and direct_drive_token_payload is not None:
                    local_token_file = ws_dir / ".drive_token.json"
                    local_token_file.write_text(json.dumps(direct_drive_token_payload) + "\n", encoding="utf-8")
                    os.chmod(local_token_file, 0o600)
                    try:
                        session.upload(local_token_file, direct_drive_config["token_remote_path"])
                    finally:
                        local_token_file.unlink(missing_ok=True)
                session.exec_file(bootstrap_py, timeout_s=900.0)
                batch_started = True
                session.exec_file(batch_py, timeout_s=self.config.command_timeout_s)
                session.download("/content/hp_sim5_results.tar.gz", results_tar, timeout_s=900.0)
            except BaseException:
                body_failed = True
                if batch_started and not results_tar.is_file():
                    self._best_effort_rescue(
                        session=session,
                        ws_dir=ws_dir,
                        local_ws=local_ws,
                        effective_store=effective_store,
                    )
                raise
            finally:
                if own_session and not self.config.keep_session:
                    try:
                        session.stop()
                    except ColabShutdownError as err:
                        if body_failed:
                            # Do not mask the primary run failure; stop() already printed manual command
                            # and preserved sessions.json.
                            pass
                        else:
                            # Defer raising until after downloaded artifacts are safely persisted.
                            shutdown_error = err

            extract_dir = ws_dir / "extracted"
            extract_dir.mkdir(parents=True)
            with tarfile.open(results_tar, "r:gz") as tar:
                tar.extractall(extract_dir, filter="data")

            summary = json.loads((extract_dir / "batch_summary.json").read_text(encoding="utf-8"))
            results = [ExperimentResult.from_dict(item) for item in summary["results"]]

            upload_errors: list[ArtifactUploadError] = []

            def _record_upload_failure(
                res: ExperimentResult,
                preserved_dir: Path,
                store_uri: str,
                meta: dict[str, Any],
                err: Exception,
            ) -> None:
                local_ws.preserve()
                err_text = f"{type(err).__name__}: {err}"
                write_upload_pending(
                    preserved_dir,
                    experiment_id=res.experiment_id,
                    store_uri=store_uri,
                    metadata=meta,
                    error=err_text,
                )
                retry_cmd = retry_upload_command(res.experiment_id, preserved_dir, store_uri)
                res.scratch_dir = str(preserved_dir)
                res.artifact_uri = None
                res.error = (
                    f"{res.error}\nArtifact upload failed: {err_text}"
                    if res.error
                    else f"Artifact upload failed: {err_text}"
                )
                print(
                    f"[hp_sim5_sim] WARNING: Artifact upload failed for {res.experiment_id} -> {store_uri}: {err_text}\n"
                    f"[hp_sim5_sim] Preserved local scratch at: {preserved_dir}\n"
                    f"[hp_sim5_sim] Retry upload with: {retry_cmd}",
                    file=sys.stderr,
                    flush=True,
                )
                upload_errors.append(
                    ArtifactUploadError(
                        f"Artifact upload failed for {res.experiment_id} -> {store_uri}: {err_text}. "
                        f"Preserved scratch at {preserved_dir}. Retry with: {retry_cmd}",
                        experiment_id=res.experiment_id,
                        scratch_dir=preserved_dir,
                        store_uri=store_uri,
                        result=res,
                    )
                )

            if direct_drive_config is not None and effective_store is not None:
                # Direct Colab -> Drive mode: retry any rescued experiments locally, preserving on failure.
                for res in results:
                    if res.status != "upload_failed":
                        continue
                    rescue_dir = extract_dir / "rescue" / res.experiment_id
                    if not rescue_dir.is_dir():
                        upload_errors.append(
                            ArtifactUploadError(
                                f"Direct Drive upload failed on Colab for {res.experiment_id} and no rescue archive was found",
                                experiment_id=res.experiment_id,
                                store_uri=effective_store.uri,
                                result=res,
                            )
                        )
                        continue
                    pending_path = rescue_dir / UPLOAD_PENDING_NAME
                    meta: dict[str, Any] = {}
                    if pending_path.is_file():
                        try:
                            meta = dict(json.loads(pending_path.read_text(encoding="utf-8")).get("metadata") or {})
                        except ValueError:
                            meta = {}
                    meta.setdefault("kind", res.kind)
                    meta.setdefault("status", res.metrics.get("experiment_status", "ok"))
                    meta.setdefault("backend", res.backend)
                    try:
                        manifest = effective_store.put_directory(res.experiment_id, rescue_dir, metadata=meta)
                        res.status = str(res.metrics.pop("experiment_status", "ok"))
                        res.metrics.pop("upload_error", None)
                        res.error = None
                        res.manifest = manifest
                        res.artifact_uri = f"{effective_store.uri}/experiments/{res.experiment_id}"
                        pending_path.unlink(missing_ok=True)
                    except Exception as err:
                        _record_upload_failure(res, rescue_dir, effective_store.uri, meta, err)
            elif self.config.remote_store_uri:
                for res in results:
                    if res.status != "upload_failed":
                        continue
                    rescue_dir = extract_dir / "rescue" / res.experiment_id
                    if rescue_dir.is_dir():
                        local_ws.preserve()
                        res.scratch_dir = str(rescue_dir)
                        res.artifact_uri = None
                        retry_cmd = retry_upload_command(res.experiment_id, rescue_dir, self.config.remote_store_uri)
                        print(
                            f"[hp_sim5_sim] WARNING: Remote store upload failed for {res.experiment_id}.\n"
                            f"[hp_sim5_sim] Preserved rescued local copy at: {rescue_dir}\n"
                            f"[hp_sim5_sim] Retry upload with: {retry_cmd}",
                            file=sys.stderr,
                            flush=True,
                        )
                        upload_errors.append(
                            ArtifactUploadError(
                                f"Remote store upload failed for {res.experiment_id}. Preserved rescued copy at {rescue_dir}.",
                                experiment_id=res.experiment_id,
                                scratch_dir=rescue_dir,
                                store_uri=self.config.remote_store_uri,
                                result=res,
                            )
                        )
            elif effective_store is not None:
                if hasattr(effective_store, "ensure_experiments_folder"):
                    try:
                        effective_store.ensure_experiments_folder()  # type: ignore[attr-defined]
                    except Exception:
                        pass
                for res in results:
                    exp_dir = extract_dir / "store/experiments" / res.experiment_id
                    if not exp_dir.is_dir():
                        continue
                    meta = directory_metadata(exp_dir)
                    if res.manifest:
                        meta.update(res.manifest.get("metadata") or {})
                    meta.setdefault("kind", res.kind)
                    meta.setdefault("status", res.status)
                    meta.setdefault("backend", res.backend)
                    try:
                        manifest = effective_store.put_directory(res.experiment_id, exp_dir, metadata=meta)
                        res.manifest = manifest
                        res.artifact_uri = f"{effective_store.uri}/experiments/{res.experiment_id}"
                    except Exception as err:
                        _record_upload_failure(res, exp_dir, effective_store.uri, meta, err)

            # Never return an artifact URI pointing to the terminated Colab VM's /content filesystem.
            for res in results:
                if res.artifact_uri and res.artifact_uri.startswith("local:///content/"):
                    res.artifact_uri = None
                if res.manifest and str(res.manifest.get("store_uri", "")).startswith("local:///content/"):
                    res.manifest["store_uri"] = None

            if keep_scratch:
                for res in results:
                    if res.scratch_dir is not None:
                        continue
                    for candidate in (
                        extract_dir / "store/experiments" / res.experiment_id,
                        extract_dir / "rescue" / res.experiment_id,
                    ):
                        if candidate.is_dir():
                            res.scratch_dir = str(candidate)
                            break

            if upload_errors:
                if len(upload_errors) > 1:
                    first = upload_errors[0]
                    raise ArtifactUploadError(
                        "\n".join(map(str, upload_errors)),
                        experiment_id=first.experiment_id,
                        scratch_dir=first.scratch_dir,
                        store_uri=first.store_uri,
                        result=first.result,
                    )
                raise upload_errors[0]
            if shutdown_error is not None:
                raise shutdown_error

            return results
