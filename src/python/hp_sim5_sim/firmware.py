"""Reusable RepRapFirmware host simulator service with clean lifecycle management."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import tempfile
import time
from urllib.error import URLError
from urllib.request import Request, urlopen

from .interfaces import FirmwareReply


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def stop_process_tree(process: subprocess.Popen | None, *, timeout_s: float = 5.0) -> None:
    """Terminate a subprocess and its process group cleanly without leaving orphans."""
    if process is None or process.poll() is not None:
        return
    try:
        is_group_leader = os.getpgid(process.pid) == process.pid
    except OSError:
        is_group_leader = False
    try:
        if is_group_leader:
            os.killpg(process.pid, signal.SIGTERM)
        else:
            process.terminate()
    except ProcessLookupError:
        process.wait()
        return
    try:
        process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        try:
            if is_group_leader:
                os.killpg(process.pid, signal.SIGKILL)
            else:
                process.kill()
        except ProcessLookupError:
            pass
        process.wait()


class RrfFirmwareService:
    """Owns one isolated `rrf_simulator --server` instance."""

    def __init__(
        self,
        repo_root: Path,
        config: str,
        *,
        port: int | None = None,
        binary: Path | None = None,
        vsd_root: Path | None = None,
        log_path: Path | None = None,
        isolate_vsd: bool = False,
        config_override_text: str | None = None,
        own_process_group: bool = True,
    ) -> None:
        self.repo_root = Path(repo_root).resolve()
        self.config = config
        self.port = port or free_port()
        self.binary = Path(binary).resolve() if binary else (self.repo_root / "RRF/build/rrf_simulator")
        self.source_vsd = Path(vsd_root).resolve() if vsd_root else (self.repo_root / "RRF/run/vsd")
        self.log_path = Path(log_path) if log_path else None
        self.isolate_vsd = isolate_vsd or (config_override_text is not None)
        self.config_override_text = config_override_text
        self.own_process_group = own_process_group
        self._url = f"http://127.0.0.1:{self.port}"
        self._vsd_temp: tempfile.TemporaryDirectory[str] | None = None
        self._process: subprocess.Popen | None = None
        self._config_sha256 = ""
        self._binary_sha256 = ""

    @property
    def url(self) -> str:
        return self._url

    @property
    def pid(self) -> int | None:
        return None if self._process is None else self._process.pid

    @property
    def process(self) -> subprocess.Popen | None:
        return self._process

    @property
    def config_sha256(self) -> str:
        return self._config_sha256

    @property
    def binary_sha256(self) -> str:
        return self._binary_sha256

    def start(self, *, ready_timeout_s: float = 30.0) -> "RrfFirmwareService":
        if not self.binary.is_file():
            raise FileNotFoundError(
                f"rrf_simulator binary not found at {self.binary}. "
                "Build it with: cmake -S RRF -B RRF/build && cmake --build RRF/build --target rrf_simulator -j"
            )
        self._binary_sha256 = hashlib.sha256(self.binary.read_bytes()).hexdigest()

        if self.isolate_vsd:
            # Keep VSD path short: RRF's MaxFilenameLength is 120 chars including host path.
            self._vsd_temp = tempfile.TemporaryDirectory(prefix="hp-rrf-")
            vsd_dir = Path(self._vsd_temp.name)
            shutil.copytree(self.source_vsd / "sys", vsd_dir / "sys")
            if self.config_override_text is not None:
                cfg_bytes = self.config_override_text.encode("utf-8")
                (vsd_dir / "sys/override.g").write_bytes(cfg_bytes)
                effective_config = "sys/override.g"
            else:
                cfg_bytes = (self.source_vsd / self.config).read_bytes()
                effective_config = self.config
            vsd_arg = str(vsd_dir)
        else:
            cfg_path = self.source_vsd / self.config
            cfg_bytes = cfg_path.read_bytes()
            effective_config = self.config
            # Use relative path when inside repo_root to stay well below RRF's 120-char path limit.
            try:
                vsd_arg = str(self.source_vsd.relative_to(self.repo_root))
            except ValueError:
                vsd_arg = str(self.source_vsd)
        self._config_sha256 = hashlib.sha256(cfg_bytes).hexdigest()

        cmd = [str(self.binary), "--vsd", vsd_arg, "-c", effective_config, "--server", "-p", str(self.port)]
        if self.log_path is not None:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            log_file = self.log_path.open("a")
            try:
                self._process = subprocess.Popen(
                    cmd,
                    cwd=self.repo_root,
                    stdin=subprocess.DEVNULL,
                    stdout=log_file,
                    stderr=log_file,
                    start_new_session=self.own_process_group,
                )
            finally:
                log_file.close()
        else:
            self._process = subprocess.Popen(
                cmd,
                cwd=self.repo_root,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=self.own_process_group,
            )

        try:
            self._wait_ready(timeout_s=ready_timeout_s)
        except Exception:
            self.close()
            raise
        return self

    def _wait_ready(self, *, timeout_s: float = 30.0) -> None:
        deadline = time.monotonic() + timeout_s
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            if self._process is not None and self._process.poll() is not None:
                raise RuntimeError(
                    f"rrf_simulator exited with code {self._process.returncode}"
                    + (f"; see {self.log_path}" if self.log_path else "")
                )
            try:
                reply = self.send_gcode("M115", timeout_s=2.0)
                if "RepRapFirmware" in reply.reply or reply.reply.strip():
                    return
            except (OSError, URLError, RuntimeError) as err:
                last_error = err
            time.sleep(0.05)
        raise RuntimeError(
            f"rrf_simulator at {self._url} did not become ready within {timeout_s}s ({last_error})"
        )

    def send_gcode(self, line: str, *, timeout_s: float = 30.0) -> FirmwareReply:
        req = Request(
            f"{self._url}/machine/code",
            data=line.encode("utf-8"),
            headers={"Content-Type": "text/plain"},
            method="POST",
        )
        with urlopen(req, timeout=timeout_s) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
        if "\n---MOTION---\n" in raw:
            reply_text, motion_blob = raw.split("\n---MOTION---\n", 1)
            motion_lines = tuple(l for l in motion_blob.splitlines() if l.strip())
        else:
            reply_text = raw
            motion_lines = ()
        return FirmwareReply(gcode=line, reply=reply_text.strip(), motion_lines=motion_lines)

    def close(self) -> None:
        stop_process_tree(self._process)
        self._process = None
        if self._vsd_temp is not None:
            self._vsd_temp.cleanup()
            self._vsd_temp = None

    def __enter__(self) -> "RrfFirmwareService":
        return self.start()

    def __exit__(self, *_exc: object) -> None:
        self.close()
