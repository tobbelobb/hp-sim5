"""Tests for hp_sim5_sim: clean interfaces, local/Drive artifact storage, runner, and Colab scheduler."""

from __future__ import annotations

import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tarfile
import threading
from urllib.parse import parse_qs, urlparse

import pytest

from hp_sim5_sim import (
    ArtifactStore,
    ColabConfig,
    ColabScheduler,
    ColabSession,
    DriveMountArtifactStore,
    ExecutionScheduler,
    ExperimentResult,
    ExperimentSpec,
    FirmwareService,
    GoogleDriveArtifactStore,
    LocalArtifactStore,
    LocalScheduler,
    ObservationBatch,
    RrfFirmwareService,
    ScratchWorkspace,
    WorldStepRequest,
    resolve_colab_cli,
    store_from_uri,
)
from hp_sim5_sim.colab import bundle_workspace
from hp_sim5_sim.storage import (
    ArtifactIntegrityError,
    DriveAuthError,
    OAuthRefreshTokenProvider,
    compute_file_digests,
)

ROOT = Path(__file__).resolve().parents[2]


# ---------------------------------------------------------------------------
# Interfaces & contracts
# ---------------------------------------------------------------------------


def test_experiment_spec_and_result_roundtrip():
    spec = ExperimentSpec(
        kind="autocal",
        machine="hangprinter_4",
        experiment_id="exp-001",
        args=["--max-steps", "5"],
        collector_args=["--line-layering"],
        recording_mode="events",
        solver="native",
        tags=["bench", "hp4"],
        metadata={"note": "baseline"},
    )
    restored = ExperimentSpec.from_dict(spec.to_dict())
    assert restored == spec
    with pytest.raises(ValueError, match="Unknown ExperimentSpec"):
        ExperimentSpec.from_dict({"kind": "autocal", "bogus": 1})

    res = ExperimentResult(
        experiment_id="exp-001",
        kind="autocal",
        status="ok",
        exit_code=0,
        backend="colab-cpu-8c",
        wall_s=12.5,
        simulated_s=300.0,
        total_steps=150000,
        metrics={"accuracy_mm": 1.2},
    )
    assert ExperimentResult.from_dict(res.to_dict()) == res


def test_protocol_conformance_and_gpu_batch_dataclasses(tmp_path):
    assert isinstance(LocalArtifactStore(tmp_path), ArtifactStore)
    assert isinstance(DriveMountArtifactStore(tmp_path), ArtifactStore)
    assert isinstance(
        GoogleDriveArtifactStore(token_provider=lambda: "tok", root_folder_id="root"),
        ArtifactStore,
    )
    assert isinstance(LocalScheduler(ROOT), ExecutionScheduler)
    assert isinstance(ColabScheduler(ROOT), ExecutionScheduler)
    assert isinstance(RrfFirmwareService(ROOT, "sys/config_hp3.g"), FirmwareService)

    req = WorldStepRequest(world_id=3, steps=50, commands=({"type": "Move", "A": 1.0},))
    batch = ObservationBatch(
        world_ids=(0, 3),
        steps=(100, 150),
        sim_times_s=(0.2, 0.3),
        queue_lengths=(0, 5),
        active_mask=(True, True),
    )
    assert req.world_id == 3 and batch.world_ids == (0, 3)


# ---------------------------------------------------------------------------
# ScratchWorkspace & LocalArtifactStore
# ---------------------------------------------------------------------------


def test_scratch_workspace_cleanup_and_keep(tmp_path):
    with ScratchWorkspace("exp-clean", base_dir=tmp_path, keep=False) as ws:
        assert ws.path is not None and ws.path.is_dir()
        (ws.path / "temp.txt").write_text("active")
        cleaned_path = ws.path
    assert not cleaned_path.exists()

    with ScratchWorkspace("exp-keep", base_dir=tmp_path, keep=True) as ws:
        assert ws.path is not None
        (ws.path / "kept.txt").write_text("saved")
        kept_path = ws.path
    assert (kept_path / "kept.txt").read_text() == "saved"


def test_local_artifact_store_roundtrip_dedupe_and_integrity(tmp_path):
    assert store_from_uri("local://output/my-store").root == (Path.cwd() / "output/my-store").resolve()
    store = store_from_uri(f"local://{tmp_path / 'store'}")
    assert isinstance(store, LocalArtifactStore)

    src = tmp_path / "run_dir"
    (src / "sub").mkdir(parents=True)
    (src / "sweeps.json").write_text('{"sweeps": []}\n')
    (src / "sub/recording.rrd").write_bytes(b"RRD-MOCK-DATA" * 1000)
    (src / "ignore.tmp").write_text("transient")

    manifest = store.put_directory(
        "exp-a",
        src,
        metadata={"kind": "autocal", "machine": "hangprinter_3", "status": "ok", "tags": ["nightly", "hp3"]},
    )
    assert manifest["file_count"] == 2
    assert store.exists("exp-a")
    assert store.exists("exp-a", "sub/recording.rrd")
    assert not store.exists("exp-a", "ignore.tmp")

    # Listing & filtering
    listed = store.list_experiments(tags=["hp3"], kind="autocal")
    assert len(listed) == 1 and listed[0]["experiment_id"] == "exp-a"
    assert store.list_experiments(tags=["hp4"]) == []

    # Retrieval
    out = tmp_path / "restored"
    store.get_experiment("exp-a", out)
    assert (out / "sweeps.json").read_text() == '{"sweeps": []}\n'
    assert (out / "sub/recording.rrd").read_bytes() == b"RRD-MOCK-DATA" * 1000

    # Integrity check detects corruption
    corrupt_target = tmp_path / "store/experiments/exp-a/sweeps.json"
    corrupt_target.write_text('{"sweeps": ["corrupted"]}\n')
    with pytest.raises(ArtifactIntegrityError):
        store.get_file("exp-a", "sweeps.json", tmp_path / "should_fail.json")


# ---------------------------------------------------------------------------
# Fake Google Drive v3 Server for real HTTP resumable upload/resume/verify tests
# ---------------------------------------------------------------------------


class _FakeDriveState:
    def __init__(self, *, fail_once_on_chunk_offset: int | None = None, deny_scope: bool = False):
        self.lock = threading.Lock()
        self.next_id = 1
        self.files: dict[str, dict] = {}
        self.blobs: dict[str, bytes] = {}
        self.sessions: dict[str, dict] = {}
        self.fail_once_on_chunk_offset = fail_once_on_chunk_offset
        self.failed_offsets: set[int] = set()
        self.resumable_inits = 0
        self.deny_scope = deny_scope
        self.corrupt_file_name: str | None = None

    def alloc_id(self) -> str:
        fid = f"fid_{self.next_id}"
        self.next_id += 1
        return fid


def _start_fake_drive_server(state: _FakeDriveState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def _send_json(self, code: int, payload: dict, extra_headers: dict | None = None):
            body = json.dumps(payload).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            for k, v in (extra_headers or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _check_auth(self) -> bool:
            if state.deny_scope:
                self._send_json(
                    403,
                    {"error": {"code": 403, "message": "ACCESS_TOKEN_SCOPE_INSUFFICIENT"}},
                )
                return False
            if self.headers.get("Authorization") != "Bearer test-token":
                self._send_json(401, {"error": {"code": 401, "message": "Unauthorized"}})
                return False
            return True

        def do_GET(self):
            if not self._check_auth():
                return
            parsed = urlparse(self.path)
            qs = parse_qs(parsed.query)
            if parsed.path == "/drive/v3/files":
                q = qs.get("q", [""])[0]
                with state.lock:
                    matches = []
                    for meta in state.files.values():
                        ok = True
                        if f"'{meta['parents'][0]}' in parents" not in q:
                            ok = False
                        if f"name = '{meta['name']}'" not in q:
                            ok = False
                        if "mimeType = " in q and f"mimeType = '{meta['mimeType']}'" not in q:
                            ok = False
                        if ok:
                            matches.append(meta)
                self._send_json(200, {"files": matches})
                return
            if parsed.path.startswith("/drive/v3/files/") and qs.get("alt") == ["media"]:
                fid = parsed.path.rsplit("/", 1)[1]
                with state.lock:
                    blob = state.blobs.get(fid)
                    fname = state.files.get(fid, {}).get("name")
                    if blob is not None and state.corrupt_file_name and fname == state.corrupt_file_name:
                        state.corrupt_file_name = None
                        blob = b"CORRUPTED" + blob[9:]
                if blob is None:
                    self._send_json(404, {"error": "not found"})
                    return
                self.send_response(200)
                self.send_header("Content-Length", str(len(blob)))
                self.end_headers()
                self.wfile.write(blob)
                return
            self._send_json(404, {"error": f"unknown GET {parsed.path}"})

        def do_POST(self):
            if not self._check_auth():
                return
            parsed = urlparse(self.path)
            qs = parse_qs(parsed.query)
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length) if length else b""
            if parsed.path == "/drive/v3/files":
                meta = json.loads(raw.decode("utf-8"))
                with state.lock:
                    fid = state.alloc_id()
                    rec = {
                        "id": fid,
                        "name": meta["name"],
                        "mimeType": meta["mimeType"],
                        "parents": meta.get("parents", ["root"]),
                        "appProperties": meta.get("appProperties", {}),
                    }
                    state.files[fid] = rec
                self._send_json(200, rec)
                return
            if parsed.path == "/upload/drive/v3/files" and qs.get("uploadType") == ["resumable"]:
                meta = json.loads(raw.decode("utf-8"))
                total = int(self.headers.get("X-Upload-Content-Length", "0"))
                with state.lock:
                    state.resumable_inits += 1
                    sid = f"sess_{state.resumable_inits}"
                    state.sessions[sid] = {
                        "file_id": None,
                        "meta": meta,
                        "total": total,
                        "buf": bytearray(),
                    }
                loc = f"http://127.0.0.1:{self.server.server_port}/upload/session/{sid}"
                self.send_response(200)
                self.send_header("Location", loc)
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            self._send_json(404, {"error": f"unknown POST {parsed.path}"})

        def do_PATCH(self):
            if not self._check_auth():
                return
            parsed = urlparse(self.path)
            qs = parse_qs(parsed.query)
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length) if length else b""
            if parsed.path.startswith("/drive/v3/files/"):
                fid = parsed.path.rsplit("/", 1)[1]
                patch = json.loads(raw.decode("utf-8"))
                with state.lock:
                    state.files[fid].update(patch)
                    rec = dict(state.files[fid])
                self._send_json(200, rec)
                return
            if parsed.path.startswith("/upload/drive/v3/files/") and qs.get("uploadType") == ["resumable"]:
                fid = parsed.path.rsplit("/", 1)[1]
                meta = json.loads(raw.decode("utf-8"))
                total = int(self.headers.get("X-Upload-Content-Length", "0"))
                with state.lock:
                    state.resumable_inits += 1
                    sid = f"sess_{state.resumable_inits}"
                    state.sessions[sid] = {
                        "file_id": fid,
                        "meta": meta,
                        "total": total,
                        "buf": bytearray(),
                    }
                loc = f"http://127.0.0.1:{self.server.server_port}/upload/session/{sid}"
                self.send_response(200)
                self.send_header("Location", loc)
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            self._send_json(404, {"error": f"unknown PATCH {parsed.path}"})

        def do_PUT(self):
            if not self._check_auth():
                return
            parsed = urlparse(self.path)
            if not parsed.path.startswith("/upload/session/"):
                self._send_json(404, {"error": "unknown session"})
                return
            sid = parsed.path.rsplit("/", 1)[1]
            length = int(self.headers.get("Content-Length", "0"))
            chunk = self.rfile.read(length) if length else b""
            cr = self.headers.get("Content-Range", "")
            with state.lock:
                sess = state.sessions[sid]
                if cr.startswith("bytes */"):
                    committed = len(sess["buf"])
                    self.send_response(308)
                    if committed > 0:
                        self.send_header("Range", f"bytes=0-{committed - 1}")
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
                if cr.startswith("bytes "):
                    span, _total = cr.removeprefix("bytes ").split("/")
                    start, _end = map(int, span.split("-"))
                else:
                    start = 0
                if (
                    state.fail_once_on_chunk_offset is not None
                    and start == state.fail_once_on_chunk_offset
                    and start not in state.failed_offsets
                ):
                    state.failed_offsets.add(start)
                    self._send_json(503, {"error": "simulated transient chunk error"})
                    return
                if start == len(sess["buf"]):
                    sess["buf"].extend(chunk)
                if len(sess["buf"]) < sess["total"]:
                    self.send_response(308)
                    self.send_header("Range", f"bytes=0-{len(sess['buf']) - 1}")
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
                data = bytes(sess["buf"])
                fid = sess["file_id"] or state.alloc_id()
                parents = sess["meta"].get("parents") or state.files.get(fid, {}).get("parents", ["root"])
                rec = {
                    "id": fid,
                    "name": sess["meta"]["name"],
                    "mimeType": self.headers.get("Content-Type", "application/octet-stream"),
                    "parents": parents,
                    "size": str(len(data)),
                    "md5Checksum": hashlib.md5(data).hexdigest(),
                    "sha256Checksum": hashlib.sha256(data).hexdigest(),
                    "appProperties": sess["meta"].get("appProperties", {}),
                }
                state.files[fid] = rec
                state.blobs[fid] = data
            self._send_json(200, rec)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    return server, base


def test_google_drive_store_chunked_resumable_retry_dedupe_and_verify(tmp_path):
    chunk_size = 256 * 1024
    # Simulate a transient 503 on the second chunk (offset 256 KiB) to test 308 resume recovery!
    state = _FakeDriveState(fail_once_on_chunk_offset=chunk_size)
    server, base = _start_fake_drive_server(state)
    try:
        store = GoogleDriveArtifactStore(
            root_folder="hp-sim5-test/runs",
            token_provider=lambda: "test-token",
            api_base=f"{base}/drive/v3",
            upload_base=f"{base}/upload/drive/v3",
            chunk_bytes=chunk_size,
        )
        run_dir = tmp_path / "scratch"
        (run_dir / "nested").mkdir(parents=True)
        # 650 KiB binary payload -> 3 chunks (256K + 256K [503->308 resume] + 138K)
        big_bytes = os.urandom(650 * 1024)
        (run_dir / "nested/recording.rrd").write_bytes(big_bytes)
        (run_dir / "sweeps.json").write_text('{"sweeps":[1,2,3]}\n')

        manifest = store.put_directory(
            "drv-exp-1",
            run_dir,
            metadata={"kind": "autocal", "machine": "hangprinter_4", "status": "ok", "tags": ["drive", "hp4"]},
        )
        assert manifest["file_count"] == 2
        assert chunk_size in state.failed_offsets  # proved transient failure happened and resumed!
        assert store.exists("drv-exp-1")
        assert store.exists("drv-exp-1", "nested/recording.rrd")

        # Deduplication: uploading the identical file again skips resumable upload!
        inits_before = state.resumable_inits
        rec = store.put_file("drv-exp-1", "nested/recording.rrd", run_dir / "nested/recording.rrd")
        assert state.resumable_inits == inits_before
        assert rec.size_bytes == len(big_bytes)

        # Listing & retrieval
        listed = store.list_experiments(tags=["hp4"], kind="autocal")
        assert len(listed) == 1 and listed[0]["experiment_id"] == "drv-exp-1"

        restored = tmp_path / "restored"
        store.get_experiment("drv-exp-1", restored)
        assert (restored / "nested/recording.rrd").read_bytes() == big_bytes
        assert (restored / "sweeps.json").read_text() == '{"sweeps":[1,2,3]}\n'

        # Download integrity failure detection
        state.corrupt_file_name = "recording.rrd"
        with pytest.raises(ArtifactIntegrityError):
            store.get_file("drv-exp-1", "nested/recording.rrd", tmp_path / "bad.rrd")
    finally:
        server.shutdown()


def test_google_drive_store_reports_clear_scope_error():
    state = _FakeDriveState(deny_scope=True)
    server, base = _start_fake_drive_server(state)
    try:
        store = GoogleDriveArtifactStore(
            root_folder="hp-sim5",
            token_provider=lambda: "test-token",
            api_base=f"{base}/drive/v3",
            upload_base=f"{base}/upload/drive/v3",
        )
        with pytest.raises(DriveAuthError, match="insufficient scopes"):
            store.list_experiments()
    finally:
        server.shutdown()


def test_oauth_refresh_token_provider_caches_and_refreshes():
    calls = {"count": 0}

    class TokenHandler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def do_POST(self):
            calls["count"] += 1
            body = json.dumps({"access_token": f"tok-{calls['count']}", "expires_in": 3600}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    srv = ThreadingHTTPServer(("127.0.0.1", 0), TokenHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        provider = OAuthRefreshTokenProvider(
            client_id="cid",
            client_secret="csec",
            refresh_token="rtok",
            token_url=f"http://127.0.0.1:{srv.server_port}/token",
        )
        assert provider() == "tok-1"
        assert provider() == "tok-1"
        assert calls["count"] == 1
    finally:
        srv.shutdown()


# ---------------------------------------------------------------------------
# LocalScheduler & CLI
# ---------------------------------------------------------------------------


def test_local_scheduler_parallel_batch_and_store_sync(tmp_path):
    store = LocalArtifactStore(tmp_path / "artifacts")
    scheduler = LocalScheduler(ROOT, scratch_base=tmp_path / "scratch")
    specs = [
        ExperimentSpec(
            experiment_id=f"cmd-{i}",
            kind="command",
            machine="hangprinter_3",
            tags=["unit-batch"],
            command=[
                "{python}",
                "-c",
                f"from pathlib import Path; (Path('{{scratch}}') / 'out.txt').write_text('val={i}\\n')",
            ],
        )
        for i in range(3)
    ]
    results = scheduler.run_batch(specs, max_workers=2, store=store)
    assert [r.status for r in results] == ["ok", "ok", "ok"]
    assert all(r.artifact_uri is not None for r in results)
    for i in range(3):
        dest = tmp_path / f"check-{i}.txt"
        store.get_file(f"cmd-{i}", "out.txt", dest)
        assert dest.read_text() == f"val={i}\n"


def test_cli_artifacts_roundtrip(tmp_path):
    src = tmp_path / "exp_src"
    src.mkdir()
    (src / "data.json").write_text('{"x": 42}\n')
    store_uri = f"local://{tmp_path / 'cli_store'}"

    p_push = subprocess.run(
        [sys.executable, "scripts/hp_sim5_run.py", "artifacts", "--store", store_uri, "push", "cli-1", str(src), "--tag", "demo"],
        cwd=ROOT, capture_output=True, text=True, check=True,
    )
    assert json.loads(p_push.stdout)["experiment_id"] == "cli-1"

    p_ls = subprocess.run(
        [sys.executable, "scripts/hp_sim5_run.py", "artifacts", "--store", store_uri, "list", "--tag", "demo"],
        cwd=ROOT, capture_output=True, text=True, check=True,
    )
    rows = json.loads(p_ls.stdout)
    assert len(rows) == 1 and rows[0]["experiment_id"] == "cli-1"

    out_dir = tmp_path / "cli_out"
    subprocess.run(
        [sys.executable, "scripts/hp_sim5_run.py", "artifacts", "--store", store_uri, "get", "cli-1", str(out_dir)],
        cwd=ROOT, capture_output=True, text=True, check=True,
    )
    assert (out_dir / "data.json").read_text() == '{"x": 42}\n'


# ---------------------------------------------------------------------------
# Colab CLI resolution, bundling, and scheduler lifecycle (stubbed CLI)
# ---------------------------------------------------------------------------


def test_resolve_colab_cli_wrapper_vs_plain(tmp_path):
    wrapper = tmp_path / "colab_wrapped"
    wrapper.write_text("#!/bin/sh\nexec /usr/bin/colab --auth=adc \"$@\"\n")
    wrapper.chmod(wrapper.stat().st_mode | stat.S_IEXEC)
    assert resolve_colab_cli(wrapper) == [str(wrapper)]

    plain = tmp_path / "colab_plain"
    plain.write_text("#!/bin/sh\necho ok\n")
    plain.chmod(plain.stat().st_mode | stat.S_IEXEC)
    assert resolve_colab_cli(plain) == [str(plain), "--auth=adc"]


def test_bundle_workspace_excludes_heavy_artifacts(tmp_path):
    tar_path = tmp_path / "bundle.tar.gz"
    info = bundle_workspace(ROOT, tar_path, include_portable_native=False)
    assert info["file_count"] > 50
    with tarfile.open(tar_path, "r:gz") as tar:
        names = set(tar.getnames())
    assert "autocal/autocal.py" in names
    assert "scripts/autocal_headless.mjs" in names
    assert "RRF/CMakeLists.txt" in names
    assert "public/wasm/cable_solver.wasm" in names
    assert not any(n.startswith("node_modules/") or "/.git/" in n or n.startswith("autocal/data/references/") for n in names)


def test_colab_scheduler_lifecycle_and_guaranteed_stop(tmp_path):
    """Verify ColabScheduler invokes new -> upload -> exec -> download -> stop and stores artifacts."""
    log_file = tmp_path / "colab_calls.jsonl"
    fake_remote = tmp_path / "fake_remote"
    fake_remote.mkdir()

    stub = tmp_path / "fake_colab"
    stub.write_text(
        f"""#!{sys.executable}
import json, shutil, sys, tarfile
from pathlib import Path

log_file = Path({str(log_file)!r})
remote = Path({str(fake_remote)!r})
argv = sys.argv[1:]
with log_file.open("a") as fh:
    fh.write(json.dumps(argv) + "\\n")

# Strip global options (--auth=..., --config PATH)
i = 0
while i < len(argv) and argv[i].startswith("-"):
    if argv[i] == "--config":
        i += 2
    else:
        i += 1
cmd = argv[i]
rest = argv[i + 1:]

if cmd == "upload":
    # upload -s NAME LOCAL REMOTE
    local_p = Path(rest[2])
    remote_name = Path(rest[3]).name
    shutil.copy2(local_p, remote / remote_name)
elif cmd == "exec":
    # On batch script execution, synthesize a results archive.
    script_path = Path(rest[rest.index("-f") + 1])
    if script_path.name == "run_batch.py":
        specs = json.loads((remote / "hp_sim5_specs.json").read_text())
        out_dir = remote / "out"
        out_dir.mkdir(exist_ok=True)
        results = []
        for s in specs:
            exp_dir = out_dir / "store/experiments" / s["experiment_id"]
            exp_dir.mkdir(parents=True, exist_ok=True)
            (exp_dir / "sweeps.json").write_text(json.dumps({{"sweeps": [], "id": s["experiment_id"]}}) + "\\n")
            results.append({{
                "experiment_id": s["experiment_id"], "kind": s["kind"], "status": "ok",
                "exit_code": 0, "backend": "colab-cpu-8c", "wall_s": 1.25, "metrics": {{}},
                "manifest": {{"metadata": {{"machine": s["machine"], "tags": s["tags"]}}}},
            }})
        (out_dir / "batch_summary.json").write_text(json.dumps({{"results": results}}))
        with tarfile.open(remote / "hp_sim5_results.tar.gz", "w:gz") as tar:
            for p in sorted(out_dir.rglob("*")):
                if p.is_file():
                    tar.add(p, arcname=p.relative_to(out_dir).as_posix())
elif cmd == "download":
    remote_name = Path(rest[2]).name
    local_p = Path(rest[3])
    shutil.copy2(remote / remote_name, local_p)
"""
    )
    stub.chmod(stub.stat().st_mode | stat.S_IEXEC)

    store = LocalArtifactStore(tmp_path / "store")
    cfg = ColabConfig(colab_bin=str(stub), high_mem=True, build_portable_native=False)
    scheduler = ColabScheduler(ROOT, config=cfg)
    specs = [
        ExperimentSpec(experiment_id="c-1", kind="autocal", machine="hangprinter_3", solver="wasm", tags=["colab"]),
        ExperimentSpec(experiment_id="c-2", kind="autocal", machine="hangprinter_4", solver="wasm", tags=["colab"]),
    ]
    results = scheduler.run_batch(specs, max_workers=2, store=store)
    assert [r.status for r in results] == ["ok", "ok"]
    assert all(store.exists(r.experiment_id, "sweeps.json") for r in results)

    calls = [json.loads(line) for line in log_file.read_text().splitlines()]
    subcommands = []
    for argv in calls:
        idx = 0
        while idx < len(argv) and argv[idx].startswith("-"):
            idx += 2 if argv[idx] == "--config" else 1
        subcommands.append(argv[idx])
    assert subcommands[0] == "new"
    assert subcommands[-1] == "stop"
    assert "upload" in subcommands and "exec" in subcommands and "download" in subcommands


@pytest.mark.slow
def test_rrf_firmware_service_lifecycle(tmp_path):
    with RrfFirmwareService(ROOT, "sys/config_hp3.g", log_path=tmp_path / "rrf.log", isolate_vsd=True) as rrf:
        reply = rrf.send_gcode("M115")
        assert "RepRapFirmware" in reply.reply
        pid = rrf.pid
        assert pid is not None
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)
