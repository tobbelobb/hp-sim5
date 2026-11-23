"""Tests for hp_sim5_sim: clean interfaces, local/Drive artifact storage, runner, and Colab scheduler."""

from __future__ import annotations

import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
import tarfile
import threading
from urllib.parse import parse_qs, urlparse

import pytest

from hp_sim5_sim import (
    ArtifactStore,
    ArtifactUploadError,
    ColabConfig,
    ColabScheduler,
    ColabSession,
    ColabShutdownError,
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
    UPLOAD_PENDING_NAME,
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
                        if " in parents" in q and f"'{meta['parents'][0]}' in parents" not in q:
                            ok = False
                        if "name = " in q and f"name = '{meta['name']}'" not in q:
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
                    if "appProperties" in patch:
                        merged = dict(state.files[fid].get("appProperties") or {})
                        merged.update(patch["appProperties"])
                        state.files[fid]["appProperties"] = merged
                        patch = {k: v for k, v in patch.items() if k != "appProperties"}
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


def test_local_scheduler_excludes_bulky_events_only_on_ok_non_recording_runs(tmp_path):
    store = LocalArtifactStore(tmp_path / "artifacts")
    scheduler = LocalScheduler(ROOT, scratch_base=tmp_path / "scratch")

    def _writer_cmd(exit_code: int = 0) -> list[str]:
        return [
            "{python}",
            "-c",
            (
                "import sys; from pathlib import Path; sc = Path('{scratch}'); "
                "(sc / 'sweeps.headless').mkdir(); (sc / '.matplotlib').mkdir(); "
                "(sc / 'sweeps.json').write_text('{{\"sweeps\": []}}\\n'); "
                "(sc / 'sweeps.headless/rrf.log').write_text('rrf ok\\n'); "
                "(sc / 'sweeps.headless/events.jsonl').write_text('bulky event trace\\n'); "
                "(sc / '.matplotlib/fontlist.json').write_text('cache'); "
                f"sys.exit({exit_code})"
            ),
        ]

    # 1. Successful recording_mode="none": skips events.jsonl and .matplotlib/*, keeps sweeps.json & rrf.log.
    res_ok = scheduler.run(
        ExperimentSpec(experiment_id="lean-ok", kind="command", recording_mode="none", command=_writer_cmd(0)),
        store=store,
    )
    assert res_ok.status == "ok"
    assert store.exists("lean-ok", "sweeps.json")
    assert store.exists("lean-ok", "sweeps.headless/rrf.log")
    assert not store.exists("lean-ok", "sweeps.headless/events.jsonl")
    assert not store.exists("lean-ok", ".matplotlib/fontlist.json")

    # 2. Failed run: retains events.jsonl for post-mortem debugging!
    res_fail = scheduler.run(
        ExperimentSpec(experiment_id="lean-fail", kind="command", recording_mode="none", command=_writer_cmd(1)),
        store=store,
    )
    assert res_fail.status == "failed"
    assert store.exists("lean-fail", "sweeps.headless/events.jsonl")
    assert not store.exists("lean-fail", ".matplotlib/fontlist.json")

    # 3. Recording enabled (recording_mode="events"): retains events.jsonl!
    res_rec = scheduler.run(
        ExperimentSpec(experiment_id="lean-rec", kind="command", recording_mode="events", command=_writer_cmd(0)),
        store=store,
    )
    assert res_rec.status == "ok"
    assert store.exists("lean-rec", "sweeps.headless/events.jsonl")

    # 4. Explicit metadata opt-in (keep_events_log=True): retains events.jsonl!
    res_keep = scheduler.run(
        ExperimentSpec(
            experiment_id="lean-keep",
            kind="command",
            recording_mode="none",
            metadata={"keep_events_log": True},
            command=_writer_cmd(0),
        ),
        store=store,
    )
    assert res_keep.status == "ok"
    assert store.exists("lean-keep", "sweeps.headless/events.jsonl")


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


# ---------------------------------------------------------------------------
# Review-fix regression tests: persistence, upload recovery, concurrency,
# verified Colab shutdown, and direct Colab -> Google Drive upload
# ---------------------------------------------------------------------------


def _make_stub_colab(
    tmp_path: Path,
    *,
    log_file: Path,
    fake_remote: Path,
    stop_mode: str = "ok",
    direct_mode: str = "ok",
    exec_mode: str = "ok",
) -> Path:
    """Create an executable stub `colab` CLI for deterministic lifecycle tests."""
    stub = tmp_path / f"fake_colab_{stop_mode}_{direct_mode}_{exec_mode}"
    stub.write_text(
        f"""#!{sys.executable}
import json, shutil, sys, tarfile
from pathlib import Path

log_file = Path({str(log_file)!r})
remote = Path({str(fake_remote)!r})
stop_mode = {stop_mode!r}
direct_mode = {direct_mode!r}
exec_mode = {exec_mode!r}
argv = sys.argv[1:]
with log_file.open("a") as fh:
    fh.write(json.dumps(argv) + "\\n")

config_path = None
i = 0
while i < len(argv) and argv[i].startswith("-"):
    if argv[i] == "--config":
        config_path = Path(argv[i + 1])
        i += 2
    else:
        i += 1
cmd = argv[i]
rest = argv[i + 1:]

if cmd == "new":
    session_name = rest[rest.index("-s") + 1]
    if config_path is not None:
        config_path.parent.mkdir(parents=True, exist_ok=True)
        config_path.write_text(json.dumps({{"sessions": [session_name]}}))
elif cmd == "upload":
    local_p = Path(rest[2])
    remote_name = Path(rest[3]).name
    shutil.copy2(local_p, remote / remote_name)
elif cmd == "exec":
    script_path = Path(rest[rest.index("-f") + 1])
    shutil.copy2(script_path, remote / script_path.name)
    if script_path.name == "run_batch.py":
        if exec_mode == "fail_mid_batch":
            print("simulated mid-batch disconnect", file=sys.stderr)
            sys.exit(1)
        script_text = script_path.read_text()
        specs = json.loads((remote / "hp_sim5_specs.json").read_text())
        out_dir = remote / "out"
        if out_dir.exists():
            shutil.rmtree(out_dir)
        out_dir.mkdir(parents=True)
        results = []
        is_direct = "direct_drive_cfg = {{" in script_text
        for s in specs:
            if is_direct and direct_mode == "ok":
                results.append({{
                    "experiment_id": s["experiment_id"], "kind": s["kind"], "status": "ok",
                    "exit_code": 0, "backend": "colab-cpu-8c", "wall_s": 1.0, "metrics": {{}},
                    "artifact_uri": f"gdrive://hp-sim5-direct/experiments/{{s['experiment_id']}}",
                    "manifest": {{"metadata": {{"machine": s["machine"], "tags": s["tags"]}}}},
                }})
            elif is_direct and direct_mode == "rescue":
                rdir = out_dir / "rescue" / s["experiment_id"]
                rdir.mkdir(parents=True, exist_ok=True)
                (rdir / "sweeps.json").write_text(json.dumps({{"rescued": s["experiment_id"]}}) + "\\n")
                (rdir / "upload_pending.json").write_text(json.dumps({{
                    "experiment_id": s["experiment_id"],
                    "metadata": {{"kind": s["kind"], "machine": s["machine"], "tags": s["tags"], "status": "ok"}},
                }}))
                results.append({{
                    "experiment_id": s["experiment_id"], "kind": s["kind"], "status": "upload_failed",
                    "exit_code": 0, "backend": "colab-cpu-8c", "wall_s": 1.0,
                    "metrics": {{"experiment_status": "ok", "upload_error": "simulated Drive 503 on Colab"}},
                    "error": "Artifact upload failed: simulated Drive 503 on Colab",
                }})
            else:
                exp_dir = out_dir / "store/experiments" / s["experiment_id"]
                exp_dir.mkdir(parents=True, exist_ok=True)
                (exp_dir / "sweeps.json").write_text(json.dumps({{"sweeps": [], "id": s["experiment_id"]}}) + "\\n")
                # Intentionally emit a remote /content URI to verify ColabScheduler scrubs it.
                results.append({{
                    "experiment_id": s["experiment_id"], "kind": s["kind"], "status": "ok",
                    "exit_code": 0, "backend": "colab-cpu-8c", "wall_s": 1.25, "metrics": {{}},
                    "artifact_uri": f"local:///content/hp_sim5_stage_store/experiments/{{s['experiment_id']}}",
                    "manifest": {{"store_uri": "local:///content/hp_sim5_stage_store", "metadata": {{"machine": s["machine"], "tags": s["tags"]}}}},
                }})
        (out_dir / "batch_summary.json").write_text(json.dumps({{"results": results}}))
        with tarfile.open(remote / "hp_sim5_results.tar.gz", "w:gz") as tar:
            for p in sorted(out_dir.rglob("*")):
                if p.is_file():
                    tar.add(p, arcname=p.relative_to(out_dir).as_posix())
    elif script_path.name == "rescue.py":
        # Synthesize a remote rescue archive containing 1 completed staged experiment + 1 partial scratch dir.
        resc_dir = remote / "rescue_stage"
        if resc_dir.exists():
            shutil.rmtree(resc_dir)
        done_exp = resc_dir / "store/experiments/exp-done-1"
        done_exp.mkdir(parents=True)
        (done_exp / "sweeps.json").write_text(json.dumps({{"recovered": True}}) + "\\n")
        (done_exp / "experiment_manifest.json").write_text(json.dumps({{
            "experiment_id": "exp-done-1",
            "metadata": {{"kind": "autocal", "machine": "hangprinter_3", "status": "ok", "tags": ["recovered"]}},
        }}))
        wip_dir = resc_dir / "partial_scratch/exp-wip-2-abc"
        wip_dir.mkdir(parents=True)
        (wip_dir / "stdout.log").write_text("partial progress log\\n")
        with tarfile.open(remote / "hp_sim5_rescue.tar.gz", "w:gz") as tar:
            for p in sorted(resc_dir.rglob("*")):
                if p.is_file():
                    tar.add(p, arcname=p.relative_to(resc_dir).as_posix())
elif cmd == "download":
    remote_name = Path(rest[2]).name
    local_p = Path(rest[3])
    shutil.copy2(remote / remote_name, local_p)
elif cmd == "stop":
    if stop_mode in ("fail_still_running", "fail_already_gone"):
        print("simulated colab stop error", file=sys.stderr)
        sys.exit(1)
elif cmd == "sessions":
    if stop_mode == "fail_still_running" and config_path and config_path.is_file():
        data = json.loads(config_path.read_text())
        for s in data.get("sessions", []):
            print(s)
    # fail_already_gone prints nothing (session already pruned)
"""
    )
    stub.chmod(stub.stat().st_mode | stat.S_IEXEC)
    return stub


def test_colab_defaults_to_persistent_local_store_and_never_returns_content_uri(tmp_path):
    """Finding 1: Colab without --store must persist locally and never return a dead /content URI."""
    log_file = tmp_path / "calls.jsonl"
    fake_remote = tmp_path / "remote"
    fake_remote.mkdir()
    stub = _make_stub_colab(tmp_path, log_file=log_file, fake_remote=fake_remote)

    default_store_dir = tmp_path / "default_experiments"
    cfg = ColabConfig(
        colab_bin=str(stub),
        build_portable_native=False,
        default_local_store_dir=default_store_dir,
    )
    scheduler = ColabScheduler(ROOT, config=cfg)
    spec = ExperimentSpec(experiment_id="colab-default-1", kind="autocal", machine="hangprinter_3", solver="wasm")

    # store=None, keep_scratch=False -> persisted in default_store_dir!
    res = scheduler.run(spec, store=None, keep_scratch=False)
    assert res.status == "ok"
    assert res.artifact_uri is not None
    assert "/content/" not in res.artifact_uri
    assert str(default_store_dir.resolve()) in res.artifact_uri
    default_store = LocalArtifactStore(default_store_dir)
    assert default_store.exists("colab-default-1", "sweeps.json")

    # store=None, keep_scratch=True -> scratch kept locally, artifact_uri is None (never /content/...)
    spec2 = ExperimentSpec(experiment_id="colab-scratch-2", kind="autocal", machine="hangprinter_3", solver="wasm")
    res2 = scheduler.run(spec2, store=None, keep_scratch=True)
    assert res2.artifact_uri is None
    assert res2.manifest is None or res2.manifest.get("store_uri") is None
    assert res2.scratch_dir is not None and (Path(res2.scratch_dir) / "sweeps.json").is_file()


def test_failed_upload_preserves_scratch_and_supports_cli_retry(tmp_path):
    """Finding 2: Upload failures preserve scratch on both Local and Colab and can be retried via CLI."""

    class FailingStore(LocalArtifactStore):
        def put_directory(self, *args, **kwargs):
            raise RuntimeError("simulated Drive 503 outage")

    failing_store = FailingStore(tmp_path / "real_store")
    scheduler = LocalScheduler(ROOT, scratch_base=tmp_path / "scratch")
    spec = ExperimentSpec(
        experiment_id="retry-local-1",
        kind="command",
        machine="hangprinter_4",
        tags=["retry-test", "hp4"],
        command=[
            "{python}",
            "-c",
            "from pathlib import Path; (Path('{scratch}') / 'sweeps.json').write_text('{{\"ok\": true}}\\n')",
        ],
    )
    with pytest.raises(ArtifactUploadError) as exc_info:
        scheduler.run(spec, store=failing_store, keep_scratch=False)

    err = exc_info.value
    assert err.scratch_dir is not None
    preserved = Path(err.scratch_dir)
    assert preserved.is_dir()
    assert (preserved / "sweeps.json").read_text() == '{"ok": true}\n'
    assert (preserved / UPLOAD_PENDING_NAME).is_file()

    # Retry upload via CLI to the healthy LocalArtifactStore at the same path.
    store_uri = f"local://{tmp_path / 'real_store'}"
    p = subprocess.run(
        [sys.executable, "scripts/hp_sim5_run.py", "artifacts", "--store", store_uri, "push", "retry-local-1", str(preserved)],
        cwd=ROOT, capture_output=True, text=True, check=True,
    )
    manifest = json.loads(p.stdout)
    assert manifest["experiment_id"] == "retry-local-1"
    assert not (preserved / UPLOAD_PENDING_NAME).exists()

    healthy_store = LocalArtifactStore(tmp_path / "real_store")
    assert healthy_store.exists("retry-local-1", "sweeps.json")
    assert not healthy_store.exists("retry-local-1", UPLOAD_PENDING_NAME)
    listed = healthy_store.list_experiments(tags=["retry-test"], kind="command")
    assert len(listed) == 1 and listed[0]["machine"] == "hangprinter_4"

    # Also verify ColabScheduler preserves downloaded artifacts when store.put_directory fails!
    log_file = tmp_path / "colab_fail_calls.jsonl"
    fake_remote = tmp_path / "colab_fail_remote"
    fake_remote.mkdir()
    stub = _make_stub_colab(tmp_path, log_file=log_file, fake_remote=fake_remote)
    colab_sched = ColabScheduler(ROOT, config=ColabConfig(colab_bin=str(stub), build_portable_native=False))
    colab_spec = ExperimentSpec(experiment_id="retry-colab-1", kind="autocal", machine="hangprinter_3", solver="wasm")
    with pytest.raises(ArtifactUploadError) as colab_exc:
        colab_sched.run(colab_spec, store=failing_store, keep_scratch=False)
    colab_preserved = Path(colab_exc.value.scratch_dir or "")
    assert (colab_preserved / "sweeps.json").is_file()
    assert (colab_preserved / UPLOAD_PENDING_NAME).is_file()


def test_parallel_index_updates_local_and_drive_no_lost_entries(tmp_path):
    """Finding 3: Parallel uploads never overwrite each other's index entries (local & Drive)."""
    from concurrent.futures import ThreadPoolExecutor

    # 1. LocalArtifactStore: concurrent threads + concurrent subprocesses.
    local_store = LocalArtifactStore(tmp_path / "concurrent_local")

    def _push_local(idx: int) -> None:
        d = tmp_path / f"lsrc_{idx}"
        d.mkdir()
        (d / "data.txt").write_text(f"payload-{idx}\n")
        local_store.put_directory(f"local-exp-{idx:02d}", d, metadata={"kind": "autocal", "tags": ["par"]})

    with ThreadPoolExecutor(max_workers=8) as pool:
        list(pool.map(_push_local, range(12)))

    procs = []
    for idx in range(12, 18):
        d = tmp_path / f"lsrc_{idx}"
        d.mkdir()
        (d / "data.txt").write_text(f"payload-{idx}\n")
        procs.append(
            subprocess.Popen(
                [
                    sys.executable,
                    "scripts/hp_sim5_run.py",
                    "artifacts",
                    "--store",
                    f"local://{local_store.root}",
                    "push",
                    f"local-exp-{idx:02d}",
                    str(d),
                    "--kind",
                    "autocal",
                    "--tag",
                    "par",
                ],
                cwd=ROOT,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        )
    for p in procs:
        out, err = p.communicate(timeout=30)
        assert p.returncode == 0, f"subprocess push failed: {out}\n{err}"

    listed_local = local_store.list_experiments(tags=["par"], limit=100)
    assert len(listed_local) == 18
    assert {r["experiment_id"] for r in listed_local} == {f"local-exp-{i:02d}" for i in range(18)}

    # 2. GoogleDriveArtifactStore: lock-free per-experiment folder appProperties (no shared index.json).
    state = _FakeDriveState()
    server, base = _start_fake_drive_server(state)
    try:
        drive_store = GoogleDriveArtifactStore(
            root_folder="hp-sim5-par",
            token_provider=lambda: "test-token",
            api_base=f"{base}/drive/v3",
            upload_base=f"{base}/upload/drive/v3",
        )
        drive_store.ensure_experiments_folder()

        def _push_drive(idx: int) -> None:
            d = tmp_path / f"dsrc_{idx}"
            d.mkdir()
            (d / "data.txt").write_text(f"drive-{idx}\n")
            drive_store.put_directory(
                f"drv-exp-{idx:02d}",
                d,
                metadata={"kind": "autocal", "machine": "hangprinter_3", "status": "ok", "tags": ["par-drive"]},
            )

        with ThreadPoolExecutor(max_workers=8) as pool:
            list(pool.map(_push_drive, range(12)))

        listed_drive = drive_store.list_experiments(tags=["par-drive"], limit=100)
        assert len(listed_drive) == 12
        assert {r["experiment_id"] for r in listed_drive} == {f"drv-exp-{i:02d}" for i in range(12)}
        # Verify no shared index.json was written and only one `experiments` folder exists.
        assert not any(f["name"] == "index.json" for f in state.files.values())
        assert sum(1 for f in state.files.values() if f["name"] == "experiments") == 1
    finally:
        server.shutdown()


def test_colab_session_stop_failure_preserves_config_and_raises(tmp_path):
    """Finding 4: Failed `colab stop` raises ColabShutdownError, preserves sessions.json, and still persists results."""
    # Case A: stop fails and session is still listed -> raises ColabShutdownError & preserves sessions.json.
    log_a = tmp_path / "stop_fail_calls.jsonl"
    remote_a = tmp_path / "stop_fail_remote"
    remote_a.mkdir()
    stub_fail = _make_stub_colab(tmp_path, log_file=log_a, fake_remote=remote_a, stop_mode="fail_still_running")

    sess = ColabSession(ColabConfig(colab_bin=str(stub_fail), session_name="leaked-vm-1"))
    sess.start()
    cfg_path = sess.config_path
    assert cfg_path.is_file()
    with pytest.raises(ColabShutdownError) as exc_info:
        sess.stop()
    assert exc_info.value.session_name == "leaked-vm-1"
    assert str(cfg_path) in exc_info.value.cleanup_command
    assert cfg_path.is_file()  # sessions.json MUST survive stop failure!
    shutil.rmtree(cfg_path.parent, ignore_errors=True)

    # Case B: stop returns non-zero, but `colab sessions` confirms session is already gone -> clean exit.
    log_b = tmp_path / "stop_gone_calls.jsonl"
    remote_b = tmp_path / "stop_gone_remote"
    remote_b.mkdir()
    stub_gone = _make_stub_colab(tmp_path, log_file=log_b, fake_remote=remote_b, stop_mode="fail_already_gone")
    sess_b = ColabSession(ColabConfig(colab_bin=str(stub_gone), session_name="pruned-vm-2"))
    sess_b.start()
    cfg_b = sess_b.config_path
    sess_b.stop()
    assert not cfg_b.exists()

    # Case C: ColabScheduler persists downloaded artifacts BEFORE raising ColabShutdownError!
    store = LocalArtifactStore(tmp_path / "saved_despite_stop_fail")
    sched = ColabScheduler(ROOT, config=ColabConfig(colab_bin=str(stub_fail), build_portable_native=False))
    spec = ExperimentSpec(experiment_id="keep-on-stop-err", kind="autocal", machine="hangprinter_3", solver="wasm")
    with pytest.raises(ColabShutdownError) as sched_exc:
        sched.run(spec, store=store)
    assert store.exists("keep-on-stop-err", "sweeps.json")
    assert Path(sched_exc.value.config_path).is_file()
    shutil.rmtree(Path(sched_exc.value.config_path).parent, ignore_errors=True)


def test_colab_direct_drive_upload_bypasses_local_and_rescues_failures(tmp_path):
    """Direct Colab -> Drive is opt-in, sends ONLY a short-lived access token (no refresh token), and rescues failures."""
    state = _FakeDriveState()
    drive_srv, drive_base = _start_fake_drive_server(state)

    class TokenHandler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def do_POST(self):
            body = json.dumps({"access_token": "test-token", "expires_in": 3600}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    tok_srv = ThreadingHTTPServer(("127.0.0.1", 0), TokenHandler)
    threading.Thread(target=tok_srv.serve_forever, daemon=True).start()
    adc_path = tmp_path / "adc.json"
    adc_path.write_text(json.dumps({
        "type": "authorized_user",
        "client_id": "cid",
        "client_secret": "SUPER_SECRET_NEVER_UPLOAD",
        "refresh_token": "LONG_LIVED_REFRESH_NEVER_UPLOAD",
        "quota_project_id": "test-qp-123",
    }))

    try:
        store = GoogleDriveArtifactStore(
            root_folder="hp-sim5-direct",
            credentials_path=adc_path,
            api_base=f"{drive_base}/drive/v3",
            upload_base=f"{drive_base}/upload/drive/v3",
        )
        store._token_provider.token_url = f"http://127.0.0.1:{tok_srv.server_port}/token"  # type: ignore[attr-defined]

        # 1. Default mode (direct_drive_upload=False): NO token or credentials sent to Colab!
        log_def = tmp_path / "direct_default.jsonl"
        remote_def = tmp_path / "remote_default"
        remote_def.mkdir()
        stub_def = _make_stub_colab(tmp_path, log_file=log_def, fake_remote=remote_def, direct_mode="ok")
        sched_def = ColabScheduler(ROOT, config=ColabConfig(colab_bin=str(stub_def), build_portable_native=False))
        spec0 = ExperimentSpec(experiment_id="default-vialocal-1", kind="autocal", machine="hangprinter_3", solver="wasm")
        res0 = sched_def.run(spec0, store=store)
        assert res0.status == "ok"
        assert not (remote_def / ".hp_sim5_drive_token.json").exists()
        assert not (remote_def / ".hp_sim5_drive_creds.json").exists()
        assert store.exists("default-vialocal-1", "sweeps.json")

        # 2. Opt-in direct Drive mode: sends ONLY short-lived access_token + quota_project (no refresh_token/client_secret).
        log_ok = tmp_path / "direct_ok.jsonl"
        remote_ok = tmp_path / "remote_ok"
        remote_ok.mkdir()
        stub_ok = _make_stub_colab(tmp_path, log_file=log_ok, fake_remote=remote_ok, direct_mode="ok")
        sched_ok = ColabScheduler(
            ROOT,
            config=ColabConfig(colab_bin=str(stub_ok), build_portable_native=False, direct_drive_upload=True),
        )
        spec1 = ExperimentSpec(experiment_id="direct-1", kind="autocal", machine="hangprinter_3", solver="wasm")
        res1 = sched_ok.run(spec1, store=store)
        assert res1.status == "ok"
        assert res1.artifact_uri == "gdrive://hp-sim5-direct/experiments/direct-1"
        tok_file = remote_ok / ".hp_sim5_drive_token.json"
        assert tok_file.is_file()
        tok_data = json.loads(tok_file.read_text())
        assert tok_data == {"access_token": "test-token", "quota_project": "test-qp-123"}
        assert "LONG_LIVED_REFRESH_NEVER_UPLOAD" not in tok_file.read_text()
        assert "SUPER_SECRET_NEVER_UPLOAD" not in tok_file.read_text()
        run_batch_src = (remote_ok / "run_batch.py").read_text()
        assert "experiments_folder_id" in run_batch_src
        assert "'root_folder_id': None" in run_batch_src
        # Token file must be unlinked BEFORE scheduler.run_batch spawns experiment subprocesses.
        assert run_batch_src.index("token_path.unlink(missing_ok=True)") < run_batch_src.index("scheduler.run_batch(")
        with tarfile.open(remote_ok / "hp_sim5_results.tar.gz", "r:gz") as tar:
            assert tar.getnames() == ["batch_summary.json"]

        # 3. Rescue path: direct Drive upload fails on Colab (e.g. token expired or 503) -> rescued & uploaded locally!
        log_r = tmp_path / "direct_rescue.jsonl"
        remote_r = tmp_path / "remote_rescue"
        remote_r.mkdir()
        stub_r = _make_stub_colab(tmp_path, log_file=log_r, fake_remote=remote_r, direct_mode="rescue")
        sched_r = ColabScheduler(
            ROOT,
            config=ColabConfig(colab_bin=str(stub_r), build_portable_native=False, direct_drive_upload=True),
        )
        spec2 = ExperimentSpec(experiment_id="rescue-1", kind="autocal", machine="hangprinter_4", solver="wasm", tags=["rescued"])
        res2 = sched_r.run(spec2, store=store)
        assert res2.status == "ok"
        assert res2.artifact_uri == "gdrive://hp-sim5-direct/experiments/rescue-1"
        assert store.exists("rescue-1", "sweeps.json")
    finally:
        drive_srv.shutdown()
        tok_srv.shutdown()


def test_colab_best_effort_rescue_on_remote_exec_failure(tmp_path, capsys):
    """When remote batch execution fails/times out mid-run, recover completed & partial artifacts before stopping VM."""
    log_file = tmp_path / "midfail_calls.jsonl"
    fake_remote = tmp_path / "midfail_remote"
    fake_remote.mkdir()
    stub = _make_stub_colab(tmp_path, log_file=log_file, fake_remote=fake_remote, exec_mode="fail_mid_batch")

    store = LocalArtifactStore(tmp_path / "recovered_store")
    sched = ColabScheduler(ROOT, config=ColabConfig(colab_bin=str(stub), build_portable_native=False))
    specs = [
        ExperimentSpec(experiment_id="exp-done-1", kind="autocal", machine="hangprinter_3", solver="wasm"),
        ExperimentSpec(experiment_id="exp-wip-2", kind="autocal", machine="hangprinter_3", solver="wasm"),
    ]
    with pytest.raises(RuntimeError, match="colab exec"):
        sched.run_batch(specs, max_workers=2, store=store, keep_scratch=False)

    # Completed experiment from before the crash was recovered and persisted to store!
    assert store.exists("exp-done-1", "sweeps.json")
    listed = store.list_experiments(tags=["recovered"])
    assert len(listed) == 1 and listed[0]["experiment_id"] == "exp-done-1"

    # Partial scratch was preserved locally on disk and reported to stderr!
    err_out = capsys.readouterr().err
    assert "RECOVERY: Rescued remote artifacts" in err_out
    partial_line = [ln for ln in err_out.splitlines() if "partial scratch:" in ln][0]
    partial_path = Path(partial_line.split("partial scratch:", 1)[1].strip())
    assert (partial_path / "stdout.log").read_text() == "partial progress log\n"
    shutil.rmtree(partial_path.parents[2], ignore_errors=True)

    # Verify rescue happened BEFORE stop, and stop STILL ran!
    calls = [json.loads(line) for line in log_file.read_text().splitlines()]
    subcommands = []
    for argv in calls:
        idx = 0
        while idx < len(argv) and argv[idx].startswith("-"):
            idx += 2 if argv[idx] == "--config" else 1
        subcommands.append(argv[idx])
    assert subcommands[-1] == "stop"
    assert "download" in subcommands  # rescue.tar.gz was downloaded before stop
    assert subcommands.index("download") < len(subcommands) - 1


@pytest.mark.slow
def test_rrf_firmware_service_lifecycle(tmp_path):
    with RrfFirmwareService(ROOT, "sys/config_hp3.g", log_path=tmp_path / "rrf.log", isolate_vsd=True) as rrf:
        reply = rrf.send_gcode("M115")
        assert "RepRapFirmware" in reply.reply
        pid = rrf.pid
        assert pid is not None
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)
