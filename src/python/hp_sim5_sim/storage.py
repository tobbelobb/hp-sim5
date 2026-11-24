"""Persistent artifact storage for hp-sim5 (Local scratch, Local store, Drive mount, Google Drive API v3).

Active simulation and recording I/O always happens on local scratch storage
(`ScratchWorkspace`). Finalized (and optional partial) artifacts are synced to
an `ArtifactStore` with streaming SHA-256 verification and structured metadata.
"""

from __future__ import annotations

from contextlib import AbstractContextManager
from datetime import datetime, timezone
import fcntl
import fnmatch
import hashlib
import http.client
import json
import mimetypes
import os
from pathlib import Path
import shutil
import tempfile
import threading
import time
from typing import Any, Callable, Mapping, Sequence
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlencode, urlparse
from urllib.request import Request, urlopen

from .interfaces import ArtifactRecord

DRIVE_API_BASE = "https://www.googleapis.com/drive/v3"
DRIVE_UPLOAD_BASE = "https://www.googleapis.com/upload/drive/v3"
DEFAULT_DRIVE_URI = "gdrive://hp-sim5-artifacts"
LOCAL_STORE_DIR = "output"
EXPERIMENTS_DIR = "experiments"
UPLOAD_PENDING_DIR = "upload-pending"
GOOGLE_TOKEN_URL = "https://oauth2.googleapis.com/token"
FOLDER_MIME = "application/vnd.google-apps.folder"
DEFAULT_CHUNK_BYTES = 8 * 1024 * 1024  # 8 MiB (multiple of 256 KiB required by Drive resumable API)
MANIFEST_NAME = "experiment_manifest.json"
INDEX_NAME = "index.jsonl"
UPLOAD_PENDING_NAME = "upload_pending.json"
DEFAULT_EXCLUDE_GLOBS: tuple[str, ...] = ("*.tmp", ".tmp-*", UPLOAD_PENDING_NAME, ".matplotlib/*")

_LOCAL_INDEX_LOCKS_GUARD = threading.Lock()
_LOCAL_INDEX_LOCKS: dict[Path, threading.Lock] = {}


def _local_index_lock(root: Path) -> threading.Lock:
    resolved = root.resolve()
    with _LOCAL_INDEX_LOCKS_GUARD:
        lock = _LOCAL_INDEX_LOCKS.get(resolved)
        if lock is None:
            lock = threading.Lock()
            _LOCAL_INDEX_LOCKS[resolved] = lock
        return lock


class DriveAuthError(RuntimeError):
    """Raised when Google Drive credentials or required Drive scopes are unavailable."""


class ArtifactIntegrityError(RuntimeError):
    """Raised when an uploaded or downloaded artifact fails checksum verification."""


class ArtifactUploadError(RuntimeError):
    """Raised when persisting artifacts fails; local scratch is preserved for retry."""

    def __init__(
        self,
        message: str,
        *,
        experiment_id: str | None = None,
        scratch_dir: Path | str | None = None,
        store_uri: str | None = None,
        result: Any = None,
    ) -> None:
        super().__init__(message)
        self.experiment_id = experiment_id
        self.scratch_dir = str(scratch_dir) if scratch_dir is not None else None
        self.store_uri = store_uri
        self.result = result


def retry_upload_command(experiment_id: str, scratch_dir: Path | str, store_uri: str) -> str:
    """Return a copy-pastable CLI command to retry uploading a preserved scratch directory."""
    import shlex

    return shlex.join([
        ".venv/bin/python",
        "scripts/hp_sim5_run.py",
        "artifacts",
        "--store",
        store_uri,
        "push",
        experiment_id,
        str(scratch_dir),
    ])


def write_upload_pending(
    scratch_dir: Path,
    *,
    experiment_id: str,
    store_uri: str,
    metadata: Mapping[str, Any],
    error: str,
) -> Path:
    """Persist retry metadata inside a preserved scratch directory after an upload failure."""
    pending_path = Path(scratch_dir) / UPLOAD_PENDING_NAME
    _atomic_write_json(
        pending_path,
        {
            "experiment_id": experiment_id,
            "store_uri": store_uri,
            "scratch_dir": str(Path(scratch_dir).resolve()),
            "failed_at": utc_now_iso(),
            "error": error,
            "metadata": dict(metadata),
            "retry_command": retry_upload_command(experiment_id, Path(scratch_dir).resolve(), store_uri),
        },
    )
    return pending_path


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def manifest_summary(manifest: Mapping[str, Any]) -> dict[str, Any]:
    meta = manifest.get("metadata", {})
    return {
        "experiment_id": manifest["experiment_id"],
        "stored_at": manifest.get("stored_at", ""),
        "total_bytes": manifest.get("total_bytes", 0),
        "file_count": manifest.get("file_count", 0),
        **{key: meta.get(key) for key in ("kind", "machine", "status", "backend", "git_revision", "summary")},
        "tags": list(meta.get("tags") or []),
    }


def _selected_files(manifest: Mapping[str, Any], rel_paths: Sequence[str] | None) -> list[dict[str, Any]]:
    files = manifest.get("files", [])
    if rel_paths is None:
        return files
    wanted = {_validate_rel_path(path) for path in rel_paths}
    missing = wanted - {item["rel_path"] for item in files}
    if missing:
        raise FileNotFoundError(f"Files absent from experiment manifest: {', '.join(sorted(missing))}")
    return [item for item in files if item["rel_path"] in wanted]


def compute_file_digests(path: Path, *, chunk_bytes: int = 1024 * 1024) -> tuple[int, str, str]:
    """Stream a file once and return (size_bytes, sha256_hex, md5_hex)."""
    sha = hashlib.sha256()
    md5 = hashlib.md5()
    size = 0
    with Path(path).open("rb") as fh:
        while True:
            chunk = fh.read(chunk_bytes)
            if not chunk:
                break
            size += len(chunk)
            sha.update(chunk)
            md5.update(chunk)
    return size, sha.hexdigest(), md5.hexdigest()


def _validate_experiment_id(experiment_id: str) -> str:
    if not experiment_id or "/" in experiment_id or "\\" in experiment_id or experiment_id in (".", ".."):
        raise ValueError(f"Invalid experiment_id: {experiment_id!r}")
    return experiment_id


def _validate_rel_path(rel_path: str) -> str:
    norm = Path(rel_path).as_posix().lstrip("/")
    parts = Path(norm).parts
    if not norm or ".." in parts or Path(rel_path).is_absolute():
        raise ValueError(f"Artifact relative path must stay within experiment directory: {rel_path!r}")
    return norm


def _atomic_write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent, prefix=".tmp-", delete=False) as tmp:
        json.dump(data, tmp, indent=2, sort_keys=True)
        tmp.write("\n")
        tmp_path = Path(tmp.name)
    os.replace(tmp_path, path)


def _guess_mime(rel_path: str) -> str:
    if rel_path.endswith(".rrd"):
        return "application/octet-stream"
    if rel_path.endswith((".jsonl", ".log", ".usda", ".g")):
        return "text/plain"
    guessed, _ = mimetypes.guess_type(rel_path)
    return guessed or "application/octet-stream"


def _matches_any(rel_path: str, patterns: Sequence[str]) -> bool:
    return any(fnmatch.fnmatch(rel_path, pat) or fnmatch.fnmatch(Path(rel_path).name, pat) for pat in patterns)


class ScratchWorkspace(AbstractContextManager["ScratchWorkspace"]):
    """Local scratch directory for active simulation I/O; never points at network/Drive."""

    def __init__(
        self,
        experiment_id: str,
        *,
        base_dir: Path | None = None,
        keep: bool = False,
    ) -> None:
        self.experiment_id = _validate_experiment_id(experiment_id)
        env_base = os.environ.get("HP_SIM5_SCRATCH_DIR")
        if base_dir is not None:
            self._base = Path(base_dir).expanduser().resolve()
        elif env_base:
            self._base = Path(env_base).expanduser().resolve()
        elif Path("/content").is_dir() and os.access("/content", os.W_OK):
            self._base = Path("/content/hp_sim5_scratch")
        else:
            self._base = Path(tempfile.gettempdir()) / "hp_sim5_scratch"
        self.keep = keep
        self.path: Path | None = None

    def preserve(self) -> Path | None:
        """Retain this scratch workspace on exit (e.g. after an upload failure)."""
        self.keep = True
        return self.path

    def __enter__(self) -> "ScratchWorkspace":
        self._base.mkdir(parents=True, exist_ok=True)
        self.path = Path(tempfile.mkdtemp(prefix=f"{self.experiment_id}-", dir=self._base))
        return self

    def __exit__(self, *_exc: object) -> None:
        if self.path is not None and not self.keep:
            shutil.rmtree(self.path, ignore_errors=True)


# ---------------------------------------------------------------------------
# Local & Mounted-Drive Artifact Stores
# ---------------------------------------------------------------------------


class LocalArtifactStore:
    """Content-verified persistent artifact store on a local filesystem."""

    def __init__(self, root: Path | str, *, scheme: str = "local") -> None:
        self.root = Path(root).expanduser().resolve()
        self._scheme = scheme
        self.experiments_dir = self.root / EXPERIMENTS_DIR
        self.index_path = self.root / INDEX_NAME

    @property
    def uri(self) -> str:
        return f"{self._scheme}://{self.root}"

    def _exp_dir(self, experiment_id: str) -> Path:
        exp_id = _validate_experiment_id(experiment_id)
        return self.experiments_dir / exp_id

    def put_file(
        self,
        experiment_id: str,
        rel_path: str,
        local_path: Path,
        *,
        metadata: Mapping[str, Any] | None = None,
    ) -> ArtifactRecord:
        exp_id = _validate_experiment_id(experiment_id)
        rel = _validate_rel_path(rel_path)
        src = Path(local_path).resolve()
        size, sha256, _ = compute_file_digests(src)
        dest = self._exp_dir(exp_id) / rel
        dest.parent.mkdir(parents=True, exist_ok=True)

        # Skip copy if destination already holds identical bytes.
        if dest.is_file() and dest.stat().st_size == size:
            _, existing_sha, _ = compute_file_digests(dest)
            if existing_sha == sha256:
                return ArtifactRecord(
                    experiment_id=exp_id,
                    rel_path=rel,
                    size_bytes=size,
                    sha256=sha256,
                    uri=f"{self.uri}/experiments/{exp_id}/{rel}",
                    content_type=_guess_mime(rel),
                )

        with tempfile.NamedTemporaryFile(dir=dest.parent, prefix=".put-", delete=False) as tmp:
            tmp_path = Path(tmp.name)
        try:
            shutil.copyfile(src, tmp_path)
            _, copied_sha, _ = compute_file_digests(tmp_path)
            if copied_sha != sha256:
                raise ArtifactIntegrityError(f"Checksum mismatch copying {src} -> {dest}")
            os.replace(tmp_path, dest)
        finally:
            tmp_path.unlink(missing_ok=True)

        return ArtifactRecord(
            experiment_id=exp_id,
            rel_path=rel,
            size_bytes=size,
            sha256=sha256,
            uri=f"{self.uri}/experiments/{exp_id}/{rel}",
            content_type=_guess_mime(rel),
        )

    def put_directory(
        self,
        experiment_id: str,
        local_dir: Path,
        *,
        metadata: Mapping[str, Any] | None = None,
        exclude_globs: Sequence[str] = DEFAULT_EXCLUDE_GLOBS,
    ) -> dict[str, Any]:
        exp_id = _validate_experiment_id(experiment_id)
        src_root = Path(local_dir).resolve()
        if not src_root.is_dir():
            raise FileNotFoundError(f"Artifact source directory not found: {src_root}")
        records: list[dict[str, Any]] = []
        total_bytes = 0
        for file_path in sorted(src_root.rglob("*")):
            if not file_path.is_file() or file_path.is_symlink():
                continue
            rel = file_path.relative_to(src_root).as_posix()
            if rel == MANIFEST_NAME or _matches_any(rel, exclude_globs):
                continue
            rec = self.put_file(exp_id, rel, file_path)
            records.append(rec.to_dict())
            total_bytes += rec.size_bytes

        manifest = {
            "schema_version": 1,
            "experiment_id": exp_id,
            "stored_at": utc_now_iso(),
            "store_uri": self.uri,
            "total_bytes": total_bytes,
            "file_count": len(records),
            "metadata": dict(metadata or {}),
            "files": records,
        }
        _atomic_write_json(self._exp_dir(exp_id) / MANIFEST_NAME, manifest)
        self._update_index(manifest)
        return manifest

    def _update_index(self, manifest: Mapping[str, Any]) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        lock_file = self.root / ".index.lock"
        # Serialize concurrent index updates across both threads and independent processes.
        with _local_index_lock(self.root), lock_file.open("a", encoding="utf-8") as lf:
            fcntl.flock(lf.fileno(), fcntl.LOCK_EX)
            try:
                entries: dict[str, dict[str, Any]] = {}
                if self.index_path.is_file():
                    for line in self.index_path.read_text(encoding="utf-8").splitlines():
                        if not line.strip():
                            continue
                        try:
                            row = json.loads(line)
                            entries[row["experiment_id"]] = row
                        except (ValueError, KeyError):
                            continue
                entries[manifest["experiment_id"]] = manifest_summary(manifest)
                with tempfile.NamedTemporaryFile(
                    mode="w", encoding="utf-8", dir=self.root, prefix=".idx-", delete=False
                ) as tmp:
                    for row in sorted(entries.values(), key=lambda r: r.get("stored_at", ""), reverse=True):
                        tmp.write(json.dumps(row, sort_keys=True) + "\n")
                    tmp_path = Path(tmp.name)
                os.replace(tmp_path, self.index_path)
            finally:
                fcntl.flock(lf.fileno(), fcntl.LOCK_UN)

    def get_file(self, experiment_id: str, rel_path: str, dest_path: Path) -> Path:
        exp_id = _validate_experiment_id(experiment_id)
        rel = _validate_rel_path(rel_path)
        src = self._exp_dir(exp_id) / rel
        if not src.is_file():
            raise FileNotFoundError(f"Artifact not found: {exp_id}/{rel}")
        dest = Path(dest_path)
        dest.parent.mkdir(parents=True, exist_ok=True)
        expected_sha = None
        manifest_path = self._exp_dir(exp_id) / MANIFEST_NAME
        if manifest_path.is_file():
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            for item in manifest.get("files", []):
                if item.get("rel_path") == rel:
                    expected_sha = item.get("sha256")
                    break
        with tempfile.NamedTemporaryFile(dir=dest.parent, prefix=".get-", delete=False) as tmp:
            tmp_path = Path(tmp.name)
        try:
            shutil.copyfile(src, tmp_path)
            if expected_sha:
                _, actual_sha, _ = compute_file_digests(tmp_path)
                if actual_sha != expected_sha:
                    raise ArtifactIntegrityError(
                        f"Integrity failure reading {exp_id}/{rel}: expected {expected_sha}, got {actual_sha}"
                    )
            os.replace(tmp_path, dest)
        finally:
            tmp_path.unlink(missing_ok=True)
        return dest

    def get_experiment(
        self,
        experiment_id: str,
        dest_dir: Path,
        *,
        rel_paths: Sequence[str] | None = None,
    ) -> Path:
        manifest = self.get_manifest(experiment_id)
        dest = Path(dest_dir)
        dest.mkdir(parents=True, exist_ok=True)
        for item in _selected_files(manifest, rel_paths):
            rel = _validate_rel_path(item["rel_path"])
            self.get_file(experiment_id, rel, dest / rel)
        _atomic_write_json(dest / MANIFEST_NAME, manifest)
        return dest

    def get_manifest(self, experiment_id: str) -> dict[str, Any]:
        exp_id = _validate_experiment_id(experiment_id)
        path = self._exp_dir(exp_id) / MANIFEST_NAME
        if not path.is_file():
            raise FileNotFoundError(f"Experiment manifest not found: {exp_id}")
        return json.loads(path.read_text(encoding="utf-8"))

    def list_experiments(
        self,
        *,
        tags: Sequence[str] = (),
        kind: str | None = None,
        limit: int | None = 100,
    ) -> list[dict[str, Any]]:
        indexed: dict[str, dict[str, Any]] = {}
        if self.index_path.is_file():
            for line in self.index_path.read_text(encoding="utf-8").splitlines():
                if line.strip():
                    try:
                        row = json.loads(line)
                        indexed[row["experiment_id"]] = row
                    except (ValueError, KeyError):
                        continue
        # A synced/mounted store may contain manifests written through the Drive API.
        rows = []
        for manifest_path in sorted(self.experiments_dir.glob(f"*/{MANIFEST_NAME}")):
            row = indexed.get(manifest_path.parent.name)
            if row is None:
                try:
                    row = manifest_summary(json.loads(manifest_path.read_text(encoding="utf-8")))
                except (FileNotFoundError, ValueError, KeyError):
                    continue
            rows.append(row)
        required_tags = set(tags)
        filtered = [
            row
            for row in rows
            if (kind is None or row.get("kind") == kind)
            and required_tags.issubset(set(row.get("tags") or []))
        ]
        filtered.sort(key=lambda r: r.get("stored_at", ""), reverse=True)
        return filtered[:limit]

    def exists(self, experiment_id: str, rel_path: str | None = None) -> bool:
        exp_dir = self._exp_dir(experiment_id)
        if rel_path is None:
            return (exp_dir / MANIFEST_NAME).is_file()
        return (exp_dir / _validate_rel_path(rel_path)).is_file()


class DriveMountArtifactStore(LocalArtifactStore):
    """ArtifactStore backed by a mounted Google Drive folder (e.g. /content/drive/MyDrive/...)."""

    def __init__(self, mount_root: Path | str) -> None:
        super().__init__(mount_root, scheme="drivemount")


# ---------------------------------------------------------------------------
# Google Drive REST API v3 Artifact Store (pure stdlib, resumable uploads)
# ---------------------------------------------------------------------------


class OAuthRefreshTokenProvider:
    """Refreshes OAuth2 authorized_user credentials using pure Python stdlib."""

    def __init__(
        self,
        *,
        client_id: str,
        client_secret: str,
        refresh_token: str,
        token_url: str = GOOGLE_TOKEN_URL,
        quota_project_id: str | None = None,
        timeout_s: float = 30.0,
    ) -> None:
        self.client_id = client_id
        self.client_secret = client_secret
        self.refresh_token = refresh_token
        self.token_url = token_url
        self.quota_project_id = quota_project_id
        self.timeout_s = timeout_s
        self._access_token: str | None = None
        self._expires_at: float = 0.0

    def __call__(self) -> str:
        now = time.monotonic()
        if self._access_token and now < self._expires_at - 60:
            return self._access_token
        body = urlencode({
            "client_id": self.client_id,
            "client_secret": self.client_secret,
            "refresh_token": self.refresh_token,
            "grant_type": "refresh_token",
        }).encode("utf-8")
        req = Request(
            self.token_url,
            data=body,
            headers={"Content-Type": "application/x-www-form-urlencoded"},
            method="POST",
        )
        try:
            with urlopen(req, timeout=self.timeout_s) as resp:
                payload = json.loads(resp.read().decode("utf-8"))
            token = payload.get("access_token") if isinstance(payload, dict) else None
            if not isinstance(token, str) or not token.strip():
                raise ValueError("missing access token")
            self._expires_at = now + float(payload.get("expires_in", 3600))
            self._access_token = token
        except HTTPError as err:
            detail = err.read().decode("utf-8", errors="replace")
            raise DriveAuthError(f"OAuth2 token refresh failed (HTTP {err.code}): {detail}") from None
        except (OSError, ValueError, TypeError, http.client.HTTPException) as err:
            raise DriveAuthError(f"Could not refresh Drive access token: {type(err).__name__}") from None
        return self._access_token


def default_adc_path(credentials_path: Path | str | None = None) -> Path | None:
    """Return the active standard Google ADC JSON path if one exists on disk."""
    configured = credentials_path or os.environ.get("GOOGLE_APPLICATION_CREDENTIALS")
    path = (Path(configured).expanduser() if configured else
            Path.home() / ".config/gcloud/application_default_credentials.json")
    return path if path.is_file() else None


def load_drive_token_provider(
    *,
    credentials_path: Path | str | None = None,
    timeout_s: float = 30.0,
) -> tuple[Callable[[], str], str | None]:
    """Resolve Google Drive access token provider from standard Google ADC credentials."""
    cand = default_adc_path(credentials_path)
    if cand is not None:
        try:
            data = json.loads(cand.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            raise DriveAuthError(f"Could not read Google ADC credentials at {cand}") from None
        fields = ("client_id", "client_secret", "refresh_token")
        if not isinstance(data, dict) or not all(isinstance(data.get(key), str) and data[key] for key in fields):
            raise DriveAuthError(f"Expected authorized-user Google ADC credentials at {cand}")
        qp = data.get("quota_project_id")
        provider = OAuthRefreshTokenProvider(
            client_id=data["client_id"],
            client_secret=data["client_secret"],
            refresh_token=data["refresh_token"],
            quota_project_id=qp,
            timeout_s=timeout_s,
        )
        return provider, qp
    if configured := credentials_path or os.environ.get("GOOGLE_APPLICATION_CREDENTIALS"):
        raise DriveAuthError(f"Google ADC credentials file not found: {Path(configured).expanduser()}")

    raise DriveAuthError(
        "No Google Application Default Credentials found. Mint ADC with Colab + Drive file scope:\n"
        "  gcloud auth application-default login --scopes="
        "openid,https://www.googleapis.com/auth/cloud-platform,"
        "https://www.googleapis.com/auth/userinfo.email,"
        "https://www.googleapis.com/auth/colaboratory,"
        "https://www.googleapis.com/auth/drive.file"
    )


def _clip_app_property(key: str, value: str) -> str:
    """Clip a Drive appProperties value so key+value stays within Google's 124-byte UTF-8 limit."""
    budget = max(0, 124 - len(key.encode("utf-8")))
    encoded = value.encode("utf-8")
    if len(encoded) <= budget:
        return value
    return encoded[:budget].decode("utf-8", errors="ignore")


class GoogleDriveArtifactStore:
    """Reliable Google Drive REST API v3 artifact store with resumable chunked uploads.

    Works identically from local machines and from Colab (no interactive Drive mount required).
    Per-experiment metadata is stored on each experiment folder's `appProperties` and
    `experiment_manifest.json` rather than a shared mutable index file, making parallel
    uploads lock-free and race-free.
    """

    def __init__(
        self,
        root_folder: str = "hp-sim5-artifacts",
        *,
        root_folder_id: str | None = None,
        experiments_folder_id: str | None = None,
        token_provider: Callable[[], str] | None = None,
        quota_project: str | None = None,
        credentials_path: Path | str | None = None,
        api_base: str = DRIVE_API_BASE,
        upload_base: str = DRIVE_UPLOAD_BASE,
        chunk_bytes: int = DEFAULT_CHUNK_BYTES,
        max_retries: int = 5,
        timeout_s: float = 120.0,
    ) -> None:
        if chunk_bytes <= 0 or chunk_bytes % (256 * 1024) != 0:
            raise ValueError("chunk_bytes must be a positive multiple of 256 KiB")
        self.root_folder = root_folder.strip("/") or "hp-sim5-artifacts"
        self._explicit_root_id = root_folder_id
        self._credentials_path = Path(credentials_path).expanduser() if credentials_path else None
        self.api_base = api_base.rstrip("/")
        self.upload_base = upload_base.rstrip("/")
        self.chunk_bytes = chunk_bytes
        self.max_retries = max_retries
        self.timeout_s = timeout_s
        if token_provider is not None:
            self._token_provider = token_provider
            self._quota_project = quota_project
            self._uses_adc = False
        else:
            self._token_provider, detected_qp = load_drive_token_provider(
                credentials_path=credentials_path,
                timeout_s=min(timeout_s, 30.0),
            )
            self._quota_project = quota_project if quota_project is not None else detected_qp
            self._uses_adc = True
        self._folder_lock = threading.Lock()
        self._folder_cache: dict[tuple[str, str], str] = {}
        self._root_id_cache: str | None = root_folder_id
        self._experiments_id_cache: str | None = experiments_folder_id

    def mint_access_token(self) -> tuple[str, str | None]:
        """Return a short-lived OAuth bearer access token and optional quota project (never a refresh token)."""
        return self._token_provider(), self._quota_project

    @property
    def uri(self) -> str:
        if self._explicit_root_id:
            return f"gdrive://folder_id:{self._explicit_root_id}"
        return f"gdrive://{self.root_folder}"

    def _headers(self, extra: Mapping[str, str] | None = None) -> dict[str, str]:
        headers = {"Authorization": f"Bearer {self._token_provider()}"}
        if self._quota_project:
            headers["x-goog-user-project"] = self._quota_project
        if extra:
            headers.update(extra)
        return headers

    def _request(
        self,
        method: str,
        url: str,
        *,
        body: bytes | None = None,
        headers: Mapping[str, str] | None = None,
        ok_statuses: tuple[int, ...] = (200, 201),
    ) -> tuple[int, dict[str, str], bytes]:
        last_err: Exception | None = None
        for attempt in range(self.max_retries):
            req = Request(url, data=body, headers=self._headers(headers), method=method)
            try:
                with urlopen(req, timeout=self.timeout_s) as resp:
                    status = resp.status
                    resp_headers = {k.lower(): v for k, v in resp.headers.items()}
                    payload = resp.read()
                    if status in ok_statuses:
                        return status, resp_headers, payload
                    raise RuntimeError(f"Unexpected HTTP {status} for {method} {url}")
            except HTTPError as err:
                payload = err.read()
                resp_headers = {k.lower(): v for k, v in err.headers.items()}
                if err.code in ok_statuses:
                    return err.code, resp_headers, payload
                detail = payload.decode("utf-8", errors="replace")
                if err.code in (401, 403):
                    if "ACCESS_TOKEN_SCOPE_INSUFFICIENT" in detail or "insufficientPermissions" in detail:
                        raise DriveAuthError(
                            "Google Drive API rejected the access token due to insufficient scopes. "
                            "Re-mint ADC with https://www.googleapis.com/auth/drive.file:\n"
                            "  gcloud auth application-default login --scopes="
                            "openid,https://www.googleapis.com/auth/cloud-platform,"
                            "https://www.googleapis.com/auth/userinfo.email,"
                            "https://www.googleapis.com/auth/colaboratory,"
                            "https://www.googleapis.com/auth/drive.file\n"
                            f"Drive response: {detail}"
                        ) from None
                    raise DriveAuthError(f"Google Drive auth error (HTTP {err.code}): {detail}") from None
                if err.code in (429, 500, 502, 503, 504) and attempt + 1 < self.max_retries:
                    time.sleep(min(16.0, 0.25 * (2**attempt)))
                    last_err = err
                    continue
                raise RuntimeError(f"Drive API {method} {url} failed (HTTP {err.code}): {detail}") from None
            except (URLError, TimeoutError, http.client.HTTPException, OSError) as err:
                last_err = err
                if attempt + 1 < self.max_retries:
                    time.sleep(min(16.0, 0.25 * (2**attempt)))
                    continue
                raise RuntimeError(f"Drive API network failure on {method} {url}: {err}") from None
        raise RuntimeError(f"Drive API request exhausted retries: {last_err}")

    def _request_json(
        self,
        method: str,
        url: str,
        *,
        json_body: Any = None,
        headers: Mapping[str, str] | None = None,
    ) -> dict[str, Any]:
        req_headers = dict(headers or {})
        body: bytes | None = None
        if json_body is not None:
            body = json.dumps(json_body).encode("utf-8")
            req_headers["Content-Type"] = "application/json; charset=utf-8"
        _, _, payload = self._request(method, url, body=body, headers=req_headers)
        return json.loads(payload.decode("utf-8")) if payload else {}

    def _find_child(self, parent_id: str, name: str, *, mime_type: str | None = None) -> dict[str, Any] | None:
        escaped = name.replace("\\", "\\\\").replace("'", "\\'")
        clauses = [f"'{parent_id}' in parents", f"name = '{escaped}'", "trashed = false"]
        if mime_type:
            clauses.append(f"mimeType = '{mime_type}'")
        q = " and ".join(clauses)
        params = urlencode({
            "q": q,
            "fields": "files(id,name,mimeType,size,md5Checksum,sha256Checksum,appProperties)",
            "pageSize": "10",
        })
        data = self._request_json("GET", f"{self.api_base}/files?{params}")
        files = data.get("files", [])
        return files[0] if files else None

    def _ensure_folder(
        self,
        parent_id: str,
        name: str,
        *,
        app_properties: Mapping[str, str] | None = None,
    ) -> str:
        cache_key = (parent_id, name)
        with self._folder_lock:
            if cache_key in self._folder_cache and not app_properties:
                return self._folder_cache[cache_key]
            existing = self._find_child(parent_id, name, mime_type=FOLDER_MIME)
            if existing:
                folder_id = existing["id"]
                if app_properties:
                    self._request_json(
                        "PATCH",
                        f"{self.api_base}/files/{quote(folder_id)}?fields=id",
                        json_body={"appProperties": dict(app_properties)},
                    )
                self._folder_cache[cache_key] = folder_id
                return folder_id
            body: dict[str, Any] = {"name": name, "mimeType": FOLDER_MIME, "parents": [parent_id]}
            if app_properties:
                body["appProperties"] = dict(app_properties)
            created = self._request_json("POST", f"{self.api_base}/files?fields=id", json_body=body)
            folder_id = created["id"]
            self._folder_cache[cache_key] = folder_id
            return folder_id

    def _ensure_root(self) -> str:
        if self._root_id_cache:
            return self._root_id_cache
        parent = "root"
        for segment in self.root_folder.split("/"):
            if segment:
                parent = self._ensure_folder(parent, segment)
        self._root_id_cache = parent
        return parent

    def ensure_experiments_folder(self) -> str:
        """Create/resolve the shared `<root>/experiments` folder once (safe before parallel fan-out)."""
        if self._experiments_id_cache:
            return self._experiments_id_cache
        root_id = self._ensure_root()
        self._experiments_id_cache = self._ensure_folder(root_id, "experiments")
        return self._experiments_id_cache

    def _ensure_path_folder(
        self,
        experiment_id: str,
        rel_dir: str,
        *,
        app_properties: Mapping[str, str] | None = None,
    ) -> str:
        exps_id = self.ensure_experiments_folder()
        parent = self._ensure_folder(exps_id, experiment_id, app_properties=app_properties)
        if rel_dir and rel_dir != ".":
            for segment in Path(rel_dir).parts:
                parent = self._ensure_folder(parent, segment)
        return parent

    def _find_experiments_folder(self) -> str | None:
        if self._experiments_id_cache:
            return self._experiments_id_cache
        # Lookup must not create folders when pointed at a missing store.
        root_id = self._root_id_cache
        if not root_id:
            root_id = "root"
            for segment in filter(None, self.root_folder.split("/")):
                folder = self._find_child(root_id, segment, mime_type=FOLDER_MIME)
                if not folder:
                    return None
                root_id = folder["id"]
            self._root_id_cache = root_id
        exps = self._find_child(root_id, "experiments", mime_type=FOLDER_MIME)
        if not exps:
            return None
        self._experiments_id_cache = exps["id"]
        return self._experiments_id_cache

    def _find_path_file(self, experiment_id: str, rel_path: str) -> dict[str, Any] | None:
        exps_id = self._find_experiments_folder()
        if not exps_id:
            return None
        exp = self._find_child(exps_id, experiment_id, mime_type=FOLDER_MIME)
        if not exp:
            return None
        parent_id = exp["id"]
        rel = Path(_validate_rel_path(rel_path))
        for segment in rel.parts[:-1]:
            child = self._find_child(parent_id, segment, mime_type=FOLDER_MIME)
            if not child:
                return None
            parent_id = child["id"]
        return self._find_child(parent_id, rel.name)

    def _upload_resumable(
        self,
        *,
        parent_id: str,
        name: str,
        local_path: Path,
        size: int,
        sha256: str,
        md5: str,
        mime_type: str,
        existing_file_id: str | None = None,
        app_properties: Mapping[str, str] | None = None,
    ) -> dict[str, Any]:
        fields = "id,name,size,md5Checksum,sha256Checksum"
        meta: dict[str, Any] = {"name": name}
        if app_properties:
            meta["appProperties"] = dict(app_properties)
        if existing_file_id:
            init_method = "PATCH"
            init_url = f"{self.upload_base}/files/{quote(existing_file_id)}?uploadType=resumable&fields={fields}"
        else:
            meta["parents"] = [parent_id]
            init_method = "POST"
            init_url = f"{self.upload_base}/files?uploadType=resumable&fields={fields}"

        _, init_headers, _ = self._request(
            init_method,
            init_url,
            body=json.dumps(meta).encode("utf-8"),
            headers={
                "Content-Type": "application/json; charset=utf-8",
                "X-Upload-Content-Type": mime_type,
                "X-Upload-Content-Length": str(size),
            },
            ok_statuses=(200,),
        )
        session_url = init_headers.get("location")
        if not session_url:
            raise RuntimeError("Drive resumable upload initiation did not return a Location header")

        # Handle empty 0-byte files cleanly in a single PUT.
        if size == 0:
            _, _, payload = self._request(
                "PUT",
                session_url,
                body=b"",
                headers={"Content-Type": mime_type, "Content-Length": "0"},
                ok_statuses=(200, 201),
            )
            return json.loads(payload.decode("utf-8"))

        offset = 0
        with local_path.open("rb") as fh:
            while offset < size:
                fh.seek(offset)
                chunk = fh.read(min(self.chunk_bytes, size - offset))
                end = offset + len(chunk) - 1
                chunk_headers = {
                    "Content-Type": mime_type,
                    "Content-Length": str(len(chunk)),
                    "Content-Range": f"bytes {offset}-{end}/{size}",
                }
                for attempt in range(self.max_retries):
                    try:
                        status, resp_headers, payload = self._request(
                            "PUT",
                            session_url,
                            body=chunk,
                            headers=chunk_headers,
                            ok_statuses=(200, 201, 308),
                        )
                        if status in (200, 201):
                            result = json.loads(payload.decode("utf-8"))
                            self._verify_remote_checksum(result, sha256=sha256, md5=md5, label=name)
                            return result
                        # 308 Resume Incomplete: advance offset to server-confirmed byte + 1.
                        rng = resp_headers.get("range", "")
                        if rng.startswith("bytes="):
                            offset = int(rng.split("-", 1)[1]) + 1
                        else:
                            offset = end + 1
                        break
                    except RuntimeError:
                        if attempt + 1 >= self.max_retries:
                            raise
                        # Query server for last committed byte before retrying.
                        time.sleep(min(8.0, 0.25 * (2**attempt)))
                        q_status, q_headers, q_payload = self._request(
                            "PUT",
                            session_url,
                            body=b"",
                            headers={"Content-Range": f"bytes */{size}"},
                            ok_statuses=(200, 201, 308),
                        )
                        if q_status in (200, 201):
                            result = json.loads(q_payload.decode("utf-8"))
                            self._verify_remote_checksum(result, sha256=sha256, md5=md5, label=name)
                            return result
                        rng = q_headers.get("range", "")
                        if rng.startswith("bytes="):
                            offset = int(rng.split("-", 1)[1]) + 1
                        break
        raise RuntimeError(f"Resumable upload ended without completion for {name}")

    @staticmethod
    def _verify_remote_checksum(remote: Mapping[str, Any], *, sha256: str, md5: str, label: str) -> None:
        if remote.get("sha256Checksum") and remote["sha256Checksum"].lower() != sha256.lower():
            raise ArtifactIntegrityError(
                f"Drive SHA-256 mismatch for {label}: expected {sha256}, got {remote['sha256Checksum']}"
            )
        if remote.get("md5Checksum") and remote["md5Checksum"].lower() != md5.lower():
            raise ArtifactIntegrityError(
                f"Drive MD5 mismatch for {label}: expected {md5}, got {remote['md5Checksum']}"
            )

    def put_file(
        self,
        experiment_id: str,
        rel_path: str,
        local_path: Path,
        *,
        metadata: Mapping[str, Any] | None = None,
    ) -> ArtifactRecord:
        exp_id = _validate_experiment_id(experiment_id)
        rel = _validate_rel_path(rel_path)
        src = Path(local_path).resolve()
        size, sha256, md5 = compute_file_digests(src)
        rel_parent = Path(rel).parent.as_posix()
        file_name = Path(rel).name
        parent_id = self._ensure_path_folder(exp_id, rel_parent)
        existing = self._find_child(parent_id, file_name)
        mime_type = _guess_mime(rel)

        # Deduplicate unchanged files using Drive's server-side checksums.
        if existing and int(existing.get("size", -1)) == size:
            if (existing.get("sha256Checksum") or "").lower() == sha256.lower() or (
                existing.get("md5Checksum") or ""
            ).lower() == md5.lower():
                return ArtifactRecord(
                    experiment_id=exp_id,
                    rel_path=rel,
                    size_bytes=size,
                    sha256=sha256,
                    uri=f"{self.uri}/experiments/{exp_id}/{rel}",
                    remote_id=existing["id"],
                    content_type=mime_type,
                )

        uploaded = self._upload_resumable(
            parent_id=parent_id,
            name=file_name,
            local_path=src,
            size=size,
            sha256=sha256,
            md5=md5,
            mime_type=mime_type,
            existing_file_id=existing["id"] if existing else None,
            app_properties={"hp_sim5_experiment_id": exp_id, "hp_sim5_sha256": sha256},
        )
        return ArtifactRecord(
            experiment_id=exp_id,
            rel_path=rel,
            size_bytes=size,
            sha256=sha256,
            uri=f"{self.uri}/experiments/{exp_id}/{rel}",
            remote_id=uploaded["id"],
            content_type=mime_type,
        )

    def put_directory(
        self,
        experiment_id: str,
        local_dir: Path,
        *,
        metadata: Mapping[str, Any] | None = None,
        exclude_globs: Sequence[str] = DEFAULT_EXCLUDE_GLOBS,
    ) -> dict[str, Any]:
        exp_id = _validate_experiment_id(experiment_id)
        src_root = Path(local_dir).resolve()
        if not src_root.is_dir():
            raise FileNotFoundError(f"Artifact source directory not found: {src_root}")
        meta = dict(metadata or {})
        self._ensure_path_folder(exp_id, ".")

        records: list[dict[str, Any]] = []
        total_bytes = 0
        for file_path in sorted(src_root.rglob("*")):
            if not file_path.is_file() or file_path.is_symlink():
                continue
            rel = file_path.relative_to(src_root).as_posix()
            if rel == MANIFEST_NAME or _matches_any(rel, exclude_globs):
                continue
            rec = self.put_file(exp_id, rel, file_path)
            records.append(rec.to_dict())
            total_bytes += rec.size_bytes

        stored_at = utc_now_iso()
        manifest = {
            "schema_version": 1,
            "experiment_id": exp_id,
            "stored_at": stored_at,
            "store_uri": self.uri,
            "total_bytes": total_bytes,
            "file_count": len(records),
            "metadata": meta,
            "files": records,
        }
        with tempfile.TemporaryDirectory(prefix="hp-manifest-") as tmp:
            manifest_file = Path(tmp) / MANIFEST_NAME
            _atomic_write_json(manifest_file, manifest)
            self.put_file(exp_id, MANIFEST_NAME, manifest_file)

        # Store per-experiment index metadata on this experiment's own folder (zero shared-index races).
        tags_csv = ",".join(str(t) for t in (meta.get("tags") or []))
        app_props = {
            k: _clip_app_property(k, str(v))
            for k, v in {
                "hp_sim5_experiment_id": exp_id,
                "hp_sim5_kind": meta.get("kind") or "",
                "hp_sim5_machine": meta.get("machine") or "",
                "hp_sim5_status": meta.get("status") or "",
                "hp_sim5_backend": meta.get("backend") or "",
                "hp_sim5_stored_at": stored_at,
                "hp_sim5_total_bytes": str(total_bytes),
                "hp_sim5_file_count": str(len(records)),
                "hp_sim5_git_revision": meta.get("git_revision") or "",
                "hp_sim5_tags": tags_csv,
                "hp_sim5_summary": " ".join(str(meta.get("summary") or "").split()),
            }.items()
        }
        self._ensure_path_folder(exp_id, ".", app_properties=app_props)
        return manifest

    def _download_file_id(self, file_id: str, dest_path: Path, *, expected_sha256: str | None = None) -> Path:
        dest = Path(dest_path)
        dest.parent.mkdir(parents=True, exist_ok=True)
        url = f"{self.api_base}/files/{quote(file_id)}?alt=media"
        with tempfile.NamedTemporaryFile(dir=dest.parent, prefix=".dl-", delete=False) as tmp:
            tmp_path = Path(tmp.name)
        try:
            for attempt in range(self.max_retries):
                req = Request(url, headers=self._headers(), method="GET")
                try:
                    sha = hashlib.sha256()
                    with urlopen(req, timeout=self.timeout_s) as resp, tmp_path.open("wb") as out:
                        while True:
                            chunk = resp.read(1024 * 1024)
                            if not chunk:
                                break
                            sha.update(chunk)
                            out.write(chunk)
                    if expected_sha256 and sha.hexdigest().lower() != expected_sha256.lower():
                        raise ArtifactIntegrityError(
                            f"Download SHA-256 mismatch for file {file_id}: "
                            f"expected {expected_sha256}, got {sha.hexdigest()}"
                        )
                    os.replace(tmp_path, dest)
                    return dest
                except HTTPError as err:
                    if err.code in (429, 500, 502, 503, 504) and attempt + 1 < self.max_retries:
                        time.sleep(min(16.0, 0.25 * (2**attempt)))
                        continue
                    raise RuntimeError(f"Drive download failed (HTTP {err.code}): {err.read().decode(errors='replace')}") from None
                except (URLError, TimeoutError, OSError) as err:
                    if attempt + 1 < self.max_retries:
                        time.sleep(min(16.0, 0.25 * (2**attempt)))
                        continue
                    raise RuntimeError(f"Drive download network error: {err}") from None
        finally:
            tmp_path.unlink(missing_ok=True)
        raise RuntimeError(f"Drive download failed for {file_id}")

    def _list_experiment_folders(self) -> list[dict[str, Any]]:
        exps_id = self._find_experiments_folder()
        if not exps_id:
            return []
        q = f"'{exps_id}' in parents and mimeType = '{FOLDER_MIME}' and trashed = false"
        folders: list[dict[str, Any]] = []
        page_token: str | None = None
        while True:
            query: dict[str, str] = {
                "q": q,
                "fields": "nextPageToken,files(id,name,appProperties,modifiedTime)",
                "pageSize": "100",
            }
            if page_token:
                query["pageToken"] = page_token
            data = self._request_json("GET", f"{self.api_base}/files?{urlencode(query)}")
            folders.extend(data.get("files", []))
            page_token = data.get("nextPageToken")
            if not page_token:
                break
        return folders

    def get_manifest(self, experiment_id: str) -> dict[str, Any]:
        exp_id = _validate_experiment_id(experiment_id)
        remote = self._find_path_file(exp_id, MANIFEST_NAME)
        if not remote:
            raise FileNotFoundError(f"Experiment manifest not found in Drive: {exp_id}")
        with tempfile.TemporaryDirectory(prefix="hp-man-") as tmp:
            dest = self._download_file_id(remote["id"], Path(tmp) / MANIFEST_NAME)
            return json.loads(dest.read_text(encoding="utf-8"))

    def get_file(self, experiment_id: str, rel_path: str, dest_path: Path) -> Path:
        exp_id = _validate_experiment_id(experiment_id)
        rel = _validate_rel_path(rel_path)
        expected_sha = None
        remote_id = None
        try:
            manifest = self.get_manifest(exp_id)
            for item in manifest.get("files", []):
                if item.get("rel_path") == rel:
                    expected_sha = item.get("sha256")
                    remote_id = item.get("remote_id")
                    break
        except FileNotFoundError:
            pass
        if not remote_id:
            remote = self._find_path_file(exp_id, rel)
            if not remote:
                raise FileNotFoundError(f"Artifact not found in Drive: {exp_id}/{rel}")
            remote_id = remote["id"]
            expected_sha = expected_sha or remote.get("sha256Checksum")
        return self._download_file_id(remote_id, dest_path, expected_sha256=expected_sha)

    def get_experiment(
        self,
        experiment_id: str,
        dest_dir: Path,
        *,
        rel_paths: Sequence[str] | None = None,
    ) -> Path:
        manifest = self.get_manifest(experiment_id)
        dest = Path(dest_dir)
        dest.mkdir(parents=True, exist_ok=True)
        for item in _selected_files(manifest, rel_paths):
            rel = _validate_rel_path(item["rel_path"])
            remote_id = item.get("remote_id")
            if remote_id:
                self._download_file_id(remote_id, dest / rel, expected_sha256=item.get("sha256"))
            else:
                self.get_file(experiment_id, rel, dest / rel)
        _atomic_write_json(dest / MANIFEST_NAME, manifest)
        return dest

    def list_experiments(
        self,
        *,
        tags: Sequence[str] = (),
        kind: str | None = None,
        limit: int | None = 100,
    ) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        for folder in self._list_experiment_folders():
            props = folder.get("appProperties") or {}
            if "hp_sim5_stored_at" in props:
                tags_raw = props.get("hp_sim5_tags", "")
                row = {
                    "experiment_id": props.get("hp_sim5_experiment_id") or folder["name"],
                    "stored_at": props.get("hp_sim5_stored_at", ""),
                    "store_uri": self.uri,
                    "total_bytes": int(props.get("hp_sim5_total_bytes") or 0),
                    "file_count": int(props.get("hp_sim5_file_count") or 0),
                    "kind": props.get("hp_sim5_kind") or None,
                    "machine": props.get("hp_sim5_machine") or None,
                    "status": props.get("hp_sim5_status") or None,
                    "backend": props.get("hp_sim5_backend") or None,
                    "tags": [t for t in tags_raw.split(",") if t],
                    "git_revision": props.get("hp_sim5_git_revision") or None,
                    "summary": props.get("hp_sim5_summary") or None,
                }
            else:
                try:
                    manifest = self.get_manifest(folder["name"])
                except FileNotFoundError:
                    continue
                row = {**manifest_summary({"experiment_id": folder["name"], **manifest}), "store_uri": self.uri}
            rows.append(row)
        required_tags = set(tags)
        filtered = [
            row
            for row in rows
            if (kind is None or row.get("kind") == kind)
            and required_tags.issubset(set(row.get("tags") or []))
        ]
        filtered.sort(key=lambda r: r.get("stored_at", ""), reverse=True)
        return filtered[:limit]

    def exists(self, experiment_id: str, rel_path: str | None = None) -> bool:
        target = MANIFEST_NAME if rel_path is None else _validate_rel_path(rel_path)
        return self._find_path_file(_validate_experiment_id(experiment_id), target) is not None


def store_from_uri(uri: str, **kwargs: Any) -> LocalArtifactStore | DriveMountArtifactStore | GoogleDriveArtifactStore:
    """Create an ArtifactStore from a URI: local://..., drivemount://..., gdrive://..."""
    parsed = urlparse(uri)
    if not parsed.scheme or parsed.scheme == "local" or parsed.scheme == "file":
        if not parsed.scheme:
            path = uri
        elif parsed.netloc:
            path = f"{parsed.netloc}{parsed.path}"
        else:
            path = parsed.path
        return LocalArtifactStore(Path(path))
    if parsed.scheme == "drivemount":
        path = f"{parsed.netloc}{parsed.path}" if parsed.netloc else parsed.path
        return DriveMountArtifactStore(Path(path))
    if parsed.scheme == "gdrive":
        target = f"{parsed.netloc}{parsed.path}".strip("/")
        if target.startswith("folder_id:"):
            return GoogleDriveArtifactStore(root_folder_id=target.removeprefix("folder_id:"), **kwargs)
        return GoogleDriveArtifactStore(root_folder=target or "hp-sim5-artifacts", **kwargs)
    raise ValueError(f"Unsupported artifact store URI scheme: {uri!r}")
