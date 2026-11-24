"""Source revision shared by local runs and portable Colab workspaces."""

from pathlib import Path
import subprocess


def git_provenance(root: Path) -> dict[str, str | bool | None]:
    try:
        revision = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=root, stderr=subprocess.DEVNULL, text=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        revision_file = root / ".hp_sim5_git_revision"
        dirty_file = root / ".hp_sim5_git_dirty"
        revision = revision_file.read_text(encoding="utf-8").strip() if revision_file.is_file() else "unknown"
        dirty = dirty_file.read_text(encoding="utf-8").strip() if dirty_file.is_file() else ""
        return {"git_revision": revision or "unknown", "git_dirty": {"true": True, "false": False}.get(dirty)}

    try:
        changes = subprocess.check_output(
            ["git", "status", "--porcelain", "--untracked-files=normal"],
            cwd=root, stderr=subprocess.DEVNULL, text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        changes = None
    return {"git_revision": revision, "git_dirty": bool(changes) if changes is not None else None}
