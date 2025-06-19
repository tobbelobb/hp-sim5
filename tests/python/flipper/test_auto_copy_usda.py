import os
import shutil
import tempfile
import time
from pathlib import Path

# Following the pattern from test_flipper_integration.py, we assume
# 'examples/python_flipper' is in PYTHONPATH during tests.
from server import _copy_usd_on_change

def test_usd_file_copy_on_change():
    """
    Tests the file copy logic from the flipper server.
    """
    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir_path = Path(tmpdir)

        # Mock project structure
        source_dir = tmpdir_path / "examples" / "usd_scenes"
        dest_dir = tmpdir_path / "public" / "examples" / "usd_scenes"
        source_dir.mkdir(parents=True, exist_ok=True)

        source_file = source_dir / "flipper_scene.usda"
        other_file = source_dir / "other_scene.usda"
        dest_file = dest_dir / "flipper_scene_copy_for_vite.usda.txt"

        # --- Test 1: Correct file change triggers copy ---
        source_content_v1 = "#USD 1.0 (v1)"
        source_file.write_text(source_content_v1)

        _copy_usd_on_change(source_file, tmpdir_path)

        assert dest_file.exists(), "Destination file should have been created."
        assert dest_file.read_text() == source_content_v1, "Destination file content is incorrect."

        # --- Test 2: Other file change does not trigger copy ---
        other_file.write_text("other file content")
        dest_file.unlink() # remove destination file to be sure
        assert not dest_file.exists()

        _copy_usd_on_change(other_file, tmpdir_path)
        assert not dest_file.exists(), "Destination file should not have been created for other file change."

        # Re-create for next test
        _copy_usd_on_change(source_file, tmpdir_path)
        assert dest_file.exists()

        # --- Test 3: Lock mechanism prevents concurrent copy ---
        lock_file = dest_dir / (dest_file.name + ".lock")

        # Create a lock file manually
        lock_fd = os.open(lock_file, os.O_CREAT | os.O_EXCL | os.O_WRONLY)

        # Modify source again
        source_content_v2 = "#USD 1.0 (v2)"
        source_file.write_text(source_content_v2)

        # This call should not be able to copy because of the lock
        _copy_usd_on_change(source_file, tmpdir_path)

        # Content should still be v1
        assert dest_file.read_text() == source_content_v1, "File should not have been copied while locked."

        # --- Test 4: Copy succeeds after lock release ---
        os.close(lock_fd)
        os.remove(lock_file)

        _copy_usd_on_change(source_file, tmpdir_path)
        assert dest_file.read_text() == source_content_v2, "File should have been copied after lock was released."
