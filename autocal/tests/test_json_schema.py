import pytest

import autocal.json_schema as schema


def test_failed_dataset_write_preserves_existing_file_and_removes_temporary(tmp_path, monkeypatch):
    path = tmp_path / "sweeps.json"
    path.write_text('{"original": true}\n')

    def failed_dump(payload, stream, **kwargs):
        stream.write('{"partial":')
        raise OSError("disk full")

    monkeypatch.setattr(schema.json, "dump", failed_dump)
    with pytest.raises(OSError, match="disk full"):
        schema.write_json_file(path, {"new": True})
    assert path.read_text() == '{"original": true}\n'
    assert list(tmp_path.iterdir()) == [path]
