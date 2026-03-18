import json

from autocal import merge_sweep_datasets as merge_tool


def _write_dataset(path, *, timestamp: str, config: dict, sweeps: list[dict]) -> None:
    payload = {
        "version": "1.0",
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "timestamp": timestamp,
        "config": config,
        "sweeps": sweeps,
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def _sweep() -> dict:
    return {
        "id": "sweep_001",
        "fixed_anchors": [0],
        "fixed_lengths": [10.0],
        "drive_anchor": 1,
        "sensor_anchor": 2,
        "data_points": [],
    }


def test_merge_sweep_datasets_main_merges_and_renumbers(tmp_path, capsys):
    base = tmp_path / "base.json"
    extra = tmp_path / "extra.json"
    out = tmp_path / "merged.json"

    _write_dataset(
        base,
        timestamp="2026-03-01T00:00:00",
        config={"m666_before_data_collection": {"R": [30.0, 30.0, 30.0]}},
        sweeps=[_sweep()],
    )
    _write_dataset(
        extra,
        timestamp="2026-03-01T00:00:01",
        config={
            "m666_adjusted_by_data_collector": {"R": [31.0, 31.0, 31.0], "Q": 0.5},
            "notes": {"base_radii_forced_mm": [30.0, 30.0, 30.0]},
        },
        sweeps=[_sweep()],
    )

    rc = merge_tool.main([str(base), str(extra), "-o", str(out)])

    assert rc == 0
    merged = json.loads(out.read_text(encoding="utf-8"))
    assert [sweep["id"] for sweep in merged["sweeps"]] == ["sweep_001", "sweep_002"]
    assert merged["config"]["m666_adjusted_by_data_collector"]["Q"] == 0.5
    assert merged["config"]["notes"]["base_radii_forced_mm"] == [30.0, 30.0, 30.0]
    assert f"; merged -> {out} sweeps=2" in capsys.readouterr().out
