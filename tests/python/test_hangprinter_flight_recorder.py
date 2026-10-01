from copy import deepcopy

import numpy as np
import pytest
from rerun.chunk import RrdReader

from scripts.hangprinter_flight_recorder import FlightRecording


def sample(step=0):
    return {
        "version": 1, "session": "test", "generation": 1,
        "step": step, "time": step * 0.01, "dt": 0.01 if step else 0,
        "frames": [{
            "path": "world/machines/test/anchors/A", "name": "A", "kind": "anchor",
            "position": [step, 0, 0], "quaternion": [0, 0, 0, 1],
        }],
        "cables": [{
            "machine": "test", "name": "A", "color": "#ffaa00", "radius": 0.001,
            "segments": [{
                "name": "span", "points": [[0, 0, 0], [1, 0, 0]],
                "force_n": step + 2, "force_vector_n": [step + 2, 0, 0], "origin": [0, 0, 0],
            }],
            "wraps": [],
            "lengths": {"commanded": 1, "actual": 1.1, "geometric": 1.12, "error": 0.1, "stretch": 0.02},
        }],
    }


def rows(recording, path, component):
    chunks = RrdReader(recording.path).stream().filter(content=path, components=component).to_chunks()
    result = []
    for chunk in chunks:
        batch = chunk.to_record_batch()
        result.extend(zip(batch.column("sim_step").to_pylist(), batch.column(component).to_pylist()))
    return sorted(result, key=lambda row: row[0])


def test_saved_rrd_retains_every_timestep_and_requested_archetypes(tmp_path):
    recording = FlightRecording(tmp_path, None, sample(), 0.01)
    try:
        for step in range(3):
            recording.log_sample(sample(step))
    finally:
        recording.close()

    lengths = rows(recording, "/line_lengths/test/A", "Scalars:scalars")
    assert [step for step, _ in lengths] == [0, 1, 2]
    assert lengths[-1][1] == [1, 1.1, 1.12]
    assert rows(recording, "/cable_forces/test/A", "Scalars:scalars") == [(0, [2]), (1, [3]), (2, [4])]
    positions = rows(recording, "/world/machines/test/anchors/A", "Transform3D:translation")
    assert [step for step, _ in positions] == [0, 1, 2]
    assert np.allclose(positions[-1][1], [[2, 0, 0]])
    strips = rows(recording, "/world/machines/test/cables/A/segments", "LineStrips3D:strips")
    assert [step for step, _ in strips] == [0, 1, 2]
    assert np.allclose(strips[-1][1], [[[0, 0, 0], [1, 0, 0]]])
    assert RrdReader(recording.path).blueprints()


def test_timestep_gaps_are_rejected_before_logging(tmp_path):
    recording = FlightRecording(tmp_path, None, sample(), 0.01)
    try:
        recording.log_sample(sample())
        with pytest.raises(ValueError, match="Nonconsecutive timestep"):
            recording.log_sample(sample(2))
        invalid_clock = sample(1)
        invalid_clock["time"] = 0.02
        with pytest.raises(ValueError, match="clock"):
            recording.log_sample(invalid_clock)
    finally:
        recording.close()
    assert rows(recording, "/line_lengths/test/A", "Scalars:scalars") == [(0, [1, 1.1, 1.12])]


def test_removing_cables_clears_old_geometry_and_telemetry(tmp_path):
    recording = FlightRecording(tmp_path, None, sample(), 0.01)
    try:
        recording.log_sample(sample())
        without_cable = deepcopy(sample(1))
        without_cable["cables"] = []
        recording.log_sample(without_cable)
    finally:
        recording.close()
    assert rows(recording, "/world/machines/test/cables/A/segments", "Clear:is_recursive") == [(1, [True])]
    assert rows(recording, "/line_lengths/test/A", "Clear:is_recursive") == [(1, [True])]
