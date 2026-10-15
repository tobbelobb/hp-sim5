import numpy as np
import rerun as rr
from rerun.chunk import RrdReader

from cable_joints_3d.columnar_recording import ColumnarRecording


def test_columns_keep_sparse_ragged_rows_all_clocks_and_partial_final_batch(tmp_path):
    output = tmp_path / 'columns.rrd'
    stream = rr.RecordingStream('bounded live columns')
    stream.set_sinks(rr.FileSink(output))
    columns = ColumnarRecording(stream, max_samples=4)
    expected = []
    try:
        for step in range(11):
            columns.begin(scene_generation=('sequence', 3), sim_step=('sequence', step),
                          sim_time=('duration', step * .002),
                          wall_time=('timestamp', np.datetime64(1800000000000 + step, 'ms')))
            # Zero, one or two instances; a signal can skip samples completely.
            values = list(range(step % 3))
            columns.append('ragged', rr.Scalars, scalars=values)
            if step % 2:
                columns.append('sparse', rr.Transform3D, translation=[[step, 0, 0]], quaternion=[[0, 0, 0, 1]])
            columns.end()
            expected.append((step, values))
        columns.flush()
        columns.flush()  # owner finalization can follow an explicit live flush
        stream.flush(timeout_sec=5)
    finally:
        stream.disconnect()
    reader = RrdReader(output)
    chunks = reader.stream().filter(content='/ragged', components='Scalars:scalars').to_chunks()
    rows = []
    for chunk in chunks:
        batch = chunk.to_record_batch()
        steps = batch.column('sim_step').to_pylist()
        assert batch.column('scene_generation').to_pylist() == [3] * len(steps)
        assert batch.column('sim_time').cast('int64').to_pylist() == [step * 2000000 for step in steps]
        assert batch.column('wall_time').cast('int64').to_pylist() == [(1800000000000 + step) * 1000000 for step in steps]
        rows.extend(zip(steps, batch.column('Scalars:scalars').to_pylist()))
    assert sorted(rows) == expected
    sparse = reader.stream().filter(content='/sparse', components='Transform3D:translation').to_chunks()
    assert sorted(step for chunk in sparse for step in chunk.to_record_batch().column('sim_step').to_pylist()) == [1, 3, 5, 7, 9]


def test_live_and_instance_limits_flush_without_waiting_for_a_full_batch(monkeypatch):
    class Sink:
        calls = []

        def send_columns(self, path, **kwargs):
            self.calls.append(path)

    now = [0.]
    monkeypatch.setattr('cable_joints_3d.columnar_recording.time.monotonic', lambda: now[0])
    sink = Sink()
    columns = ColumnarRecording(sink, max_instances=3)
    for step in range(3):
        columns.begin(sim_step=('sequence', step))
        columns.append('signal', rr.Scalars, scalars=[step])
        columns.end()
    assert sink.calls == ['signal']
    columns.begin(sim_step=('sequence', 3))
    columns.append('signal', rr.Scalars, scalars=[3])
    now[0] = 1.1
    columns.end()
    assert sink.calls == ['signal', 'signal']
