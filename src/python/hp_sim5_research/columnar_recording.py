"""Bounded column batches for live simulation telemetry."""
import time

import rerun as rr


class ColumnarRecording:
    def __init__(self, recording, *, max_samples=128, max_instances=65536, flush_interval=1.):
        if max_samples < 1 or max_instances < 1 or flush_interval <= 0:
            raise ValueError('Column batch limits must be positive')
        self.recording = recording
        self.max_samples = max_samples
        self.max_instances = max_instances
        self.flush_interval = flush_interval
        self._groups = {}
        self._times = []
        self._time = ()
        self._row = None
        self._samples = self._instances = 0
        self._last_flush = time.monotonic()

    def begin(self, **timelines):
        """Set this sample's explicit name=(sequence/duration/timestamp, value) indexes."""
        self._time = tuple((name, kind, value) for name, (kind, value) in timelines.items())
        self._row = None

    def append(self, path, archetype, **fields):
        """Append detached per-instance values; construct no Rerun objects per sample."""
        count = len(next(iter(fields.values())))
        if any(len(values) != count for values in fields.values()):
            raise ValueError('Column fields must have the same instance count')
        if self._row is None:
            self._row = len(self._times)
            self._times.append(self._time)
        key = (path, archetype, tuple(fields))
        if key not in self._groups:
            self._groups[key] = ([], [], {name: [] for name in fields})
        rows, lengths, columns = self._groups[key]
        rows.append(self._row)
        lengths.append(count)
        for name, values in fields.items():
            columns[name].extend(values)
        self._instances += count

    def end(self):
        self._samples += 1
        if (self._samples >= self.max_samples or self._instances >= self.max_instances
                or time.monotonic() - self._last_flush >= self.flush_interval):
            self.flush()

    def flush(self):
        indexes = {}
        for (path, archetype, _), (rows, lengths, fields) in self._groups.items():
            selection = tuple(rows)
            if selection not in indexes:
                times = [self._times[row] for row in rows]
                indexes[selection] = [rr.TimeColumn(name, **{kind: [sample[i][2] for sample in times]})
                                      for i, (name, kind, _) in enumerate(times[0])]
            self.recording.send_columns(path, indexes=indexes[selection],
                                        columns=archetype.columns(**fields).partition(lengths), strict=True)
        self._groups.clear()
        self._times.clear()
        self._row = None
        self._samples = self._instances = 0
        self._last_flush = time.monotonic()
