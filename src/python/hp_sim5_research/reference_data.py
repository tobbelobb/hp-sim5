"""Read-only access to the external DaRUS cable datasets.

The raw files live outside Git (see ai_docs/externally_captured_reference_data/README.md),
usually on a Google Drive mount that sometimes returns transient I/O errors. Nothing
here writes into a dataset directory.
"""
from dataclasses import dataclass, field
from pathlib import Path
import time

import numpy as np

REFERENCE = Path(__file__).resolve().parents[3] / 'ai_docs' / 'externally_captured_reference_data'
DATASETS = {
    '3600': 'Cable_Actuator_Test_Rig_Stuttgart_2023_DaRUS-3600',
    '4614': 'Experimental_Investigation_2025_DaRUS-4614',
    '5171': 'Experimental_modelling_of_synthetic_fibre_ropes_for_compensation_2026_DaRUS-5171',
}


def dataset_root(dataset):
    return REFERENCE / DATASETS[dataset] / 'data'


def incomplete(directory):
    """True while a fetch_darus.py download is still writing into the directory tree."""
    return any(Path(directory).rglob('*.download'))


def _read_text(path, attempts=5):
    for attempt in range(attempts):
        try:
            return Path(path).read_text(encoding='latin-1')
        except OSError:
            if attempt == attempts - 1:
                raise
            time.sleep(2 ** attempt)


@dataclass
class Scope:
    """One TwinCAT Scope export: a shared time base and named channels."""
    path: Path
    t: np.ndarray  # s, from the first channel's time column
    channels: dict = field(default_factory=dict)
    sample_ms: float = 1.0

    def __getitem__(self, name):
        return self.channels[name]

    def recorded(self, trigger):
        """Copy restricted to samples where `trigger` is nonzero, time zeroed at its start."""
        keep = self.channels[trigger] != 0
        t = self.t[keep]
        return Scope(self.path, t - t[0], {k: v[keep] for k, v in self.channels.items()}, self.sample_ms)


def read_scope(path):
    """Parse a TwinCAT Scope CSV export (tab separated, a time column before every channel).

    The header length varies between exports, so channel names come from the first
    `Name` row with more than one channel and data start at the first numeric row.
    """
    lines = _read_text(path).splitlines()
    names = sample_ms = None
    start = None
    for i, line in enumerate(lines):
        cells = line.rstrip('\t').split('\t')
        if names is None and cells[0] == 'Name' and len(cells) > 2:
            names = cells[1::2]
        elif names is not None and cells[0] == 'SampleTime[ms]':
            sample_ms = {float(c) for c in cells[1::2]}
        elif names is not None and cells[0] and cells[0].lstrip('-').replace('.', '', 1).isdigit():
            start = i
            break
    if names is None or start is None:
        raise ValueError(f'Not a TwinCAT Scope export: {path}')
    if sample_ms is not None and len(sample_ms) != 1:
        raise ValueError(f'Channels have different sample times {sorted(sample_ms)}: {path}')
    rows = [l.rstrip('\t').split('\t') for l in lines[start:] if l.strip()]
    width = 2 * len(names)
    data = np.array([r[:width] for r in rows if len(r) >= width], dtype=float)
    times = data[:, 0::2]
    if not np.allclose(times, times[:, :1]):
        raise ValueError(f'Channel time columns disagree: {path}')
    step = sample_ms.pop() if sample_ms else 1.0
    return Scope(Path(path), times[:, 0] * 1e-3, dict(zip(names, data[:, 1::2].T)), step)


def read_tracker(path):
    """FARO laser-tracker CSV of DaRUS-4614: `x; y; z; unix time; ...` with decimal commas.

    Returns an (n, 3) position array in metres and the unix time stamps in seconds.
    """
    rows = [l.replace(',', '.').split(';') for l in _read_text(path).splitlines() if l.strip()]
    data = np.array(rows, dtype=float)
    return data[:, :3], data[:, 3]
