import copy

import numpy as np

import autocal.fit_recovery as recovery


def test_sparse_recovery_subset_preserves_endpoints_and_source_dataset():
    dataset = {
        "machine_type": "slideprinter",
        "sweeps": [{
            "drive_anchor": 1,
            "data_points": [
                {
                    "source_drive_anchor": 1,
                    "drive_setpoint_mm": float(i),
                    "l_drive": float(i),
                }
                for i in range(10)
            ],
        }],
    }
    before = copy.deepcopy(dataset)
    sparse = recovery.sparse_recovery_dataset(dataset)

    assert dataset == before
    selected = sparse["sweeps"][0]["data_points"]
    assert len(selected) == 5
    assert selected[0]["drive_setpoint_mm"] == 0.0
    assert selected[-1]["drive_setpoint_mm"] == 9.0


def test_recovery_geometry_rejects_boundary_and_degenerate_candidates():
    valid = {
        "machine_type": "slideprinter",
        "anchors": np.asarray([[0.0, -10.0], [8.0, 5.0], [-8.0, 5.0]]),
    }
    boundary = {
        "machine_type": "hangprinter_4",
        "anchors": np.asarray([[0.0, -10.0, -1.0], [8.0, 5.0, -1.0], [-8.0, 5.0, -1.0], [0.0, 0.0, 1e-6]]),
    }
    degenerate = {
        "machine_type": "slideprinter",
        "anchors": np.asarray([[0.0, -10.0], [0.0, 5.0], [0.0, 15.0]]),
    }

    assert recovery.recovery_geometry_valid(valid)
    assert not recovery.recovery_geometry_valid(boundary)
    assert not recovery.recovery_geometry_valid(degenerate)

