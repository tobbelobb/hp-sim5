import numpy as np

from autocal.predictive_validation import score_future_sweeps


def _dataset():
    anchors = np.array([[-400.0, 0.0], [400.0, 0.0], [0.0, 500.0]])
    phi = np.linspace(0.0, np.pi, 40)
    positions = anchors[0] + 600.0 * np.column_stack([np.cos(phi), np.sin(phi)])
    points = [
        {
            "l_drive": float(d - np.linalg.norm(anchors[1])),
            "l_sensor": float(s - np.linalg.norm(anchors[2])),
        }
        for d, s in zip(
            np.linalg.norm(positions - anchors[1], axis=1),
            np.linalg.norm(positions - anchors[2], axis=1),
        )
    ]
    sweep = {
        "id": "heldout",
        "fixed_anchors": [0],
        "fixed_lengths": [200.0],
        "drive_anchor": 1,
        "sensor_anchor": 2,
        "data_points": points,
    }
    rng = np.random.default_rng(12)
    for point in points:
        point["l_drive"] += float(rng.normal(0.0, 0.2))
        point["l_sensor"] += float(rng.normal(0.0, 0.2))
    points[9]["l_sensor"] += 20.0
    return {
        "machine_type": "slideprinter",
        "num_anchors": 3,
        "dimensions": 2,
        "sweeps": [sweep],
    }, anchors


def test_score_future_sweeps_prefers_geometry_that_predicts_sensor_lengths():
    dataset, anchors = _dataset()

    exact = score_future_sweeps(dataset, anchors, training_sweep_ids=[])
    wrong = score_future_sweeps(
        dataset,
        anchors + np.array([[0.0, 0.0], [35.0, -20.0], [-15.0, 25.0]]),
        training_sweep_ids=[],
    )

    assert exact is not None and exact[1] == 1
    assert wrong is not None and wrong[1] == 1
    assert exact[0] < wrong[0]


def test_score_future_sweeps_requires_a_whole_sweep_outside_training():
    dataset, anchors = _dataset()

    assert score_future_sweeps(dataset, anchors, training_sweep_ids=["heldout"]) is None
