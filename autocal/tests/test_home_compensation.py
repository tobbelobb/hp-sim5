import numpy as np
import pytest

from autocal.home_compensation import compensate_home

ANCHORS = np.array([[0., -1900., -280.], [1645., 950., -280.],
                    [-1645., 950., -280.], [0., 0., 1900.]])


def evidence_for(q):
    # Construct independent extensions satisfying material-length conservation.
    home = np.zeros(4)
    offsets = np.array([0., 0., 0., 0.4])
    measured = np.linalg.norm(ANCHORS, axis=1) + offsets + home - np.linalg.norm(ANCHORS - q, axis=1)
    return offsets, {"provenance": "synthetic known extension", "measurement_extension_mm": measured,
                     "home_extension_mm": home, "range_uncertainty_mm": 0.01}


def test_recovers_physical_home_and_translation_sign():
    q = np.array([0.0, 0.0, -0.3])
    offsets, evidence = evidence_for(q)
    anchors, actual = compensate_home(ANCHORS, offsets, evidence)
    np.testing.assert_allclose(actual, q, atol=1e-9)
    np.testing.assert_allclose(anchors, ANCHORS - q, atol=1e-9)


def test_missing_inconsistent_and_unobservable_evidence_rejected():
    offsets, evidence = evidence_for(np.array([0.0, 0.0, -0.3]))
    with pytest.raises(ValueError):
        compensate_home(ANCHORS, offsets, None)
    with pytest.raises(ValueError, match='loaded encoder home'):
        compensate_home(ANCHORS, offsets, dict(evidence, home_extension_mm=[0.1] * 4))
    bad = dict(evidence, measurement_extension_mm=evidence['measurement_extension_mm'] + [0., 0., 0., 1.])
    with pytest.raises(ValueError, match='inconsistent'):
        compensate_home(ANCHORS, offsets, bad)
    flat = ANCHORS.copy()
    flat[:, 2] = 0
    flat[3, 0] = 1000
    with pytest.raises(ValueError, match='ambiguous'):
        compensate_home(flat, offsets, evidence)
    with pytest.raises(ValueError, match='uncertain'):
        compensate_home(ANCHORS, offsets, dict(evidence, range_uncertainty_mm=2.))
