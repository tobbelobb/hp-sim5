import numpy as np

from autocal.flex import FlexModel


def test_flex_model_corrects_length_monotonically():
    m666 = {"S": 20000.0, "U": [1.0], "Y": [500.0]}
    model = FlexModel.from_m666(m666, num_axes=1)
    assert model is not None

    l = np.array([2000.0], dtype=float)
    d0 = model.corrected_distance_mm(l, np.array([0.0]), axis=0)[0]
    d1 = model.corrected_distance_mm(l, np.array([10.0]), axis=0)[0]

    assert d1 > d0
    assert np.isclose(d0, 2000.0)


def test_flex_model_handles_nan_tension_as_zero():
    m666 = {"S": 20000.0, "U": [1.0], "Y": [0.0]}
    model = FlexModel.from_m666(m666, num_axes=1)
    assert model is not None

    l = np.array([1234.0], dtype=float)
    out = model.corrected_distance_mm(l, np.array([np.nan]), axis=0)[0]
    assert np.isclose(out, 1234.0)



def test_flex_inverse_matches_firmware_ma_squared_and_strand_scaling():
    for ma, strands in ((1., 2.), (2., 2.), (4., 4.)):
        model = FlexModel.from_m666({'S':20000., 'U':[ma], 'N':[strands], 'Y':[429.1]}, num_axes=1)
        distance, force = np.array([1000., 1800., 2200.]), np.array([2., 4., 12.])
        material = distance - force*(ma*distance+429.1)/(20000.*ma**2*strands)
        assert np.allclose(model.corrected_distance_mm(material, force, axis=0), distance, atol=1e-10)
