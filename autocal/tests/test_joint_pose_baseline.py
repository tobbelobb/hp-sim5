import numpy as np
import pytest

from autocal.tools.joint_pose_baseline import fit_joint_ranges
from autocal.tests.test_pose_fit import _dataset, ANCHORS, SEED


@pytest.mark.parametrize("offset", [0., 1.5])
def test_joint_range_baseline_recovers_geometry_and_its_own_offsets(offset):
    fit = fit_joint_ranges(_dataset((0., 0., 0., offset)), SEED)
    assert fit["success"]
    assert np.max(np.abs(fit["anchors"] - ANCHORS)) < .05
    assert fit["home_offsets_mm"][-1] == pytest.approx(offset, abs=.01)
