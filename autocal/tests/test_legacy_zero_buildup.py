import importlib.util
import sys
from pathlib import Path

import numpy as np


def _load_legacy_util():
    legacy_dir = Path(__file__).resolve().parents[1] / "auto-calibration-simulation-for-hangprinter"
    util_path = legacy_dir / "util.py"
    sys.path.insert(0, str(legacy_dir))
    spec = importlib.util.spec_from_file_location("legacy_util", util_path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    finally:
        if sys.path and sys.path[0] == str(legacy_dir):
            sys.path.pop(0)
    return module


def test_motor_pos_samples_to_distances_zero_buildup_no_div0():
    util = _load_legacy_util()
    motor_deg = np.array([[10.0, 20.0, 30.0]], dtype=float)
    spool_r = np.array([75.0, 75.0, 75.0], dtype=float)
    mech_adv = np.array([2.0, 2.0, 2.0], dtype=float)
    out = util.motor_pos_samples_to_distances_relative_to_origin(
        motor_deg,
        spool_buildup_factor=0.0,
        spool_r=spool_r,
        mech_adv_=mech_adv,
        lines_per_spool_=np.ones(3),
    )
    assert np.all(np.isfinite(out))
    gear = float(util.spool_gear_teeth) / float(util.motor_gear_teeth)
    scale = (2.0 * np.pi * spool_r) / (gear * mech_adv * 360.0)
    assert np.allclose(out, motor_deg * scale)
