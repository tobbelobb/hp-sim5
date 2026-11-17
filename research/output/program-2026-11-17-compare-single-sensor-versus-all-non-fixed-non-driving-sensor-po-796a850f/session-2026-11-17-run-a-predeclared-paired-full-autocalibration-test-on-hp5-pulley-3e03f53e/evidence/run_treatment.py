"""Launch one treatment full autocal run with the experiment-only fifth range.

The production command line is unchanged; this launcher installs the range
fitter/stretch patches in-process and lets autocal.py's own equal-strain phase
and planner use the experiment collector selected by HP5_EXTRA_RANGE_COLLECTOR.

Baseline runs use autocal/autocal.py directly and never import this module.
"""
from __future__ import annotations

import hashlib
import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SESSION = HERE.parent
REPO = next(p for p in SESSION.parents if (p / "autocal/autocal.py").exists())
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO))

import extra_range_fit  # noqa: E402


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    collector = os.environ.get("HP5_EXTRA_RANGE_COLLECTOR")
    if not collector:
        print("HP5_EXTRA_RANGE_COLLECTOR must point at the experiment collector", file=sys.stderr)
        return 2
    collector_path = Path(collector)
    if not collector_path.is_absolute():
        collector_path = (REPO / collector_path).resolve()
    if not collector_path.exists():
        print(f"experiment collector not found: {collector_path}", file=sys.stderr)
        return 2
    extra_range_fit.install()
    record = {
        "started_unix": time.time(),
        "argv": sys.argv,
        "repo": str(REPO),
        "session": str(SESSION),
        "collector": str(collector_path),
        "collector_sha256": sha256(collector_path),
        "fit_module_sha256": sha256(HERE / "extra_range_fit.py"),
        "gate_version": extra_range_fit.GATE_VERSION,
        "gate_constants": {
            "min_segment_points": extra_range_fit.MIN_SEGMENT_POINTS,
            "min_drive_travel_mm": extra_range_fit.MIN_DRIVE_TRAVEL_MM,
            "min_sensor_travel_mm": extra_range_fit.MIN_SENSOR_TRAVEL_MM,
            "frozen_abs_mm": extra_range_fit.FROZEN_ABS_MM,
            "low_info_abs_mm": extra_range_fit.LOW_INFO_ABS_MM,
            "low_info_fraction_of_sensor": extra_range_fit.LOW_INFO_FRACTION_OF_SENSOR,
            "min_coupling": extra_range_fit.MIN_COUPLING,
            "shape_rms_mm": extra_range_fit.SHAPE_RMS_MM,
            "shape_max_mm": extra_range_fit.SHAPE_MAX_MM,
        },
        "installed": True,
    }
    (SESSION / "runs").mkdir(parents=True, exist_ok=True)
    (SESSION / "runs" / "treatment-launcher.json").write_text(json.dumps(record, indent=2) + "\n")
    print(f"; experiment extra-range treatment launcher: collector={collector_path}")
    from autocal.autocal import main as autocal_main
    return int(autocal_main(sys.argv[1:]))


if __name__ == "__main__":
    raise SystemExit(main())
