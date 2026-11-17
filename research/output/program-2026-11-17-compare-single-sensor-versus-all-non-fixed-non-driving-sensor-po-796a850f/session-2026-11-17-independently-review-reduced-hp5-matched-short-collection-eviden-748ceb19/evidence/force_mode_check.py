"""Recompute the four-/five-line fits under the *actual* redundant-line force.

`collect_run.py` records `redundant_force_mode` as "sensor" whenever the
collector path contains the substring "prototype". The control collector is
called `prototype-control`, so both control arms were labelled "sensor" even
though their redundant line was at `force_low`. This script compares the
labelled model with the physically correct one.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import review_fit as R  # noqa: E402


def main() -> int:
    out = {}
    for run in ("ctl-1", "twosensor-1", "ctl-low1p0"):
        dataset, truth = R.load(run)
        invocation = json.loads((R.SOURCE / "runs" / run / "invocation.json").read_text())
        labelled = invocation.get("redundant_force_mode", "sensor")
        actual = "sensor" if run == "twosensor-1" else "passive"
        out[run] = {"labelled_mode": labelled, "actual_mode": actual,
                    "force_low_n": invocation["force_low_n"], "fits": {}, "truth": {}}
        for mode in sorted({labelled, actual}):
            for measure_extra in (False, True):
                key = f"{mode}_{'five_line' if measure_extra else 'four_line'}"
                out[run]["fits"][key] = R.pipeline(
                    dataset, truth, measure_extra=measure_extra, offset_lines=[3, 4],
                    redundant_force_mode=mode, start=truth,
                    extra_starts=(truth * 1.05,))
                out[run]["truth"][key] = R.truth_residual(
                    dataset, truth, measure_extra=measure_extra, redundant_force_mode=mode)
    Path(HERE / "force-mode-check.json").write_text(json.dumps(out, indent=2) + "\n")
    for run, values in out.items():
        print(f"== {run} labelled={values['labelled_mode']} actual={values['actual_mode']} "
              f"force_low={values['force_low_n']}")
        for key, fit in values["fits"].items():
            print(f"  {key:22s} anchor {fit['max_aligned_abs_coordinate_mm']:9.3f} mm  "
                  f"rms {fit['rms_mm']:.4f}  truth_rms {values['truth'][key]['rms_mm']:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
