"""Stable-estimate, patience and stop-file policy for the calibration loop."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from autocal._autocal_common import (
    FULL_AUTO_STOP_REASON_PATIENCE, STABLE_STOP_REASON,
    _float_or_none, _fmt_float,
)

# Stop once this many consecutive sweeps each moved every anchor coordinate by
# at most the stable step. Each sweep is the planner's most informative
# candidate, so an estimate that new information no longer moves has converged
# to what the data can resolve ("stabilizing predictions" stopping). Agreement
# is not accuracy: systematic bias moves no estimate.
STABLE_SWEEPS = 2
STABLE_STEP_MM = 0.5
HP5_GOOD_STEP_MM = 2.0
HP5_GOOD_STD_MM = 1.0
HP5_GOOD_RMS_MM = 0.05


def history_rank_position(history_candidates, *, iteration: int) -> Optional[int]:
    """Rank of an iteration's anchor uncertainty among all admissible fits; 1 is lowest.

    Ties go to the newer fit, which saw more data. A fit whose uncertainty is
    unknown ranks after every known one.
    """
    def key(candidate):
        value = _float_or_none(candidate.get("anchor_uncertainty_mm"))
        return (value if value is not None else float("inf"), -int(candidate.get("iteration", 0)))
    ordered = sorted(history_candidates, key=key)
    return next((index for index, candidate in enumerate(ordered, start=1)
                 if int(candidate.get("iteration", -1)) == int(iteration)), None)


@dataclass(frozen=True)
class StopDecision:
    rank: Optional[int]
    history_improved: bool
    stable_hit: bool
    step_mm: Optional[float]
    decision: str
    good_fit_hit: bool = False

    @property
    def reason(self) -> Optional[str]:
        if self.decision != "accept":
            return None
        if self.good_fit_hit:
            return "good-fit"
        return STABLE_STOP_REASON if self.stable_hit else FULL_AUTO_STOP_REASON_PATIENCE


@dataclass
class StoppingPolicy:
    patience: int
    stop_file: Path
    stable_step_mm: float = STABLE_STEP_MM
    machine_type: str = ""
    no_improve: int = field(default=0, init=False)
    stable_streak: int = field(default=0, init=False)
    previous_anchors: Optional[np.ndarray] = field(default=None, init=False)
    good_fit_streak: int = field(default=0, init=False)
    last_good_points: int = field(default=0, init=False)

    def __post_init__(self):
        self.patience = max(1, int(self.patience))

    @property
    def remaining(self) -> int:
        return max(0, self.patience - self.no_improve)

    def stop_requested(self) -> bool:
        try:
            return self.stop_file.exists()
        except OSError:
            return False

    def announce(self, log_console: Callable[[str], None]) -> None:
        if self.stable_step_mm > 0.0:
            log_console(
                f"; full-auto: stops when {STABLE_SWEEPS} consecutive sweeps each move every anchor "
                f"coordinate by at most {self.stable_step_mm:g} mm (--stable-step-mm 0 disables)"
            )
            if self.machine_type in ("hangprinter_5", "hangprinter_5_pulley_reduced"):
                log_console(f"; HP5 also accepts two good fits: range RMS <={HP5_GOOD_RMS_MM:g} mm, "
                            f"estimated anchor std <={HP5_GOOD_STD_MM:g} mm, agreement <={HP5_GOOD_STEP_MM:g} mm")

    def _step(self, history, iteration: int) -> Optional[float]:
        current = next((c for c in reversed(history) if int(c.get("iteration", -1)) == int(iteration)), None)
        anchors = (current or {}).get("plan", {}).get("anchors")
        if anchors is None:
            return None
        anchors = np.asarray(anchors, dtype=float)
        previous, self.previous_anchors = self.previous_anchors, anchors
        if previous is None or previous.shape != anchors.shape or not np.all(np.isfinite(anchors)):
            return None
        return float(np.max(np.abs(anchors - previous)))

    def update(self, history, *, iteration: int, admissible: bool,
               underconstrained: bool, log_line: Callable[[str], None]) -> StopDecision:
        rank = history_rank_position(history, iteration=iteration) if admissible else None
        improved = admissible and rank == 1
        step_mm = None
        current = next((c for c in reversed(history) if int(c.get("iteration", -1)) == int(iteration)), {})
        plan = current.get("plan") or {}
        dataset = plan.get("dataset") or {}
        points = [point for sweep in dataset.get("sweeps", []) for point in sweep.get("data_points", [])]
        extra_coverage = (sum(any(extra.get("accepted") for extra in point.get("extra_sensor_ranges", []))
                              for point in points) / len(points)) if points else 0.
        std = _float_or_none(current.get("max_std_mm"))
        rms = _float_or_none((plan.get("pose_fit") or {}).get("rms_mm"))
        hp5 = (plan.get("machine_type") in ("hangprinter_5", "hangprinter_5_pulley_reduced")
               and any(point.get("extra_sensor_ranges") for point in points))
        good = (hp5 and admissible and not underconstrained and self.stable_step_mm > 0
                and len(dataset.get("sweeps", [])) >= 4 and extra_coverage >= 0.5
                and not plan.get("info_rank_deficient", True)
                and (plan.get("pose_fit") or {}).get("stretch_corrected", False)
                and std is not None and std <= HP5_GOOD_STD_MM
                and rms is not None and rms <= HP5_GOOD_RMS_MM)
        if admissible:
            self.no_improve = 0 if improved else self.no_improve + 1
            step_mm = self._step(history, iteration)
            stable = step_mm is not None and step_mm <= self.stable_step_mm
            self.stable_streak = self.stable_streak + 1 if stable else 0
        if good:
            self.good_fit_streak = (self.good_fit_streak + 1
                                    if step_mm is not None and step_mm <= HP5_GOOD_STEP_MM
                                    and len(points) > self.last_good_points else 1)
            self.last_good_points = len(points)
        else:
            self.good_fit_streak = 0
        good_fit_hit = self.good_fit_streak >= 2
        stable_hit = self.stable_step_mm > 0.0 and self.stable_streak >= STABLE_SWEEPS and (not hp5 or good_fit_hit)
        log_line(
            f"; stable_stop step_mm={_fmt_float(step_mm)} limit_mm={self.stable_step_mm:g} "
            f"streak={self.stable_streak}/{STABLE_SWEEPS}"
            f" good_fits={self.good_fit_streak}/2"
        )
        accept = not underconstrained and (good_fit_hit or stable_hit or self.no_improve >= self.patience)
        return StopDecision(rank, bool(improved), stable_hit, step_mm,
                            "accept" if accept else "collect", good_fit_hit)
