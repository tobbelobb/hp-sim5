"""Stable-estimate, patience and stop-file policy for the calibration loop."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from autocal._autocal_common import (
    FULL_AUTO_STOP_REASON_PATIENCE, STABLE_STOP_REASON,
    _float_or_none, _fmt_float, _full_auto_history_selection_score,
)

# Stop once this many consecutive sweeps each moved every anchor coordinate by
# at most the stable step. Each sweep is the planner's most informative
# candidate, so an estimate that new information no longer moves has converged
# to what the data can resolve ("stabilizing predictions" stopping). Agreement
# is not accuracy: systematic bias moves no estimate.
STABLE_SWEEPS = 2
STABLE_STEP_MM = 0.5


def history_rank_position(history_candidates, *, iteration: int) -> Optional[int]:
    if not history_candidates:
        return None
    scored = []
    for candidate in history_candidates:
        candidate_rank = _float_or_none(candidate.get("rank_score"))
        selection_score, _selection_info = _full_auto_history_selection_score(
            candidate_rank,
            iteration_index=int(candidate.get("iteration", 1)),
            coverage_adjust=_float_or_none(candidate.get("rank_coverage_adjust")),
        )
        rel_std = _float_or_none(candidate.get("rel_std"))
        cost = _float_or_none(candidate.get("cost"))
        sort_key = (
            float(selection_score),
            float(candidate_rank) if candidate_rank is not None else float("inf"),
            float(rel_std) if rel_std is not None else float("inf"),
            float(cost) if cost is not None else float("inf"),
            -float(candidate.get("iteration", 0)),
        )
        scored.append((sort_key, candidate))
    scored.sort(key=lambda item: item[0])
    for idx, (_sort_key, candidate) in enumerate(scored, start=1):
        if int(candidate.get("iteration", -1)) == int(iteration):
            return idx
    return None



@dataclass(frozen=True)
class StopDecision:
    rank: Optional[int]
    history_improved: bool
    stable_hit: bool
    step_mm: Optional[float]
    decision: str

    @property
    def reason(self) -> Optional[str]:
        if self.decision != "accept":
            return None
        return STABLE_STOP_REASON if self.stable_hit else FULL_AUTO_STOP_REASON_PATIENCE


@dataclass
class StoppingPolicy:
    patience: int
    stop_file: Path
    stable_step_mm: float = STABLE_STEP_MM
    no_improve: int = field(default=0, init=False)
    stable_streak: int = field(default=0, init=False)
    previous_anchors: Optional[np.ndarray] = field(default=None, init=False)

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
        if admissible:
            self.no_improve = 0 if improved else self.no_improve + 1
            step_mm = self._step(history, iteration)
            stable = step_mm is not None and step_mm <= self.stable_step_mm
            self.stable_streak = self.stable_streak + 1 if stable else 0
        stable_hit = self.stable_step_mm > 0.0 and self.stable_streak >= STABLE_SWEEPS
        log_line(
            f"; stable_stop step_mm={_fmt_float(step_mm)} limit_mm={self.stable_step_mm:g} "
            f"streak={self.stable_streak}/{STABLE_SWEEPS}"
        )
        accept = not underconstrained and (stable_hit or self.no_improve >= self.patience)
        return StopDecision(rank, bool(improved), stable_hit, step_mm,
                            "accept" if accept else "collect")
