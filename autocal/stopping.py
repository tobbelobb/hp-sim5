"""Patience, geometric-agreement and stop-file policy for the calibration loop."""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from autocal._autocal_common import (
    FULL_AUTO_STOP_REASON_PATIENCE, SPREAD_STOP_REASON,
    _float_or_none, _fmt_float, _full_auto_history_selection_score,
)
from autocal.history_selection import (
    SPREAD_STOP_CONSECUTIVE, SPREAD_STOP_FRAC, SPREAD_STOP_MIN_CANDIDATES,
    consensus_anchor_estimate, consensus_spread_hit,
)


def spread_stop_fraction() -> float:
    """Optional early-stop heuristic based on geometric fit agreement.

    Agreement is not calibrated uncertainty or proof of accuracy. The default
    and its override are retained pending broader acquisition evidence.
    `HP_SIM5_AUTOCAL_SPREAD_STOP_FRAC=off` disables this heuristic; invalid or
    non-finite values retain the default.
    """
    raw = os.environ.get("HP_SIM5_AUTOCAL_SPREAD_STOP_FRAC", "").strip()
    if not raw:
        return float(SPREAD_STOP_FRAC)
    if raw.lower() in {"off", "false", "no", "none", "disabled"}:
        return 0.0
    try:
        value = float(raw)
    except ValueError:
        return float(SPREAD_STOP_FRAC)
    if not np.isfinite(value):
        return float(SPREAD_STOP_FRAC)
    return 0.0 if value <= 0.0 else float(value)


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
    spread_hit: bool
    spread_mm: Optional[float]
    spread_limit_mm: Optional[float]
    decision: str

    @property
    def reason(self) -> Optional[str]:
        if self.decision != "accept":
            return None
        return SPREAD_STOP_REASON if self.spread_hit else FULL_AUTO_STOP_REASON_PATIENCE


@dataclass
class StoppingPolicy:
    patience: int
    stop_file: Path
    spread_fraction: float = field(default_factory=spread_stop_fraction)
    no_improve: int = field(default=0, init=False)
    spread_streak: int = field(default=0, init=False)

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
        if self.spread_fraction > 0.0:
            log_console(
                f"; full-auto: early stop when the two closest distinct fits agree to "
                f"{self.spread_fraction:g} of the machine size, sustained "
                f"{SPREAD_STOP_CONSECUTIVE} iterations, from "
                f"{SPREAD_STOP_MIN_CANDIDATES} candidates "
                f"(disable with HP_SIM5_AUTOCAL_SPREAD_STOP_FRAC=off)"
            )

    def update(self, history, *, iteration: int, admissible: bool,
               underconstrained: bool, log_line: Callable[[str], None]) -> StopDecision:
        rank = history_rank_position(history, iteration=iteration) if admissible else None
        improved = admissible and rank == 1
        if admissible:
            self.no_improve = 0 if improved else self.no_improve + 1

        spread_hit = False
        spread_mm = limit_mm = None
        if self.spread_fraction > 0.0 and len(history) >= SPREAD_STOP_MIN_CANDIDATES:
            anchors, _members, spread = consensus_anchor_estimate(history)
            hit, spread_mm, limit_mm = consensus_spread_hit(
                anchors, spread, frac=self.spread_fraction,
            )
            self.spread_streak = self.spread_streak + 1 if hit else 0
            spread_hit = self.spread_streak >= SPREAD_STOP_CONSECUTIVE
            log_line(
                f"; history_select: spread_stop spread_mm={_fmt_float(spread_mm)} "
                f"limit_mm={_fmt_float(limit_mm)} "
                f"streak={self.spread_streak}/{SPREAD_STOP_CONSECUTIVE}"
            )
        accept = not underconstrained and (spread_hit or self.no_improve >= self.patience)
        return StopDecision(rank, bool(improved), spread_hit, spread_mm, limit_mm,
                            "accept" if accept else "collect")
