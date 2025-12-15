from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Sequence, Union

import numpy as np


def _axis_value(values: Any, axis: int, default: float) -> float:
    if values is None:
        return float(default)
    if isinstance(values, (int, float)):
        try:
            return float(values)
        except Exception:
            return float(default)
    if isinstance(values, (list, tuple)) and axis < len(values):
        try:
            return float(values[axis])
        except Exception:
            return float(default)
    return float(default)


@dataclass(frozen=True)
class FlexModel:
    """
    Linear spring-based flex correction modeled after RRF HangprinterKinematics.

    RRF uses:
      springK(springLength) = springKPerUnitLength / springLength
      flex = -F / (springK * mechanicalAdvantage)
          = -F * springLength / (springKPerUnitLength * mechanicalAdvantage)

    Here we invert that to correct measured (unstretched/material) lengths into
    geometric distances (mm).
    """

    spring_k_per_unit_length: float
    mechanical_advantage: Sequence[float]
    guy_wire_lengths: Sequence[float]
    spring_k_multiplier: Union[float, Sequence[float]] = 1.0

    @classmethod
    def from_m666(
        cls,
        m666: Optional[dict],
        *,
        num_axes: int,
        spring_k_multiplier: Union[float, Sequence[float]] = 1.0,
    ) -> Optional["FlexModel"]:
        if not isinstance(m666, dict):
            return None
        if num_axes <= 0:
            return None

        s = m666.get("S")
        u = m666.get("U")
        y = m666.get("Y")

        spring_k_per_unit_length = _axis_value(s, 0, np.nan)
        if not np.isfinite(spring_k_per_unit_length) or spring_k_per_unit_length <= 0:
            return None

        mechanical_advantage = [_axis_value(u, i, 1.0) for i in range(num_axes)]
        guy_wire_lengths = [_axis_value(y, i, 0.0) for i in range(num_axes)]
        return cls(
            spring_k_per_unit_length=float(spring_k_per_unit_length),
            mechanical_advantage=mechanical_advantage,
            guy_wire_lengths=guy_wire_lengths,
            spring_k_multiplier=spring_k_multiplier,
        )

    def corrected_distance_mm(
        self,
        unstretched_length_mm: np.ndarray,
        tension_n: np.ndarray,
        *,
        axis: int,
    ) -> np.ndarray:
        """
        Convert an 'unstretched/material' absolute length (mm) into a geometric distance (mm).

        Uses the same spring model as RRF and assumes the measured value corresponds
        to RRF's `distance + flex` (i.e. distance minus extension).
        """

        unstretched_length_mm = np.asarray(unstretched_length_mm, dtype=float)
        tension_n = np.asarray(tension_n, dtype=float)
        if unstretched_length_mm.shape != tension_n.shape:
            raise ValueError("unstretched_length_mm and tension_n must have the same shape")

        ma = float(_axis_value(self.mechanical_advantage, axis, 1.0))
        ma = 1e-9 if ma <= 0 else ma
        guy = float(_axis_value(self.guy_wire_lengths, axis, 0.0))
        s_mult = float(_axis_value(self.spring_k_multiplier, axis, 1.0))
        s_eff = float(self.spring_k_per_unit_length) * max(s_mult, 1e-9)

        tension = np.where(np.isfinite(tension_n) & (tension_n > 0.0), tension_n, 0.0)

        # Solve for distance in:
        #   l_meas = d + flex(d)
        #   flex(d) = -F * (d*ma + guy) / (S_eff * ma)
        # => l_meas = d*(1 - F/S_eff) - F*guy/(S_eff*ma)
        denom = 1.0 - (tension / s_eff)
        denom = np.where(denom > 1e-9, denom, 1e-9)
        return (unstretched_length_mm + (tension * guy) / (s_eff * ma)) / denom

