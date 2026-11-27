from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Union

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


def _scalar_value(value: Any, default: float) -> float:
    if value is None:
        return float(default)
    if isinstance(value, (list, tuple)):
        return _axis_value(value, 0, default)
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return float(default)
    return parsed if np.isfinite(parsed) else float(default)


@dataclass(frozen=True)
class FlexModel:
    """
    Linear spring-based flex and drum-storage model matching RRF HangprinterKinematics
    and the native cable solver (`engine.rs`).

    Span elasticity (RRF):
      springK(springLength) = springKPerUnitLength / springLength
      flex = -F * springLength / (springKPerUnitLength * U**2 * N) + homeFlex

    Spool storage (`M666 K`, `M666 V`):
      rho(T_strand) = max(1 - K * T_strand / R^2, 0.5) / (1 + (T_strand / EA if V else 0))
    where `T_strand = F_axis / (U * N)`. Each mm of nominal drum rotation winds/unwinds
    `rho` mm of unstretched cable material.
    """

    spring_k_per_unit_length: float
    mechanical_advantage: Sequence[float]
    guy_wire_lengths: Sequence[float]
    lines_per_axis: Sequence[float] = ()
    spring_k_multiplier: Union[float, Sequence[float]] = 1.0
    spool_radii_mm: Sequence[float] = ()
    spool_flattening: float = 0.0
    strained_spool_storage: bool = False
    home_tension_n: Sequence[float] = ()

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
        spool_radii = [_axis_value(m666.get("R"), i, 0.0) for i in range(num_axes)]
        spool_flattening = max(_scalar_value(m666.get("K"), 0.0), 0.0)
        strained_spool_storage = _scalar_value(m666.get("V"), 0.0) != 0.0
        # Buildup (Q != 0) is a separate drum model in RRF and excludes K/V.
        buildup = _scalar_value(m666.get("Q"), 0.0)
        if abs(buildup) > 1e-12:
            spool_flattening = 0.0
            strained_spool_storage = False
        raw_t = m666.get("T")
        home_tension = (
            [max(_axis_value(raw_t, i, 0.0), 0.0) for i in range(num_axes)]
            if raw_t is not None
            else ()
        )
        return cls(
            spring_k_per_unit_length=float(spring_k_per_unit_length),
            mechanical_advantage=mechanical_advantage,
            guy_wire_lengths=guy_wire_lengths,
            lines_per_axis=[_axis_value(m666.get("N"), i, 1.0) for i in range(num_axes)],
            spring_k_multiplier=spring_k_multiplier,
            spool_radii_mm=spool_radii,
            spool_flattening=float(spool_flattening),
            strained_spool_storage=bool(strained_spool_storage),
            home_tension_n=home_tension,
        )

    @property
    def has_spool_physics(self) -> bool:
        return bool(self.spool_flattening > 0.0 or self.strained_spool_storage)

    @property
    def has_home_tension(self) -> bool:
        return bool(self.home_tension_n and any(float(t) > 0.0 for t in self.home_tension_n))

    def with_spring_k_multiplier(self, spring_k_multiplier: Union[float, Sequence[float]]) -> "FlexModel":
        return FlexModel(
            spring_k_per_unit_length=self.spring_k_per_unit_length,
            mechanical_advantage=self.mechanical_advantage,
            guy_wire_lengths=self.guy_wire_lengths,
            lines_per_axis=self.lines_per_axis,
            spring_k_multiplier=spring_k_multiplier,
            spool_radii_mm=self.spool_radii_mm,
            spool_flattening=self.spool_flattening,
            strained_spool_storage=self.strained_spool_storage,
            home_tension_n=self.home_tension_n,
        )

    def effective_stiffness_n(self, axis: int) -> float:
        s_mult = float(_axis_value(self.spring_k_multiplier, axis, 1.0))
        return float(self.spring_k_per_unit_length) * max(s_mult, 1e-9)

    def spool_rho(self, tension_n: np.ndarray, *, axis: int) -> np.ndarray:
        """Material rest length stored per nominal geometric drum arc (`rho <= 1`)."""
        tension_arr = np.asarray(tension_n, dtype=float)
        if not self.has_spool_physics:
            return np.ones_like(tension_arr, dtype=float)

        ma = max(float(_axis_value(self.mechanical_advantage, axis, 1.0)), 1e-9)
        strands = max(float(_axis_value(self.lines_per_axis, axis, 1.0)), 1e-9)
        tension = np.where(np.isfinite(tension_arr) & (tension_arr > 0.0), tension_arr, 0.0)
        t_strand = tension / (ma * strands)

        radius = float(_axis_value(self.spool_radii_mm, axis, 0.0)) if self.spool_radii_mm else 0.0
        if self.spool_flattening > 0.0 and np.isfinite(radius) and radius > 1e-9:
            r_scale = np.maximum(1.0 - self.spool_flattening * t_strand / (radius * radius), 0.5)
        else:
            r_scale = np.ones_like(t_strand)

        if self.strained_spool_storage:
            ea_eff = self.effective_stiffness_n(axis)
            s_scale = 1.0 / (1.0 + t_strand / ea_eff)
        else:
            s_scale = np.ones_like(t_strand)
        return r_scale * s_scale

    def home_flex_mm(self, home_distance_mm: float, *, axis: int) -> float:
        """Elastic extension already present in line `axis` at encoder zero (`M666 T`)."""
        if not self.home_tension_n:
            return 0.0
        t_home = float(_axis_value(self.home_tension_n, axis, 0.0))
        if not np.isfinite(t_home) or t_home <= 0.0:
            return 0.0
        ma = max(float(_axis_value(self.mechanical_advantage, axis, 1.0)), 1e-9)
        guy = float(_axis_value(self.guy_wire_lengths, axis, 0.0))
        strands = max(float(_axis_value(self.lines_per_axis, axis, 1.0)), 1e-9)
        s_eff = self.effective_stiffness_n(axis) * ma * strands
        return float(t_home * (float(home_distance_mm) * ma + guy) / (s_eff * ma))

    def corrected_distance_mm(
        self,
        unstretched_length_mm: np.ndarray,
        tension_n: np.ndarray,
        *,
        axis: int,
        home_distance_mm: Optional[float] = None,
    ) -> np.ndarray:
        """
        Convert an 'unstretched/material' absolute length (mm) into a geometric distance (mm).

        Uses the same spring model as RRF and assumes the measured value corresponds
        to RRF's `distance + flex` (i.e. distance minus extension plus optional homeFlex).
        """
        unstretched_length_mm = np.asarray(unstretched_length_mm, dtype=float)
        tension_n = np.asarray(tension_n, dtype=float)
        if unstretched_length_mm.shape != tension_n.shape:
            raise ValueError("unstretched_length_mm and tension_n must have the same shape")

        ma = max(float(_axis_value(self.mechanical_advantage, axis, 1.0)), 1e-9)
        guy = float(_axis_value(self.guy_wire_lengths, axis, 0.0))
        strands = max(float(_axis_value(self.lines_per_axis, axis, 1.0)), 1e-9)
        s_eff = self.effective_stiffness_n(axis) * ma * strands

        tension = np.where(np.isfinite(tension_n) & (tension_n > 0.0), tension_n, 0.0)
        home_flex = (
            self.home_flex_mm(home_distance_mm, axis=axis)
            if home_distance_mm is not None and self.has_home_tension
            else 0.0
        )

        # Solve for distance in:
        #   l_meas = d + flex(d) = d - F * (d*ma + guy) / (S_eff * ma) + home_flex
        # => d * (1 - F/S_eff) = l_meas - home_flex + F*guy/(S_eff*ma)
        denom = 1.0 - (tension / s_eff)
        denom = np.where(denom > 1e-9, denom, 1e-9)
        return (unstretched_length_mm - home_flex + (tension * guy) / (s_eff * ma)) / denom


def extract_spool_history(dataset: dict) -> Dict[str, object]:
    """Extract per-sweep-direction winding metadata needed by `spool_rest_correction_mm`."""
    config = dataset.get("config") or {}
    ft = config.get("force_tuning") or {}
    default_pullout = ft.get("pullout_force_n", ft.get("force_max_n"))
    default_pullout_n = (
        float(default_pullout)
        if isinstance(default_pullout, (int, float)) and np.isfinite(default_pullout) and float(default_pullout) > 0.0
        else 0.0
    )

    passes: List[Dict[str, object]] = []
    idx = 0
    for sweep in dataset.get("sweeps", []):
        pts = sweep.get("data_points", [])
        if not pts:
            continue
        meta = sweep.get("metadata") or {}
        reserves = meta.get("spool_reserves") or []
        dir_forces = meta.get("direction_forces") or []
        canon_drive = int(sweep["drive_anchor"])
        canon_sensor = int(sweep["sensor_anchor"])

        by_dir: Dict[int, List[int]] = {0: [], 1: []}
        for i, pt in enumerate(pts):
            src_sensor = int(pt.get("source_sensor_anchor", canon_sensor))
            d_idx = 0 if src_sensor == canon_sensor else 1
            by_dir[d_idx].append(idx + i)

        for d_idx in (0, 1):
            rows = by_dir[d_idx]
            if not rows:
                continue
            first_pt = pts[rows[0] - idx]
            sa = int(first_pt.get("source_sensor_anchor", canon_sensor if d_idx == 0 else canon_drive))
            da = int(first_pt.get("source_drive_anchor", canon_drive if d_idx == 0 else canon_sensor))
            payout = 0.0
            if d_idx < len(reserves) and isinstance(reserves[d_idx], dict):
                raw_po = reserves[d_idx].get("sensor_payout_mm")
                if isinstance(raw_po, (int, float)) and np.isfinite(raw_po) and float(raw_po) > 0.0:
                    payout = float(raw_po)
            pullout_n = default_pullout_n
            if d_idx < len(dir_forces) and isinstance(dir_forces[d_idx], dict):
                raw_pf = dir_forces[d_idx].get("pullout_force_n")
                if isinstance(raw_pf, (int, float)) and np.isfinite(raw_pf) and float(raw_pf) > 0.0:
                    pullout_n = float(raw_pf)
            passes.append({
                "rows": np.asarray(rows, dtype=int),
                "drive_anchor": da,
                "sensor_anchor": sa,
                "sensor_payout_mm": payout,
                "pullout_force_n": pullout_n,
            })
        idx += len(pts)

    return {"num_points": idx, "passes": passes}


def slice_spool_history(
    history: Optional[Dict[str, object]],
    mask: np.ndarray,
) -> Optional[Dict[str, object]]:
    """Slice extracted spool-history pass rows by a boolean point mask."""
    if not isinstance(history, dict):
        return None
    mask_arr = np.asarray(mask, dtype=bool).reshape(-1)
    if history.get("num_points") != mask_arr.size:
        return None
    remap = np.full(mask_arr.size, -1, dtype=int)
    remap[mask_arr] = np.arange(int(np.sum(mask_arr)), dtype=int)
    passes: List[Dict[str, object]] = []
    for entry in history.get("passes", ()):
        rows = np.asarray(entry["rows"], dtype=int)
        valid = (rows >= 0) & (rows < mask_arr.size)
        kept = remap[rows[valid]]
        kept = kept[kept >= 0]
        if kept.size:
            passes.append({**entry, "rows": kept})
    return {"num_points": int(np.sum(mask_arr)), "passes": passes}


def spool_rest_correction_mm(
    lengths_mm: np.ndarray,
    tensions_n: np.ndarray,
    model: Optional[FlexModel],
    history: Optional[Dict[str, object]] = None,
) -> np.ndarray:
    """Compute material-length transfer delta `l_rest - l_enc` (mm) from spool flattening & strained storage.

    When cable is wound onto a drum under tension (`T`), radial flattening (`M666 K`) and
    strained storage (`M666 V`) store `rho(T) <= 1` mm of unstretched rope per mm of nominal
    drum arc, whereas pre-wound rope from initialization unwinds at `rho_init = rho(T_home)`
    (`1.0` when `M666 T` is absent or zero).

    Adding the returned correction to encoder-reported `lengths_mm` recovers the unstretched
    material length change before applying free-span elasticity.
    """
    lengths = np.asarray(lengths_mm, dtype=float)
    if model is None or not model.has_spool_physics or lengths.size == 0:
        return np.zeros_like(lengths, dtype=float)

    tensions = np.asarray(tensions_n, dtype=float)
    num_points, num_axes = lengths.shape
    rho = np.column_stack([model.spool_rho(tensions[:, a], axis=a) for a in range(num_axes)])
    rho_init = np.array(
        [
            float(
                model.spool_rho(
                    np.array([_axis_value(model.home_tension_n, a, 0.0) if model.home_tension_n else 0.0]),
                    axis=a,
                )[0]
            )
            for a in range(num_axes)
        ],
        dtype=float,
    )

    finite = np.isfinite(lengths)
    safe_l = np.where(finite, lengths, 0.0)
    # Default for wound lines (l < 0): coils stored at current tension; unwound lines (l > 0)
    # release pre-wound rope at rho_init.
    corr = safe_l * (rho_init[None, :] - 1.0) + np.maximum(-safe_l, 0.0) * (rho_init[None, :] - rho)

    if isinstance(history, dict) and history.get("num_points") == num_points:
        for entry in history.get("passes", ()):
            rows = entry["rows"]
            if rows.size == 0:
                continue
            da = int(entry["drive_anchor"])
            sa = int(entry["sensor_anchor"])
            pullout_n = float(entry["pullout_force_n"])
            if 0 <= da < num_axes and pullout_n > 0.0:
                rho_pull = float(model.spool_rho(np.array([pullout_n]), axis=da)[0])
                l_d = safe_l[rows, da]
                corr[rows, da] = l_d * (rho_init[da] - 1.0) + np.maximum(-l_d, 0.0) * (rho_init[da] - rho_pull)
            payout = float(entry["sensor_payout_mm"])
            if 0 <= sa < num_axes and payout > 0.0:
                l_raw = lengths[rows, sa]
                s_finite = finite[rows, sa]
                drho = rho_init[sa] - rho[rows, sa]
                # Sensor unwound pre-wound rope to `payout` at rho_init, then rewound to the first
                # finite point and monotonically through the sweep pass, laying LIFO coils at each
                # point's tension. Skip non-finite/rejected points cleanly without zeroing them.
                prev_l: Optional[float] = None
                prev_drho = 0.0
                acc = 0.0
                for k in range(len(rows)):
                    if not s_finite[k]:
                        corr[rows[k], sa] = 0.0
                        continue
                    cur_l = float(l_raw[k])
                    cur_drho = float(drho[k])
                    if prev_l is None:
                        acc = max(payout - cur_l, 0.0) * cur_drho
                    else:
                        dl = prev_l - cur_l
                        if dl >= 0.0:
                            acc += dl * 0.5 * (prev_drho + cur_drho)
                        else:
                            acc = max(0.0, acc + dl * cur_drho)
                    corr[rows[k], sa] = cur_l * (rho_init[sa] - 1.0) + acc
                    prev_l = cur_l
                    prev_drho = cur_drho

    return np.where(finite, corr, 0.0)
