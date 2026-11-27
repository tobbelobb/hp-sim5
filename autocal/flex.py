from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

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
    _vec_cache: Dict[int, Dict[str, np.ndarray]] = field(
        default_factory=dict, init=False, repr=False, compare=False
    )

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

    def _arrays_for(self, num_axes: int) -> Dict[str, np.ndarray]:
        cached = self._vec_cache.get(num_axes)
        if cached is not None:
            return cached
        ma = np.array([max(_axis_value(self.mechanical_advantage, a, 1.0), 1e-9) for a in range(num_axes)], dtype=float)
        strands = np.array([max(_axis_value(self.lines_per_axis, a, 1.0), 1e-9) for a in range(num_axes)], dtype=float)
        guy = np.array([_axis_value(self.guy_wire_lengths, a, 0.0) for a in range(num_axes)], dtype=float)
        ea_eff = np.array([self.effective_stiffness_n(a) for a in range(num_axes)], dtype=float)
        s_eff = ea_eff * ma * strands
        inv_ma_strands = 1.0 / (ma * strands)
        radii = np.array(
            [_axis_value(self.spool_radii_mm, a, 0.0) if self.spool_radii_mm else 0.0 for a in range(num_axes)],
            dtype=float,
        )
        flat_coeff = np.where(
            (self.spool_flattening > 0.0) & np.isfinite(radii) & (radii > 1e-9),
            self.spool_flattening / np.maximum(radii * radii, 1e-18),
            0.0,
        )
        t_home_raw = np.array(
            [_axis_value(self.home_tension_n, a, 0.0) if self.home_tension_n else 0.0 for a in range(num_axes)],
            dtype=float,
        )
        t_home = np.where(np.isfinite(t_home_raw) & (t_home_raw > 0.0), t_home_raw, 0.0)
        vec = {
            "ma": ma,
            "strands": strands,
            "guy": guy,
            "ea_eff": ea_eff,
            "s_eff": s_eff,
            "s_eff_ma": s_eff * ma,
            "inv_ma_strands": inv_ma_strands,
            "flat_coeff": flat_coeff,
            "t_home": t_home,
        }
        if self.has_spool_physics:
            t_strand_h = t_home * inv_ma_strands
            r_h = np.maximum(1.0 - t_strand_h * flat_coeff, 0.5) if self.spool_flattening > 0.0 else 1.0
            s_h = (1.0 / (1.0 + t_strand_h / ea_eff)) if self.strained_spool_storage else 1.0
            vec["rho_init"] = np.asarray(r_h * s_h, dtype=float)
        else:
            vec["rho_init"] = np.ones(num_axes, dtype=float)
        self._vec_cache[num_axes] = vec
        return vec

    def spool_rho_matrix(self, tensions_n: np.ndarray) -> np.ndarray:
        """Vectorized `spool_rho` across all axes for a `(P, N)` tension matrix."""
        t_arr = np.asarray(tensions_n, dtype=float)
        if not self.has_spool_physics:
            return np.ones_like(t_arr, dtype=float)
        vec = self._arrays_for(t_arr.shape[1])
        tension = np.where(np.isfinite(t_arr) & (t_arr > 0.0), t_arr, 0.0)
        t_strand = tension * vec["inv_ma_strands"][None, :]
        r_scale = (
            np.maximum(1.0 - t_strand * vec["flat_coeff"][None, :], 0.5)
            if self.spool_flattening > 0.0
            else 1.0
        )
        s_scale = (
            1.0 / (1.0 + t_strand / vec["ea_eff"][None, :])
            if self.strained_spool_storage
            else 1.0
        )
        return np.asarray(r_scale * s_scale, dtype=float)

    def corrected_distance_matrix(
        self,
        unstretched_length_mm: np.ndarray,
        tensions_n: np.ndarray,
        *,
        home_distance_mm: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Vectorized `corrected_distance_mm` across all axes for `(P, N)` matrices."""
        mat = np.asarray(unstretched_length_mm, dtype=float)
        t_arr = np.asarray(tensions_n, dtype=float)
        vec = self._arrays_for(mat.shape[1])
        tension = np.where(np.isfinite(t_arr) & (t_arr > 0.0), t_arr, 0.0)
        if home_distance_mm is not None and self.has_home_tension:
            hd = np.asarray(home_distance_mm, dtype=float)
            home_flex = (vec["t_home"] * (hd * vec["ma"] + vec["guy"]) / vec["s_eff_ma"])[None, :]
        else:
            home_flex = 0.0
        denom = np.maximum(1.0 - tension / vec["s_eff"][None, :], 1e-9)
        return (mat - home_flex + (tension * vec["guy"][None, :]) / vec["s_eff_ma"][None, :]) / denom


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


def _compute_pass_spool_overrides(
    lengths: np.ndarray,
    finite: np.ndarray,
    rho: np.ndarray,
    model: FlexModel,
    history: Dict[str, object],
) -> List[Tuple[np.ndarray, int, np.ndarray]]:
    """Compute drive pullout and LIFO sensor rewind overrides for each pass in `history`."""
    num_axes = lengths.shape[1]
    vec = model._arrays_for(num_axes)
    rho_init = vec["rho_init"]
    inv_ma_strands = vec["inv_ma_strands"]
    flat_coeff = vec["flat_coeff"]
    ea_eff = vec["ea_eff"]
    safe_l = np.where(finite, lengths, 0.0)
    overrides: List[Tuple[np.ndarray, int, np.ndarray]] = []
    for entry in history.get("passes", ()):
        rows = entry["rows"]
        if rows.size == 0:
            continue
        da = int(entry["drive_anchor"])
        sa = int(entry["sensor_anchor"])
        pullout_n = float(entry["pullout_force_n"])
        if 0 <= da < num_axes and pullout_n > 0.0:
            t_strand = pullout_n * float(inv_ma_strands[da])
            r_pull = max(1.0 - t_strand * float(flat_coeff[da]), 0.5) if model.spool_flattening > 0.0 else 1.0
            s_pull = (1.0 / (1.0 + t_strand / float(ea_eff[da]))) if model.strained_spool_storage else 1.0
            rho_pull = r_pull * s_pull
            l_d = safe_l[rows, da]
            drive_corr = l_d * (rho_init[da] - 1.0) + np.maximum(-l_d, 0.0) * (rho_init[da] - rho_pull)
            overrides.append((rows, da, drive_corr))
        payout = float(entry["sensor_payout_mm"])
        if 0 <= sa < num_axes and payout > 0.0:
            l_raw = lengths[rows, sa]
            s_finite = finite[rows, sa]
            drho = rho_init[sa] - rho[rows, sa]
            if bool(np.all(s_finite)):
                if len(rows) == 1:
                    acc = np.array([max(payout - float(l_raw[0]), 0.0) * float(drho[0])], dtype=float)
                    overrides.append((rows, sa, l_raw * (rho_init[sa] - 1.0) + acc))
                    continue
                dl = l_raw[:-1] - l_raw[1:]
                if bool(np.all(dl >= 0.0)):
                    acc = np.empty(len(rows), dtype=float)
                    acc[0] = max(payout - float(l_raw[0]), 0.0) * float(drho[0])
                    acc[1:] = acc[0] + np.cumsum(dl * 0.5 * (drho[:-1] + drho[1:]))
                    overrides.append((rows, sa, l_raw * (rho_init[sa] - 1.0) + acc))
                    continue
            # Sensor unwound pre-wound rope to `payout` at rho_init, then rewound to the first
            # finite point and monotonically through the sweep pass, laying LIFO coils at each
            # point's tension. Skip genuinely missing (non-finite) raw measurements cleanly.
            sensor_corr = np.zeros(len(rows), dtype=float)
            prev_l: Optional[float] = None
            prev_drho = 0.0
            acc_val = 0.0
            for k in range(len(rows)):
                if not s_finite[k]:
                    sensor_corr[k] = 0.0
                    continue
                cur_l = float(l_raw[k])
                cur_drho = float(drho[k])
                if prev_l is None:
                    acc_val = max(payout - cur_l, 0.0) * cur_drho
                else:
                    dl_k = prev_l - cur_l
                    if dl_k >= 0.0:
                        acc_val += dl_k * 0.5 * (prev_drho + cur_drho)
                    else:
                        acc_val = max(0.0, acc_val + dl_k * cur_drho)
                sensor_corr[k] = cur_l * (rho_init[sa] - 1.0) + acc_val
                prev_l = cur_l
                prev_drho = cur_drho
            overrides.append((rows, sa, sensor_corr))
    return overrides


def spool_rest_correction_mm(
    lengths_mm: np.ndarray,
    tensions_n: np.ndarray,
    model: Optional[FlexModel],
    history: Optional[Dict[str, object]] = None,
    *,
    pass_overrides: Optional[List[Tuple[np.ndarray, int, np.ndarray]]] = None,
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
    vec = model._arrays_for(num_axes)
    rho = model.spool_rho_matrix(tensions)
    rho_init = vec["rho_init"]

    finite = np.isfinite(lengths)
    safe_l = np.where(finite, lengths, 0.0)
    # Default for wound lines (l < 0): coils stored at current tension; unwound lines (l > 0)
    # release pre-wound rope at rho_init.
    corr = safe_l * (rho_init[None, :] - 1.0) + np.maximum(-safe_l, 0.0) * (rho_init[None, :] - rho)

    if pass_overrides is not None:
        for rows, axis_idx, values in pass_overrides:
            corr[rows, axis_idx] = values
    elif isinstance(history, dict) and history.get("num_points") == num_points:
        for rows, axis_idx, values in _compute_pass_spool_overrides(lengths, finite, rho, model, history):
            corr[rows, axis_idx] = values

    return np.where(finite, corr, 0.0)
