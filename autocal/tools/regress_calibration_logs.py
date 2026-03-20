#!/usr/bin/env python3
"""
Regression test runner for autocal/autocal.py full-auto runs.

What it does
- Runs fixed commands (one per dataset in DATASETS)
- Locates the generated log path from stdout/stderr (looks for "Writing additional info to log:")
- Compares generated vs reference logs by parsing:
  * Final "== Calibration summary ==" block (fit/UI score when present + M669 anchors + M666 radii)
  * Each iteration triple:
    - "Anchors:"
    - "line_model ... effective=[...]"
    - machine-readable "; selected run=..." line
    - legacy fallback console "Score: ..."

Comparison metrics
- Anchor distance: sum_i ||anchor_i - anchor_i_ref||  (Euclidean, mm)
- Radius distance: 2*pi * sum_i |R_i - R_i_ref|
- "Total parameter distance" = anchor_distance + radius_distance

True values (simulation ground truth) used for "closer/further" reporting
- Anchors: [[0,-1900],[1645.44826719,950],[-1645.44826719,950]]
- R: 39.184 (all)
- buildup factor is provided via CLI and not evaluated here

Exit code
- 0 if everything is EXACTLY EQUAL, or changed but stays within tolerance (default 10mm total) AND
    no score/fit direction mismatches
- 1 otherwise

Tip: put this file somewhere like tools/regress_autocal.py and run from repo root.
"""

from __future__ import annotations

import argparse
import ast
import concurrent.futures
import math
import os
import re
import shutil
import subprocess
import sys
import uuid
from dataclasses import dataclass, field
from itertools import zip_longest
from pathlib import Path
from typing import List, Optional, Tuple


Coord = Tuple[float, ...]
DEFAULT_REFERENCE_LOG_NAMES = (
    "full_auto_reference_run_march_19.log",
    "full_auto_reference_run_march_11.log",
    "full_auto_reference_run_march_6.log",
)

SIM_2D_TRUE_ANCHORS: List[Coord] = [
    (0.0, -1900.0, 0.0),
    (1645.44826719, 950.0, 0.0),
    (-1645.44826719, 950.0, 0.0),
]
SIM_2D_TRUE_RADII = [39.184, 39.184, 39.184]

SIM_3D_TRUE_ANCHORS: List[Coord] = [
    (0.0, -1900.0, -280.0),
    (1645.45, 950.0, -280.0),
    (-1645.45, 950.0, -280.0),
    (0.0, 0.0, 1900.0),
]
SIM_3D_TRUE_RADII = [39.1845, 39.1845, 39.1845, 39.1845]


@dataclass(frozen=True)
class DatasetSpec:
    name: str
    machine_type: str
    base_radii: str
    buildup_factor: str
    true_anchors: List[Coord]
    true_radii: List[float]
    extra_args: Tuple[str, ...] = ()
    reference_log_names: Tuple[str, ...] = field(default_factory=lambda: DEFAULT_REFERENCE_LOG_NAMES)


DATASETS = [
    DatasetSpec(
        name="five_points_bigger_deltas",
        machine_type="slideprinter",
        base_radii="30.0",
        buildup_factor="0.636619",
        true_anchors=SIM_2D_TRUE_ANCHORS,
        true_radii=SIM_2D_TRUE_RADII,
    ),
    DatasetSpec(
        name="flexible_lines_2000",
        machine_type="slideprinter",
        base_radii="30.0",
        buildup_factor="0.636619",
        true_anchors=SIM_2D_TRUE_ANCHORS,
        true_radii=SIM_2D_TRUE_RADII,
    ),
    DatasetSpec(
        name="layered_manual_some_bad_points",
        machine_type="slideprinter",
        base_radii="30.0",
        buildup_factor="0.636619",
        true_anchors=SIM_2D_TRUE_ANCHORS,
        true_radii=SIM_2D_TRUE_RADII,
    ),
    DatasetSpec(
        name="ten_points_bigger_deltas",
        machine_type="slideprinter",
        base_radii="30.0",
        buildup_factor="0.636619",
        true_anchors=SIM_2D_TRUE_ANCHORS,
        true_radii=SIM_2D_TRUE_RADII,
    ),
    DatasetSpec(
        name="ten_points_w_buildup",
        machine_type="slideprinter",
        base_radii="30.0",
        buildup_factor="0.636619",
        true_anchors=SIM_2D_TRUE_ANCHORS,
        true_radii=SIM_2D_TRUE_RADII,
    ),
    DatasetSpec(
        name="ten_points_w_buildup2",
        machine_type="slideprinter",
        base_radii="30.0",
        buildup_factor="0.636619",
        true_anchors=SIM_2D_TRUE_ANCHORS,
        true_radii=SIM_2D_TRUE_RADII,
    ),
    DatasetSpec(
        name="fifth_hp3_dataset",
        machine_type="hangprinter_4",
        base_radii="30",
        buildup_factor="0.636619",
        true_anchors=SIM_3D_TRUE_ANCHORS,
        true_radii=SIM_3D_TRUE_RADII,
        extra_args=("--verbose", "--r0-bounds", "39,40"),
        reference_log_names=("full_auto_reference_run_march_19.log",),
    ),
    DatasetSpec(
        name="seventh_hp3_dataset",
        machine_type="hangprinter_4",
        base_radii="30",
        buildup_factor="0.636619",
        true_anchors=SIM_3D_TRUE_ANCHORS,
        true_radii=SIM_3D_TRUE_RADII,
        extra_args=("--verbose", "--r0-bounds", "39,40"),
        reference_log_names=("full_auto_reference_run_march_20.log",),
    ),
]


@dataclass
class Params:
    anchors: Optional[List[Coord]]
    radii: Optional[List[float]]
    fit_score_ui: Optional[float]  # "Fit quality score" from the summary block


@dataclass
class Iteration:
    anchors: Optional[List[Coord]]
    radii: Optional[List[float]]
    fit_score_ui: Optional[float] = None
    rank_score: Optional[float] = None
    history_rank_score: Optional[float] = None
    anchors_line: Optional[str] = None
    radii_line: Optional[str] = None
    fit_score_line: Optional[str] = None
    rank_score_line: Optional[str] = None


@dataclass
class ParsedLog:
    summary: Optional[Params]
    iterations: List[Iteration]
    summary_block_lines: List[str]


@dataclass
class DatasetRunResult:
    name: str
    ok: bool
    lines: List[str]
    generated_log: Optional[Path]
    reference_log: Optional[Path]
    true_err_total_delta: Optional[float] = None
    true_iter_mean_delta: Optional[float] = None
    true_gen_iter_std: Optional[float] = None
    true_gen_iter_count: int = 0


# ---------- Parsing helpers ----------

_RE_LOGPATH = re.compile(r"Writing additional info to log:\s*(.+)\s*$")
_RE_ANCHORS = re.compile(r"Anchors:\s*(\[\[.*\]\])")
_RE_EFFECTIVE = re.compile(r"effective=\[([^\]]+)\]")
_RE_LEGACY_FIT_SCORE = re.compile(r"\bScore:\s*([0-9.+-eE]+)\b")
_RE_SELECTED_FIT_SCORE_UI = re.compile(r"\b(?:fit_score_ui|score_ui)=([0-9.+-eE]+)\b")
_RE_SELECTED_RANK_SCORE = re.compile(r"\brank_score=([0-9.+-eE]+)\b")
_RE_SELECTED_HISTORY_RANK_SCORE = re.compile(r"\b(?:history_rank_score|selection_rank)=([0-9.+-eE]+)\b")
_NUM_PATTERN = r"[+-]?\d+(?:\.\d+)?(?:e[+-]?\d+)?"
_RE_M669_PARTS = re.compile(rf"([A-Z])\s*((?:{_NUM_PATTERN})(?::{_NUM_PATTERN})+)", re.IGNORECASE)
_RE_M666_R = re.compile(rf"\bR\s*((?:{_NUM_PATTERN})(?::{_NUM_PATTERN})+)", re.IGNORECASE)
_RE_FIT_SCORE = re.compile(r"Fit(?:/UI)? quality score.*?:\s*([0-9.+-eE]+)\s*$")
_RE_ANSI = re.compile(r"\x1b\[[0-9;]*m")


def _safe_float(x: str) -> Optional[float]:
    try:
        return float(x)
    except Exception:
        return None


def parse_generated_log_path(run_output: str) -> Optional[str]:
    """
    Parses stdout/stderr combined output from autocal.py and returns the produced log path.
    We accept both raw and "Wrote to console:" prefixed lines.
    """
    for line in run_output.splitlines():
        m = _RE_LOGPATH.search(line)
        if m:
            return m.group(1).strip()
    return None


def parse_iterations(text: str) -> List[Iteration]:
    """
    Extracts per-iteration (anchors, effective radii, score fields) tuples in order.
    Newer logs terminate iterations on the machine-readable "; selected run=..." line.
    Older logs may only have a console "Score:" line, which we still accept.
    """
    iters: List[Iteration] = []
    cur: dict = {}
    for line in text.splitlines():
        if "Anchors:" in line:
            m = _RE_ANCHORS.search(line)
            if m:
                try:
                    arr = ast.literal_eval(m.group(1))
                    cur["anchors"] = [tuple(float(v) for v in a) for a in arr]
                    cur["anchors_line"] = line.strip()
                except Exception:
                    # keep going; we'll still report parse issues later
                    cur["anchors"] = None
                    cur["anchors_line"] = line.strip()
        if "effective=[" in line:
            m = _RE_EFFECTIVE.search(line)
            if m:
                try:
                    radii = ast.literal_eval("[" + m.group(1) + "]")
                    cur["radii"] = [float(x) for x in radii]
                    cur["radii_line"] = line.strip()
                except Exception:
                    cur["radii"] = None
                    cur["radii_line"] = line.strip()
        if "; selected run=" in line:
            if "anchors" not in cur and "radii" not in cur:
                continue
            fit_score_ui = None
            rank_score = None
            history_rank_score = None
            m_fit = _RE_SELECTED_FIT_SCORE_UI.search(line)
            if m_fit:
                fit_score_ui = _safe_float(m_fit.group(1))
            m_rank = _RE_SELECTED_RANK_SCORE.search(line)
            if m_rank:
                rank_score = _safe_float(m_rank.group(1))
            m_history_rank = _RE_SELECTED_HISTORY_RANK_SCORE.search(line)
            if m_history_rank:
                history_rank_score = _safe_float(m_history_rank.group(1))
            iters.append(
                Iteration(
                    anchors=cur.get("anchors"),
                    radii=cur.get("radii"),
                    fit_score_ui=fit_score_ui,
                    rank_score=rank_score,
                    history_rank_score=history_rank_score,
                    anchors_line=cur.get("anchors_line"),
                    radii_line=cur.get("radii_line"),
                    fit_score_line=line.strip() if m_fit else None,
                    rank_score_line=line.strip() if (m_rank or m_history_rank) else None,
                )
            )
            cur = {}
            continue
        if "Score:" in line and cur:
            m = _RE_LEGACY_FIT_SCORE.search(line)
            if m:
                fit_score_ui = _safe_float(m.group(1))
                iters.append(
                    Iteration(
                        anchors=cur.get("anchors"),
                        radii=cur.get("radii"),
                        fit_score_ui=fit_score_ui,
                        anchors_line=cur.get("anchors_line"),
                        radii_line=cur.get("radii_line"),
                        fit_score_line=line.strip(),
                    )
                )
                cur = {}
    return iters


def extract_summary_block_lines(text: str, max_lines: int = 40) -> List[str]:
    """
    Returns the lines following the last occurrence of "== Calibration summary ==".
    """
    idx = text.rfind("== Calibration summary ==")
    if idx == -1:
        return []
    lines = text[idx:].splitlines()[:max_lines]
    return lines


def parse_summary(text: str) -> Optional[Params]:
    """
    Parses the last calibration summary for fit score, anchors, and radii.
    """
    lines = extract_summary_block_lines(text, max_lines=60)
    if not lines:
        return None

    fit_score: Optional[float] = None
    anchors: Optional[List[Coord]] = None
    radii: Optional[List[float]] = None

    for line in lines:
        if "quality score" in line:
            m = _RE_FIT_SCORE.search(line)
            if m:
                fit_score = _safe_float(m.group(1))
        if "Parameters (M669)" in line or "Anchors (M669)" in line:
            parsed_anchors: List[Coord] = []
            for _label, coords_text in _RE_M669_PARTS.findall(line):
                values = [_safe_float(part) for part in coords_text.split(":")]
                if any(v is None for v in values):
                    parsed_anchors = []
                    break
                parsed_anchors.append(tuple(float(v) for v in values if v is not None))
            if parsed_anchors:
                anchors = parsed_anchors
        if "Line model (M666)" in line or "Spools (M666)" in line:
            m = _RE_M666_R.search(line)
            if m:
                parsed_radii = [_safe_float(part) for part in m.group(1).split(":")]
                if any(v is None for v in parsed_radii):
                    radii = None
                else:
                    radii = [float(v) for v in parsed_radii if v is not None]

    if radii is None or anchors is None:
        return None

    return Params(anchors=anchors, radii=radii, fit_score_ui=fit_score)


def parse_log_file(path: Path) -> ParsedLog:
    text = path.read_text(errors="replace")
    summary = parse_summary(text)
    iters = parse_iterations(text)
    summary_lines = extract_summary_block_lines(text)
    return ParsedLog(summary=summary, iterations=iters, summary_block_lines=summary_lines)


# ---------- Metrics ----------

def _anchors_compatible(a: List[Coord], b: List[Coord]) -> bool:
    return len(a) == len(b) and all(len(left) == len(right) for left, right in zip(a, b))


def anchor_distance(a: List[Coord], b: List[Coord]) -> float:
    return sum(math.dist(left, right) for left, right in zip(a, b))


def _anchor_distance_common_dims(a: List[Coord], b: List[Coord]) -> Optional[float]:
    if len(a) != len(b):
        return None
    total = 0.0
    for left, right in zip(a, b):
        dim = min(len(left), len(right))
        if dim == 0:
            return None
        total += math.dist(left[:dim], right[:dim])
    return total


def radius_distance(r: List[float], r2: List[float]) -> float:
    # 2*pi*distance for the R values
    return (2.0 * math.pi) * sum(abs(x - y) for x, y in zip(r, r2))


def total_param_distance(
    anchors1: Optional[List[Coord]],
    radii1: Optional[List[float]],
    anchors2: Optional[List[Coord]],
    radii2: Optional[List[float]],
) -> Optional[Tuple[float, float, float]]:
    if anchors1 is None or anchors2 is None or radii1 is None or radii2 is None:
        return None
    if not _anchors_compatible(anchors1, anchors2) or len(radii1) != len(radii2):
        return None
    ad = anchor_distance(anchors1, anchors2)
    rd = radius_distance(radii1, radii2)
    return (ad + rd, ad, rd)


def error_to_true(
    anchors: Optional[List[Coord]],
    radii: Optional[List[float]],
    true_anchors: List[Coord],
    true_radii: List[float],
) -> Optional[Tuple[float, float, float]]:
    if anchors is None or radii is None:
        return None
    if len(anchors) != len(true_anchors) or len(radii) != len(true_radii):
        return None
    ad = _anchor_distance_common_dims(anchors, true_anchors)
    if ad is None:
        return None
    rd = radius_distance(radii, true_radii)
    return (ad + rd, ad, rd)


def compute_true_gen_iter_stats(
    iterations: List[Iteration],
    true_anchors: List[Coord],
    true_radii: List[float],
) -> Tuple[Optional[float], Optional[float], int]:
    values: List[float] = []
    for it in iterations:
        v = error_to_true(it.anchors, it.radii, true_anchors, true_radii)
        if v is None:
            continue
        total = float(v[0])
        if math.isfinite(total):
            values.append(total)

    n = len(values)
    if n == 0:
        return None, None, 0
    mean = sum(values) / n
    if n == 1:
        return mean, 0.0, 1
    var = sum((x - mean) ** 2 for x in values) / n
    return mean, math.sqrt(var), n


def compute_true_iter_mean_delta(
    ref_iterations: List[Iteration],
    gen_iterations: List[Iteration],
    true_anchors: List[Coord],
    true_radii: List[float],
) -> Tuple[Optional[float], int]:
    deltas: List[float] = []
    for ref_iter, gen_iter in zip_longest(ref_iterations, gen_iterations):
        if ref_iter is None or gen_iter is None:
            continue
        ref_true = error_to_true(ref_iter.anchors, ref_iter.radii, true_anchors, true_radii)
        gen_true = error_to_true(gen_iter.anchors, gen_iter.radii, true_anchors, true_radii)
        if ref_true is None or gen_true is None:
            continue
        ref_total = float(ref_true[0])
        gen_total = float(gen_true[0])
        if not math.isfinite(ref_total) or not math.isfinite(gen_total):
            continue
        deltas.append(gen_total - ref_total)

    n = len(deltas)
    if n == 0:
        return None, 0
    return sum(deltas) / n, n


def dir_label(delta: float, eps: float = 0.0) -> str:
    if abs(delta) <= eps:
        return "equal"
    return "better" if delta < 0 else "worse"  # lower is better


def use_color(mode: str) -> bool:
    if mode == "always":
        return True
    if mode == "never":
        return False
    if os.environ.get("NO_COLOR") is not None:
        return False
    return bool(getattr(sys.stdout, "isatty", lambda: False)())


def _ansi(code: str, text: str, enabled: bool) -> str:
    if not enabled:
        return text
    return f"\033[{code}m{text}\033[0m"


def verdict_text(delta: Optional[float], enabled: bool) -> str:
    if delta is None:
        return "N/A"
    label = dir_label(delta)
    if label == "better":
        return _ansi("32", label, enabled)
    if label == "worse":
        return _ansi("31", label, enabled)
    return _ansi("33", label, enabled)


def tol_status_text(exact: bool, small: bool, tol_mm_total: float, enabled: bool) -> str:
    if exact:
        return _ansi("36", "exact", enabled)
    if small:
        return _ansi("32", f"within {fmt(tol_mm_total)}mm", enabled)
    return _ansi("31", f"exceeds {fmt(tol_mm_total)}mm", enabled)


def _visible_len(text: str) -> int:
    return len(_RE_ANSI.sub("", text))


def _pad_cell(text: str, width: int) -> str:
    extra = width - _visible_len(text)
    if extra <= 0:
        return text
    return text + (" " * extra)


def format_table(headers: List[str], rows: List[List[str]]) -> List[str]:
    widths = [len(h) for h in headers]
    for row in rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], _visible_len(cell))

    header_line = " | ".join(_pad_cell(h, widths[i]) for i, h in enumerate(headers))
    sep_line = "-+-".join("-" * w for w in widths)
    out = [header_line, sep_line]
    for row in rows:
        out.append(" | ".join(_pad_cell(row[i], widths[i]) for i in range(len(headers))))
    return out


# ---------- Running commands ----------

def _flatten_cli_args(args: Sequence[object]) -> List[str]:
    flat: List[str] = []
    for arg in args:
        if isinstance(arg, (list, tuple)):
            flat.extend(str(item) for item in arg)
        else:
            flat.append(str(arg))
    return flat


def run_autocal(
    repo_root: Path,
    dataset_path: Path,
    dataset_spec: DatasetSpec,
    full_auto_log: Optional[Path] = None,
) -> Tuple[int, str]:
    extra_args = _flatten_cli_args(dataset_spec.extra_args)
    cmd = [
        sys.executable,
        str(repo_root / "autocal" / "autocal.py"),
        "--sim",
        "--machine-type",
        dataset_spec.machine_type,
        "--dataset",
        str(dataset_path),
        "--find-radii",
        "global",
        "--base-radii",
        dataset_spec.base_radii,
        "--buildup-factor",
        dataset_spec.buildup_factor,
        "--no-collect",
        *extra_args,
    ]
    if full_auto_log is not None:
        cmd.extend(["--full-auto-log", str(full_auto_log)])

    # Reduce nondeterminism where possible (won't hurt even if irrelevant)
    env = os.environ.copy()
    env.setdefault("PYTHONHASHSEED", "0")
    env.setdefault("OMP_NUM_THREADS", "1")
    env.setdefault("OPENBLAS_NUM_THREADS", "1")
    env.setdefault("MKL_NUM_THREADS", "1")
    env.setdefault("NUMEXPR_NUM_THREADS", "1")

    proc = subprocess.run(
        cmd,
        cwd=str(repo_root),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    return proc.returncode, proc.stdout


# ---------- Reporting / Comparison ----------

def fmt(x: Optional[float], nd: int = 3) -> str:
    if x is None:
        return "N/A"
    # Keep scientific if huge
    if abs(x) >= 1e6:
        return f"{x:.3e}"
    return f"{x:.{nd}f}"


def compute_final_score(total_error_sum: float, mean_delta_sum: float) -> float:
    return total_error_sum + mean_delta_sum


def iteration_effective_rank_score(iteration: Iteration) -> Optional[float]:
    if iteration.history_rank_score is not None:
        return iteration.history_rank_score
    if iteration.rank_score is not None:
        return iteration.rank_score
    return None


def iteration_effective_rank_kind(iteration: Iteration) -> Optional[str]:
    if iteration.history_rank_score is not None:
        return "history_rank"
    if iteration.rank_score is not None:
        return "rank_score"
    return None


def report_dataset(
    name: str,
    ref: ParsedLog,
    gen: ParsedLog,
    dataset_spec: DatasetSpec,
    tol_mm_total: float,
    fail_on_score_mismatch: bool,
    color: bool = False,
) -> Tuple[bool, List[str], Optional[float]]:
    """
    Returns (ok, lines).
    ok=False if changes exceed tolerance or mismatch rules trigger.
    """
    lines: List[str] = []
    ok = True
    true_err_total_delta: Optional[float] = None
    true_iter_mean_delta, _true_iter_pair_count = compute_true_iter_mean_delta(
        ref.iterations,
        gen.iterations,
        dataset_spec.true_anchors,
        dataset_spec.true_radii,
    )
    _true_gen_iter_mean, true_gen_iter_std, true_gen_iter_count = compute_true_gen_iter_stats(
        gen.iterations,
        dataset_spec.true_anchors,
        dataset_spec.true_radii,
    )

    # ---- Summary ----
    lines.append(f"\n========= {name} =========")
    if ref.summary is None:
        lines.append("REF: ERROR: could not parse calibration summary")
        ok = False
    if gen.summary is None:
        lines.append("GEN: ERROR: could not parse calibration summary")
        ok = False

    if ref.summary and gen.summary:
        exact = False
        delta = total_param_distance(gen.summary.anchors, gen.summary.radii, ref.summary.anchors, ref.summary.radii)
        fit_score_ui_delta = (
            gen.summary.fit_score_ui - ref.summary.fit_score_ui
            if (gen.summary.fit_score_ui is not None and ref.summary.fit_score_ui is not None)
            else None
        )

        ref_true = error_to_true(ref.summary.anchors, ref.summary.radii, dataset_spec.true_anchors, dataset_spec.true_radii)
        gen_true = error_to_true(gen.summary.anchors, gen.summary.radii, dataset_spec.true_anchors, dataset_spec.true_radii)

        if delta is None:
            lines.append("SUMMARY: ERROR: missing params in summary parse")
            ok = False
        else:
            total_d, ad, rd = delta
            exact = (total_d == 0.0) and (fit_score_ui_delta in (None, 0.0))
            small = total_d < tol_mm_total

            summary_rows: List[List[str]] = [
                ["param_total", "-", "-", fmt(total_d), tol_status_text(exact, small, tol_mm_total, color)],
                ["param_anchors", "-", "-", fmt(ad), "-"],
                ["param_R*2π", "-", "-", fmt(rd), "-"],
            ]

            if fit_score_ui_delta is None:
                summary_rows.append(["fit_score_ui", fmt(ref.summary.fit_score_ui), fmt(gen.summary.fit_score_ui), "N/A", "optional"])
            else:
                summary_rows.append(
                    [
                        "fit_score_ui",
                        fmt(ref.summary.fit_score_ui),
                        fmt(gen.summary.fit_score_ui),
                        fmt(fit_score_ui_delta),
                        verdict_text(fit_score_ui_delta, color),
                    ]
                )

            if ref_true and gen_true:
                true_err_total_delta = float(gen_true[0] - ref_true[0])
                summary_rows.append(
                    [
                        "true_err_total",
                        fmt(ref_true[0]),
                        fmt(gen_true[0]),
                        fmt(true_err_total_delta),
                        verdict_text(true_err_total_delta, color),
                    ]
                )
                summary_rows.append(["true_err_anchors", fmt(ref_true[1]), fmt(gen_true[1]), fmt(gen_true[1] - ref_true[1]), "-"])
                summary_rows.append(["true_err_R*2π", fmt(ref_true[2]), fmt(gen_true[2]), fmt(gen_true[2] - ref_true[2]), "-"])
                mean_delta_label = dir_label(true_iter_mean_delta) if true_iter_mean_delta is not None else "N/A"
                lines.append(
                    f"RUN_TRACKER: true_err_total delta(gen-ref)={fmt(true_err_total_delta)} [{dir_label(true_err_total_delta)}], "
                    f"true_iter_mean_delta={fmt(true_iter_mean_delta)} [{mean_delta_label}], "
                    f"true_gen_iter_std={fmt(true_gen_iter_std)} "
                    f"(n={true_gen_iter_count})"
                )

            lines.append("Calibration summary:")
            lines.extend(["  " + x for x in format_table(
                headers=["metric", "ref", "gen", "delta(gen-ref)", "verdict"],
                rows=summary_rows,
            )])

            if exact:
                lines.append("SUMMARY: EXACTLY EQUAL")
            elif small:
                lines.append(f"SUMMARY: CHANGED, but within {tol_mm_total}mm total-param tolerance")
            else:
                lines.append(f"SUMMARY: CHANGED and EXCEEDS tolerance ({tol_mm_total}mm) => FAIL")
                ok = False

        if not exact:
            # Print the extracted summary blocks to make it easy to eyeball
            lines.append("REF summary block (for quick eyeballing):")
            lines.extend(["  " + l for l in ref.summary_block_lines[:8]])
            lines.append("GEN summary block (for quick eyeballing):")
            lines.extend(["  " + l for l in gen.summary_block_lines[:8]])

    # ---- Iterations ----
    n_ref = len(ref.iterations)
    n_gen = len(gen.iterations)
    lines.append(f"ITERATIONS: ref={n_ref} gen={n_gen}")

    if n_ref != n_gen:
        lines.append("ITERATIONS: COUNT MISMATCH => FAIL")
        ok = False

    n = max(n_ref, n_gen)
    # Track worst param delta among iterations (between ref and gen)
    worst_iter_delta = 0.0
    mismatches = 0
    mixed_rank_scales = 0
    missing_rank_pairs = 0

    # Print per-iter details only if something changed a lot or count mismatch,
    # but we still compute everything.
    detailed_rows: List[List[str]] = []
    for i in range(n):
        r = ref.iterations[i] if i < n_ref else None
        g = gen.iterations[i] if i < n_gen else None

        # If one side is missing, keep row visible so users can see extra iterations.
        if r is None or g is None:
            r_true = error_to_true(r.anchors, r.radii, dataset_spec.true_anchors, dataset_spec.true_radii) if r is not None else None
            g_true = error_to_true(g.anchors, g.radii, dataset_spec.true_anchors, dataset_spec.true_radii) if g is not None else None
            detailed_rows.append(
                [
                    str(i),
                    "N/A",
                    "N/A",
                    "N/A",
                    "missing",
                    fmt(r_true[0]) if r_true else "N/A",
                    fmt(g_true[0]) if g_true else "N/A",
                    "N/A",
                    "N/A",
                    fmt(iteration_effective_rank_score(r)) if r is not None else "N/A",
                    fmt(iteration_effective_rank_score(g)) if g is not None else "N/A",
                    "N/A",
                    "N/A",
                    fmt(r.fit_score_ui) if r is not None else "N/A",
                    fmt(g.fit_score_ui) if g is not None else "N/A",
                    "N/A",
                ]
            )
            ok = False
            continue

        d = total_param_distance(g.anchors, g.radii, r.anchors, r.radii)
        if d is None:
            detailed_rows.append(
                [
                    str(i),
                    "N/A",
                    "N/A",
                    "N/A",
                    "N/A",
                    "N/A",
                    "N/A",
                    "N/A",
                    "N/A",
                    "N/A",
                    "N/A",
                    "N/A",
                    "N/A",
                    "N/A",
                    "N/A",
                    _ansi("31", "parse-error", color),
                ]
            )
            ok = False
            continue

        total_d, ad, rd = d
        worst_iter_delta = max(worst_iter_delta, total_d)

        # "closer/further to true" per iteration
        r_true = error_to_true(r.anchors, r.radii, dataset_spec.true_anchors, dataset_spec.true_radii)
        g_true = error_to_true(g.anchors, g.radii, dataset_spec.true_anchors, dataset_spec.true_radii)

        r_rank_score = iteration_effective_rank_score(r)
        g_rank_score = iteration_effective_rank_score(g)
        r_rank_kind = iteration_effective_rank_kind(r)
        g_rank_kind = iteration_effective_rank_kind(g)
        rank_scale_mixed = (
            r_rank_kind is not None
            and g_rank_kind is not None
            and r_rank_kind != g_rank_kind
        )
        if rank_scale_mixed:
            mixed_rank_scales += 1
        elif r_rank_score is None or g_rank_score is None:
            missing_rank_pairs += 1

        # Ranking score reflection check (direction)
        rank_delta_iter = None
        if (not rank_scale_mixed) and r_rank_score is not None and g_rank_score is not None:
            rank_delta_iter = g_rank_score - r_rank_score

        fit_delta_iter = None
        if r_true and g_true:
            fit_delta_iter = g_true[0] - r_true[0]

        exact_iter = (total_d == 0.0) and (rank_delta_iter in (None, 0.0))
        small_iter = total_d < tol_mm_total

        # Rank score reflects fit?
        reflect = "N/A"
        mismatch = False
        if rank_scale_mixed:
            reflect = "mixed-scale"
        elif fit_delta_iter is not None and rank_delta_iter is not None:
            fit_dir = dir_label(fit_delta_iter)   # better/worse/equal (lower is better)
            rank_dir = dir_label(rank_delta_iter)
            reflect = "OK" if (fit_dir == rank_dir) or (fit_dir == "equal") or (rank_dir == "equal") else "MISMATCH"
            mismatch = (reflect == "MISMATCH")
            if mismatch:
                mismatches += 1

        detailed_rows.append(
            [
                str(i),
                fmt(total_d),
                fmt(ad),
                fmt(rd),
                "exact" if exact_iter else ("<tol" if small_iter else ">=tol"),
                fmt(r_true[0]) if r_true else "N/A",
                fmt(g_true[0]) if g_true else "N/A",
                fmt(fit_delta_iter),
                verdict_text(fit_delta_iter, color),
                fmt(r_rank_score),
                fmt(g_rank_score),
                fmt(rank_delta_iter),
                verdict_text(rank_delta_iter, color),
                fmt(r.fit_score_ui),
                fmt(g.fit_score_ui),
                reflect if reflect != "MISMATCH" else _ansi("31", reflect, color),
            ]
        )

        # Failing rules
        if total_d >= tol_mm_total:
            ok = False
        if mismatch and fail_on_score_mismatch:
            ok = False

    lines.append(f"ITERATIONS: worst_param_delta_total={fmt(worst_iter_delta)} (tol={tol_mm_total}mm)")
    lines.append("ITERATIONS: rank columns use history_rank when present, otherwise raw rank_score")
    if mixed_rank_scales:
        lines.append(
            f"ITERATIONS: mixed rank scales in {mixed_rank_scales} rows "
            f"(history_rank vs raw rank_score)"
        )
    if missing_rank_pairs:
        lines.append(f"ITERATIONS: rank comparisons unavailable in {missing_rank_pairs} rows (missing rank score)")
    if mismatches:
        lines.append(f"ITERATIONS: rank/true direction mismatches: {mismatches}" + (" => FAIL" if fail_on_score_mismatch else " (warn-only)"))
    else:
        lines.append("ITERATIONS: rank/true direction mismatches: 0")

    # Decide if we should print detailed rows:
    if (not ok) or (worst_iter_delta > 0.0) or (n_ref != n_gen):
        lines.append("ITERATIONS detail:")
        lines.extend(["  " + x for x in format_table(
            headers=[
                "iter",
                "Δparam_total",
                "Δanchors",
                "ΔR*2π",
                "tol",
                "true_ref",
                "true_gen",
                "Δtrue",
                "true_verdict",
                "rank_ref",
                "rank_gen",
                "Δrank",
                "rank_verdict",
                "fit_ui_ref",
                "fit_ui_gen",
                "reflect",
            ],
            rows=detailed_rows,
        )])

    return ok, lines, true_err_total_delta


def find_file_first_existing(candidates: List[Path]) -> Optional[Path]:
    for p in candidates:
        if p.exists():
            return p
    return None


def prepare_isolated_dataset_copy(dataset_name: str, dataset_path: Path, scratch_root: Path) -> Path:
    run_dir = scratch_root / f"{dataset_name}_{uuid.uuid4().hex[:8]}"
    run_dir.mkdir(parents=True, exist_ok=False)
    isolated = run_dir / dataset_path.name
    shutil.copy2(dataset_path, isolated)
    return isolated


def run_one_dataset(
    dataset_spec: DatasetSpec,
    repo_root: Path,
    dataset_path: Path,
    ref_log_path: Path,
    tol_mm_total: float,
    fail_on_score_mismatch: bool,
    color: bool,
    scratch_root: Path,
) -> DatasetRunResult:
    name = dataset_spec.name
    isolated_dataset = prepare_isolated_dataset_copy(name, dataset_path, scratch_root)
    isolated_jsonl = isolated_dataset.with_name(f"{isolated_dataset.stem}.full_auto_log.jsonl")

    rc, output = run_autocal(
        repo_root=repo_root,
        dataset_path=isolated_dataset,
        dataset_spec=dataset_spec,
        full_auto_log=isolated_jsonl,
    )
    if rc != 0:
        return DatasetRunResult(
            name=name,
            ok=False,
            lines=[
                f"=== {name} ===",
                f"ERROR: autocal.py exited with {rc}",
                "---- combined stdout/stderr ----",
                output.rstrip(),
            ],
            generated_log=None,
            reference_log=ref_log_path,
        )

    gen_log_rel = parse_generated_log_path(output)
    if not gen_log_rel:
        return DatasetRunResult(
            name=name,
            ok=False,
            lines=[
                f"=== {name} ===",
                "ERROR: could not find generated log path in stdout/stderr (looked for 'Writing additional info to log:')",
                "---- combined stdout/stderr ----",
                output.rstrip(),
            ],
            generated_log=None,
            reference_log=ref_log_path,
        )

    gen_log_path = (repo_root / gen_log_rel).resolve() if not Path(gen_log_rel).is_absolute() else Path(gen_log_rel).resolve()
    if not gen_log_path.exists():
        return DatasetRunResult(
            name=name,
            ok=False,
            lines=[
                f"=== {name} ===",
                f"ERROR: generated log path does not exist: {gen_log_path}",
                "---- combined stdout/stderr ----",
                output.rstrip(),
            ],
            generated_log=gen_log_path,
            reference_log=ref_log_path,
        )

    ref_parsed = parse_log_file(ref_log_path)
    gen_parsed = parse_log_file(gen_log_path)
    true_iter_mean_delta, _true_iter_pair_count = compute_true_iter_mean_delta(
        ref_parsed.iterations,
        gen_parsed.iterations,
        dataset_spec.true_anchors,
        dataset_spec.true_radii,
    )
    _true_gen_iter_mean, true_gen_iter_std, true_gen_iter_count = compute_true_gen_iter_stats(
        gen_parsed.iterations,
        dataset_spec.true_anchors,
        dataset_spec.true_radii,
    )

    ok, lines, true_err_total_delta = report_dataset(
        name=name,
        ref=ref_parsed,
        gen=gen_parsed,
        dataset_spec=dataset_spec,
        tol_mm_total=tol_mm_total,
        fail_on_score_mismatch=fail_on_score_mismatch,
        color=color,
    )
    return DatasetRunResult(
        name=name,
        ok=ok,
        lines=lines,
        generated_log=gen_log_path,
        reference_log=ref_log_path,
        true_err_total_delta=true_err_total_delta,
        true_iter_mean_delta=true_iter_mean_delta,
        true_gen_iter_std=true_gen_iter_std,
        true_gen_iter_count=true_gen_iter_count,
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", type=str, default=".", help="Path to repo root (must contain autocal/autocal.py).")
    ap.add_argument("--data-dir", type=str, default="autocal/data/references", help="Directory containing the datasets (.json).")
    ap.add_argument("--ref-dir", type=str, default="autocal/data/references", help="Directory containing the reference logs (.log).")
    ap.add_argument("--tol-mm", type=float, default=0.01, help="Tolerance on total parameter distance (anchors + 2*pi*R).")
    ap.add_argument("--no-fail-score-mismatch", action="store_true", help="Do not fail on score/fit direction mismatch (still reported).")
    ap.add_argument("--keep-going", action="store_true", help="Run all datasets even if one fails.")
    ap.add_argument("--color", choices=["auto", "always", "never"], default="auto", help="Colorize output verdicts.")
    ap.add_argument(
        "--only",
        choices=sorted({dataset.machine_type for dataset in DATASETS}),
        help="Run only datasets for one machine type.",
    )
    args = ap.parse_args()
    color = use_color(args.color)

    repo_root = Path(args.repo_root).resolve()
    if not (repo_root / "autocal" / "autocal.py").exists():
        print(f"ERROR: repo_root={repo_root} does not contain autocal/autocal.py", file=sys.stderr)
        return 1

    data_dir = (repo_root / args.data_dir).resolve() if not Path(args.data_dir).is_absolute() else Path(args.data_dir).resolve()
    ref_dir = (repo_root / args.ref_dir).resolve() if not Path(args.ref_dir).is_absolute() else Path(args.ref_dir).resolve()

    # Also try /mnt/data as a convenience if user runs in a sandbox-like environment.
    alt_dir = Path("/mnt/data")

    overall_ok = True
    jobs: List[Tuple[DatasetSpec, Path, Path]] = []
    dataset_specs = DATASETS
    if args.only is not None:
        dataset_specs = [dataset for dataset in DATASETS if dataset.machine_type == args.only]

    for dataset_spec in dataset_specs:
        ds = dataset_spec.name
        dataset_path = find_file_first_existing([
            data_dir / f"{ds}.json",
            alt_dir / f"{ds}.json",
        ])
        ref_candidates = [ref_dir / f"{ds}.{name}" for name in dataset_spec.reference_log_names]
        ref_candidates.extend(alt_dir / f"{ds}.{name}" for name in dataset_spec.reference_log_names)
        ref_log_path = find_file_first_existing(ref_candidates)

        if dataset_path is None:
            print(f"=== {ds} ===")
            print(f"ERROR: dataset not found (tried {data_dir}/{ds}.json and /mnt/data/{ds}.json)")
            overall_ok = False
            continue
        if ref_log_path is None:
            print(f"=== {ds} ===")
            print(f"ERROR: reference log not found (tried {', '.join(str(path) for path in ref_candidates)})")
            overall_ok = False
            continue
        jobs.append((dataset_spec, dataset_path, ref_log_path))

    scratch_root = (repo_root / "autocal" / "data" / ".regress_parallel_runs").resolve()
    scratch_root.mkdir(parents=True, exist_ok=True)

    if jobs:
        completed_results: List[DatasetRunResult] = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(jobs)) as pool:
            fut_to_name = {
                pool.submit(
                    run_one_dataset,
                    dataset_spec=dataset_spec,
                    repo_root=repo_root,
                    dataset_path=dataset_path,
                    ref_log_path=ref_log_path,
                    tol_mm_total=float(args.tol_mm),
                    fail_on_score_mismatch=not args.no_fail_score_mismatch,
                    color=color,
                    scratch_root=scratch_root,
                ): dataset_spec.name
                for dataset_spec, dataset_path, ref_log_path in jobs
            }
            for fut in concurrent.futures.as_completed(fut_to_name):
                ds = fut_to_name[fut]
                try:
                    result = fut.result()
                except Exception as exc:
                    print(f"=== {ds} ===")
                    print(f"ERROR: unexpected worker failure: {exc}")
                    overall_ok = False
                    continue

                print("\n".join(result.lines))
                if result.generated_log is not None:
                    print(f"Generated log: {result.generated_log}")
                if result.reference_log is not None:
                    print(f"Reference log: {result.reference_log}")
                completed_results.append(result)
                if not result.ok:
                    overall_ok = False

        if completed_results:
            completed_sorted = sorted(completed_results, key=lambda r: r.name)
            rows: List[List[str]] = []
            sum_delta = 0.0
            sum_true_mean_delta = 0.0
            sum_count = 0
            for r in completed_sorted:
                d = r.true_err_total_delta
                if d is not None and math.isfinite(d):
                    rows.append([
                        r.name,
                        fmt(d),
                        dir_label(d),
                        fmt(r.true_iter_mean_delta),
                        fmt(r.true_gen_iter_std),
                        str(r.true_gen_iter_count),
                    ])
                    sum_delta += float(d)
                    sum_count += 1
                else:
                    rows.append([
                        r.name,
                        "N/A",
                        "N/A",
                        fmt(r.true_iter_mean_delta),
                        fmt(r.true_gen_iter_std),
                        str(r.true_gen_iter_count),
                    ])
                if r.true_iter_mean_delta is not None and math.isfinite(r.true_iter_mean_delta):
                    sum_true_mean_delta += float(r.true_iter_mean_delta)

            print("\nRUN_TRACKER: true_err_total delta(gen-ref) and true_iter mean delta by dataset")
            for line in format_table(
                headers=["dataset", "delta(gen-ref)", "verdict", "true_iter_mean_delta", "true_gen_std", "n_true_gen"],
                rows=rows,
            ):
                print("  " + line)
            if sum_count > 0:
                print(
                    f"RUN_TRACKER: sum_true_err_total_delta(gen-ref)={fmt(sum_delta)} over {sum_count} datasets [{dir_label(sum_delta)}]"
                )
                final_score = compute_final_score(sum_delta, sum_true_mean_delta)
                print(
                    f"RUN_TRACKER: final_score={fmt(final_score)} "
                    f"(sum_true_err_total_delta + sum_true_mean_delta, "
                    f"sum_true_mean_delta={fmt(sum_true_mean_delta)})"
                )
            else:
                print("RUN_TRACKER: sum_true_err_total_delta(gen-ref)=N/A (parse missing)")

    if overall_ok:
        print("\nALL DATASETS: PASS")
        return 0
    else:
        print("\nONE OR MORE DATASETS: FAIL")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
