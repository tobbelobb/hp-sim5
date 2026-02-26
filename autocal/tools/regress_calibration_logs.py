#!/usr/bin/env python3
"""
Regression test runner for autocal/active_calibrate.py full-auto runs.

What it does
- Runs 5 fixed commands (one per dataset)
- Locates the generated log path from stdout/stderr (looks for "Writing additional info to log:")
- Compares generated vs reference logs by parsing:
  * Final "== Calibration summary ==" block (fit score + M669 anchors + M666 radii)
  * Each iteration triple: "Anchors:", "line_model ... effective=[...]", "Score: ..."

Comparison metrics
- Anchor distance: sum_i ||anchor_i - anchor_i_ref||  (Euclidean, mm)
- Radius distance: 60*pi * sum_i |R_i - R_i_ref|  (as requested)
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
import math
import os
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple


DATASETS = [
    "five_points_bigger_deltas",
    "layered_manual_some_bad_points",
    "ten_points_bigger_deltas",
    "ten_points_w_buildup",
    "ten_points_w_buildup2",
]

# Ground truth for --sim datasets (provided by user)
TRUE_ANCHORS: List[Tuple[float, float]] = [
    (0.0, -1900.0),
    (1645.44826719, 950.0),
    (-1645.44826719, 950.0),
]
TRUE_R: float = 39.184


@dataclass
class Params:
    anchors: Optional[List[Tuple[float, float]]]  # len=3
    radii: Optional[List[float]]  # len=3
    score: Optional[float]  # "Fit quality score" for summary, or iteration Score


@dataclass
class Iteration:
    anchors: Optional[List[Tuple[float, float]]]
    radii: Optional[List[float]]
    score: Optional[float]
    anchors_line: Optional[str] = None
    radii_line: Optional[str] = None
    score_line: Optional[str] = None


@dataclass
class ParsedLog:
    summary: Optional[Params]
    iterations: List[Iteration]
    summary_block_lines: List[str]


# ---------- Parsing helpers ----------

_RE_LOGPATH = re.compile(r"Writing additional info to log:\s*(.+)\s*$")
_RE_ANCHORS = re.compile(r"Anchors:\s*(\[\[.*\]\])")
_RE_EFFECTIVE = re.compile(r"effective=\[([^\]]+)\]")
_RE_SCORE = re.compile(r"\bScore:\s*([0-9.+-eE]+)\b")
_RE_M669_PARTS = re.compile(
    r"([ABC])\s*([+-]?\d+(?:\.\d+)?(?:e[+-]?\d+)?)\:([+-]?\d+(?:\.\d+)?(?:e[+-]?\d+)?)\:([+-]?\d+(?:\.\d+)?(?:e[+-]?\d+)?)",
    re.IGNORECASE,
)
_RE_M666_R = re.compile(
    r"R\s*([+-]?\d+(?:\.\d+)?(?:e[+-]?\d+)?):([+-]?\d+(?:\.\d+)?(?:e[+-]?\d+)?):([+-]?\d+(?:\.\d+)?(?:e[+-]?\d+)?)",
    re.IGNORECASE,
)
_RE_FIT_SCORE = re.compile(r"Fit quality score.*?:\s*([0-9.+-eE]+)\s*$")


def _safe_float(x: str) -> Optional[float]:
    try:
        return float(x)
    except Exception:
        return None


def parse_generated_log_path(run_output: str) -> Optional[str]:
    """
    Parses stdout/stderr combined output from active_calibrate.py and returns the produced log path.
    We accept both raw and "Wrote to console:" prefixed lines.
    """
    for line in run_output.splitlines():
        m = _RE_LOGPATH.search(line)
        if m:
            return m.group(1).strip()
    return None


def parse_iterations(text: str) -> List[Iteration]:
    """
    Extracts per-iteration (anchors, effective radii, score) triplets in order.
    Assumes score line terminates an iteration.
    """
    iters: List[Iteration] = []
    cur: dict = {}
    for line in text.splitlines():
        if "Anchors:" in line:
            m = _RE_ANCHORS.search(line)
            if m:
                try:
                    arr = ast.literal_eval(m.group(1))
                    anchors_xy = [(float(a[0]), float(a[1])) for a in arr]
                    cur["anchors"] = anchors_xy
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
        if "Score:" in line:
            m = _RE_SCORE.search(line)
            if m:
                score = _safe_float(m.group(1))
                iters.append(
                    Iteration(
                        anchors=cur.get("anchors"),
                        radii=cur.get("radii"),
                        score=score,
                        anchors_line=cur.get("anchors_line"),
                        radii_line=cur.get("radii_line"),
                        score_line=line.strip(),
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
    Parses the last calibration summary for:
    - fit score
    - anchors (A,B,C)
    - radii (R1:R2:R3)
    """
    lines = extract_summary_block_lines(text, max_lines=60)
    if not lines:
        return None

    fit_score: Optional[float] = None
    anchors_map: dict[str, Tuple[float, float]] = {}
    radii: Optional[List[float]] = None

    for line in lines:
        if "Fit quality score" in line:
            m = _RE_FIT_SCORE.search(line)
            if m:
                fit_score = _safe_float(m.group(1))
        if "Parameters (M669)" in line:
            parts = _RE_M669_PARTS.findall(line)
            for (k, x, y, _z) in parts:
                fx, fy = _safe_float(x), _safe_float(y)
                if fx is not None and fy is not None:
                    anchors_map[k.upper()] = (fx, fy)
        if "Line model (M666)" in line:
            m = _RE_M666_R.search(line)
            if m:
                radii = [_safe_float(m.group(1)), _safe_float(m.group(2)), _safe_float(m.group(3))]
                if any(v is None for v in radii):
                    radii = None
                else:
                    radii = [float(v) for v in radii]  # type: ignore

    if fit_score is None or radii is None or len(anchors_map) != 3:
        return None

    anchors = [anchors_map["A"], anchors_map["B"], anchors_map["C"]]
    return Params(anchors=anchors, radii=radii, score=fit_score)


def parse_log_file(path: Path) -> ParsedLog:
    text = path.read_text(errors="replace")
    summary = parse_summary(text)
    iters = parse_iterations(text)
    summary_lines = extract_summary_block_lines(text)
    return ParsedLog(summary=summary, iterations=iters, summary_block_lines=summary_lines)


# ---------- Metrics ----------

def anchor_distance(a: List[Tuple[float, float]], b: List[Tuple[float, float]]) -> float:
    return sum(math.hypot(ax - bx, ay - by) for (ax, ay), (bx, by) in zip(a, b))


def radius_distance(r: List[float], r2: List[float]) -> float:
    # As requested: 60*pi*distance for the R values
    return (60.0 * math.pi) * sum(abs(x - y) for x, y in zip(r, r2))


def total_param_distance(
    anchors1: Optional[List[Tuple[float, float]]],
    radii1: Optional[List[float]],
    anchors2: Optional[List[Tuple[float, float]]],
    radii2: Optional[List[float]],
) -> Optional[Tuple[float, float, float]]:
    if anchors1 is None or anchors2 is None or radii1 is None or radii2 is None:
        return None
    ad = anchor_distance(anchors1, anchors2)
    rd = radius_distance(radii1, radii2)
    return (ad + rd, ad, rd)


def error_to_true(anchors: Optional[List[Tuple[float, float]]], radii: Optional[List[float]]) -> Optional[Tuple[float, float, float]]:
    if anchors is None or radii is None:
        return None
    ad = anchor_distance(anchors, TRUE_ANCHORS)
    rd = radius_distance(radii, [TRUE_R, TRUE_R, TRUE_R])
    return (ad + rd, ad, rd)


def dir_label(delta: float, eps: float = 0.0) -> str:
    if abs(delta) <= eps:
        return "equal"
    return "better" if delta < 0 else "worse"  # lower is better


# ---------- Running commands ----------

def run_active_calibrate(repo_root: Path, dataset_path: Path) -> Tuple[int, str]:
    cmd = [
        sys.executable,
        str(repo_root / "autocal" / "active_calibrate.py"),
        "--sim",
        "--machine-type",
        "slideprinter",
        "--full-auto",
        "--dataset",
        str(dataset_path),
        "--find-radii",
        "global",
        "--base-radii",
        "30.0",
        "--buildup-factor",
        "0.636619",
        "--no-collect",
    ]

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


def report_dataset(
    name: str,
    ref: ParsedLog,
    gen: ParsedLog,
    tol_mm_total: float,
    fail_on_score_mismatch: bool,
) -> Tuple[bool, List[str]]:
    """
    Returns (ok, lines).
    ok=False if changes exceed tolerance or mismatch rules trigger.
    """
    lines: List[str] = []
    ok = True

    # ---- Summary ----
    lines.append(f"=== {name} ===")
    if ref.summary is None:
        lines.append("REF: ERROR: could not parse calibration summary")
        ok = False
    if gen.summary is None:
        lines.append("GEN: ERROR: could not parse calibration summary")
        ok = False

    if ref.summary and gen.summary:
        delta = total_param_distance(gen.summary.anchors, gen.summary.radii, ref.summary.anchors, ref.summary.radii)
        score_delta = (gen.summary.score - ref.summary.score) if (gen.summary.score is not None and ref.summary.score is not None) else None

        ref_true = error_to_true(ref.summary.anchors, ref.summary.radii)
        gen_true = error_to_true(gen.summary.anchors, gen.summary.radii)

        if delta is None:
            lines.append("SUMMARY: ERROR: missing params in summary parse")
            ok = False
        else:
            total_d, ad, rd = delta
            exact = (total_d == 0.0) and (score_delta == 0.0)
            small = total_d < tol_mm_total

            lines.append(f"SUMMARY: param_delta_total={fmt(total_d)} (anchors={fmt(ad)}, R*60π={fmt(rd)})  tol={tol_mm_total}mm")
            if score_delta is None:
                lines.append("SUMMARY: score_delta=N/A (parse missing)")
                ok = False
            else:
                lines.append(f"SUMMARY: score_ref={fmt(ref.summary.score)} score_gen={fmt(gen.summary.score)}  score_delta={fmt(score_delta)}")

            if ref_true and gen_true:
                lines.append(
                    f"SUMMARY vs TRUE: err_ref={fmt(ref_true[0])} (anchors={fmt(ref_true[1])}, R*60π={fmt(ref_true[2])}) "
                    f"err_gen={fmt(gen_true[0])} (anchors={fmt(gen_true[1])}, R*60π={fmt(gen_true[2])}) "
                    f"delta(gen-ref)={fmt(gen_true[0]-ref_true[0])} => {dir_label(gen_true[0]-ref_true[0])}"
                )

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

    n = min(n_ref, n_gen)
    # Track worst param delta among iterations (between ref and gen)
    worst_iter_delta = 0.0
    mismatches = 0

    # Print per-iter details only if something changed a lot or count mismatch,
    # but we still compute everything.
    detailed_rows: List[str] = []
    for i in range(n):
        r = ref.iterations[i]
        g = gen.iterations[i]

        d = total_param_distance(g.anchors, g.radii, r.anchors, r.radii)
        if d is None:
            detailed_rows.append(f"  iter {i}: ERROR: could not compare params (parse missing)")
            ok = False
            continue

        total_d, ad, rd = d
        worst_iter_delta = max(worst_iter_delta, total_d)

        # "closer/further to true" per iteration
        r_true = error_to_true(r.anchors, r.radii)
        g_true = error_to_true(g.anchors, g.radii)

        # Score reflection check (direction)
        score_delta_iter = None
        if r.score is not None and g.score is not None:
            score_delta_iter = g.score - r.score

        fit_delta_iter = None
        if r_true and g_true:
            fit_delta_iter = g_true[0] - r_true[0]

        exact_iter = (total_d == 0.0) and (score_delta_iter == 0.0)
        small_iter = total_d < tol_mm_total

        # Score reflects fit?
        reflect = "N/A"
        mismatch = False
        if fit_delta_iter is not None and score_delta_iter is not None:
            fit_dir = dir_label(fit_delta_iter)   # better/worse/equal (lower is better)
            score_dir = dir_label(score_delta_iter)
            reflect = "OK" if (fit_dir == score_dir) or (fit_dir == "equal") or (score_dir == "equal") else "MISMATCH"
            mismatch = (reflect == "MISMATCH")
            if mismatch:
                mismatches += 1

        row = (
            f"  iter {i}: param_delta_total={fmt(total_d)} (anchors={fmt(ad)}, R*60π={fmt(rd)}) "
            f"{'EXACT' if exact_iter else ('<tol' if small_iter else '>=tol')} "
        )
        if r_true and g_true:
            row += f"| true_err_ref={fmt(r_true[0])} true_err_gen={fmt(g_true[0])} Δ={fmt(fit_delta_iter)}=>{dir_label(fit_delta_iter)} "
        else:
            row += "| true_err: N/A "
        if score_delta_iter is not None:
            row += f"| score_ref={fmt(r.score)} score_gen={fmt(g.score)} Δ={fmt(score_delta_iter)}=>{dir_label(score_delta_iter)} "
        else:
            row += "| score: N/A "
        row += f"| score_reflects_fit={reflect}"
        detailed_rows.append(row)

        # Failing rules
        if total_d >= tol_mm_total:
            ok = False
        if mismatch and fail_on_score_mismatch:
            ok = False

    lines.append(f"ITERATIONS: worst_param_delta_total={fmt(worst_iter_delta)} (tol={tol_mm_total}mm)")
    if mismatches:
        lines.append(f"ITERATIONS: score/fit direction mismatches: {mismatches}" + (" => FAIL" if fail_on_score_mismatch else " (warn-only)"))
    else:
        lines.append("ITERATIONS: score/fit direction mismatches: 0")

    # Decide if we should print detailed rows:
    if (not ok) or (worst_iter_delta > 0.0) or (n_ref != n_gen):
        lines.append("ITERATIONS detail:")
        lines.extend(detailed_rows)

    return ok, lines


def find_file_first_existing(candidates: List[Path]) -> Optional[Path]:
    for p in candidates:
        if p.exists():
            return p
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", type=str, default=".", help="Path to repo root (must contain autocal/active_calibrate.py).")
    ap.add_argument("--data-dir", type=str, default="autocal/data", help="Directory containing the datasets (.json).")
    ap.add_argument("--ref-dir", type=str, default="autocal/data", help="Directory containing the reference logs (.log).")
    ap.add_argument("--tol-mm", type=float, default=10.0, help="Tolerance on total parameter distance (anchors + 60*pi*R).")
    ap.add_argument("--no-fail-score-mismatch", action="store_true", help="Do not fail on score/fit direction mismatch (still reported).")
    ap.add_argument("--keep-going", action="store_true", help="Run all datasets even if one fails.")
    args = ap.parse_args()

    repo_root = Path(args.repo_root).resolve()
    if not (repo_root / "autocal" / "active_calibrate.py").exists():
        print(f"ERROR: repo_root={repo_root} does not contain autocal/active_calibrate.py", file=sys.stderr)
        return 1

    data_dir = (repo_root / args.data_dir).resolve() if not Path(args.data_dir).is_absolute() else Path(args.data_dir).resolve()
    ref_dir = (repo_root / args.ref_dir).resolve() if not Path(args.ref_dir).is_absolute() else Path(args.ref_dir).resolve()

    # Also try /mnt/data as a convenience if user runs in a sandbox-like environment.
    alt_dir = Path("/mnt/data")

    overall_ok = True
    for ds in DATASETS:
        dataset_path = find_file_first_existing([
            data_dir / f"{ds}.json",
            alt_dir / f"{ds}.json",
        ])
        ref_log_path = find_file_first_existing([
            ref_dir / f"{ds}.full_auto_reference_run_feb_26.log",
            alt_dir / f"{ds}.full_auto_reference_run_feb_26.log",
        ])

        if dataset_path is None:
            print(f"=== {ds} ===")
            print(f"ERROR: dataset not found (tried {data_dir}/{ds}.json and /mnt/data/{ds}.json)")
            overall_ok = False
            if not args.keep_going:
                break
            continue
        if ref_log_path is None:
            print(f"=== {ds} ===")
            print(f"ERROR: reference log not found (tried {ref_dir}/{ds}.full_auto_reference_run_feb_26.log and /mnt/data/...)")
            overall_ok = False
            if not args.keep_going:
                break
            continue

        rc, output = run_active_calibrate(repo_root, dataset_path)
        if rc != 0:
            print(f"=== {ds} ===")
            print(f"ERROR: active_calibrate.py exited with {rc}")
            print("---- combined stdout/stderr ----")
            print(output.rstrip())
            overall_ok = False
            if not args.keep_going:
                break
            continue

        gen_log_rel = parse_generated_log_path(output)
        if not gen_log_rel:
            print(f"=== {ds} ===")
            print("ERROR: could not find generated log path in stdout/stderr (looked for 'Writing additional info to log:')")
            print("---- combined stdout/stderr ----")
            print(output.rstrip())
            overall_ok = False
            if not args.keep_going:
                break
            continue

        gen_log_path = (repo_root / gen_log_rel).resolve() if not Path(gen_log_rel).is_absolute() else Path(gen_log_rel).resolve()
        if not gen_log_path.exists():
            print(f"=== {ds} ===")
            print(f"ERROR: generated log path does not exist: {gen_log_path}")
            print("---- combined stdout/stderr ----")
            print(output.rstrip())
            overall_ok = False
            if not args.keep_going:
                break
            continue

        ref_parsed = parse_log_file(ref_log_path)
        gen_parsed = parse_log_file(gen_log_path)

        ok, lines = report_dataset(
            name=ds,
            ref=ref_parsed,
            gen=gen_parsed,
            tol_mm_total=float(args.tol_mm),
            fail_on_score_mismatch=not args.no_fail_score_mismatch,
        )
        print("\n".join(lines))
        print(f"Generated log: {gen_log_path}")
        print(f"Reference log: {ref_log_path}")

        if not ok:
            overall_ok = False
            if not args.keep_going:
                break

    if overall_ok:
        print("\nALL DATASETS: PASS")
        return 0
    else:
        print("\nONE OR MORE DATASETS: FAIL")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
