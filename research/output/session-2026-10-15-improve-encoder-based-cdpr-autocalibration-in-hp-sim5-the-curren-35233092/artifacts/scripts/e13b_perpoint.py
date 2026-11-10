"""E13b (corrected): per-point perturbation sensitivity of a well-identified 9-sweep fit.

The 0.01 -> 0.05 mm fixed-anchor tolerance removes a ~0.03 mm correction move that
was issued before every point, so it perturbs the *per-point* collected length at
the drive and sensor anchors. The measured A/B difference (ab-matched/) is 51% of
l_drive/l_sensor readings exactly 0 and 0.005-0.094 mm at the rest (mean 0.053).
This replays the full 9-sweep reference dataset with perturbations of that shape
and magnitude, so the anchor sensitivity is measured instead of argued.

Note: `--dataset` replay fits `l_drive`/`l_sensor` (the noise-mean length pair),
not `raw_angles_deg`; a +-5 deg per-point change to raw_angles_deg leaves the fit
bit-identical. Each delta is applied to both the raw and the `_mu` length.
"""
from __future__ import annotations
import json, os, re, shutil, subprocess, sys, tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np

ROOT = Path("/home/torbjorn/repos/hp-sim5-worktrees/improve-encoder-based-cdpr-autocalibrati-af9af5fc")
sys.path.insert(0, str(ROOT))
from autocal.tools.regress_calibration_logs import SIM_3D_TRUE_ANCHORS, error_to_true

S = ROOT / ("output/research/sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-"
            "hp-sim5-the-curren-35233092")
SRC = S / "artifacts/reference-003/sweeps.json"
OUT = S / "artifacts/sensitivity-perpoint"
TRUE = [tuple(map(float, a)) for a in SIM_3D_TRUE_ANCHORS]
MEASURED_MAX = 0.0942          # max per-point |d| on l_drive/l_sensor, ab-matched/
MEASURED_MIN = 0.0052          # one 0.01 deg encoder quantum at 30 mm radius
MEASURED_NONZERO_FRAC = 0.488

CASES = [("baseline", None, "none", 0.0)]
for s in range(6):
    CASES.append((f"measured_s{s}", s, "measured", MEASURED_MAX))
for s in range(6):
    CASES.append((f"uniform_s{s}", s, "uniform", MEASURED_MAX))
for s in range(4):
    CASES.append((f"wide_s{s}", s, "uniform", 0.150))


def perturb(dataset, seed, kind, mag_mm):
    d = json.loads(json.dumps(dataset))
    if kind == "none":
        return d
    rng = np.random.default_rng(4321 + seed)
    out = 0
    for sw in d["sweeps"]:
        for pt in sw["data_points"]:
            for raw, mu in (("l_drive", "l_drive_mu"), ("l_sensor", "l_sensor_mu")):
                base = pt.get(raw)
                if not isinstance(base, (int, float)):
                    continue
                if kind == "uniform":
                    delta = rng.uniform(-mag_mm, mag_mm)
                else:
                    if rng.random() > MEASURED_NONZERO_FRAC:
                        continue
                    delta = rng.uniform(MEASURED_MIN, mag_mm) * (1 if rng.random() < 0.5 else -1)
                pt[raw] = base + delta
                if isinstance(pt.get(mu), (int, float)):
                    pt[mu] = pt[mu] + delta
                out += 1
    return d


def run(case):
    name, seed, kind, mag = case
    dest = OUT / name
    dest.mkdir(parents=True, exist_ok=True)
    scratch = Path(tempfile.mkdtemp(prefix=f"pp-{name}-", dir="/tmp"))
    ds = scratch / f"{name}.json"
    ds.write_text(json.dumps(perturb(json.load(open(SRC)), seed, kind, mag)))
    cmd = [sys.executable, "-m", "autocal.autocal", "--sim", "--machine-type", "hangprinter_3",
           "--dataset", str(ds), "--find-radii", "off", "--find-buildup-factor", "off",
           "--no-collect", "--full-auto-log", str(dest / "run.jsonl")]
    env = os.environ.copy()
    env["PYTHONHASHSEED"] = "0"
    for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        env[k] = "1"
    env["MPLCONFIGDIR"] = "/tmp/mpl"
    proc = subprocess.run(cmd, cwd=str(ROOT), env=env, stdout=subprocess.PIPE,
                          stderr=subprocess.DEVNULL, text=True)
    (dest / "stdout.txt").write_text(proc.stdout)
    shutil.rmtree(scratch, ignore_errors=True)
    m = re.search(r"Anchors \(M669\): M669 (.+)", proc.stdout)
    q = re.search(r"Fit quality score \(lower is better\): ([\d.eE+-]+)", proc.stdout)
    if not m:
        return {"case": name, "kind": kind, "mag": mag, "true_err": None, "fit_quality": None}
    anchors = [[float(x) for x in tok[1:].split(":")] for tok in m.group(1).split()]
    te = float(error_to_true([tuple(a) for a in anchors], [1.0] * 4, TRUE, [1.0] * 4)[1])
    return {"case": name, "kind": kind, "mag": mag, "true_err": te,
            "fit_quality": float(q.group(1)) if q else None}


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    only = os.environ.get("ONLY", "")
    cases = [c for c in CASES if c[0].startswith(only)] if only else CASES
    rows = []
    with ThreadPoolExecutor(max_workers=int(os.environ.get("JOBS", "4"))) as ex:
        for r in ex.map(run, cases):
            print(f"{r['case']:14s} kind={r['kind']:8s} mag={r['mag']:.3f} "
                  f"true_err={r['true_err'] if r['true_err'] is None else round(r['true_err'],3)} "
                  f"fit_q={r['fit_quality']}", flush=True)
            rows.append(r)
    if not only:
        json.dump(rows, open(OUT / "perpoint.json", "w"), indent=1)
