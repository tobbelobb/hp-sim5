"""Replay patience ranking, newest_trusted_index and labels with J0 (classical) vs J1 (robust) folds."""
import json, sys
from pathlib import Path
import numpy as np
from scipy.stats import spearmanr

WT = Path("/home/torbjorn/.t3/worktrees/hp-sim5/t3-244bb580-382b3b0103efa274")
sys.path.insert(0, str(WT))
from autocal.anchor_uncertainty import newest_trusted_index, uncertainty_label, fold_spread_mm
from autocal.stopping import history_rank_position

MAIN = Path("/home/torbjorn/repos/hp-sim5")


def j0(f):
    n = len(f); return float(np.max(np.sqrt((n - 1) / n * np.sum((f - f.mean(0)) ** 2, axis=0))))


def patience_of(run):
    for line in open(MAIN / run["f"]):
        e = json.loads(line)
        if e.get("patience") is not None:
            return int(e["patience"])
    return 8


def replay(iters, stat, patience):
    hist, no_improve, stop = [], 0, None
    for it in iters:
        hist.append(dict(iteration=it["iteration"], anchor_uncertainty_mm=it[stat]))
        rank = history_rank_position(hist, iteration=it["iteration"])
        no_improve = 0 if rank == 1 else no_improve + 1
        if no_improve >= patience and stop is None:
            stop = it["iteration"]
    return stop


rows = [json.loads(l) for l in open(sys.argv[1])]
runs, finals, changes = [], [], []
for run in rows:
    iters = []
    for it in run["iters"]:
        if "err" not in it:
            continue
        if it["sweeps"] < 3:
            it["J0"] = it["J1"] = None
        elif it["folds"] is None:
            it["J0"] = it["J1"] = float("inf")
        else:
            f = np.asarray(it["folds"]); it["J0"], it["J1"] = j0(f), fold_spread_mm(f)
        iters.append(it)
    if not iters:
        continue
    P = patience_of(run)
    s0, s1 = replay(iters, "J0", P), replay(iters, "J1", P)
    last = iters[-1]["iteration"]
    # decision at each possible stop point: actual end, and each replayed patience stop
    def select(stat, upto):
        sub = [it for it in iters if it["iteration"] <= upto]
        idx = newest_trusted_index([it[stat] for it in sub])
        return None if idx is None else sub[idx]
    rec = dict(f=run["f"], machine=run["machine"], stop=run["stop"], key=run["key"], P=P, last=last,
               n=len(iters), stop_J0=s0, stop_J1=s1)
    for name, upto in (("end", last), ("p0", s0 or last), ("p1", s1 or last)):
        a, b = select("J0", upto), select("J1", upto)
        rec[f"sel_{name}"] = (a and a["iteration"], b and b["iteration"], a and round(a["err"], 2), b and round(b["err"], 2))
    fin = iters[-1]
    rec.update(err=fin["err"], J0=fin["J0"], J1=fin["J1"])
    runs.append(rec)
# dedupe by key
seen, uniq = set(), []
for r in runs:
    if r["key"] in seen:
        continue
    seen.add(r["key"]); uniq.append(r)
print(f"runs replayed {len(runs)}, distinct {len(uniq)}")
ok = [r for r in uniq if r["J0"] is not None and np.isfinite(r["J0"])]
e = np.array([r["err"] for r in ok]); a0 = np.array([r["J0"] for r in ok]); a1 = np.array([r["J1"] for r in ok])
print("final-fit Spearman vs err: J0 %.3f  J1 %.3f  (n=%d)" % (spearmanr(a0, e)[0], spearmanr(a1, e)[0], len(ok)))
for m in sorted({r["machine"] for r in ok}):
    sel = [r for r in ok if r["machine"] == m]
    if len(sel) > 3:
        ee = [r["err"] for r in sel]
        print(f"  {m:30s} n={len(sel):3d} Spearman J0 {spearmanr([r['J0'] for r in sel], ee)[0]:+.2f} J1 {spearmanr([r['J1'] for r in sel], ee)[0]:+.2f}"
              f" median J/err J0 {np.median([r['J0']/r['err'] for r in sel]):.2f} J1 {np.median([r['J1']/r['err'] for r in sel]):.2f}")
for thr in (5, 10):
    big = [r for r in ok if r["err"] > thr]
    print(f">{thr} mm: n={len(big)}  min J0 {min([r['J0'] for r in big], default=None)}  min J1 {min([r['J1'] for r in big], default=None)}")
    for r in big:
        print(f"   err {r['err']:6.2f} J0 {r['J0']:7.2f} J1 {r['J1']:7.2f} {r['machine']} {r['f']}")
lab = lambda v: uncertainty_label(v)
print("label confusion (final fits): err<=1.5 & poor:", sum(r["err"] <= 1.5 and lab(r["J0"]) == "poor" for r in ok),
      sum(r["err"] <= 1.5 and lab(r["J1"]) == "poor" for r in ok), " err>3 & not poor:",
      sum(r["err"] > 3 and lab(r["J0"]) != "poor" for r in ok), sum(r["err"] > 3 and lab(r["J1"]) != "poor" for r in ok))
print("\nDecision differences (distinct runs):")
for r in uniq:
    diff = r["stop_J0"] != r["stop_J1"] or any(r[k][0] != r[k][1] for k in ("sel_end", "sel_p0", "sel_p1"))
    if diff:
        print(json.dumps({k: r[k] for k in ("f", "machine", "stop", "P", "last", "stop_J0", "stop_J1", "sel_end", "sel_p0", "sel_p1")}))
print("n distinct with any difference:", sum(1 for r in uniq if r["stop_J0"] != r["stop_J1"] or any(r[k][0] != r[k][1] for k in ("sel_end", "sel_p0", "sel_p1"))))
print("patience fires within run length: J0", sum(1 for r in uniq if r["stop_J0"]), "J1", sum(1 for r in uniq if r["stop_J1"]))
Path(sys.argv[2]).write_text(json.dumps(runs, indent=1))
