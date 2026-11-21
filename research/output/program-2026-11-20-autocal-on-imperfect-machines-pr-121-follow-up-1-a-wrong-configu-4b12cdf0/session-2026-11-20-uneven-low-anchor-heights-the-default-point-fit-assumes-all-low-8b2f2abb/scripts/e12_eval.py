"""E12 evaluation: classical jackknife (J0) vs the structure-aware spread (J2) at fold-ratio limit R.

(a) the 9 HP5/HP5r equal-height live runs (plus every other live run, for no-regression);
(b) the reviewer's single-sweep injections (10 and 20 mm): J2 must call "poor" every case J0 does;
(c) the reviewer's corpus replay: patience stop and newest_trusted_index choice, J0 vs J2.
"""
import json, os, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from structural import classical, structural_spread
from autocal.anchor_uncertainty import newest_trusted_index, uncertainty_label
from autocal.stopping import history_rank_position

R = Path(__file__).resolve().parents[1] / "results"
MAIN = Path("/home/torbjorn/repos/hp-sim5")
live = json.loads((R / "e12_live.json").read_text())
inject = json.loads((R / "e12_inject10.json").read_text()) + json.loads((R / "e12_inject20.json").read_text())
corpus = json.loads((R / "e12_corpus_0.json").read_text()) + json.loads((R / "e12_corpus_1.json").read_text())


def j0(folds):
    return float(classical(folds).max())


MAX_LOST = int(os.environ.get("E12_MAX_LOST", "1"))  # folds that may be excluded per coordinate


def j2(folds, ratios, limit):
    return structural_spread(np.asarray(folds), np.asarray(ratios), limit, max_lost=MAX_LOST)


def patience_of(run):
    for line in open(MAIN / run["f"]):
        e = json.loads(line)
        if e.get("patience") is not None:
            return int(e["patience"])
    return 8


def replay_stop(iters, key, patience):
    hist, streak = [], 0
    for it in iters:
        hist.append(dict(iteration=it["iteration"], anchor_uncertainty_mm=it[key]))
        streak = 0 if history_rank_position(hist, iteration=it["iteration"]) == 1 else streak + 1
        if streak >= patience:
            return it["iteration"]
    return None


def evaluate(limit):
    res = {}
    focus = [r for r in live if r["machine"].startswith("hangprinter_5") and "_A_" not in r["case"]]
    res["a_runs"] = [(r["case"], round(r["err"], 2), round(j0(r["folds"]), 2), round(j2(r["folds"], r["ratios"], limit)[0], 2),
                      len(j2(r["folds"], r["ratios"], limit)[1])) for r in focus]
    res["a_poor_J0"] = sum(uncertainty_label(x[2]) == "poor" for x in res["a_runs"])
    res["a_poor_J2"] = sum(uncertainty_label(x[3]) == "poor" for x in res["a_runs"])
    others = [r for r in live if r not in focus]
    res["live_other_labels_changed"] = [(r["case"], uncertainty_label(j0(r["folds"])), uncertainty_label(j2(r["folds"], r["ratios"], limit)[0]))
                                        for r in others if uncertainty_label(j0(r["folds"])) != uncertainty_label(j2(r["folds"], r["ratios"], limit)[0])]
    rows = []
    for r in inject:
        if r["folds"] is None:
            continue
        a, (b, lim) = j0(r["folds"]), j2(r["folds"], r["ratios"], limit)
        rows.append(dict(case=r["case"], d=r["d_mm"], err=r["err"], J0=a, J2=b, limited=len(lim)))
    res["b_n"] = len(rows)
    res["b_lost_poor"] = [(x["case"], x["d"], round(x["err"], 2), round(x["J0"], 2), round(x["J2"], 2)) for x in rows
                          if uncertainty_label(x["J0"]) == "poor" and uncertainty_label(x["J2"]) != "poor"]
    res["b_label_worse_any"] = [(x["case"], x["d"], round(x["err"], 2), round(x["J0"], 2), round(x["J2"], 2)) for x in rows
                                if ["good", "usable", "poor"].index(uncertainty_label(x["J2"])) < ["good", "usable", "poor"].index(uncertainty_label(x["J0"])) and x["err"] > 1.5]
    big = [x for x in rows if x["err"] > 5]
    res["b_err_gt5_not_poor"] = (sum(uncertainty_label(x["J0"]) != "poor" for x in big), sum(uncertainty_label(x["J2"]) != "poor" for x in big), len(big))
    res["b_false_poor_le1_5"] = (sum(x["err"] <= 1.5 and uncertainty_label(x["J0"]) == "poor" for x in rows),
                                 sum(x["err"] <= 1.5 and uncertainty_label(x["J2"]) == "poor" for x in rows))
    changes, seen = [], set()
    for run in corpus:
        if run["key"] in seen:
            continue
        seen.add(run["key"])
        iters = []
        for it in run["iters"]:
            if "err" not in it:
                continue
            if it["sweeps"] < 3:
                it["J0"] = it["J2"] = None
            elif it.get("folds") is None:
                it["J0"] = it["J2"] = float("inf")
            else:
                it["J0"] = j0(it["folds"])
                it["J2"] = j2(it["folds"], it["ratios"], limit)[0] if "ratios" in it else it["J0"]
            iters.append(it)
        if not iters:
            continue
        P = patience_of(run); s0, s2 = replay_stop(iters, "J0", P), replay_stop(iters, "J2", P)
        def applied(key, upto):
            sub = [it for it in iters if it["iteration"] <= upto]
            i = newest_trusted_index([it[key] for it in sub])
            return None if i is None else (sub[i]["iteration"], sub[i]["err"])
        e0, e2 = applied("J0", s0 or iters[-1]["iteration"]), applied("J2", s2 or iters[-1]["iteration"])
        if s0 != s2 or (e0 and e2 and e0[0] != e2[0]):
            changes.append(dict(f=run["f"], machine=run["machine"], P=P, stop=(s0, s2), applied=(e0, e2),
                                worse=bool(e0 and e2 and e2[1] > e0[1] + 1e-9)))
    res["c_distinct"] = len(seen); res["c_changes"] = changes; res["c_worse"] = sum(c["worse"] for c in changes)
    return res


limits = [float(v) for v in (sys.argv[2].split(",") if len(sys.argv) > 2 else ["2.5", "3", "4", "5", "6"])]
out = {str(l): evaluate(l) for l in limits}
for l, r in out.items():
    print(f"R={l}: (a) poor J0 {r['a_poor_J0']}/9 -> J2 {r['a_poor_J2']}/9 | other live label changes {r['live_other_labels_changed']}")
    print(f"       (b) n={r['b_n']} lost 'poor' {len(r['b_lost_poor'])} {r['b_lost_poor']}; err>5 not poor J0/J2/n {r['b_err_gt5_not_poor']}; false poor<=1.5 J0/J2 {r['b_false_poor_le1_5']}")
    print(f"       (c) distinct {r['c_distinct']}, decision changes {len(r['c_changes'])}, worse {r['c_worse']}")
Path(sys.argv[1]).write_text(json.dumps(out, indent=1))
