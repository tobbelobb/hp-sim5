"""Per-iteration leave-one-group-out folds for corpus runs (candidate code), J0 and J1.

usage: replay_folds.py runs.json out.jsonl worker_index worker_count
"""
import copy, hashlib, json, re, sys, time
from pathlib import Path
import numpy as np

WT = Path("/home/torbjorn/.t3/worktrees/hp-sim5/t3-244bb580-382b3b0103efa274")
sys.path.insert(0, str(WT))
sys.path.insert(0, str(next(WT.glob("research/output/program-2026-11-20-*/session-2026-11-20-uneven*/scripts"))))
from lowz_common import fit_default, anchor_error
from autocal._autocal_common import _normalize_dataset_point_roles
from autocal.dataset_roles import measurement_groups
from autocal.pose_fit import HOME_OFFSET_PRIOR_SIGMA_MM, fit_pose_least_squares

MAIN = Path("/home/torbjorn/repos/hp-sim5")


def truth_of(run_dir, manifest):
    inp = run_dir / "input.json"
    if inp.exists():
        t = json.loads(inp.read_text()).get("truth")
        if t is not None:
            return np.asarray(t, float), "input.json"
    m669 = (manifest.get("firmware_initial") or {}).get("M669", "")
    rows = re.findall(r"^([A-DI]):([-\d.]+), ([-\d.]+), ([-\d.]+)", m669, re.M)
    if not rows:
        return None, None
    return np.asarray([[float(x) for x in r[1:]] for r in rows]), "firmware_initial"


def iteration_counts(console):
    counts, current, collected = {}, None, []
    for line in console.splitlines():
        m = re.match(r"; === full-auto iteration (\d+)/", line)
        if m:
            current = int(m.group(1)); counts[current] = None; continue
        m = re.match(r"Wrote to console: Collected sweep nr (\d+)", line)
        if m and current is not None and counts[current] is None:
            counts[current] = int(m.group(1)) - 1
    keys = sorted(counts)
    for i, k in enumerate(keys):
        if counts[k] is None:
            counts[k] = counts[keys[i - 1]] + 1 if i else None
    return counts


def folds_of(data, anchors):
    sweeps = data["sweeps"]; out = []
    for left in measurement_groups(data):
        sub = dict(data, sweeps=[s for i, s in enumerate(sweeps) if i not in left])
        fit = fit_pose_least_squares(sub, anchors, fit_home_offsets=True,
                                     home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM)
        if not fit["success"] or not np.all(np.isfinite(fit["anchors"])):
            return None
        out.append(fit["anchors"])
    return np.stack(out)


def main():
    runs, out, wi, wc = json.load(open(sys.argv[1])), Path(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
    for ri, run in enumerate(runs):
        if ri % wc != wi:
            continue
        f = MAIN / run["f"]; run_dir = f.parent; base = f.name.replace(".full_auto_log.jsonl", "")
        manifest = json.loads((run_dir / f"{base}.headless/manifest.json").read_text())
        console = run_dir / f"{base}.full_auto.log"
        ds_path = run_dir / f"{base}.json"
        truth, tsrc = truth_of(run_dir, manifest)
        if truth is None or not console.exists() or not ds_path.exists():
            continue
        counts = iteration_counts(console.read_text(errors="replace"))
        data = json.loads(ds_path.read_text()); _normalize_dataset_point_roles(data)
        groups = measurement_groups(data)
        iters = []
        t0 = time.time()
        for k in sorted(counts):
            c = counts[k]
            if c is None or c < 1 or c > len(groups):
                iters.append(dict(iteration=k, sweeps=c, skip="count")); continue
            keep = sorted(i for g in groups[:c] for i in g)
            sub = dict(copy.deepcopy(data), sweeps=[data["sweeps"][i] for i in keep])
            try:
                fit = fit_default(sub)[0]
                folds = folds_of(sub, fit["anchors"]) if c >= 3 else None
            except Exception as e:
                iters.append(dict(iteration=k, sweeps=c, skip=repr(e)[:200])); continue
            iters.append(dict(iteration=k, sweeps=c, err=anchor_error(fit["anchors"], truth),
                              anchors=np.asarray(fit["anchors"]).tolist(),
                              folds=None if folds is None else folds.tolist()))
        key = hashlib.sha256(json.dumps([data["sweeps"][:max([i['sweeps'] or 0 for i in iters] + [0])],
                                         truth.tolist()], sort_keys=True).encode()).hexdigest()[:16]
        with out.open("a") as fh:
            fh.write(json.dumps(dict(f=run["f"], machine=run["machine"], stop=manifest.get("stop_reason"),
                                     truth_source=tsrc, key=key, wall=time.time() - t0, iters=iters)) + "\n")
        print(ri, run["f"], len(iters), round(time.time() - t0, 1), flush=True)


main()
