"""E14: is a stronger solver a free accuracy win?  Unique dataset path per run."""
import json, os, re, shutil, subprocess, sys, copy, tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
ROOT=Path("/home/torbjorn/repos/hp-sim5-worktrees/improve-encoder-based-cdpr-autocalibrati-af9af5fc")
sys.path.insert(0,str(ROOT))
from autocal.tools.regress_calibration_logs import SIM_3D_TRUE_ANCHORS, error_to_true
S=ROOT/("output/research/sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092")
SRC=S/"artifacts/reference-003/sweeps.json"
TRUE=[tuple(map(float,a)) for a in SIM_3D_TRUE_ANCHORS]
base=json.load(open(SRC))

def perturbed(mag_mm, seed):
    d=copy.deepcopy(base); rng=np.random.default_rng(4321+seed)
    for sw in d["sweeps"]:
        for pt in sw["data_points"]:
            for raw,mu in (("l_drive","l_drive_mu"),("l_sensor","l_sensor_mu")):
                b=pt.get(raw)
                if not isinstance(b,(int,float)): continue
                delta=rng.uniform(-mag_mm,mag_mm)
                pt[raw]=b+delta
                if isinstance(pt.get(mu),(int,float)): pt[mu]=pt[mu]+delta
    return d

SETTINGS=[("dflt",[]),
          ("trf",["--solve-optimizer","trf"]),
          ("trf2000",["--solve-optimizer","trf","--solve-iterations","2000"]),
          ("lm",["--solve-optimizer","lm"])]

def run(arg):
    tag,payload,extra=arg
    scratch=Path(tempfile.mkdtemp(prefix=f"e14-{tag.replace('/','_')}-",dir="/tmp"))
    ds=scratch/"d.json"; ds.write_text(json.dumps(payload))
    r=subprocess.run([sys.executable,"-m","autocal.autocal","--sim","--machine-type","hangprinter_3",
      "--dataset",str(ds),"--find-radii","off","--find-buildup-factor","off","--no-collect",
      "--full-auto-log",str(scratch/"log.jsonl"),*extra],capture_output=True,text=True,cwd=str(ROOT),
      env={**os.environ,"MPLCONFIGDIR":"/tmp/mpl","PYTHONHASHSEED":"0","OMP_NUM_THREADS":"1"})
    shutil.rmtree(scratch,ignore_errors=True)
    m=re.search(r"Anchors \(M669\): M669 (.+)",r.stdout); q=re.search(r"Fit quality score \(lower is better\): ([\d.eE+-]+)",r.stdout)
    if not m: return (tag,None,None)
    anch=[[float(x) for x in tok[1:].split(":")] for tok in m.group(1).split()]
    te=float(error_to_true([tuple(a) for a in anch],[1.0]*4,TRUE,[1.0]*4)[1])
    return (tag,te,float(q.group(1)) if q else None)

DS=[("base",base)]
for s_ in range(4):
    DS.append((f"u020_s{s_}",perturbed(0.020,s_)))
DS.append(("u050_s0",perturbed(0.050,0)))
cases=[(f"{t}/{d}",p,e) for t,e in SETTINGS for d,p in DS]
with ThreadPoolExecutor(max_workers=4) as ex:
    for tag,te,q in ex.map(run,cases):
        print(f"{tag:20s} true_err={te if te is None else round(te,4)} fit_q={q}",flush=True)
