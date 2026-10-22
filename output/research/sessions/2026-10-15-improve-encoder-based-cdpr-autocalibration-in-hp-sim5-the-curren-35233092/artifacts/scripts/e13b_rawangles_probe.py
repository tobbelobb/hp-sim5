import json, subprocess, sys, re, copy
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
ROOT=Path("/home/torbjorn/repos/hp-sim5-worktrees/improve-encoder-based-cdpr-autocalibrati-af9af5fc")
sys.path.insert(0,str(ROOT))
from autocal.tools.regress_calibration_logs import SIM_3D_TRUE_ANCHORS, error_to_true
S=ROOT/("output/research/sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092")
SRC=S/"artifacts/reference-003/sweeps.json"
TRUE=[tuple(map(float,a)) for a in SIM_3D_TRUE_ANCHORS]
MMD=0.5235987755982988
base=json.load(open(SRC))

def variant(mode, mag_deg):
    d=copy.deepcopy(base)
    rng=np.random.default_rng(7)
    for sw in d["sweeps"]:
        for pt in sw["data_points"]:
            a=pt.get("raw_angles_deg")
            if mode in ("raw","both") and a:
                for i in range(len(a)):
                    if a[i] is None or a[i]==0.0: continue
                    a[i]+=rng.uniform(-mag_deg,mag_deg)
                if pt.get("mu"): pt["mu"]=list(a)
            if mode in ("len","both"):
                for f in ("l_drive","l_sensor","l_drive_mu","l_sensor_mu"):
                    if isinstance(pt.get(f),(int,float)):
                        pt[f]=pt[f]+rng.uniform(-1,1)*mag_deg*MMD
    return d

def run(arg):
    tag,mode,mag=arg
    p=Path(f"/tmp/wf_{tag}.json"); p.write_text(json.dumps(variant(mode,mag)))
    r=subprocess.run([sys.executable,"-m","autocal.autocal","--sim","--machine-type","hangprinter_3",
      "--dataset",str(p),"--find-radii","off","--find-buildup-factor","off","--no-collect"],
      capture_output=True,text=True,cwd=str(ROOT),env={**__import__("os").environ,"MPLCONFIGDIR":"/tmp/mpl","PYTHONHASHSEED":"0","OMP_NUM_THREADS":"1"})
    m=re.search(r"Anchors \(M669\): M669 (.+)",r.stdout); q=re.search(r"Fit quality score \(lower is better\): ([\d.eE+-]+)",r.stdout)
    anch=[[float(x) for x in tok[1:].split(":")] for tok in m.group(1).split()] if m else None
    te=float(error_to_true([tuple(a) for a in anch],[1.0]*4,TRUE,[1.0]*4)[1]) if anch else None
    return tag, te, (float(q.group(1)) if q else None), anch

cases=[("base","none",0.0),("raw_pp","raw",0.1795),("raw_big","raw",5.0),("len_pp","len",0.1795),("both_pp","both",0.1795)]
with ThreadPoolExecutor(max_workers=5) as ex:
    for tag,te,q,anch in ex.map(run,cases):
        print(f"{tag:9s} true_err={te if te is None else round(te,4)} fit_q={q}")
        print(f"          {[[round(v,3) for v in a] for a in anch] if anch else None}")
