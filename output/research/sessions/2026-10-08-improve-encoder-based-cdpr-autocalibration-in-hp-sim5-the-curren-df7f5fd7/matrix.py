from pathlib import Path
import sys, time, json, concurrent.futures, hashlib, subprocess
sys.path.insert(0, str(Path.cwd()))
from autocal.tools import regress_calibration_logs as r
s=Path(__file__).resolve().parent
phase=sys.argv[1]
out=s/phase
out.mkdir(exist_ok=True)
root=Path.cwd()
data=root/'autocal/data/references'
refdir=Path(sys.argv[2]).resolve() if len(sys.argv)>2 else data
start=time.perf_counter()
def run(spec):
    ref=(refdir/(spec.name+'.log')) if (refdir/(spec.name+'.log')).exists() else r.resolve_reference_log_path(dataset_name=spec.name, ref_dir=refdir, alt_dir=Path('/mnt/data'), reference_log_names=())
    t=time.perf_counter()
    result=r.run_one_dataset(spec,root,data/(spec.name+'.json'),ref,0.01,False,False,out)
    elapsed=time.perf_counter()-t
    (out/(spec.name+'.txt')).write_text('\n'.join(result.lines))
    if result.generated_log:
        parsed=r.parse_log_file(result.generated_log)
        (out/(spec.name+'.log')).write_text(result.generated_log.read_text())
        err=r.error_to_true(parsed.summary.anchors,parsed.summary.radii,spec.true_anchors,spec.true_radii)
        row=dict(name=spec.name,ok=result.ok,seconds=elapsed,log=str(result.generated_log),reference=str(ref),error=err,iterations=len(parsed.iterations),audit=r.history_ranking_audit(parsed.iterations,spec),final_delta=result.true_err_total_delta,trajectory_delta=result.true_iter_mean_delta)
    else:
        row=dict(name=spec.name,ok=False,seconds=elapsed,error=None)
    print(json.dumps(row),flush=True)
    return row
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    rows=list(pool.map(run,r.DATASETS))
(out/'results.json').write_text(json.dumps(dict(seconds=time.perf_counter()-start,rows=rows,revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()),indent=2))
