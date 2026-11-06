import os,sys,json,hashlib,time,shutil,importlib,fcntl
from pathlib import Path
ROOT=Path.cwd();sys.path.insert(0,str(ROOT))
from autocal.headless_sim import HeadlessSimulation
from autocal.autocal import main
S=Path(__file__).resolve().parents[1];H=S/'harness'
assert os.popen('node --version').read().strip()=='v26.13.0', 'Source cloud client env.sh'
world,runid=sys.argv[1:3];extra=sys.argv[3:];policy=False; extended="--extended" in extra; point_filter="--point-filter" in extra; robust_mm="--robust-mm" in extra; halfturn="--half-turn" in extra; fourmid="--four-mid" in extra; projection_budget="--projection-budget" in extra; settle120="--settle120" in extra; extra=[x for x in extra if x not in ("--extended","--point-filter","--half-turn","--four-mid","--robust-mm","--projection-budget","--settle120")]
spec=json.loads((H/(world+'.json')).read_text());run=S/'runs'/runid;run.mkdir(parents=True,exist_ok=False)
shutil.copyfile(H/'autocal_headless.mjs',run/'service-snapshot.mjs')
shutil.copyfile(Path(__file__),run/'runner-snapshot.py')
shutil.copytree(H/('settle120' if settle120 else ('halfturn4mid' if halfturn and fourmid else ('halfturn' if halfturn else 'default'))),run/'collector-policy')
os.environ['FRICTION_WORLD_SPEC']=str(H/(world+'.json'))
os.environ['HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS']='30000000' if projection_budget else '8000000'
os.environ['OMP_NUM_THREADS']='1';os.environ['OPENBLAS_NUM_THREADS']='1'
orig_init=HeadlessSimulation.__init__;orig_spawn=HeadlessSimulation.spawn
# Session-local adaptation. Production scene/configuration files remain untouched.
def init(self,args,collector_args):
    orig_init(self,args,collector_args);self.scene=Path(spec['scene']);self.manifest['scene']=str(self.scene);self.manifest['intervention']=spec

def spawn(self,name,command):
    if name=='physics':
        assert command[1]=='scripts/autocal_headless.mjs';command=[command[0],str(run/'service-snapshot.mjs'),*command[2:]]
    return orig_spawn(self,name,command)
HeadlessSimulation.__init__=init;HeadlessSimulation.spawn=spawn
if point_filter or robust_mm:
    candidate_file=H/('robust_mm_candidate.py' if robust_mm else 'candidate_filter.py')
    shutil.copyfile(candidate_file,run/'candidate-filter-snapshot.py')
    import importlib.util
    modspec=importlib.util.spec_from_file_location('candidate_filter_snapshot', run/'candidate-filter-snapshot.py')
    candidate_module=importlib.util.module_from_spec(modspec);modspec.loader.exec_module(candidate_module)
    filter_command=candidate_module.filter_command
    original_apply=HeadlessSimulation.apply_parameters
    def filtered_apply(self,commands):
        cmds=list(commands); original=cmds[0]
        try:
            cmds[0],diagnostic=filter_command(json.loads((run/'sweeps.json').read_text()),original)
        except Exception as exc:
            diagnostic={'action':'fallback','reason':f'{type(exc).__name__}: {exc}'}
            cmds[0]=original
        (run/'point-filter.json').write_text(json.dumps(diagnostic,indent=2)+'\n')
        print('; experimental encoder filter:',diagnostic.get('action'),'; stock quality score does not score replacement',flush=True)
        return original_apply(self,cmds)
    HeadlessSimulation.apply_parameters=filtered_apply

if True:
    import subprocess
    original_popen=subprocess.Popen
    def popen(command,*a,**kw):
        if isinstance(command,(list,tuple)) and len(command)>1 and command[0]=='node' and command[1]=='autocal/control/cli/collect_sweep_data.mjs':
            command=[command[0],str(run/'collector-policy/collect_sweep_data.mjs'),*command[2:]]
        return original_popen(command,*a,**kw)
    subprocess.Popen=popen

argv=['--headless-sim','--recording-mode','events','--machine-type','hangprinter_3','--dataset',str(run/'sweeps.json'),'--rrf-config','sys/config_hp3.g','--base-radii','30','--buildup-factor','0','--find-radii','off','--find-buildup-factor','off','--optimizer-mode','fast','--solve-optimizer','lbfgsb','--solve-iterations','400','--solve-restarts','4','--spool-outer-iters','3','--max-steps','12','--sweep-points','10','--speedup','25','--stage-artifacts',str(run/'stages'),*extra]
(run/'invocation.json').write_text(json.dumps(dict(world=spec,policy='settle120-v1' if settle120 else ('robust-mm-v1' if robust_mm else ('point-filter-v1' if point_filter else ('halfturn-4mid-v1' if halfturn and fourmid else ('halfturn-v1' if halfturn else ('extended-min-iteration-9' if extended else 'stock'))))),policy_files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (run/'collector-policy').glob('*.mjs')},argv=argv,extra=extra,script_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in ([Path(__file__),H/'autocal_headless.mjs'] + ([H/('robust_mm_candidate.py' if robust_mm else 'candidate_filter.py')] if point_filter or robust_mm else []))},started_unix=time.time()),indent=2)+'\n')
module=importlib.import_module('autocal.autocal')
if extended:
    from dataclasses import replace
    BasePolicy=module.StoppingPolicy
    class ExtendedPolicy(BasePolicy):
        def update(self, history, *, iteration, admissible, underconstrained, log_line):
            decision=super().update(history, iteration=iteration, admissible=admissible, underconstrained=underconstrained, log_line=log_line)
            if iteration < 9 and decision.decision == 'accept':
                log_line('; experiment: postpone acceptance until iteration 9')
                return replace(decision,decision='collect')
            return decision
    module.StoppingPolicy=ExtendedPolicy

original_plan=module.plan_next_ellipse_sweep
ledger=S/'fit-ledger.json'
def counted_plan(*a,**kw):
    with (S/'fit-ledger.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        state=json.loads(ledger.read_text()) if ledger.exists() else dict(reserved_uninstrumented_fits=0,calls=[])
        if len(state['calls'])+state['reserved_uninstrumented_fits'] >= 80: raise RuntimeError('Session 80-fit budget exhausted')
        state['calls'].append(dict(run=runid,utc=time.time(),index=len(state['calls'])+1))
        tmp=ledger.with_suffix('.tmp');tmp.write_text(json.dumps(state,indent=2)+'\n');tmp.replace(ledger)
    return original_plan(*a,**kw)
module.plan_next_ellipse_sweep=counted_plan
try:code=main(argv)
except BaseException as e:
    (run/'failure.json').write_text(json.dumps(dict(type=type(e).__name__,message=str(e))));raise
(run/'exit.json').write_text(json.dumps(dict(exit_code=code,finished_unix=time.time()))+'\n');sys.exit(code)
