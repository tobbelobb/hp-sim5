from argparse import Namespace
from pathlib import Path
import subprocess,sys,os
sys.path.insert(0,str(Path(__file__).resolve().parents[4]))
from autocal.headless_sim import HeadlessSimulation
s=Path(__file__).resolve().parent
name,machine,rpc=sys.argv[1:]
out=s/name;out.mkdir()
repo=s.parents[3]
for f in ['controlled_pullout.mjs','pullout_profiles.mjs']:(out/f).write_text((repo/'autocal/control/behaviors'/f).read_text())
(out/'changes.diff').write_text(subprocess.check_output(['git','diff'],text=True))
args=Namespace(machine_type='hangprinter_'+('3'if machine=='hp3'else'4'),find_radii='off',find_buildup_factor='off',rrf_config=None,config=None,dataset=out/'world.json',keep_sim_alive=False,recording_mode='sampled')
os.environ['HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS']='500000'
with HeadlessSimulation(args,[])as sim:
 with(out/'trial.log').open('w')as log:
  result=subprocess.run(['node',str(s/'smoke_trial.mjs'),sim.url,str(out/'result.json'),machine,rpc],stdout=log,stderr=log,timeout=300)
 sim.manifest['exit_code']=result.returncode
print((out/'result.json').read_text()[:500])
