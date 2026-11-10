"""Run unmodified HP3 autocal in a fresh scene with a changed physical reference.

Only isolated scene/firmware inputs and HeadlessSimulation's scene selection are
overridden. The fitter, collector, initialization and stopping code are untouched.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sys

ROOT=Path(__file__).resolve().parents[5];sys.path.insert(0,str(ROOT))
SESSION=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--variant',choices=['baseline','center','offset','offset-small'],required=True)
parser.add_argument('--label',required=True);parser.add_argument('--max-steps',type=int,default=1)
parser.add_argument('--recording',choices=['none','events','sampled'],default='sampled')
args=parser.parse_args()
work=SESSION/args.label;work.mkdir()
p={'baseline':[0.,0.,0.],'center':[0.,0.,.265],'offset':[.5,.2,.265],'offset-small':[.25,.1,.265]}[args.variant]
base=ROOT/'public/usd_scenes/hp3_rigid_body.usda';text=base.read_text();changes=[]
pattern=r'(def (?:Xform|Circle) "(Spool[ABCD]|Anchor(?:[ABC][LR]|D[012]))".*?double3 xformOp:translate = )\(([^)]+)\)'
def shift(match):
 old=[float(x.strip()) for x in match[3].split(',')];new=[a-b for a,b in zip(old,p)];changes.append({'prim':match[2],'old':old,'new':new})
 return match[1]+'('+', '.join(format(x,'.16g') for x in new)+')'
scene_text=re.sub(pattern,shift,text,flags=re.S)
assert len(changes)==13,len(changes)
scene=work/'authored-scene.usda';scene.write_text(scene_text)
truth=[[0.,-1900.,-280.],[1645.44826719,950.,-280.],[-1645.44826719,950.,-280.],[0.,0.,1900.]]
truth=[[a-1000*b for a,b in zip(anchor,p)] for anchor in truth]
config=(ROOT/'RRF/run/vsd/sys/config_hp3.g').read_text()
line='M669 '+' '.join(letter+':'.join(format(x,'.12g') for x in anchor) for letter,anchor in zip('ABCD',truth))
config,n=re.subn(r'^M669 A[^\n]+',line,config,flags=re.M);assert n==1
cfg=work/'firmware-config.g';cfg.write_text(config)
# RRF config paths are relative to its virtual SD. Retain the original under work.
cfg_name=f'sys/research_ae0174ce_{args.label}.g';live_cfg=ROOT/'RRF/run/vsd'/cfg_name
assert not live_cfg.exists();live_cfg.write_text(config)
contract={'variant':args.variant,'reference_position_in_original_frame_mm':[1000*x for x in p],
 'physical_truth_anchors_mm':truth,'plant_settings':{'machine':'hp3','layering':False,'base_radii_mm':[30]*4,'buildup':0,'substeps':1},
 'forces_n':[.0223,.4455,6.5739],'recording':args.recording,'max_iterations':args.max_steps,
 'max_physics_steps':int(os.environ['HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS']),
 'base_scene_sha256':hashlib.sha256(base.read_bytes()).hexdigest(),'scene_sha256':hashlib.sha256(scene.read_bytes()).hexdigest(),
 'config_sha256':hashlib.sha256(cfg.read_bytes()).hexdigest(),'static_changes':changes,
 'intervention':'Translate static spool/anchor assembly by -reference position; mover/collector/noise/solver unchanged.',
 'truth_use':'plant construction, matching firmware initialization and evaluation only; not passed as an anchor-fit seed'}
(work/'contract.json').write_text(json.dumps(contract,indent=2)+'\n')
import autocal.headless_sim as headless
Base=headless.HeadlessSimulation
class TrialSimulation(Base):
 def __init__(self,*a,**kw):
  super().__init__(*a,**kw);self.scene=scene
  self.manifest.update(scene=str(scene),research_contract=str(work/'contract.json'),reference_position_mm=contract['reference_position_in_original_frame_mm'])
headless.HeadlessSimulation=TrialSimulation
from autocal.autocal import main
argv=['--headless-sim','--machine-type','hangprinter_3','--speedup','25',
 '--dataset',str(work/'sweeps.json'),'--base-radii','30','--buildup-factor','0','--find-radii','off','--find-buildup-factor','off',
 '--rrf-config',cfg_name,'--recording-mode',args.recording,'--max-steps',str(args.max_steps),
 '--stage-artifacts',str(work/'stages'),'--full-auto-log',str(work/'iterations.jsonl'),
 '--collector-args','--no-auto-tune-force','--force-low','0.0223','--force-mid','0.4455','--force-max','6.5739']
(work/'argv.json').write_text(json.dumps(argv,indent=2)+'\n')
try:
 code=main(argv)
 (work/'exit.json').write_text(json.dumps({'code':code})+'\n')
 sys.exit(code)
finally:
 live_cfg.unlink(missing_ok=True)
