"""Post-hoc scoring only; never imported by collector, solver, or application."""
from pathlib import Path
import json,math,re,sys
p=Path(sys.argv[1]);m=json.loads((p/'sweeps.headless/manifest.json').read_text());truth=[[0,-2000,-280],[1725.448267190434,950,-280],[-1715.448267190434,950,-280],[0,0,1980]]
commands=[s for s in m.get('applied_parameters',[]) if s.startswith('M669')];out={'run':str(p),'status':m.get('status'),'stop_reason':m.get('stop_reason'),'normal_completion':m.get('normal_completion'),'total_steps':m.get('total_steps'),'simulated_s':m.get('simulated_s'),'wall_s':m.get('wall_s'),'baked_scene_sha256':m.get('baked_scene_sha256'),'truth_from_authored_translations_mm':truth,'applied_command':commands[-1] if commands else None,'firmware_final':m.get('firmware_final')}
if commands:
 coords=[]
 for axis in 'ABCD':
  mm=re.search(r'\b'+axis+r'([-+\d.eE]+):([-+\d.eE]+):([-+\d.eE]+)',commands[-1]);assert mm;coords.append(list(map(float,mm.groups())))
 errs=[math.dist(a,b) for a,b in zip(coords,truth)];out.update(anchors=coords,errors_mm=dict(zip('ABCD',errs)),max_mm=max(errs),rmse_mm=math.sqrt(sum(x*x for x in errs)/4))
print(json.dumps(out,indent=2))
