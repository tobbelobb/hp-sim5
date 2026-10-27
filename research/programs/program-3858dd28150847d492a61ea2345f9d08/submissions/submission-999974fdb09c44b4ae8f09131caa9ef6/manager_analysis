"""Manager analysis of saved native artifacts. No optimization or simulation."""
import hashlib
import json
import math
from pathlib import Path

OUT=Path(__file__).resolve().parent
SESSION=OUT.parent/'attempt-982dc44e4c9b4e5da9b3232c26d54bf3'
PINNED=SESSION/'pinned'
def read(p): return json.loads(p.read_text())
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,v): p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
truth=[[0.,-1900.,-280.],[1645.45,950.,-280.],[-1645.45,950.,-280.],[0.,0.,1900.]]
collections=read(PINNED/'collection-results.json')
raw=read(PINNED/'fit-results.json')['results']
clean=read(PINNED/'clean-fit-results.json')['results']
assert len(raw)==len(clean)==4
summary={'author':'manager, from preserved partial worker evidence after usage-limit interruption',
 'scope':'Two native HP3 bootstrap collections, four raw and four geometry-metadata-stripped ordinary fit stages. No new fitting or simulation during this analysis.',
 'conventional_scene_home_truth_mm':truth,
 'truth_caveat':'This is the repository conventional effective scene-home geometry, not an observed encoder-zero pose. Reference pose/rotation were unavailable. No final automatic acceptance or hardware accuracy was measured.',
 'worlds':[],'fits':[],'experimental_fit_calls':8,'recovery_fit_calls':0,
 'driver_validity_caveat':'The saved driver defines valid as finite primary/rank cost and covariance_ok. This is not a complete production acceptance decision. geometry_valid and underconstrained_penalty are separate production helper observations.'}
for c in collections:
 w=c['world'];d=read(PINNED/w/'sweeps.json');f=read(PINNED/w/'fit-input.json')
 assert 'm669' not in f['config']
 dc=json.loads(json.dumps(d));dc['config'].pop('m669',None)
 assert dc==f, 'Stripping must change only config.m669'
 assert sha(PINNED/w/'sweeps.json')==c['dataset_sha256']
 m=read(PINNED/w/'sweeps.headless/manifest.json')
 assert m['status']=='complete' and m['kept_alive'] is False and m['physics_engine']=='rust-native'
 sweeps=[]
 for s in d['sweeps']:
  pts=s['data_points']
  assert len(pts)==20
  assert all(math.isfinite(pt[k]) for pt in pts for k in ('l_drive','l_sensor'))
  sweeps.append({'id':s['id'],'points':len(pts),'drive':s['drive_anchor'],'sensor':s['sensor_anchor'],
   'drive_span_mm':max(p['l_drive'] for p in pts)-min(p['l_drive'] for p in pts),
   'sensor_span_mm':max(p['l_sensor'] for p in pts)-min(p['l_sensor'] for p in pts)})
 assert len(sweeps)==3
 summary['worlds'].append({'world':w,'stiffness_n':c['stiffness_n'],'points':sum(s['points'] for s in sweeps),'sweeps':sweeps,
  'world_wall_s':c['wall_s'],'service_wall_s':m['wall_s'],'physics_steps':m['total_steps'],'simulated_s':m['simulated_s'],
  'raw_sha256':sha(PINNED/w/'sweeps.json'),'stripped_sha256':sha(PINNED/w/'fit-input.json'),
  'native_sha256':m['physics_artifact']['sha256'],'baked_scene_sha256':m['baked_scene_sha256']})
for a,b in zip(raw,clean):
 assert (a['world'],a['solver'])==(b['world'],b['solver'])
 equal=a['anchors_mm']==b['anchors_mm'] and a['metrics']==b['metrics'] and a['optimizer_runs']==b['optimizer_runs']
 assert equal
 err=math.sqrt(sum(sum((x-y)**2 for x,y in zip(p,q)) for p,q in zip(b['anchors_mm'],truth))/4)
 summary['fits'].append({'world':b['world'],'solver':b['solver'],'conventional_anchor_rmse_mm':err,
  'primary_cost':b['metrics']['primary_cost'],'driver_finite_covariance_flag':b['metrics']['valid'],
  'geometry_valid':b['metrics']['geometry_valid'],'underconstrained_penalty':b['metrics']['underconstrained_penalty'],
  'info_rank_deficient':b['metrics']['info_rank_deficient'],'max_std_mm':b['metrics']['max_std_mm'],
  'raw_stripped_numerically_identical':equal,'stripped_fit_wall_s':b['wall_s']})
summary['total_native_physics_steps']=sum(w['physics_steps'] for w in summary['worlds'])
summary['total_world_wall_s']=sum(w['world_wall_s'] for w in summary['worlds'])
write(OUT/'native-saved-summary.json',summary)
paths=['pinned/nominal/sweeps.headless/manifest.json','pinned/soft/sweeps.headless/manifest.json',
 'pinned/nominal/sweeps.headless/clock.json','pinned/soft/sweeps.headless/clock.json',
 'pinned/nominal/command.json','pinned/soft/command.json','pinned/nominal/input-check.json','pinned/soft/input-check.json',
 'nominal/sweeps.headless/manifest.json','soft/sweeps.headless/manifest.json','collection-results.json','exit.json']
write(OUT/'native-provenance-bundle.json',{'kind':'manager copy of exact original JSON bytes, represented as text with original SHA256',
 'files':[{'source':str(SESSION/p),'relative':p,'sha256':sha(SESSION/p),'text':(SESSION/p).read_text()} for p in paths]})
print(json.dumps({'worlds':summary['worlds'],'fits':summary['fits'],'total_steps':summary['total_native_physics_steps']},indent=2))
