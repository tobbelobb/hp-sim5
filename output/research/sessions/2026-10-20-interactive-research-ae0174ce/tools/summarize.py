"""Build compact cross-experiment evidence and source provenance."""
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import shutil
import subprocess

S=Path(__file__).resolve().parents[1];ROOT=S.parents[3]
def read(name):return json.loads((S/name/'results.json').read_text())
a=read('E01');b=read('E05');c=read('E06');d=read('E07')
for row in a:
    row['meaningful_adjacent_score_geometry_disagreements']=sum(
        (x['cost']-y['cost'])*(x['rmse_mm']-y['rmse_mm'])<0 and
        abs(x['cost']-y['cost'])>1e-6 and abs(x['rmse_mm']-y['rmse_mm'])>.01
        for x,y in zip(row['selected_history'],row['selected_history'][1:]))
for row in c:
    text=(S/'E06'/f"{row['name']}-{row['mode']}"/'stdout.log').read_text()
    row['terminal_reason']='spread-consensus' if 'stopping early; the two closest' in text else 'replay-exhausted-no-collect'
means=[]
for prefix in [3,5,0]:
    for solver in ['lbfgsb','trf']:
        values={mode:[r['rmse_mm'] for r in b if r['solver']==solver and r['mode']==mode and
                     (r['prefix']==prefix if prefix else r['prefix']>5)] for mode in ['original','center','edge']}
        means.append({'prefix':prefix or 'all','solver':solver,**{k:sum(v)/len(v) for k,v in values.items()}})
comparisons=[]
for r in b:
    if r['mode']=='original':continue
    original=next(x for x in b if x['name']==r['name'] and x['prefix']==r['prefix'] and x['solver']==r['solver'] and x['mode']=='original')
    comparisons.append({'name':r['name'],'prefix':r['prefix'],'solver':r['solver'],'mode':r['mode'],
                        'rmse_delta_mm':r['rmse_mm']-original['rmse_mm']})
summary={'baseline':a,'feasible_shift_means':means,'feasible_shift_comparisons':comparisons,
         'worse_comparisons':sum(r['rmse_delta_mm']>0 for r in comparisons),
         'cli_replays':c,'polished_seeds':d}
(S/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
source_dir=S/'source';source_dir.mkdir(exist_ok=True)
paths=['autocal/tools/rebase_dataset.py','autocal/tests/test_rebase_dataset.py','autocal/README.md']
for path in paths:
    dest=source_dir/path;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/path,dest)
source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
               list((ROOT/'autocal').glob('*.py'))+[ROOT/p for p in paths]}
for row in a:
    assert hashlib.sha256(Path(row['source']).read_bytes()).hexdigest()==row['sha256']
manifest={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
          'python':platform.python_version(),'packages':{p:importlib.metadata.version(p) for p in ['numpy','scipy','jax']},
          'source_sha256':source_hashes,'inputs_unchanged':True,
          'new_collection':False,'recording':'none; offline numerical fits','ground_truth_use':'metrics only',
          'constraints':'HP3 source worlds; Q=0, layering off, radius=30 mm fixed',
          'timing_limitations':'E07 overlapped part of E06; no solver or full-autocal speedup claimed'}
(S/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('worse feasible shifts:',summary['worse_comparisons'],'/',len(comparisons))
print('meaningful score disagreements:',sum(r['meaningful_adjacent_score_geometry_disagreements'] for r in a))
