"""E09: medians and separate process, iteration and geometric failure counts."""
import json
import math
from pathlib import Path
import statistics
import sys

ROOT=Path(__file__).resolve().parents[5];sys.path.insert(0,str(ROOT))
import numpy as np
from autocal.stage_artifacts import read_stage_artifact
from autocal.tools.rebase_dataset import restore_anchors
from autocal.tools.regress_calibration_logs import parse_summary
S=Path(__file__).resolve().parents[1]
TRUE=np.array([[0.,-1900.,-280.],[1645.45,950.,-280.],[-1645.45,950.,-280.],[0.,0.,1900.]])
def rmse(a):return float(np.sqrt(np.mean(np.sum((np.asarray(a)-TRUE)**2,axis=1))))
def count(records):
 return {'n':len(records),'invalid':sum(not r['valid'] for r in records),
         'unrestorable':sum(r.get('rmse_mm') is None for r in records),
         'underconstrained':sum(r['underconstrained'] for r in records),
         'over_100mm':sum(r.get('rmse_mm') is not None and r['rmse_mm']>100 for r in records),
         'median_cost':statistics.median(r['cost'] for r in records) if records else None,
         'median_rmse_mm':statistics.median(r['rmse_mm'] for r in records if r['rmse_mm'] is not None) if any(r['rmse_mm'] is not None for r in records) else None,
         'worst_rmse_mm':max((r['rmse_mm'] for r in records if r['rmse_mm'] is not None),default=None)}

out={'full_replays':{},'matched_fits':[]}
for mode in ['original','center','edge']:
 records=[];runs=[]
 for name in ['REF','W00','W08']:
  work=S/'E01'/name if mode=='original' else S/'E06'/f'{name}-{mode}'
  events=[json.loads(line) for line in (work/('run.log' if mode=='original' else 'run.jsonl')).read_text().splitlines()]
  ds=json.loads((work/'sweeps.json').read_text());q=ds.get('config',{}).get('notes',{}).get('reference_shift',{}).get('offsets_mm')
  for e in events:
   if 'iteration' not in e:continue
   for r in e['runs']:
    metrics=r['metrics'];p=work/'stages'/f"fit-{e['iteration']:03}-{r['id']}.json"
    fit=read_stage_artifact(p)['payload']['fit']
    assert abs(float(fit['cost'])-float(metrics['primary_cost']))<1e-6,(p,fit['cost'],metrics['primary_cost'])
    anchors=fit['anchors']
    try:
     a=restore_anchors(anchors,q)['anchors'] if q is not None else anchors
     error=rmse(a)
    except ValueError:
     error=None
    records.append({'dataset':name,'iteration':e['iteration'],'variant':r['id'],
                    'selected':r['id']==e['selected_run'],'valid':metrics['valid'],
                    'underconstrained':bool(metrics['underconstrained_penalty']),
                    'cost':metrics['primary_cost'],'rmse_mm':error})
  summary=parse_summary((work/'stdout.log').read_text(),fixed_radii=[30.]*4)
  a=restore_anchors(summary.anchors,q)['anchors'] if q is not None else summary.anchors
  text=(work/'stdout.log').read_text()
  runs.append({'dataset':name,'rmse_mm':rmse(a),'displayed_score':summary.fit_score_ui,
               'stopped_automatically':'stopping early; the two closest' in text,
               'replay_exhausted':'--no-collect set; stopping before live collection.' in text})
 out['full_replays'][mode]={'runs':runs,'raw_records':records,
   'cold':count([r for r in records if r['variant']=='default']),
   'warm':count([r for r in records if r['variant']=='default.warm']),
   'selected':count([r for r in records if r['selected']]),
   'all_variants':count(records),
   'final_median_rmse_mm':statistics.median(r['rmse_mm'] for r in runs),
   'final_median_displayed_score':statistics.median(r['displayed_score'] for r in runs)}
for solver in ['lbfgsb','trf']:
 data=json.loads((S/'E05/results.json').read_text())
 for budget in [3,5,'all']:
  for mode in ['original','center','edge']:
   rows=[r for r in data if r['solver']==solver and r['mode']==mode and (r['prefix']==budget if budget!='all' else r['prefix']>5)]
   out['matched_fits'].append({'solver':solver,'budget':budget,'mode':mode,'n':len(rows),
    'median_rmse_mm':statistics.median(r['rmse_mm'] for r in rows),
    'median_cost':statistics.median(r['cost'] for r in rows),
    'over_100mm':sum(r['rmse_mm']>100 for r in rows),
    'error_count':sum('error' in r for r in rows)})
(S/'E09-reliability.json').write_text(json.dumps(out,indent=2)+'\n')
for k,v in out['full_replays'].items():
 print(k,{x:v[x] for x in ['cold','warm','selected','final_median_rmse_mm','final_median_displayed_score']})
for r in out['matched_fits']:print(r)
