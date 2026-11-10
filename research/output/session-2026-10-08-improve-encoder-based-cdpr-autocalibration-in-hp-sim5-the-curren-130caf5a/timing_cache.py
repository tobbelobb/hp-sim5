import json,os,subprocess,sys
from pathlib import Path
import numpy as np
OUT=Path(__file__).resolve().parent
names=('layered_manual_some_bad_points','seventh_hp3_dataset');rows=[]
for repeat in range(3):
 variants=('baseline','cache') if repeat%2==0 else ('cache','baseline')
 for variant in variants:
  label=f'E06-{variant}-{repeat+1}'
  with (OUT/f'{label}.txt').open('w') as stream:
   proc=subprocess.run([sys.executable,str(OUT/'replay.py'),label,*names],stdout=stream,stderr=subprocess.STDOUT,check=True)
  runs=json.loads((OUT/f'{label}-runs.json').read_text())
  for row in runs:
   row.update(repeat=repeat+1,model=variant,counters=json.loads((OUT/label/row['dataset']/'counters.json').read_text()));rows.append(row)
  (OUT/'E06-isolated-runs.json').write_text(json.dumps(rows,indent=2))
  print(label,[(r['dataset'],round(r['seconds'],3),r['returncode']) for r in runs],flush=True)
summary=[]
for name in names:
 a=[r['seconds'] for r in rows if r['dataset']==name and r['model']=='baseline'];b=[r['seconds'] for r in rows if r['dataset']==name and r['model']=='cache']
 summary.append(dict(dataset=name,baseline_seconds=a,cache_seconds=b,baseline_median=float(np.median(a)),cache_median=float(np.median(b)),median_saving_percent=float(100*(1-np.median(b)/np.median(a)))))
(OUT/'E06-isolated-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
