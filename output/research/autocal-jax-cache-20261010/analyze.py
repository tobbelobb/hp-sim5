from pathlib import Path
import json
from statistics import mean
HERE=Path(__file__).resolve().parent
summaries=[]
outputs={}
for name in ['C1','C2','C3','C4','C5','C6']:
    file=HERE/name/'summary.json'
    if not file.exists(): continue
    summaries.append(json.loads(file.read_text()))
    rows=json.loads((file.parent/'datasets.json').read_text())
    outputs[name]={row['dataset']:row.get('numerical') for row in rows}
baseline=outputs['C1']
comparisons={name:dict(compared=len(rows),identical=[dataset for dataset in rows if rows[dataset]==baseline.get(dataset)],
    different=[dataset for dataset in rows if rows[dataset]!=baseline.get(dataset)]) for name,rows in outputs.items()}
off=[s['elapsed_s'] for s in summaries if s['mode']=='off']
on=[s['elapsed_s'] for s in summaries if s['mode']=='warm-on' and s['trial']!='C6']
result=dict(batteries=summaries,numerical_comparisons=comparisons,off_mean_s=mean(off) if off else None,
    warm_on_mean_s=mean(on) if on else None)
if off and on:
    result.update(speedup=mean(off)/mean(on),saved_s=mean(off)-mean(on),saved_percent=100*(1-mean(on)/mean(off)))
(HERE/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
