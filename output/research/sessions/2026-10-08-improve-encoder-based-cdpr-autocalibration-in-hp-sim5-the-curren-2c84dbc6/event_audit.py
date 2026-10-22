import json,math
from pathlib import Path
OUT=Path(__file__).resolve().parent

def numbers(value,prefix=()):
 out={}
 if isinstance(value,dict):
  for key,child in value.items():out.update(numbers(child,prefix+(key,)))
 elif isinstance(value,list):
  for key,child in enumerate(value):out.update(numbers(child,prefix+(key,)))
 elif isinstance(value,(int,float)):
  out[prefix]='NaN' if isinstance(value,float) and math.isnan(value) else value
 return out

results=[]
for path in (OUT/'cache_actual_final').glob('*/events.jsonl'):
 baseline=[json.loads(line) for line in (OUT/'baseline'/path.parent.name/'events.jsonl').read_text().splitlines()]
 candidate=[json.loads(line) for line in path.read_text().splitlines()]
 assert len(baseline)==len(candidate)
 changes=[];timings=[];count=0
 for event,(old,new) in enumerate(zip(baseline,candidate)):
  a,b=numbers(old),numbers(new)
  for key in set(a)|set(b):
   if key[-1]=='elapsed_seconds':
    if a.get(key)!=b.get(key):timings.append(dict(event=event,path=str(key),baseline=a.get(key),candidate=b.get(key)))
    continue
   count+=1
   if a.get(key)!=b.get(key):changes.append(dict(event=event,path=str(key),baseline=a.get(key),candidate=b.get(key)))
 decisions_equal=all(a.get(key)==b.get(key) for a,b in zip(baseline,candidate) for key in ('decision','selected_run','event'))
 results.append(dict(dataset=path.parent.name,events=len(baseline),numeric_fields=count,numeric_changes=changes,timing_changes=timings,decisions_equal=decisions_equal))
print(sum(r['numeric_fields'] for r in results),'non-timing numeric fields;',sum(len(r['numeric_changes']) for r in results),'changes')
(OUT/'E10-raw-events-comparison.json').write_text(json.dumps(results,indent=2)+'\n')
