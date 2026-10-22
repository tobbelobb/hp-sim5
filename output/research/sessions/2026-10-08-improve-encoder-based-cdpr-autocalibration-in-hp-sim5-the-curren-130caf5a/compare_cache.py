import json,math,sys
from pathlib import Path
sys.path.insert(0,str(Path.cwd()))
from autocal.tools.regress_calibration_logs import DATASETS,parse_log_file,resolve_reference_log_path,report_dataset
OUT=Path(__file__).resolve().parent
variant=sys.argv[1] if len(sys.argv)>1 else 'E05-cache';results=[];lines=[]
def numbers(v,path=()):
 if isinstance(v,dict):return {k:x for key,child in v.items() for k,x in numbers(child,path+(key,)).items()}
 if isinstance(v,list):return {k:x for key,child in enumerate(v) for k,x in numbers(child,path+(key,)).items()}
 if isinstance(v,(int,float)) and (not path or path[-1]!='elapsed_seconds'):return {path:'NaN' if isinstance(v,float) and math.isnan(v) else v}
 return {}
for spec in DATASETS:
 log=OUT/variant/spec.name/'calibration.log'
 if not log.exists():continue
 baseline=OUT/'E01-baseline'/spec.name/'calibration.log'
 reference=resolve_reference_log_path(spec.name,Path('autocal/data/references'),Path('autocal/data'),spec.reference_log_names)
 comparisons={}
 for label,path in [('fresh_baseline',baseline),('stored_reference',reference)]:
  a,b=parse_log_file(path),parse_log_file(log);ok,detail,delta=report_dataset(spec.name,a,b,spec,.01,True,False)
  comparisons[label]=dict(ok=ok,true_total_error_delta=delta,iterations_reference=len(a.iterations),iterations_candidate=len(b.iterations));lines.extend([label,*detail])
 old=[json.loads(l) for l in (baseline.parent/'events.jsonl').read_text().splitlines()];new=[json.loads(l) for l in (log.parent/'events.jsonl').read_text().splitlines()]
 diff=[dict(event_index=i,differences=[dict(key=str(k),old=v,new=numbers(b).get(k)) for k,v in numbers(a).items() if numbers(b).get(k)!=v]) for i,(a,b) in enumerate(zip(old,new)) if numbers(a)!=numbers(b)]
 events_equal=len(old)==len(new) and not diff
 decisions_equal=len(old)==len(new) and all(a.get(k)==b.get(k) for a,b in zip(old,new) for k in ('event','decision','selected_run'))
 results.append(dict(dataset=spec.name,comparisons=comparisons,non_timing_numeric_fields_equal=events_equal,numeric_fields=sum(len(numbers(a)) for a in old),decisions_equal=decisions_equal,differences=diff))
(OUT/f'{variant}-regressions.json').write_text(json.dumps(results,indent=2));(OUT/f'{variant}-regressions.txt').write_text('\n'.join(lines)+'\n')
print(json.dumps(dict(completed=len(results),all_pass=all(c['ok'] for r in results for c in r['comparisons'].values()),numeric_events_equal=sum(r['non_timing_numeric_fields_equal'] for r in results),numeric_fields=sum(r['numeric_fields'] for r in results),decisions_equal=all(r['decisions_equal'] for r in results)),indent=2))
