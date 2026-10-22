import json,re,sys
from pathlib import Path
sys.path.insert(0,str(Path.cwd()))
from autocal.tools.regress_calibration_logs import DATASETS,parse_log_file,resolve_reference_log_path,report_dataset
OUT=Path(__file__).resolve().parent
results=[];lines=[]
for spec in DATASETS:
 variant='E09-final-slide' if spec.machine_type=='slideprinter' else 'E09-final-hp'
 log=OUT/variant/spec.name/'calibration.log'
 if not log.exists():continue
 baseline=OUT/'E01-baseline'/spec.name/'calibration.log'
 reference=resolve_reference_log_path(spec.name,Path('autocal/data/references'),Path('autocal/data'),spec.reference_log_names)
 comparisons={}
 for label,path in [('fresh_baseline',baseline),('stored_reference',reference)]:
  a,b=parse_log_file(path),parse_log_file(log)
  ok,detail,delta=report_dataset(spec.name,a,b,spec,.01,True,False)
  comparisons[label]={'ok':ok,'true_total_error_delta':delta,'iterations_reference':len(a.iterations),'iterations_final':len(b.iterations)};lines.extend([label,*detail])
 def numbers(v,path=()):
  import math
  if isinstance(v,dict):return {k:x for key,child in v.items() for k,x in numbers(child,path+(key,)).items()}
  if isinstance(v,list):return {k:x for key,child in enumerate(v) for k,x in numbers(child,path+(key,)).items()}
  if isinstance(v,(int,float)) and (not path or path[-1]!='elapsed_seconds'):
   return {path:'NaN' if isinstance(v,float) and math.isnan(v) else v}
  return {}
 old=[json.loads(l) for l in (baseline.parent/'events.jsonl').read_text().splitlines()]
 new=[json.loads(l) for l in (log.parent/'events.jsonl').read_text().splitlines()]
 events_equal=len(old)==len(new) and all(numbers(a)==numbers(b) for a,b in zip(old,new))
 numeric_fields=sum(len(numbers(a)) for a in old)
 decisions_equal=len(old)==len(new) and all(a.get(k)==b.get(k) for a,b in zip(old,new) for k in ('event','decision','selected_run'))
 results.append({'dataset':spec.name,'comparisons':comparisons,'non_timing_numeric_fields_equal':events_equal,'numeric_fields':numeric_fields,'decisions_equal':decisions_equal})
(OUT/'E09-regressions.json').write_text(json.dumps(results,indent=2));(OUT/'E09-regressions.txt').write_text('\n'.join(lines)+'\n')
print(json.dumps({'completed':len(results),'all_pass':all(c['ok'] for r in results for c in r['comparisons'].values()),'numeric_events_equal':sum(r['non_timing_numeric_fields_equal'] for r in results),'numeric_fields':sum(r['numeric_fields'] for r in results),'decisions_equal':all(r['decisions_equal'] for r in results)},indent=2))
