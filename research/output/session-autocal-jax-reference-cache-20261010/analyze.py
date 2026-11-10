from dataclasses import asdict
import json
from pathlib import Path
import runpy
import shutil
import re
from collections import Counter
from statistics import mean
from checks import parse_fixed_radii, numeric_metrics

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
SOURCE=ROOT/'output/extended-autocal/no-buildup-reference-001/sweeps.json'
ns=runpy.run_path(str(ROOT/'autocal/tools/regress_calibration_logs.py'),run_name='reference_cache_analysis')
spec=ns['DatasetSpec'](name='no-buildup-reference-001',machine_type='hangprinter_4',base_radii='30',
    buildup_factor='0',true_anchors=ns['SIM_3D_TRUE_ANCHORS'],true_radii=[30.0]*4,find_radii='off')
baseline=parse_fixed_radii(ns,HERE/'D1/sweeps.full_auto.log')
baseline_metrics=numeric_metrics(HERE/'D1/sweeps.full_auto.log')
base_numeric=dict(summary=asdict(baseline.summary),iterations=[{k:getattr(it,k) for k in
    ('anchors','radii','fit_score_ui','rank_score','history_rank_score')} for it in baseline.iterations])
if not (HERE/'preliminary-results.json').exists():
    shutil.copy2(HERE/'results.json',HERE/'preliminary-results.json')
rows=json.loads((HERE/'results.json').read_text())
for row in rows:
    folder=HERE/row['trial']
    parsed=parse_fixed_radii(ns,folder/'sweeps.full_auto.log')
    row['numerical']=dict(summary=asdict(parsed.summary),iterations=[{k:getattr(it,k) for k in
        ('anchors','radii','fit_score_ui','rank_score','history_rank_score')} for it in parsed.iterations])
    row['exact_numerical_equal']=row['numerical']==base_numeric
    row['exact_full_precision_metrics_equal']=numeric_metrics(folder/'sweeps.full_auto.log')==baseline_metrics
    row['compiler_logs_collected']=False
    row['hits']=None
    row['misses']=None
    row['regression_pass'],lines,_=ns['report_dataset'](name=row['trial'],ref=baseline,gen=parsed,
        dataset_spec=spec,tol_mm_total=1e-6,fail_on_score_mismatch=True,color=False)
    (folder/'regression-comparison.txt').write_text('\n'.join(lines)+'\n')
    (folder/'summary.json').write_text(json.dumps(row,indent=2)+'\n')
(HERE/'results.json').write_text(json.dumps(rows,indent=2)+'\n')

original=parse_fixed_radii(ns,SOURCE.with_suffix('.full_auto.log'))
original_pass,lines,_=ns['report_dataset'](name='original-collection-vs-replay',ref=original,gen=baseline,
    dataset_spec=spec,tol_mm_total=1e-6,fail_on_score_mismatch=True,color=False)
(HERE/'original-comparison.txt').write_text('\n'.join(lines)+'\n')
integrity=json.loads((HERE/'integrity.json').read_text())
integrity['original_comparison_pass']=original_pass
(HERE/'integrity.json').write_text(json.dumps(integrity,indent=2)+'\n')
diagnostics=[]
for name in ['D8','D9','D10','D11','D14','D15']:
    folder=HERE/name
    row=json.loads((folder/'summary.json').read_text())
    combined=(folder/'stdout.log').read_text()+'\n'+(folder/'sweeps.full_auto.log').read_text()
    row['hits']=dict(Counter(module for module,key in set(re.findall(r"Persistent compilation cache hit for '([^']+)' with key '([^']+)'",combined))))
    row['misses']=dict(Counter(module for module,key in set(re.findall(r"PERSISTENT COMPILATION CACHE MISS for '([^']+)' with key '([^']+)'",combined))))
    parsed=parse_fixed_radii(ns,folder/'sweeps.full_auto.log')
    row['regression_pass'],lines,_=ns['report_dataset'](name=name,ref=baseline,gen=parsed,
        dataset_spec=spec,tol_mm_total=1e-6,fail_on_score_mismatch=True,color=False)
    row['exact_full_precision_metrics_equal']=numeric_metrics(folder/'sweeps.full_auto.log')==baseline_metrics
    (folder/'regression-comparison.txt').write_text('\n'.join(lines)+'\n')
    (folder/'summary.json').write_text(json.dumps(row,indent=2)+'\n')
    diagnostics.append(row)

fit_keys=['anchors','calibration','cost','cost_raw','cost_noise_normalized','info','covariance',
    'covariance_scaled','covariance_scale','confidence_intervals','workspace_diag_mm','info_rank',
    'info_rank_deficient','warnings','length_model','spool_model_params','line_width_mm','sigma_floor_mm',
    'sigma_used_mm','fit_structure_levels']
def fits(folder):
    result={}
    for p in sorted(folder.glob('fit-*.json')):
        fit=json.loads(p.read_text())['payload']['fit']
        fit['calibration'].pop('timestamp',None)
        result[p.name]={k:fit[k] for k in fit_keys}
    return result
off_fits=fits(HERE/'D10/stages')
fit_comparisons={name:dict(exact=fits(HERE/name/'stages')==off_fits,compared=len(off_fits),
    different=[k for k,v in off_fits.items() if fits(HERE/name/'stages').get(k)!=v]) for name in ['D8','D9','D11','D15']}
fit_comparisons['original']=dict(exact=fits(SOURCE.parent/'sweeps.stages')==off_fits,
    compared=len(off_fits),different=[k for k,v in off_fits.items()
                                    if fits(SOURCE.parent/'sweeps.stages').get(k)!=v])
valid_transfer=[json.loads((HERE/name/'summary.json').read_text()) for name in ['D12','D13']]
times={mode:[r['elapsed_s'] for r in rows if r['mode']==mode] for mode in
       ['off','cold-empty','target-warm']}
times['regression-warm']=[r['elapsed_s'] for r in valid_transfer]
means={mode:mean(t) for mode,t in times.items()}
analysis=dict(timing_seconds=times,mean_seconds=means,
    saved_seconds=means['off']-means['target-warm'],
    saved_percent=100*(1-means['target-warm']/means['off']),
    speedup=means['off']/means['target-warm'],
    regression_transfer_saved_percent=100*(1-means['regression-warm']/means['off']),
    all_timing_runs_regression_pass=all(r['regression_pass'] for r in rows+valid_transfer),
    all_timing_runs_numerically_exact=all(r['exact_numerical_equal'] for r in rows) and all(r['numerical']==base_numeric for r in valid_transfer),
    all_timing_runs_full_precision_metrics_exact=all(r['exact_full_precision_metrics_equal'] for r in rows+valid_transfer),
    original_log_regression_pass=original_pass,
    original_full_precision_metrics_exact=numeric_metrics(SOURCE.with_suffix('.full_auto.log'),include_settings=False)==numeric_metrics(HERE/'D1/sweeps.full_auto.log',include_settings=False),
    full_precision_fit_comparisons=fit_comparisons,diagnostics=diagnostics,
    relocated_cache_controls=dict(experiment_ids=['D2','D6','D8','D9','D11'],
        interpretation='Relocating the cache changes the GPU auxiliary cache path in compile options, even on CPU. These controls are excluded from transfer and warm-cache timing conclusions. D12-D14 preserve the original cache path; D15 preserves the D3 cache path.'),
    integrity=json.loads((HERE/'integrity.json').read_text()))
(HERE/'analysis.json').write_text(json.dumps(analysis,indent=2)+'\n')
print(json.dumps(analysis,indent=2))
assert all(r['regression_pass'] and r['exact_full_precision_metrics_equal'] for r in rows+diagnostics)
assert all(c['exact'] for key,c in fit_comparisons.items() if key!='original')
