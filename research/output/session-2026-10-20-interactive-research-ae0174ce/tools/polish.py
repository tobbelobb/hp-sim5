"""E07: use rebased fits only as seeds, then restore the original objective."""
import contextlib
import importlib.util
from pathlib import Path
import time
p=Path(__file__).with_name('experiments.py');spec=importlib.util.spec_from_file_location('experiments',p)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
out=m.SESSION/'E07';out.mkdir()
options=m.read_stage_artifact(m.ROOT/'output/extended-autocal/no-buildup-reference-003/sweeps.stages/fit-003-default.json')['payload']['fit_options']
options.update(robust_debug=False,buildup_factor=0.,base_radii=[30.]*4)
rows=[]
for r in m.json.loads((m.SESSION/'E05/results.json').read_text()):
    if r['solver']!='lbfgsb' or r['mode']=='original':continue
    work=out/f"{r['name']}-n{r['prefix']}-{r['mode']}";work.mkdir()
    data=m.json.loads(m.SOURCES[r['name']].read_text());data['sweeps']=data['sweeps'][:r['prefix']]
    path=work/'sweeps.json';m.save(path,data)
    started=time.perf_counter()
    with (work/'fit.log').open('w') as log,contextlib.redirect_stdout(log),contextlib.redirect_stderr(log):
        fit=m.fit_ellipse_dataset(path,**{**options,'initial_guess':m.np.asarray(r['restoration']['anchors'])})
    row={'name':r['name'],'prefix':r['prefix'],'mode':r['mode'],'seconds':time.perf_counter()-started,
         'cost':fit['cost'],'anchors':fit['anchors'],**m.metrics(fit['anchors']),
         'validation':m.score_future_sweeps(m.json.loads(m.SOURCES[r['name']].read_text()),fit['anchors'],
                      [s['id'] for s in data['sweeps']],use_flex=False)}
    rows.append(row);m.save(work/'result.json',row);m.save(out/'results.json',rows)
    print(row['name'],row['prefix'],row['mode'],row['rmse_mm'],row['cost'],flush=True)
