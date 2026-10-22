"""E06: run the complete automatic replay on files shifted to observed references."""
import importlib.util
from pathlib import Path
import subprocess
import time
p=Path(__file__).with_name('experiments.py');spec=importlib.util.spec_from_file_location('experiments',p)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
np=m.np
out=m.SESSION/'E06';out.mkdir()
pilot=m.json.loads((m.SESSION/'E05/results.json').read_text())
rows=[]
for name,source in m.SOURCES.items():
    for mode in ['center','edge']:
        transform=next(r['transform'] for r in pilot if r['name']==name and r['prefix']==3 and r['solver']=='lbfgsb' and r['mode']==mode)
        work=out/f'{name}-{mode}';work.mkdir()
        ds=work/'sweeps.json';data=m.json.loads(source.read_text());q=np.asarray(transform['offsets_mm'])
        m.save(ds,m.rebase_dataset(data,q));m.save(work/'transform.json',transform)
        cmd=[str(m.ROOT/'.venv/bin/python'),'-m','autocal.autocal','--sim','--no-collect',
             '--machine-type','hangprinter_3','--dataset',str(ds),'--base-radii','30','--buildup-factor','0',
             '--find-radii','off','--find-buildup-factor','off','--stage-artifacts',str(work/'stages'),
             '--full-auto-log',str(work/'run.jsonl')]
        started=time.perf_counter()
        with (work/'stdout.log').open('w') as log:
            process=subprocess.run(cmd,cwd=m.ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=240)
        row={'name':name,'mode':mode,'transform':transform,'command':cmd,'rc':process.returncode,'seconds':time.perf_counter()-started}
        summary=m.regression.parse_summary((work/'stdout.log').read_text(),fixed_radii=[30.]*4)
        if summary:
            restored=m.restore_anchors(summary.anchors,q)
            row.update({'restoration':restored,**m.metrics(restored['anchors'])})
        row['iterations']=[]
        for line in (work/'run.jsonl').read_text().splitlines():
            event=m.json.loads(line)
            if 'iteration' in event:
                row['iterations'].append({k:event.get(k) for k in ['iteration','selected_run','decision','stop_spread_hit','cost']})
        histories=sorted((work/'stages').glob('history-*.json'))
        if histories:
            history=m.read_stage_artifact(histories[-1])['payload']['candidates']
            row['sweeps_used']=len(history[-1]['plan']['dataset']['sweeps'])
            row['points_used']=sum(len(s['data_points']) for s in history[-1]['plan']['dataset']['sweeps'])
        m.save(work/'result.json',row);rows.append(row);m.save(out/'results.json',rows)
        print(name,mode,process.returncode,row.get('rmse_mm'),row.get('restoration',{}).get('closure_rms_mm'),row.get('sweeps_used'),flush=True)
