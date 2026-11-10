"""Bounded offline HP3 reference experiments. Ground truth is metrics-only."""
import argparse
import contextlib
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT))
import numpy as np
from autocal.stage_artifacts import read_stage_artifact
from autocal.fit_stage import fit_ellipse_dataset
from autocal.tools.rebase_dataset import point_offsets, rebase_dataset, locate_reference, restore_anchors
from autocal.theoretical_ellipse import canonicalize_anchor_gauge, get_anchor_opt_bounds, anchors_matrix_to_opt_vec
from autocal.predictive_validation import score_future_sweeps
from autocal.tools import regress_calibration_logs as regression

SESSION = Path(__file__).resolve().parents[1]
OLD = ROOT / 'output/research/sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051'
SOURCES = {'REF': ROOT / 'output/extended-autocal/no-buildup-reference-003/sweeps.json',
           'W00': OLD / 'environment-matrix-20261015-retry1/W00-nominal/sweeps.json',
           'W08': OLD / 'environment-matrix-20261015-retry1/W08-heavy-high-g/sweeps.json'}
TRUE = np.array(regression.SIM_3D_TRUE_ANCHORS)


def save(path, data):
    path.write_text(json.dumps(data, indent=2, default=lambda x: x.tolist() if isinstance(x, np.ndarray) else str(x))+'\n')


def metrics(anchors):
    a = np.asarray(anchors)
    pairs = [(i, j) for i in range(4) for j in range(i)]
    pair_errors = [np.linalg.norm(a[i]-a[j])-np.linalg.norm(TRUE[i]-TRUE[j]) for i,j in pairs]
    return {'rmse_mm': float(np.sqrt(np.mean(np.sum((a-TRUE)**2,axis=1)))),
            'pairwise_rmse_mm': float(np.sqrt(np.mean(np.square(pair_errors))))}


def baseline():
    out = SESSION / 'E01';out.mkdir()
    rows=[]
    for name, source in SOURCES.items():
        work=out/name;work.mkdir();ds=work/'sweeps.json';ds.write_bytes(source.read_bytes())
        cmd=[str(ROOT/'.venv/bin/python'), '-m','autocal.autocal','--sim','--no-collect',
             '--machine-type','hangprinter_3','--dataset',str(ds),'--base-radii','30',
             '--buildup-factor','0','--find-radii','off','--find-buildup-factor','off',
             '--stage-artifacts',str(work/'stages'),'--full-auto-log',str(work/'run.log')]
        t=time.perf_counter()
        with (work/'stdout.log').open('w') as log:
            process=subprocess.run(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=240)
        row={'name':name,'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
             'command':cmd,'rc':process.returncode,'seconds':time.perf_counter()-t,'iterations':[]}
        for p in sorted((work/'stages').glob('fit-*-default.json')):
            payload=read_stage_artifact(p)['payload'];fit=payload['fit']
            row['iterations'].append({'artifact':str(p),'sweeps':len(fit['dataset']['sweeps']),
                'cost':fit['cost'],'anchors':fit['anchors'],**metrics(fit['anchors'])})
        summary=regression.parse_summary((work/'stdout.log').read_text(),fixed_radii=[30.]*4)
        if summary:
            row['applied']={'anchors':summary.anchors,**metrics(summary.anchors)}
        history=read_stage_artifact(sorted((work/'stages').glob('history-*.json'))[-1])['payload']
        row['selected_history']=[{'iteration':c['iteration'],'cost':c['cost'],'history_rank_score':c['history_rank_score'],**metrics(c['plan']['anchors'])} for c in history['candidates']]
        row['adjacent_score_geometry_disagreements']=sum(
            (a['cost']-b['cost'])*(a['rmse_mm']-b['rmse_mm'])<0
            for a,b in zip(row['selected_history'],row['selected_history'][1:]))
        if 'applied' in row:
            row['applied_regret_mm']=row['applied']['rmse_mm']-min(c['rmse_mm'] for c in row['selected_history'])
        rows.append(row);save(out/'results.json',rows)
        print(name, process.returncode, row.get('applied'),flush=True)


def pilot(solvers, prefixes, names, label, edge_policy="feasible", reference_max_sweeps=0):
    out=SESSION/label;out.mkdir()
    template=read_stage_artifact(ROOT/'output/extended-autocal/no-buildup-reference-003/sweeps.stages/fit-003-default.json')['payload']['fit_options']
    options={**template,'robust_debug':False,'buildup_factor':0.,'base_radii':[30.]*4}
    results=[]
    for name in names:
        data=json.loads(SOURCES[name].read_text())
        # Reference locations use a first-three-sweep estimate; candidate points come from the available prefix.
        artifact=read_stage_artifact(SESSION/'E01'/name/'stages/fit-001-default.json')
        estimate=np.asarray(artifact['payload']['fit']['anchors'])
        points=[]
        for i,s in enumerate(data['sweeps']):
            for j,p in enumerate(s['data_points']):
                q=point_offsets(data,i,j)
                pos,closure=locate_reference(estimate,np.linalg.norm(estimate,axis=1)+q)
                points.append((i,j,q,pos,closure))
        center=np.mean(estimate,axis=0)
        # A matched unshifted warm seed isolates the reference effect from initialization.
        lb,ub=get_anchor_opt_bounds('hangprinter_4',4,3)
        def feasible(p):
            v=anchors_matrix_to_opt_vec(canonicalize_anchor_gauge('hangprinter_4',estimate-p[3]),'hangprinter_4')
            return bool(np.all(v>=lb) and np.all(v<=ub))
        for requested_n in prefixes:
            n=len(data['sweeps']) if requested_n==0 else requested_n
            limit=min(n,reference_max_sweeps) if reference_max_sweeps else n
            available=[p for p in points if p[0]<limit]
            choices={'center':min(available,key=lambda p:np.linalg.norm(p[3]-center)),
                     'edge':max((p for p in available if edge_policy=='unrestricted' or feasible(p)),key=lambda p:np.linalg.norm(p[3]-center))}
            train=copy.deepcopy(data);train['sweeps']=train['sweeps'][:n]
            for solver in solvers:
                for mode in ('original','center','edge'):
                    work=out/f'{name}-n{n}-{solver}-{mode}';work.mkdir()
                    ds=work/'sweeps.json';kwargs={**options,'solve_optimizer':solver}
                    meta={}
                    if mode=='original':
                        shifted=train;kwargs['initial_guess']=estimate
                        q=np.zeros(4)
                    else:
                        i,j,q,pos,closure=choices[mode]
                        shifted=rebase_dataset(train,q)
                        kwargs['initial_guess']=canonicalize_anchor_gauge('hangprinter_4',estimate-pos)
                        meta={'sweep_index':i,'point_index':j,'offsets_mm':q.tolist(),
                              'estimated_position_mm':pos.tolist(),'estimated_shift_distance_mm':float(np.linalg.norm(pos)),
                              'reference_seed_closure_mm':closure.tolist(),
                              'distance_to_estimated_centroid_mm':float(np.linalg.norm(pos-center)),
                              'seed_within_original_solver_bounds':feasible(choices[mode])}
                    save(ds,shifted);save(work/'transform.json',meta)
                    t=time.perf_counter()
                    row={'name':name,'prefix':n,'solver':solver,'mode':mode,'transform':meta}
                    try:
                        with (work/'fit.log').open('w') as log,contextlib.redirect_stdout(log),contextlib.redirect_stderr(log):
                            fit=fit_ellipse_dataset(ds,**kwargs)
                        anchors=np.asarray(fit['anchors'])
                        restored=restore_anchors(anchors,q) if mode!='original' else {'anchors':anchors,'closure_rms_mm':0.}
                        row.update({'seconds':time.perf_counter()-t,'cost':fit['cost'],
                             'fitted_anchors':anchors,'restoration':restored,**metrics(restored['anchors']),
                             'validation':score_future_sweeps(data,np.asarray(restored['anchors']),
                                            [s['id'] for s in train['sweeps']],use_flex=False),
                             'warnings':fit.get('warnings')})
                    except Exception as exc:
                        row.update({'error':repr(exc),'seconds':time.perf_counter()-t})
                    results.append(row);save(work/'result.json',row);save(out/'results.json',results)
                    print(name,n,solver,mode,{k:row.get(k) for k in ('rmse_mm','pairwise_rmse_mm','cost','seconds','error')},flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['baseline','pilot']);ap.add_argument('--solvers',nargs='+',default=['lbfgsb'])
    ap.add_argument('--prefixes',type=int,nargs='+',default=[5]);ap.add_argument('--names',nargs='+',default=list(SOURCES));ap.add_argument('--label',default='E03')
    ap.add_argument('--edge-policy',choices=['feasible','unrestricted'],default='feasible');ap.add_argument('--reference-max-sweeps',type=int,default=0)
    args=ap.parse_args()
    if args.mode=='baseline':baseline()
    else:pilot(args.solvers,args.prefixes,args.names,args.label,args.edge_policy,args.reference_max_sweeps)
