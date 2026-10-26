#!/usr/bin/env python3
"""Bounded offline probes. Run from repo root with .venv/bin/python; no fitting."""
import ast
import argparse
import copy
import hashlib
import inspect
import itertools
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

sys.dont_write_bytecode = True
PARSER = argparse.ArgumentParser(description=__doc__)
PARSER.add_argument('experiment', nargs='?', default='all', choices=['prepare','E1','E2','E3','all'])
PARSER.add_argument('--repo-root', type=Path, help='Production source root; defaults to git root of current directory')
PARSER.add_argument('--output', type=Path, default=Path(__file__).resolve().parent, help='Input and evidence directory')
ARGS = PARSER.parse_args()
ROOT = (ARGS.repo_root or Path(subprocess.check_output(['git','rev-parse','--show-toplevel'],text=True).strip())).resolve()
OUT = ARGS.output.resolve()
OUT.mkdir(parents=True,exist_ok=True)
sys.path.insert(0, str(ROOT))
os.environ['JAX_COMPILATION_CACHE_DIR'] = str(OUT / 'jax-cache')
os.environ['XDG_CACHE_HOME'] = str(OUT / 'cache')
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
os.environ['MKL_NUM_THREADS'] = '1'
import numpy as np
from autocal.ellipse_cost import EllipseCostFunction
import autocal.ellipse_objective_jax as jm
from autocal.spool_model import WinchSpoolModel, build_spool_model_params, dataset_with_modeled_lengths
from autocal.theoretical_ellipse import (anchors_matrix_to_opt_vec, anchor_opt_vec_to_matrix,
                                       compute_constraint_circle_3d, get_anchor_opt_bounds)

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def write(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')

def spool(r, b):
    return WinchSpoolModel.from_firmware(base_radius=r, buildup_factor=b,
        spool_to_motor_gearing_factor=1, mechanical_advantage=1, lines_per_spool=1)

def model(d, r, b):
    p = build_spool_model_params(d, base_radii_mm=[30.]*4,
        modeled_radii_mm=np.asarray(r).tolist(), modeled_buildup_factor=np.broadcast_to(b, (4,)).tolist(),
        spool_to_motor_gearing_factor=[1.]*4, mechanical_advantage=[1.]*4,
        lines_per_spool=[1.]*4, theta0_mode='zero')
    return dataset_with_modeled_lengths(d, p)

def cost(d, noise=False):
    return EllipseCostFunction(d, pointwise_filtering=False, sweep_wise_filtering=False,
        pointwise_global_mad=False, robust_loss=False, use_noise_mean=False,
        noise_normalized=noise, sigma_source='point')

def vec(a):
    return anchors_matrix_to_opt_vec(np.asarray(a), 'hangprinter_4')

def np_residuals(d, a):
    c = cost(d)
    l2, mm = [], []
    valid = 0
    for sw in c.sweeps:
        e = c._pointwise_sweep_residuals(sw, np.asarray(a))
        r = e['residuals']
        if r is None:
            continue
        valid += 1
        l2.extend(r.tolist())
        mm.extend((r / (e['l_drive_abs'] + e['l_sensor_abs'])).tolist())
    return np.asarray(l2), np.asarray(mm), valid

def summary(x):
    x = np.asarray(x, dtype=float)
    finite = x[np.isfinite(x)]
    return {'count':int(x.size), 'finite_count':int(finite.size),
            'rms':float(np.sqrt(np.mean(finite**2))) if finite.size else None,
            'max_abs':float(np.max(np.abs(finite))) if finite.size else None}

def delta(x, y):
    x, y = np.asarray(x), np.asarray(y)
    if x.shape != y.shape or not x.size:
        return {'shape_match':False, 'absolute':None}
    return {'shape_match':True, 'absolute':summary(x-y)}

def make_jax_residual_probe():
    """Return the actual source core's intermediate arrays without changing source.

    AST truncation stops immediately before noise normalization. Everything through
    raw Sampson residual construction is exactly the current production function.
    Separate calls below also evaluate the unmodified compiled objective.
    """
    source = inspect.getsource(jm._objective_core)
    tree = ast.parse(source)
    fn = tree.body[0]
    stop = next(i for i,n in enumerate(fn.body)
                if isinstance(n,ast.If) and isinstance(n.test,ast.Name)
                and n.test.id == 'noise_norm_available')
    fn.name = '_session_residual_probe'
    fn.body = fn.body[:stop] + [ast.parse(
        'return residuals, residuals / (drive_abs + sensor_abs), geom_valid, point_mask').body[0]]
    ast.fix_missing_locations(tree)
    namespace = dict(jm.__dict__)
    exec(compile(tree, '<session-current-source-residual-probe>', 'exec'), namespace)
    write('jax-probe-provenance.json', {
        'original_core_sha256':hashlib.sha256(source.encode()).hexdigest(),
        'transformed_ast_sha256':hashlib.sha256(ast.dump(tree).encode()).hexdigest(),
        'operation':'truncate before noise_norm_available; return residuals, residuals/(drive_abs+sensor_abs), geom_valid, point_mask',
        'source_edited':False})
    kwargs = jm._build_kwargs_for_objective(cost(INPUT['synthetic_clean']), constraint_mode=2)
    return jm.jax.jit(namespace[fn.name], static_argnames=tuple(kwargs))

PROBE = None

def measure(d, a, noise=False, jax=True):
    global PROBE
    c = cost(d, noise)
    r2, rm, valid = np_residuals(d, a)
    n = sum(len(s['data_points']) for s in d['sweeps'])
    assert r2.size == n and np.isfinite(r2).all(), (n, summary(r2),valid)
    out = {'expected_points':n, 'sweeps':len(d['sweeps']), 'numpy_valid_sweeps':valid,
           'numpy_l2_mm2':summary(r2), 'numpy_signed_mm':summary(rm),
           'numpy_objective':float(c.evaluate(vec(a))), 'noise_normalized':noise,
           'numpy_residuals_mm':rm.tolist(), 'numpy_residuals_mm2':r2.tolist()}
    if jax:
        lb, ub = [np.asarray(v) for v in get_anchor_opt_bounds('hangprinter_4',4,3)]
        layout = jm._jax_constraint_mode(c,lb,ub)
        packed = jm._prepare_sweeps(c)
        kwargs = jm._build_kwargs_for_objective(c,constraint_mode=layout[0])
        if PROBE is None:
            PROBE = make_jax_residual_probe()
        arrays = PROBE(jm.jnp.asarray(vec(a)),
                       *jm._build_packed_jax_args(packed,layout[1],lb,ub), **kwargs)
        jr2,jrm,geom,mask = [np.asarray(x) for x in arrays]
        jr2,jrm = jr2[mask],jrm[mask]
        assert np.all(geom) and jr2.size == n and np.isfinite(jr2).all()
        obj = jm.build_compiled_objective(c,lb,ub)
        assert obj is not None
        out.update(jax_l2_mm2=summary(jr2), jax_signed_mm=summary(jrm),
                   jax_valid_sweeps=int(np.sum(geom)), jax_objective=float(obj(vec(a))),
                   jax_residuals_mm=jrm.tolist(),jax_residuals_mm2=jr2.tolist(),
                   backend_delta_mm=delta(rm,jrm),backend_objective_absolute_delta=abs(out['numpy_objective']-obj(vec(a))))
    assert np.isfinite(out['numpy_objective'])
    return out

def make_sweep(a, r, b, pair, seed, idx):
    lengths = [np.linalg.norm(seed-a[k]) for k in pair]
    circle = compute_constraint_circle_3d(a,list(pair),lengths)
    assert circle is not None
    offset = seed-circle.center
    phi0 = np.arctan2(np.dot(offset,circle.v),np.dot(offset,circle.u))
    phi = phi0 + np.linspace(-.42,.42,25)
    pos = circle.center + circle.radius*(np.cos(phi)[:,None]*circle.u + np.sin(phi)[:,None]*circle.v)
    free = [k for k in range(4) if k not in pair]
    if idx % 2:
        free.reverse()
    drive,sensor = free
    points=[]
    for p in pos:
        dl = np.linalg.norm(p-a,axis=1)-np.linalg.norm(a,axis=1)
        for k in pair:
            dl[k] = lengths[list(pair).index(k)]-np.linalg.norm(a[k])
        theta=[]
        for k in range(4):
            sp=spool(r[k],b)
            assert sp.k1+dl[k]*sp.k2 > 0, 'inverse clipping would invalidate synthetic data'
            t=sp.linepos_mm_to_theta_deg(dl[k])
            assert abs(sp.theta_deg_to_linepos_mm(t)-dl[k])<1e-8
            theta.append(t)
        points.append({'l_drive':float(dl[drive]),'l_sensor':float(dl[sensor]),
                       'raw_angles_deg':theta,'sigma':[.03]*4})
    return {'id':f'synthetic_{idx:02d}','fixed_anchors':list(pair),
            'fixed_lengths':[float(v-np.linalg.norm(a[k])) for k,v in zip(pair,lengths)],
            'drive_anchor':drive,'sensor_anchor':sensor,'data_points':points}

def prepare():
    a=np.asarray([[0,-1900,-120],[1645,950,-120],[-1645,950,-120],[0,0,2050.]],dtype=float)
    r=np.asarray([39.2,40.1,38.7,39.8]); b=.636619
    pairs=[(0,1),(0,2),(1,2),(0,3),(1,3),(2,3)]
    seeds=[np.asarray([0.,-120.,500.]),np.asarray([260.,150.,750.])]
    sweeps=[make_sweep(a,r,b,pair,seed,i) for i,(seed,pair) in enumerate(itertools.product(seeds,pairs))]
    clean={'version':'2.0','machine_type':'hangprinter_4','num_anchors':4,'dimensions':3,
           'config':{'angles_unit':'deg','lengths_unit':'mm','mm_per_degree':(r*np.pi/180).tolist()},'sweeps':sweeps}
    rng=np.random.default_rng(20261026)
    noisy=copy.deepcopy(clean)
    for sw in noisy['sweeps']:
        for p in sw['data_points']:
            for k in [sw['drive_anchor'],sw['sensor_anchor']]:
                p['raw_angles_deg'][k]+=float(rng.normal(0,.03))
    source=ROOT/'autocal/data/references/fifth_hp3_dataset.json'
    frozen=json.loads(source.read_text())
    frozen['sweeps']=frozen['sweeps'][:3]
    # Preserve actual raw observations and length fields, omit unused telemetry.
    compact={k:frozen[k] for k in ('version','machine_type','num_anchors','dimensions','config','sweeps')}
    for sw in compact['sweeps']:
        sw['data_points']=[{k:p[k] for k in ('l_drive','l_sensor','raw_angles_deg','sigma') if k in p} for p in sw['data_points']]
    write('inputs.json',{'seed':20261026,'anchors_mm':a.tolist(),'radii_mm':r.tolist(),
        'buildup_mm':b,'angle_noise_sigma_deg':.03,'synthetic_clean':clean,'synthetic_noisy':noisy,
        'frozen_hp3_first_three':compact,'frozen_anchors_mm':[[0,-1900,-280],[1645.45,950,-280],[-1645.45,950,-280],[0,0,1900]],
        'frozen_source':str(source.relative_to(ROOT)), 'frozen_source_sha256':sha(source)})
    return {'prepared':True,'input_sha256':sha(OUT/'inputs.json'),'synthetic_points':300}

def e1():
    result={'experiment':'E1','fixed_encoder_perturbation_deg':.7,
            'fixed_uniform_shift_deg':1.,'drive_angle_shift_deg':1.,'drive_length_shift_mm':.5,'cases':[]}
    for label,raw,a,r in [('synthetic',INPUT['synthetic_clean'],INPUT['anchors_mm'],INPUT['radii_mm']),
                          ('frozen_hp3',INPUT['frozen_hp3_first_three'],INPUT['frozen_anchors_mm'],[39.1845]*4)]:
        base=copy.deepcopy(raw)
        if label=='synthetic':
            base['sweeps']=base['sweeps'][:3]
        # Give each point a distinct fixed encoder value, then permute values only.
        for sw in base['sweeps']:
            for i,p in enumerate(sw['data_points']):
                for k in sw['fixed_anchors']:
                    p['raw_angles_deg'][k]+=float(np.linspace(-.7,.7,len(sw['data_points']))[i])
        variants={'baseline':base,'fixed_permuted':copy.deepcopy(base),
                  'fixed_uniform_shift':copy.deepcopy(base),'drive_angle_shift':copy.deepcopy(base),
                  'drive_length_shift':copy.deepcopy(base),'aggregate_fixed_length_shift':copy.deepcopy(base)}
        for sw in variants['fixed_permuted']['sweeps']:
            for k in sw['fixed_anchors']:
                values=[p['raw_angles_deg'][k] for p in sw['data_points']][::-1]
                for p,v in zip(sw['data_points'],values): p['raw_angles_deg'][k]=v
        for sw in variants['fixed_uniform_shift']['sweeps']:
            for p in sw['data_points']:
                for k in sw['fixed_anchors']: p['raw_angles_deg'][k]+=1.
        for sw in variants['drive_angle_shift']['sweeps']:
            for p in sw['data_points']: p['raw_angles_deg'][sw['drive_anchor']]+=1.
        for sw in variants['drive_length_shift']['sweeps']:
            for p in sw['data_points']: p['l_drive']+=.5
        for sw in variants['aggregate_fixed_length_shift']['sweeps']: sw['fixed_lengths'][0]+=.5
        for mode in ['raw_lengths','modeled_lengths']:
            measurements={}
            ds={}
            for name,d in variants.items():
                ds[name]=model(d,r,INPUT['buildup_mm']) if mode=='modeled_lengths' else d
                measurements[name]=measure(ds[name],np.asarray(a))
            comparisons={}
            for name,m in measurements.items():
                if name=='baseline':continue
                comparisons[name]={backend:delta(m[f'{backend}_residuals_mm'],measurements['baseline'][f'{backend}_residuals_mm']) for backend in ['numpy','jax']}
                comparisons[name]['numpy_objective_absolute_delta']=abs(m['numpy_objective']-measurements['baseline']['numpy_objective'])
                comparisons[name]['jax_objective_absolute_delta']=abs(m['jax_objective']-measurements['baseline']['jax_objective'])
                comparisons[name]['fixed_median_length_absolute_delta_mm']=max(abs(x-y) for s,t in zip(ds[name]['sweeps'],ds['baseline']['sweeps']) for x,y in zip(s['fixed_lengths'],t['fixed_lengths']))
            result['cases'].append({'dataset':label,'length_mode':mode,'measurements':measurements,'comparisons':comparisons})
    return result

def e2():
    a=np.asarray(INPUT['anchors_mm']);r=np.asarray(INPUT['radii_mm']);b=INPUT['buildup_mm']
    result={'experiment':'E2','scales':[.95,1.,1.05],'paths':[]}
    for noise_label,key in [('clean','synthetic_clean'),('noisy_angle','synthetic_noisy')]:
        raw=INPUT[key]
        basemodel=model(raw,r,b)
        baseline=measure(basemodel,a)
        for scale in result['scales']:
            for path in ['joint_geometry_radius_buildup','geometry_only','geometry_radius_fixed_buildup','geometry_buildup_known_radius0']:
                ar=a*scale
                rr=r.copy() if path=='geometry_only' else r*scale
                bb=b*scale if path in ['joint_geometry_radius_buildup','geometry_buildup_known_radius0'] else b
                if path=='geometry_buildup_known_radius0': rr[0]=r[0]
                d=model(raw,rr,bb)
                m=measure(d,ar)
                m['residual_scaling_error_mm']=delta(m['numpy_residuals_mm'],np.asarray(baseline['numpy_residuals_mm'])*scale)
                m['length_homogeneity_error_mm']=summary([p[k]-scale*q[k] for s,t in zip(d['sweeps'],basemodel['sweeps']) for p,q in zip(s['data_points'],t['data_points']) for k in ['l_drive','l_sensor']]+[x-scale*y for s,t in zip(d['sweeps'],basemodel['sweeps']) for x,y in zip(s['fixed_lengths'],t['fixed_lengths'])])
                result['paths'].append({'noise':noise_label,'scale':scale,'path':path,'measurements':m})
            if noise_label=='noisy_angle':
                d=model(raw,r*scale,b*scale)
                fixed_sigma=measure(d,a*scale,noise=True)
                # Explicitly co-scale the mm/degree noise conversion as a normalization control.
                d_scaled_sigma=copy.deepcopy(d)
                d_scaled_sigma['config']['mm_per_degree']=(np.asarray(d['config']['mm_per_degree'])*scale).tolist()
                scaled_sigma=measure(d_scaled_sigma,a*scale,noise=True)
                result['paths'].append({'noise':noise_label,'scale':scale,'path':'joint_noise_normalization_controls',
                                        'fixed_mm_sigma':fixed_sigma,'coscaled_mm_sigma':scaled_sigma})
    return result

def e3():
    a=np.asarray(INPUT['anchors_mm']);r=np.asarray(INPUT['radii_mm']);b=INPUT['buildup_mm'];x0=vec(a)
    base=copy.deepcopy(INPUT['synthetic_clean']);base['sweeps']=base['sweeps'][:3]
    duplicate=copy.deepcopy(base);duplicate['sweeps']=copy.deepcopy(base['sweeps']*4)
    for i,s in enumerate(duplicate['sweeps']):s['id']=f'duplicate_{i}'
    direction=np.r_[x0/1000.,np.ones(5)]
    unit_direction=direction/np.linalg.norm(direction)
    result={'experiment':'E3','geometry_coordinate_unit_mm':1000.,'radius_coordinates':'log relative radius',
            'buildup_coordinate':'log relative common buildup, mm','data_sigma_mm':.02,'radius0_sigma_mm':.05,
            'scale_direction_in_coordinates':direction.tolist(),'cases':[]}
    for label,raw in [('base_3',base),('duplicated_12',duplicate),('diverse_12',INPUT['synthetic_clean'])]:
        def residual(u):
            aa=anchor_opt_vec_to_matrix(x0+1000*u[:9],'hangprinter_4',4,3)
            rr=r*np.exp(u[9:13]);bb=b*np.exp(u[13])
            d=model(raw,rr,bb)
            _,mm,valid=np_residuals(d,aa)
            assert valid==len(raw['sweeps']) and mm.size==25*len(raw['sweeps']) and np.isfinite(mm).all()
            return mm
        derivatives=[]
        for h in [1e-4,1e-5]:
            columns=[]
            for k in range(14):
                u=np.zeros(14);u[k]=h
                columns.append((residual(u)-residual(-u))/(2*h))
            j=np.column_stack(columns)
            n=j.shape[0]
            # Normalize by sqrt(n) for fair comparison with duplicated observations.
            normalized=j/np.sqrt(n)
            s=np.linalg.svd(normalized,compute_uv=False)
            radius_row=np.zeros(14);radius_row[9]=r[0]/.05
            augmented=np.vstack([j/.02,radius_row])
            s_aug=np.linalg.svd(augmented,compute_uv=False)
            j_fixed_b=j[:,:13]
            s_fixed=np.linalg.svd(j_fixed_b/np.sqrt(n),compute_uv=False)
            direct=(residual(h*direction)-residual(-h*direction))/(2*h)
            derivatives.append({'central_step':h,'rows':n,'columns':14,'finite_entries':int(np.isfinite(j).sum()),
                'jacobian_mm_per_coordinate':j.tolist(),'rms_normalized_singular_values':s.tolist(),
                'fixed_buildup_singular_values':s_fixed.tolist(),
                'augmented_sigma_normalized_singular_values':s_aug.tolist(),
                'unit_scale_direction_response_mm':summary(normalized@unit_direction),
                'scale_direction_response_mm_per_relative_scale':summary(direct),
                'radius_scale_slope_z_per_relative_scale':float(radius_row@direction),
                'numerical_rank_at_relative_1e-7':int(np.sum(s>s[0]*1e-7)),
                'augmented_rank_at_relative_1e-7':int(np.sum(s_aug>s_aug[0]*1e-7))})
        scale_control=[]
        for scale in [.95,1.,1.05]:
            m=measure(model(raw,r*scale,b*scale),a*scale,jax=False)
            scale_control.append({'scale':scale,'data_rms_mm':m['numpy_signed_mm']['rms'],
                'finite_points':m['numpy_signed_mm']['finite_count'],
                'known_radius0_residual_mm':float(r[0]*(scale-1)),
                'known_radius0_residual_z':float(r[0]*(scale-1)/.05)})
        result['cases'].append({'dataset':label,'derivatives':derivatives,'scale_controls':scale_control})
    return result

def provenance():
    paths=sorted((ROOT/'autocal').rglob('*.py'))
    return {'utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'machine':platform.uname()._asdict(),
            'python':sys.version,'python_executable':sys.executable,'numpy':np.__version__,
            'jax':jm.jax.__version__,'jax_backend':jm.jax.default_backend(),
            'jax_x64':bool(jm.jax.config.jax_enable_x64),'git_revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
            'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in paths},
            'analysis_sha256':sha(__file__),'inputs_sha256':sha(OUT/'inputs.json'),
            'units':{'anchors':'mm','radii':'mm','buildup':'mm, dimensionful coefficient k2=-MA*lines*buildup','angles':'deg',
                     'Sampson_l2':'mm^2','signed_geometric_residual':'approximate mm, r_l2/(drive_abs+sensor_abs)','objective':'dimensionless'},
            'allocation_used':{'physics_steps':0,'collections':0,'fits':0},
            'cpu_model':[line.strip() for line in Path('/proc/cpuinfo').read_text().splitlines() if line.startswith('model name')][:1]}

if __name__=='__main__':
    start=time.monotonic();command=ARGS.experiment
    if command=='prepare':
        print(json.dumps(prepare()));sys.exit(0)
    INPUT=json.loads((OUT/'inputs.json').read_text())
    before=provenance()
    selected=['E1','E2','E3'] if command=='all' else [command]
    for name in selected:
        t=time.monotonic()
        result={'E1':e1,'E2':e2,'E3':e3}[name]()
        result['elapsed_seconds']=time.monotonic()-t
        result['analysis_sha256']=sha(__file__);result['inputs_sha256']=sha(OUT/'inputs.json')
        result['git_revision']=before['git_revision']
        write(f'{name}.json',result)
        print(json.dumps({'experiment':name,'elapsed_seconds':result['elapsed_seconds'],'evidence':str(OUT/f'{name}.json')}),flush=True)
    after=provenance()
    assert before['source_sha256']==after['source_sha256'], 'source changed during experiment'
    before['source_unchanged_at_exit']=True;before['elapsed_seconds']=time.monotonic()-start
    before['evidence_sha256']={f'{name}.json':sha(OUT/f'{name}.json') for name in selected}
    write(f'manifest-{command}.json',before)
