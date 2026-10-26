#!/usr/bin/env python3
"""Summarize stored evidence and make deterministic gzip files. No numerics rerun."""
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parent)
out=parser.parse_args().output.resolve()
limit=1024*1024

def read(name):
    p=out/name
    return json.loads(p.read_text() if p.exists() else gzip.decompress(p.with_suffix(p.suffix+'.gz').read_bytes()))
def sha(data):return hashlib.sha256(data).hexdigest()
def save(name,data):
    (out/name).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')

e1,e2,e3,manifest=[read(n) for n in ['E1.json','E2.json','E3.json','manifest-all.json']]
checks=[]
rows1=[]
for c in e1['cases']:
    ms=c['measurements']
    expected=ms['baseline']['expected_points']
    for m in ms.values():
        assert m['numpy_signed_mm']['finite_count']==expected
        assert m['jax_signed_mm']['finite_count']==expected
        assert m['numpy_valid_sweeps']==m['sweeps']==m['jax_valid_sweeps']
    cmp=c['comparisons']
    assert cmp['fixed_permuted']['numpy']['absolute']['max_abs']==0
    assert cmp['fixed_permuted']['jax']['absolute']['max_abs']==0
    positive=['drive_length_shift','aggregate_fixed_length_shift'] if c['length_mode']=='raw_lengths' else ['fixed_uniform_shift','drive_angle_shift']
    assert all(cmp[k][backend]['absolute']['max_abs']>.1 for k in positive for backend in ['numpy','jax'])
    rows1.append({'dataset':c['dataset'],'length_mode':c['length_mode'],'points_per_condition_backend':expected,
        'all_conditions_finite':True,'baseline_rms_mm':ms['baseline']['numpy_signed_mm']['rms'],
        'max_backend_residual_disagreement_mm':max(m['backend_delta_mm']['absolute']['max_abs'] for m in ms.values()),
        'residual_max_absolute_changes_mm':{k:{b:v[b]['absolute']['max_abs'] for b in ['numpy','jax']} for k,v in cmp.items()},
        'objective_absolute_changes':{k:{b:v[b+'_objective_absolute_delta'] for b in ['numpy','jax']} for k,v in cmp.items()}})
checks.append('E1 complete finite NumPy/JAX coverage, exact permutation nulls, nonzero controls')

rows2=[];norm=[]
for row in e2['paths']:
    if 'measurements' in row:
        m=row['measurements']
        assert m['numpy_signed_mm']['finite_count']==m['jax_signed_mm']['finite_count']==300
        if row['path']=='joint_geometry_radius_buildup':
            assert m['length_homogeneity_error_mm']['max_abs']<1e-8
            assert m['residual_scaling_error_mm']['absolute']['max_abs']<1e-8
        elif row['noise']=='clean' and row['scale']!=1:
            assert m['numpy_signed_mm']['rms']>1
        rows2.append({'noise':row['noise'],'scale':row['scale'],'path':row['path'],
            'finite_points_per_backend':300,'numpy_rms_mm':m['numpy_signed_mm']['rms'],
            'jax_rms_mm':m['jax_signed_mm']['rms'],'numpy_objective':m['numpy_objective'],
            'jax_objective':m['jax_objective'],'length_homogeneity_max_error_mm':m['length_homogeneity_error_mm']['max_abs'],
            'residual_scaling_max_error_mm':m['residual_scaling_error_mm']['absolute']['max_abs']})
    else:
        norm.append({'scale':row['scale'],'path':row['path'],
            'fixed_mm_sigma_objective':row['fixed_mm_sigma']['numpy_objective'],
            'coscaled_mm_sigma_objective':row['coscaled_mm_sigma']['numpy_objective'],
            'fixed_drive_sigma_rms_mm':row['fixed_mm_sigma']['effective_drive_sigma_mm']['rms'],
            'coscaled_drive_sigma_rms_mm':row['coscaled_mm_sigma']['effective_drive_sigma_mm']['rms'],
            'floor_mm':row['fixed_mm_sigma']['sigma_floor_mm']})
active=[r for r in norm if r['path']=='joint_active_sigma_normalization_controls']
center=next(r for r in active if r['scale']==1)['coscaled_mm_sigma_objective']
assert max(abs(r['coscaled_mm_sigma_objective']-center) for r in active)<1e-8
assert all(abs(r['fixed_mm_sigma_objective']/center-r['scale']**2)<1e-8 for r in active)
noisy_joint=[r for r in rows2 if r['noise']=='noisy_angle' and r['path']=='joint_geometry_radius_buildup']
center_unnormalized=next(r['numpy_objective'] for r in noisy_joint if r['scale']==1)
assert all(abs(r['numpy_objective']/center_unnormalized-r['scale']**4)<1e-8 for r in noisy_joint)
checks.append('E2 clean symmetry, active positive controls, finite coverage, noisy homogeneity and corrected normalization discriminator')

rows3=[]
for c in e3['cases']:
    deriv=[]
    for d in c['derivatives']:
        assert d['finite_entries']==d['rows']*d['columns']
        assert d['numerical_rank_at_relative_1e-7']==13
        assert d['augmented_rank_at_relative_1e-7']==14
        deriv.append({k:d[k] for k in ['central_step','rows','columns','finite_entries',
            'rms_normalized_singular_values','fixed_buildup_singular_values',
            'augmented_sigma_normalized_singular_values','numerical_rank_at_relative_1e-7',
            'augmented_rank_at_relative_1e-7','scale_direction_response_mm_per_relative_scale',
            'radius_scale_slope_z_per_relative_scale']})
    rows3.append({'dataset':c['dataset'],'derivatives':deriv,'scale_controls':c['scale_controls']})
checks.append('E3 finite Jacobians, data rank 13 and augmented rank 14 at both central-difference steps')
base=rows3[0]['derivatives'][1]['rms_normalized_singular_values']
duplicate=rows3[1]['derivatives'][1]['rms_normalized_singular_values']
diverse=rows3[2]['derivatives'][1]['rms_normalized_singular_values']
assert max(abs(a-b) for a,b in zip(base,duplicate))<1e-8
checks.append('E3 duplicated-observation control preserves RMS-normalized singular values')
summary={'status':'complete, unreviewed','source_revision':manifest['git_revision'],
    'machine':manifest['machine'],'cpu_model':manifest['cpu_model'],
    'versions':{k:manifest[k] for k in ['python','python_executable','numpy','jax','jax_backend','jax_x64']},
    'analysis_sha256':manifest['analysis_sha256'],'inputs_sha256':manifest['inputs_sha256'],
    'units':manifest['units'],'allocation_used':manifest['allocation_used'],
    'current_numerical_seconds':sum(e['elapsed_seconds'] for e in [e1,e2,e3]),
    'E1':rows1,'E2':rows2,'E2_normalization_controls':norm,'E3':rows3,
    'diverse_vs_base_smallest_nonnull_singular_value_ratio':diverse[-2]/base[-2],
    'validation_checks_passed':checks,
    'preserved_initial_trial':{'results':'E2-initial-floor-limited.json.gz','script':'analysis-v1.py',
        'manifest':'manifest-E2-initial-floor-limited.json',
        'raw_result_key_in_original_manifest':'E2.json',
        'interpretation':'floor-limited normalization comparison was inactive; corrected active-sigma discriminator is in current E2'},
    'scope':'Synthetic kinematic source sensitivity, plus frozen simulated HP3 observations. No fit, physics, collection, hardware accuracy or implemented fix.',
    'constraints':'Free common dimensionful buildup permits scale symmetry; fixed buildup/radii, objective noise floors, priors, nominal spool parameters and bounds are separate constraints.'}
save('summary.json',summary)

compressed=[]
large_names={p.name for p in out.glob('*.json') if p.stat().st_size>limit}
large_names.update(p.name[:-3] for p in out.glob('*.json.gz'))
for name in sorted(large_names):
    p=out/name
    raw=p.read_bytes() if p.exists() else gzip.decompress(p.with_suffix(p.suffix+'.gz').read_bytes())
    buffer=io.BytesIO()
    with gzip.GzipFile(filename='',mode='wb',fileobj=buffer,mtime=0,compresslevel=9) as handle:
        handle.write(raw)
    packed=buffer.getvalue();dest=p.with_suffix(p.suffix+'.gz');dest.write_bytes(packed)
    assert gzip.decompress(packed)==raw
    assert len(packed)<=limit
    compressed.append({'raw':p.name,'raw_bytes':len(raw),'raw_sha256':sha(raw),
        'gzip':dest.name,'gzip_bytes':len(packed),'gzip_sha256':sha(packed),
        'gzip_mtime':0,'gzip_filename':'','decompression_verified':True})

selected=['analysis.py','analysis-v1.py','archive.py','inputs.json','summary.json',
    'E1.json','E2.json.gz','E3.json','E2-initial-floor-limited.json.gz',
    'manifest-all.json','manifest-E2-initial-floor-limited.json','jax-probe-provenance.json','source-links.json',
    'all.log','E2-initial-floor-limited.log','prepare.log','abstract.md','research.md','report.md','final-message.md']
assets=[]
for name in selected:
    p=out/name
    if not p.exists():continue
    data=p.read_bytes();assert len(data)<=limit,name
    assets.append({'path':name,'bytes':len(data),'sha256':sha(data)})
assert sum(a['bytes'] for a in assets)<8*limit
save('manifest-archive.json',{'per_file_limit_bytes':limit,'total_limit_bytes':8*limit,
    'compression':'gzip level 9, mtime=0, empty original filename; originals retained',
    'compressed_json':compressed,'recommended_assets':assets,
    'recommended_assets_bytes_excluding_this_manifest':sum(a['bytes'] for a in assets),
    'also_select':'manifest-archive.json','no_numerical_rerun_for_archiving':True})
print(json.dumps({'validation_checks_passed':len(checks),'compressed_files':compressed,
    'recommended_asset_bytes_excluding_manifest':sum(a['bytes'] for a in assets)}))
