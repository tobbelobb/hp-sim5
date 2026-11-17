import copy,json,sys,re
from pathlib import Path
import numpy as np
from autocal.pose_fit import fit_pose_least_squares,shift_dataset_home_offsets
HERE=Path(__file__).resolve().parent
truth=np.array([[0,-1900,-280],[1900,0,-280],[0,1900,-280],[-1900,0,-280],[0,0,1900.]])
def error(a):
 delta=np.array(a)-truth
 return dict(max_coordinate_error_mm=float(abs(delta).max()), aligned_max_coordinate_error_mm=float(abs(delta-delta.mean(axis=0)).max()))
result={}
for variant in (sys.argv[1:] or ['direct','reduced']):
 dataset=HERE/variant/'sweeps.json'
 data=json.loads(dataset.read_text())
 m=json.loads((dataset.parent/'manifest.json').read_text())
 points=[point for sweep in data['sweeps'] for point in sweep['data_points']]
 extras=[extra for point in points for extra in point.get('extra_sensor_ranges',[])]
 verified=dict(points=len(points),accepted=sum(e['accepted'] for e in extras),stop_reason=m['stop_reason'],simulated_s=m['simulated_s'])
 command=m['applied_parameters'][0]
 a=np.array([[float(v) for v in re.search(r'\b'+axis+r'([-+\d.eE:]+)',command)[1].split(':')] for axis in 'ABCDI'])
 verified.update(error(a));verified['applied_m669']=command
 verified['sweeps']=len(data['sweeps']); verified['wall_s']=m['wall_s']
 ablation={}
 for ranges in (4,5):
  d=copy.deepcopy(data)
  if ranges==4:
   for sweep in d['sweeps']:
    for point in sweep['data_points']:
     for extra in point.get('extra_sensor_ranges',[]):extra['accepted']=False
  fit=fit_pose_least_squares(d,truth*1.02,fit_home_offsets=True,home_offset_prior_sigma_mm=5.)
  refined=fit_pose_least_squares(shift_dataset_home_offsets(d,fit['home_offsets_mm']),fit['anchors'],fit_home_offsets=False,home_offset_prior_sigma_mm=5.)
  ablation[ranges]={**error(refined['anchors']),'rms_mm':refined['rms_mm'],'success':bool(fit['success'] and refined['success'])}
 verified['same_recording_ablation']=ablation;result[variant]=verified
 print(variant,json.dumps(verified,indent=2),flush=True)

