import copy,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path.cwd()))
from autocal.ellipse_cost import EllipseCostFunction
from autocal.calibrate import build_anchor_initial_guess
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal._autocal_common import _resolve_length_model_base_params
from autocal.spool_model import build_spool_model_params,dataset_with_modeled_lengths
from autocal.theoretical_ellipse import anchors_matrix_to_opt_vec,anchor_opt_vec_to_matrix
from autocal.tools.regress_calibration_logs import DATASETS
OUT=Path(__file__).resolve().parent

def select(d,anchor_seed,model=None,target_fraction=.5):
 normalized=copy.deepcopy(d);normalize_dataset_point_roles(normalized)
 working=model(normalized) if model else normalized
 cost=EllipseCostFunction(working,use_flex=False,pointwise_filtering=False,sweep_wise_filtering=False,noise_normalized=False)
 x=anchors_matrix_to_opt_vec(anchor_seed,d['machine_type']);dims=d['dimensions'];n=d['num_anchors']
 keys=[(s['id'],i) for s in working['sweeps'] for i in range(len(s['data_points']))]
 def values(v):
  rows={(r['sweep_id'],r['point_idx']):r['residual_mm_signed'] for r in cost.pointwise_residual_rows(v)}
  return np.array([rows.get(k,0.) for k in keys])
 v0=values(x);expected=sum(len(s['data_points']) for s in working['sweeps'])
 valid_ids={r['sweep_id'] for r in cost.pointwise_residual_rows(x)}
 invalid_indices={i for i,k in enumerate(keys) if k[0] not in valid_ids}
 jac=np.column_stack([(values(x+np.eye(len(x))[i]) - values(x-np.eye(len(x))[i]))*.5 for i in range(len(x))])
 # Full-data whitening makes the criterion independent of parameter units.
 _,s,vt=np.linalg.svd(jac,full_matrices=False);rank=sum(s>s.max()*1e-8)
 z=jac@vt[:rank].T/np.maximum(s[:rank],1e-12)
 groups=[];offset=0
 for sweep in d['sweeps']:
  points=sweep['data_points'];g=[]
  for i,p in enumerate(points):
   if g and p.get('step_index',1)==0:groups.append(g);g=[]
   g.append(offset+i)
  if g:groups.append(g)
  offset+=len(points)
 targets=[min(len(g),max(5,int(np.ceil(len(g)*target_fraction)))) for g in groups]
 chosen={g[0] for g in groups}|{g[-1] for g in groups}
 for g,k in zip(groups,targets):
  if any(i in invalid_indices for i in g):chosen.update(g[i] for i in np.linspace(0,len(g)-1,k).round().astype(int))
 info=np.eye(rank)*1e-6
 for i in sorted(chosen):info+=np.outer(z[i],z[i])
 while any(len(chosen&set(g))<k for g,k in zip(groups,targets)):
  pool=[i for g,k in zip(groups,targets) if len(chosen&set(g))<k for i in g if i not in chosen]
  inv=np.linalg.inv(info);gain=np.einsum('ij,jk,ik->i',z[pool],inv,z[pool]);best=pool[int(np.argmax(gain))]
  chosen.add(best);info+=np.outer(z[best],z[best])
 result=copy.deepcopy(d);offset=0
 for sweep in result['sweeps']:
  points=sweep['data_points'];sweep['data_points']=[p for i,p in enumerate(points) if offset+i in chosen];offset+=len(points)
 return result,dict(full_points=expected,selected_points=len(chosen),seed=anchor_seed.tolist(),invalid_seed_points=len(invalid_indices),rank=int(rank),singular_values=s.tolist(),selected_indices=sorted(chosen),information_eigenvalues=np.linalg.eigvalsh(info).tolist(),method='greedy D-optimal whitened anchor residual Jacobian; endpoints and >=5 points per physical direction; retrospective observations')

if __name__=='__main__':
 records=[]
 for spec in DATASETS:
  if spec.name not in sys.argv[1:]:continue
  raw=json.loads(Path(f'autocal/data/references/{spec.name}.json').read_text())
  r0=39.5 if spec.machine_type=='hangprinter_4' else 37.5
  seed=build_anchor_initial_guess(dict(raw,sweeps=raw['sweeps'][:3]),base_radii_mm=[r0]*raw['num_anchors'])
  params=_resolve_length_model_base_params(raw,num_anchors=raw['num_anchors'],base_radii_override=[float(spec.base_radii)])
  def model(d):return dataset_with_modeled_lengths(d,build_spool_model_params(d,**params,modeled_radii_mm=[r0]*d['num_anchors'],modeled_buildup_factor=[float(spec.buildup_factor)]*d['num_anchors']))
  try:
   data,record=select(raw,seed,model)
   target=OUT/'E03-informed'/spec.name;target.mkdir(parents=True,exist_ok=True);(target/f'{spec.name}.json').write_text(json.dumps(data))
   record.update(dataset=spec.name,radius_seed_mm=r0)
  except Exception as e:record=dict(dataset=spec.name,failure=repr(e))
  records.append(record);print(json.dumps(record),flush=True)
 (OUT/'E03-selected-points.json').write_text(json.dumps(records,indent=2))
