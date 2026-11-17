"""Compare prefix fits with/without extra ranges; no simulation or production edits.

Use production data-derived seeds and its joint-offset then shifted-range fit.
Both arms retain identical recorded loads and trajectories. Nominal anchors are
used only to score results. This does not replay adaptive planning or stopping.
"""
import copy,json,sys,time
from pathlib import Path
import numpy as np
from autocal.calibrate import build_anchor_initial_guess
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.pose_fit import fit_pose_least_squares,shift_dataset_home_offsets
HERE=Path(__file__).resolve().parent
TRUTH=np.array([[0.,-1900.,-280.],[1900.,0.,-280.],[0.,1900.,-280.],[-1900.,0.,-280.],[0.,0.,1900.]])

def score(anchors):
    delta=np.asarray(anchors)-TRUTH
    return dict(abs_mm=float(abs(delta).max()),aligned_mm=float(abs(delta-delta.mean(axis=0)).max()))

def main(variant):
    data=json.loads((HERE.parent/variant/'sweeps.json').read_text())
    normalize_dataset_point_roles(data)
    results=[]
    for count in range(3,len(data['sweeps'])+1):
        prefix=copy.deepcopy(data);prefix['sweeps']=prefix['sweeps'][:count]
        seed=build_anchor_initial_guess(prefix,machine_type=prefix['machine_type'],base_radii_mm=None,low_anchor_z=None)
        for ranges in (4,5):
            dataset=copy.deepcopy(prefix)
            if ranges==4:
                for sweep in dataset['sweeps']:
                    for point in sweep['data_points']:
                        for extra in point.get('extra_sensor_ranges',[]):extra['accepted']=False
            started=time.monotonic()
            try:
                offsets=fit_pose_least_squares(dataset,seed,fit_home_offsets=True,home_offset_prior_sigma_mm=5.)
                fitted=fit_pose_least_squares(shift_dataset_home_offsets(dataset,offsets['home_offsets_mm']),offsets['anchors'],fit_home_offsets=False,home_offset_prior_sigma_mm=5.)
                result=dict(sweeps=count,points=sum(len(s['data_points']) for s in prefix['sweeps']),ranges=ranges,
                    success=bool(offsets['success'] and fitted['success']),rms_mm=float(fitted['rms_mm']),
                    **score(fitted['anchors']))
            except (ValueError,np.linalg.LinAlgError) as error:
                result=dict(sweeps=count,ranges=ranges,success=False,error=str(error))
            result['fit_wall_s']=time.monotonic()-started;results.append(result)
            print(variant,json.dumps(result),flush=True)
            (HERE/f'{variant}.json').write_text(json.dumps(dict(variant=variant,results=results),indent=2)+'\n')

if __name__=='__main__':main(sys.argv[1])
