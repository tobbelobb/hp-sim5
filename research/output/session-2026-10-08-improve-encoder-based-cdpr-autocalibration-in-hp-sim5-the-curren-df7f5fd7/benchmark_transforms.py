from pathlib import Path
import sys,json,time,copy,statistics
import numpy as np
sys.path.insert(0,str(Path.cwd()))
from autocal.tools.regress_calibration_logs import DATASETS
from autocal._autocal_common import _resolve_length_model_base_params
from autocal.dataset_roles import normalize_dataset_point_roles
from autocal.spool_model import build_spool_model_params,dataset_with_modeled_lengths
s=Path(__file__).resolve().parent
rows=[]
for spec in DATASETS:
    data=json.loads((Path('autocal/data/references')/(spec.name+'.json')).read_text())
    normalize_dataset_point_roles(data)
    before=copy.deepcopy(data)
    n=data['num_anchors'];settings=_resolve_length_model_base_params(data,num_anchors=n,base_radii_override=[30.0])
    params=build_spool_model_params(data,base_radii_mm=settings['base_radii_mm'],modeled_radii_mm=spec.true_radii,
        modeled_buildup_factor=[float(spec.buildup_factor)]*n,spool_to_motor_gearing_factor=settings['spool_to_motor_gearing_factor'],
        mechanical_advantage=settings['mechanical_advantage'],lines_per_spool=settings['lines_per_spool'],theta0_mode='zero')
    full=[];borrowed=[]
    for i in range(20):
        output={}
        for name,flag in ([('full',True),('borrowed',False)] if i%2==0 else [('borrowed',False),('full',True)]):
            start=time.perf_counter();output[name]=dataset_with_modeled_lengths(data,params,copy_metadata=flag)
            (full if flag else borrowed).append(time.perf_counter()-start)
        assert output['full']==output['borrowed']
    assert data==before
    rows.append(dict(name=spec.name,full_seconds=full,borrowed_seconds=borrowed,full_median=statistics.median(full),borrowed_median=statistics.median(borrowed),exact=True,input_unchanged=True))
(s/'E07/transforms.json').write_text(json.dumps(rows,indent=2))
for r in rows:print(r['name'],r['full_median'],r['borrowed_median'])
