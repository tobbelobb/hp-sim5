import json,numpy as np,sys
from pathlib import Path
p=Path(sys.argv[1]); d=json.load(open(p/'sweeps.json')); out=[]
for s in d['sweeps']:
 pts=s['data_points'];axes=s['fixed_anchors']; vals=np.array([p['mu'] for p in pts])[:,axes]*np.pi/6; delta=vals-np.array(s['fixed_lengths']); out.append(dict(id=s['id'],axes=axes,mean=delta.mean(axis=0).tolist(),std=delta.std(axis=0).tolist(),span=np.ptp(vals,axis=0).tolist(),sigma_deg_median=np.median(np.array([p['sigma'] for p in pts])[:,axes],axis=0).tolist()))
print(json.dumps(out,indent=2))
