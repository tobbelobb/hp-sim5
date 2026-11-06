import json,sys,numpy as np
from pathlib import Path
p=Path(sys.argv[1]);rows=[]
with (p/'sweeps.headless/events.jsonl').open() as f:
 for line in f:
  d=json.loads(line)
  if d.get('type')=='encoder_response' and len(d.get('angles_deg',[]))==4:rows.append((d['sim_time_s'],d['angles_deg']))
end=max(t for t,_ in rows);r=[(t,a) for t,a in rows if end-30.1<=t<=end];x=np.array([a for t,a in r]);late=[(t,a) for t,a in r if t>=end-5];xx=np.array([a for t,a in late]);early=np.array([a for t,a in late if t<=end-2.5]);late2=np.array([a for t,a in late if t>=end-2.5]);print(json.dumps({'last_time_s':end,'samples_30s':len(r),'full_30s_range_deg':np.ptp(x,axis=0).tolist(),'last_5s_samples':len(late),'last_5s_range_deg':np.ptp(xx,axis=0).tolist(),'last_5s_early_to_late_mean_change_deg':(late2.mean(axis=0)-early.mean(axis=0)).tolist(),'first_values':x[0].tolist(),'last_values':x[-1].tolist(),'last_30s_first_to_last_deg':(x[-1]-x[0]).tolist()},indent=2))
