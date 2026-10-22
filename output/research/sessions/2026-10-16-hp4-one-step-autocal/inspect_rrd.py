from pathlib import Path
import numpy as np
from rerun.chunk import RrdReader
p=Path(__file__).parent
chunks=RrdReader(str(next((p/'browser').glob('*.rrd')))).stream().to_chunks()
rows=[]
for c in chunks:
 if str(c.entity_path)!='/world/machines/machine-0/bodies/Effector_58':continue
 b=c.to_record_batch()
 if 'Transform3D:translation' not in b.schema.names:continue
 ts=b.column('sim_time').cast('int64').to_pylist()
 xyz=b.column('Transform3D:translation').to_pylist()
 q=b.column('Transform3D:quaternion').to_pylist() if 'Transform3D:quaternion' in b.schema.names else [None]*len(ts)
 rows.extend([t/1e9,*v[0],*(rot[0] if rot else [float('nan')]*4)] for t,v,rot in zip(ts,xyz,q) if v)
a=np.array(sorted(rows,key=lambda r:r[0]));np.save(p/'browser-body.npy',a)
print('shape',a.shape,'last',a[-1]);
for seconds in (120,30,10):
 s=a[a[:,0]>=a[-1,0]-seconds];print('last',seconds,'first',s[0],'range',np.ptp(s[:,1:],axis=0))
print('other paths', '\n'.join(sorted({str(c.entity_path) for c in chunks if 'autocal' in str(c.entity_path)})[:15]))
