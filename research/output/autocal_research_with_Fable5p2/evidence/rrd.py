import sys, numpy as np, pickle, collections
from rerun.chunk import RrdReader
chunks=RrdReader(sys.argv[1]).stream().to_chunks()
data=collections.defaultdict(list); names={}
for c in chunks:
    ep=str(c.entity_path)
    if not (ep.startswith('/line_') or ep.startswith('/cable_forces') or '/bodies/' in ep): continue
    b=c.to_record_batch(); cols=b.schema.names
    if 'sim_time' not in cols:
        for n in cols:
            if 'names' in n.lower(): names[ep]=b.column(n).to_pylist()
        continue
    ts=np.array(b.column('sim_time').cast('int64').to_pylist())/1e9
    for n in cols:
        if n.endswith('Scalars:scalars') or n.endswith('Transform3D:translation') or n.endswith('Transform3D:quaternion'):
            for t,v in zip(ts,b.column(n).to_pylist()):
                if v is not None: data[(ep,n.split(':')[-1])].append((t,np.ravel(v)))
out={}
for k,v in data.items():
    v.sort(key=lambda r:r[0]); m=max(len(r[1]) for r in v); out[k]=(np.array([r[0] for r in v]),np.array([np.pad(r[1].astype(float),(0,m-len(r[1])),constant_values=np.nan) for r in v]))
pickle.dump((out,names),open(sys.argv[2],'wb'))
for k,(t,v) in sorted(out.items()): print(k,t.shape,v.shape,t[0],t[-1],names.get(k[0]))
