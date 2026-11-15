import sys; sys.path.insert(0,'.')
from sw import *
import sw
def mixed(label,parts,EA=12000.,seed=None,ferr=0.0,noise=0.0,Aplan=TRUE,mgplan=MG):
    rng=np.random.default_rng(seed) if seed is not None else None
    sweeps=[];Ts=[]
    for sws,policy,val in parts:
        ds,T=dataset(sws,policy,val,EA,rng=rng,ferr=ferr,noise=noise,Aplan=Aplan,mgplan=mgplan); sweeps+=ds['sweeps']; Ts.append(T)
    ds={"machine_type":"hangprinter_3","num_anchors":4,"dimensions":3,"config":{},"sweeps":sweeps}
    seed_a=TRUE+np.array([[0,30,10],[-20,15,10],[20,-15,10],[0,0,30.]]); out=[]
    for off in (False,True):
        r=fit_pose_least_squares(ds,seed_a,fit_home_offsets=off,home_offset_prior_sigma_mm=HOME_OFFSET_PRIOR_SIGMA_MM,extra_initial_anchors=[TRUE])
        e=r['anchors']-TRUE; s,q=shape(e); out.append(f'raw {np.abs(e).max():5.2f} shape {s:5.2f} Dz {e[3,2]:+5.2f} rms {r["rms_mm"]:.3f} k {r["home_offsets_mm"][3]:+.2f}')
    print(f'{label:58s} N={len(sweeps):3d} | rigid {out[0]} | +offset {out[1]}')
H=[(0,0,-390),(1,0,-390),(2,0,-390)]; M=[(0,200,-190),(1,200,-190),(2,200,-190)]; Z=[(0,0,0),(1,0,0),(2,0,0)]
if __name__=='__main__':
    mixed('boot const2 + high const2',[(BOOT+H,'const',2.0)])
    mixed('boot const2 + high equal1',[(BOOT,'const',2.0),(H,'equal',1.0)])
    mixed('boot const2 + high,mid equal1',[(BOOT,'const',2.0),(H+M,'equal',1.0)])
    mixed('all equal1: boot+high',[(BOOT+H,'equal',1.0)])
    mixed('all equal1: boot+high+mid',[(BOOT+H+M,'equal',1.0)])
    mixed('all equal1: high+mid+zero (no boot)',[(H+M+Z,'equal',1.0)])
    mixed('  EA 20k',[(BOOT+H+M,'equal',1.0)],EA=20000.)
    mixed('  EA 8.5k',[(BOOT+H+M,'equal',1.0)],EA=8500.)
    rng=np.random.default_rng(3)
    mixed('  planned anchors off ~20mm',[(BOOT+H+M,'equal',1.0)],Aplan=TRUE+rng.normal(0,20,(4,3)))
    mixed('  planned mass +25%',[(BOOT+H+M,'equal',1.0)],mgplan=MG*1.25)
    for sd in (1,2,3):
        mixed(f'  force err ±0.36N enc noise 0.01 seed{sd}',[(BOOT+H+M,'equal',1.0)],seed=sd,ferr=0.36,noise=0.01)
    for sd in (1,2):
        mixed(f'  const2 +force err ±0.36N seed{sd}',[(BOOT+H+M,'const',2.0)],seed=sd,ferr=0.36,noise=0.01)
