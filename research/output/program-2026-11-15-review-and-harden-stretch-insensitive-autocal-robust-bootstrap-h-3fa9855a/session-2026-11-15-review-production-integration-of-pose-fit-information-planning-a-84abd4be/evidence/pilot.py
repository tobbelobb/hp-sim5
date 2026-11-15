"""Synthetic counterexamples, run from repo root with .venv/bin/python <this file>."""
import json
import numpy as np
from autocal.active_learning import SweepConfig
from autocal.equal_strain import equal_strain_tensions, sensor_force_profiles
from autocal.pose_planning import _fixed_length_circle, predict_sweep_positions

hp = np.array([[0.,-1900,-280],[1645.4,950,-280],[-1645.4,950,-280],[0,0,1900]])
opts=dict(strand_tension_n=1., mover_weight_n=.081*9.81, strands=np.array([2.,2,2,3]), extra_line_length_mm=np.array([528.3,528.3,528.3,398.9]), minimum_force_n=.4)
a,b = (SweepConfig((0,3),(390.,d),1,2) for d in (0.,-150.))
p,q = (sensor_force_profiles(hp,c,**opts)[0] for c in (a,b))
lo=max(min(p['drive_mm']),min(q['drive_mm'])); hi=min(max(p['drive_mm']),max(q['drive_mm']))
x=np.linspace(lo,hi,1000)
diff=np.abs(np.interp(x,p['drive_mm'],p['force_n'])-np.interp(x,q['drive_mm'],q['force_n']))
print('same pair, different fixed target:', json.dumps({'overlap_mm':[lo,hi], 'max_force_difference_N':float(max(diff)), 'at_drive_mm':float(x[diff.argmax()])}))
sky=np.array([[-2000.,-2000,3000],[2000,-2000,3000],[2000,2000,3000],[-2000,2000,3000]])
cfg=SweepConfig((0,1),(0.,0.),2,3)
pos=predict_sweep_positions(sky,cfg,20,.05)
t=equal_strain_tensions(sky,np.zeros(3),[0,1,2,3],strand_tension_n=1.,mover_weight_n=1.,strands=np.ones(4),extra_line_length_mm=np.zeros(4))
print('synthetic skycam:',json.dumps({'predicted_positions':len(pos),'home_tensions_N':t.tolist()}))
cube=np.array([[x,y,z] for x in (-1000.,1000.) for y in (-1000.,1000.) for z in (-1000.,1000.)])
cc=SweepConfig((0,1,2,3,4,5),(0.,0.,0.,0.,0.,0.),6,7)
curve=_fixed_length_circle(cube,cc)
res=np.linalg.norm(curve[:,None,:]-cube[None,:6,:],axis=2)-np.linalg.norm(cube[:6],axis=1)
print('synthetic cube fixed lengths:',json.dumps({'circle_points':len(curve),'max_abs_error_per_fixed_line_mm':np.max(np.abs(res),axis=0).tolist()}))
print('skycam achieved mean strand tension N:',float(np.mean(t)))
