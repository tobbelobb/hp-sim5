import copy,json,numpy as np
import autocal.predictive_validation as pv
from autocal.tests.test_ellipse_cost import _synthetic_dataset
from autocal.ellipse_cost import EllipseCostFunction
from autocal.filter_pass import _modeled_point_length
D,A=_synthetic_dataset();p=D['sweeps'][0]['data_points'][0];p.update(l_drive=10.,l_sensor=20.,l_drive_mu=100.,l_sensor_mu=200.,raw_angles_zero_tension_deg=[1.,2.,3.]);capture=[]
class Spy(EllipseCostFunction):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs);capture.append({'effective_use_noise_mean':self.use_noise_mean,'drive':float(self._packed_sweeps[0].l_drive[0]),'sensor':float(self._packed_sweeps[0].l_sensor[0])})
pv.EllipseCostFunction=Spy
for annotated in [False,True]:
 d=copy.deepcopy(D)
 if annotated:d['config']={'noise_model':{'project_zero_tension':True}}
 pv.score_future_sweeps(d,A,{'sweep_002','sweep_003'},prefer_zero_tension_angles=True,use_noise_mean=True)
 capture[-1]['annotated']=annotated
print(json.dumps({'heldout_path':capture,'position_objective_direct_value':_modeled_point_length(p,primary_key='l_drive',mean_key='l_drive_mu',use_noise_mean=True)},indent=2))
