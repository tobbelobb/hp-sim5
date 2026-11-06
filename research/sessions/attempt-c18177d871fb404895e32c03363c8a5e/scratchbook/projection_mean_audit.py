import json
from autocal.ellipse_cost import EllipseCostFunction
from autocal.tests.test_ellipse_cost import _synthetic_dataset
x,_=_synthetic_dataset();p=x['sweeps'][0]['data_points'][0];p.update(l_drive=10.,l_sensor=20.,l_drive_mu=100.,l_sensor_mu=200.,raw_angles_zero_tension_deg=[1.,2.,3.]);results={}
for flag in [True,False]:
 c=EllipseCostFunction(x,use_noise_mean=flag);a,b,_,_=c._extract_point_arrays([p],1,2);results[str(flag)]={'drive':float(a[0]),'sensor':float(b[0])}
print(json.dumps(results,indent=2))
