import json,numpy as np
from pathlib import Path
r=Path('/workspace/hp-sim5/research/sessions/attempt-8c3c74b7641e45c2acd1cd976f572e08/scratchbook'); t={6301:[[0,-1900,-280],[1645.448267190434,950,-280],[-1645.448267190434,950,-280],[0,0,1900]],7301:[[0,-2000,-280],[1725.448267190434,950,-280],[-1715.448267190434,950,-280],[0,0,1980]],8301:[[0,-1840,-280],[1555.448267190434,950,-280],[-1565.448267190434,950,-280],[0,0,1800]]}
for n,truth in t.items():
 a=json.load(open(r/f'pointellipse{n}.json'))['anchors'];print(n,np.linalg.norm(np.array(a)-truth,axis=1).round(3).tolist())
