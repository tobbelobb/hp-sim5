import numpy as np
from python.vector3 import vec3, length, dot, cross

def test_vec3_helpers():
    v = vec3(1,2,2)
    assert length(v) == np.sqrt(9)
    w = vec3(0,1,0)
    assert dot(v,w) == 2
    c = cross(v,w)
    assert np.allclose(c, [ -2,0,1 ])
