import sys, numpy as np
sys.argv = [sys.argv[0], sys.argv[1], 'full.npz']
exec(open('attrib.py').read().split("if __name__")[0])
L = z['L']; M = z['model']; sw = z['sweep']
import io, contextlib
def q(Lx):
    with contextlib.redirect_stdout(io.StringIO()):
        r = fit(Lx, '')
    return r['anchors'][3, 2] - TRUE[3, 2], np.abs(r['anchors'] - TRUE).max()
print('measured', q(L))
for s in sorted(set(sw)):
    m = sw == s
    Lx = L.copy(); Lx[m] = M[m]
    Ly = M.copy(); Ly[m] = L[m]
    print('sweep', s, 'z~%.0f' % z['E'][m][:, 2].mean(), 'true lengths in this sweep only -> Dz %+.2f worst %.2f' % q(Lx), ' | measured only in this sweep -> Dz %+.2f worst %.2f' % q(Ly))
