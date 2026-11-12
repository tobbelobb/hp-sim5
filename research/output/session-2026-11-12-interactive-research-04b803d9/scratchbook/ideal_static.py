# Ideal static response of the firmware's own flex model: point effector, N_i parallel lines with MA_i
# segments to point anchors, elastic length MA*d+Y, EA=20000, unilateral lines, gravity on 0.081 kg.
import sys, numpy as np
from scipy.optimize import least_squares
EA, m, g = 20000., 0.081, 9.81
MA = np.array([2, 2, 2, 4.]); N = np.array([2, 2, 2, 3.]); Y = np.array([.5283, .5283, .5283, .3989])
anch = np.array([[0, -1.9, -.28], [1.64544826719, .95, -.28], [-1.64544826719, .95, -.28], [0, 0, 1.9]])
d0 = np.linalg.norm(anch, axis=1)
L = MA * d0 + Y
# initial rest lengths: home equilibrium with D carrying gravity, ABC exactly taut (zero tension), as in the sim
T0 = np.array([0, 0, 0, m * g / (MA[3] * N[3])])
rest0 = L - T0 * L / EA
def tensions(p, rest):
    geo = MA * np.linalg.norm(anch - p, axis=1) + Y
    return np.maximum(0, EA * (geo - rest) / rest)
def resid(p, rest):
    u = (anch - p) / np.linalg.norm(anch - p, axis=1)[:, None]
    F = (tensions(p, rest) * MA * N)[:, None] * u
    return F.sum(0) + [0, 0, -m * g]
def solve(lp_mm):
    rest = rest0 + MA * np.array(lp_mm) * 1e-3   # lp<0 shortens; reel = MA*lp of line
    sol = least_squares(resid, np.zeros(3), args=(rest,), xtol=1e-15, ftol=1e-15, gtol=1e-15)
    return sol.x * 1e3, tensions(sol.x, rest)
for lp in [[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, -1], [-.56587, -.56587, -.56587, -.08332], [-1.13174, -1.13174, -1.13174, -.33326]]:
    p, T = solve(lp)
    print(lp, 'disp mm', np.round(p, 4), '|xy| %.4f ang %.1f' % (np.hypot(*p[:2]), np.degrees(np.arctan2(p[1], p[0]))), 'T/line', np.round(T, 3))
