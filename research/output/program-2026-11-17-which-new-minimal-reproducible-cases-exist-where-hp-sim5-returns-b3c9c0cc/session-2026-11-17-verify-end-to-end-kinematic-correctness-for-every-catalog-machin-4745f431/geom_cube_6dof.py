#!/usr/bin/env python3
"""Pure 6-DOF geometric prediction for the CubeCorners effector.

Given the CubeCorners cable attachment layout and the per-motor line-length
changes (from the RRF step stream, with the simulator's effective spool
radius), solve for the rigid-body pose that keeps all eight cable paths
inextensible.  This is the geometric prediction that is independent of the
physics solver.

    .venv/bin/python geom_cube_6dof.py '{"R_eff":0.039,"steps":{"A":-32,...}}'

Prints JSON with the solution for the quantised step stream ("actual") and
for continuous, unquantised line changes ("ideal").  The difference isolates
the effect of microstep rounding.
"""
import json
import sys

import numpy as np
from scipy.optimize import least_squares

# Authored geometry from public/usd_scenes/cubecorners.usda (metres).
ANCHORS = {
    'A': (1.4849, -1.4849, 2.4), 'B': (1.4849, 1.4849, 2.4),
    'C': (-1.4849, 1.4849, 2.4), 'D': (-1.4849, -1.4849, 2.4),
    'I': (1.4849, -1.4849, -0.18), 'J': (1.4849, 1.4849, -0.18),
    'L': (-1.4849, 1.4849, -0.18), 'O': (-1.4849, -1.4849, -0.18),
}
ATTACH = {
    'A': (0.2, -0.2, 0.5), 'B': (0.2, 0.2, 0.5),
    'C': (-0.2, 0.2, 0.5), 'D': (-0.2, -0.2, 0.5),
    'I': (0.2, -0.2, 0.1), 'J': (0.2, 0.2, 0.1),
    'L': (-0.2, 0.2, 0.1), 'O': (-0.2, -0.2, 0.1),
}
CENTER = np.array([0.0, 0.0, 0.3])
TIP_LOCAL = np.array([0.0, 0.0, -0.3])
ANCHORS = {k: np.array(v) for k, v in ANCHORS.items()}
LOCAL = {k: np.array(v) - CENTER for k, v in ATTACH.items()}
L0 = {k: np.linalg.norm(LOCAL[k] + CENTER - ANCHORS[k]) for k in ANCHORS}
U = {k: (ANCHORS[k] - (LOCAL[k] + CENTER)) / L0[k] for k in ANCHORS}


def rotvec_to_R(w):
    theta = np.linalg.norm(w)
    if theta < 1e-12:
        return np.eye(3)
    k = w / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def solve(target):
    def resid(x):
        t = x[:3]
        R = rotvec_to_R(x[3:])
        return np.array([
            np.linalg.norm(R @ LOCAL[k] + CENTER + t - ANCHORS[k]) - target[k]
            for k in ANCHORS
        ])

    sol = least_squares(resid, np.zeros(6), xtol=1e-15, ftol=1e-15, gtol=1e-15)
    R = rotvec_to_R(sol.x[3:])
    tip = R @ TIP_LOCAL + CENTER + sol.x[:3]
    return {
        'translation_mm': (sol.x[:3] * 1000).round(6).tolist(),
        'rotation_axis_angle_rad': sol.x[3:].round(12).tolist(),
        'rotation_angle_deg': round(float(np.degrees(np.linalg.norm(sol.x[3:]))), 8),
        'tip_mm': ((tip - np.array([0.0, 0.0, 0.0])) * 1000).round(6).tolist(),
        'residuals_mm': (sol.fun * 1000).round(6).tolist(),
        'max_residual_mm': round(float(np.max(np.abs(sol.fun)) * 1000), 6),
    }


def main():
    data = json.loads(sys.argv[1])
    r_eff = float(data['R_eff'])
    steps = {k: float(v) for k, v in data['steps'].items()}
    move = data.get('move_mm')
    axis = data.get('axis', 'X')
    axis_index = 'XYZ'.index(axis)

    # Quantised step stream: delta stored length = -R * delta_angle (the
    # simulator's winding sign for these paths), so the free length changes
    # by +R * delta_angle.  target = L0 + free change.
    actual_target = {
        k: L0[k] + r_eff * steps[k] * 2.0 * np.pi / 3200.0 for k in ANCHORS
    }
    out = {'R_eff_mm': r_eff * 1000, 'steps': steps, 'actual': solve(actual_target)}

    if move is not None:
        mm = float(move)
        # Continuous line change for a pure axis translation: dFree = -u . dp.
        ideal_target = {k: L0[k] - (mm * 1e-3) * U[k][axis_index] for k in ANCHORS}
        out['ideal'] = solve(ideal_target)
        out['ideal']['translation_mm'][axis_index] = round(mm, 6)

    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
