"""Exact-ish HP4 line geometry from scene points, for a rigid effector pose (c, q)."""
import re
import numpy as np
from scipy.spatial.transform import Rotation

def scene_points(path):
    text = open(path).read()
    pts, rad = {}, {}
    for m in re.finditer(r'def (?:Circle|Xform) "(\w+)"[^{]*\{(.*?)\n        \}', text, re.S):
        name, body = m.group(1), m.group(2)
        t = re.search(r'double3 xformOp:translate = \(([^)]*)\)', body)
        if t: pts[name] = np.array([float(v) for v in t.group(1).split(',')]) * 1000
        r = re.search(r'double radius = ([\d.eE+-]+)', body)
        if r: rad[name] = float(r.group(1)) * 1000
    return pts, rad

class HP4:
    def __init__(self, path, home_com):
        self.p, self.r = scene_points(path)
        self.home = np.asarray(home_com, float)
        names = [n for n in self.p if n.startswith(('Pinhole', 'Wheel')) and ('bottom' in n and 'D' in n or not n.startswith('WheelD')) and 'top' not in n or n.startswith('Pinhole') or (n.startswith('Wheel') and not n.startswith('WheelD'))]
        self.moving = {n for n in self.p if n.startswith(('Pinhole',)) or (n.startswith('Wheel') and (not n.startswith('WheelD') or n.endswith('bottom')))}
    def world(self, name, c, R):
        x = self.p[name]
        return c + R @ (x - self.home) if name in self.moving else x
    def lengths(self, e, q):
        """Per-axis free length per strand-equivalent (mm, like |anchor-effector|), per line."""
        R = Rotation.from_quat(q).as_matrix() if q is not None else np.eye(3)
        c = self.home + e
        w = lambda n: self.world(n, c, R)
        out = {}
        for ax in 'ABC':
            lines = []
            for side in 'LR':
                s = f'{ax}{side}'
                lines.append(np.linalg.norm(w(f'Anchor{s}_top') - w(f'Pinhole{s}_top')) + np.linalg.norm(w(f'Pinhole{s}_bottom') - w(f'Anchor{s}_bottom')))
            out[ax] = np.array(lines) / 2
        lines = []
        r = 6.5
        for i in range(3):
            P = [f'WheelD{i}_left_top', f'WheelD{i}_left_bottom', f'WheelD{i}_right_top', f'WheelD{i}_right_bottom', f'AttachD{i}']
            X = [w(n) for n in P]
            L = np.linalg.norm(X[1] - X[0]) + np.sqrt(np.sum((X[2] - X[1])**2) - 4*r*r) + np.linalg.norm(X[3] - X[2]) + np.sqrt(np.sum((X[4] - X[3])**2) - r*r)
            # wraps: turn angles at the 4 wheels using centre-line directions
            spool = self.p['SpoolD']
            dirs = [X[0] - spool] + [X[k+1] - X[k] for k in range(4)]
            dirs = [d / np.linalg.norm(d) for d in dirs]
            for k in range(4):
                L += r * np.arccos(np.clip(dirs[k] @ dirs[k+1], -1, 1))
            lines.append(L / 4)
        out['D'] = np.array(lines)
        return out

class HP3:
    LINES = {'A': [('AnchorAL', 'AttachAL'), ('AnchorAR', 'AttachAR')], 'B': [('AnchorBL', 'AttachAR'), ('AnchorBR', 'AttachBR')],
             'C': [('AnchorCL', 'AttachBR'), ('AnchorCR', 'AttachAL')], 'D': [('AnchorD0', 'AttachAL'), ('AnchorD1', 'AttachAR'), ('AnchorD2', 'AttachBR')]}
    def __init__(self, path, home_com):
        self.p, self.r = scene_points(path); self.home = np.asarray(home_com, float)
    def lengths(self, e, q):
        R = Rotation.from_quat(q).as_matrix() if q is not None else np.eye(3)
        c = self.home + e
        return {ax: np.array([np.linalg.norm(self.p[a] - (c + R @ (self.p[m] - self.home))) for a, m in lines]) for ax, lines in self.LINES.items()}
