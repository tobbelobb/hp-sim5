"""Audit authored USD transforms against the transform the scene loader reads.

Reimplements hp-sim-3d/app/scene/usdValueReaders.js readOrientationAttribute and
the 'xformOp:translate is the position' assumption, then compares each tagged
machine body with pxr's composed local transform.  Exits nonzero when a prim's
position or orientation differs beyond a small tolerance.
"""
from pathlib import Path
import math
import sys
import numpy as np
from pxr import Usd, UsdGeom, Gf, Sdf

ROOT = Path('/home/torbjorn/repos/hp-sim5')
SCENES = sorted((ROOT / 'public/usd_scenes').glob('*.usda'))
POS_TOL_MM = 1.0
ANG_TOL_DEG = 0.5


def js_quat_from_matrix_usda(values):
    return np.array([values.GetReal(), *values.GetImaginary()], float)  # (w,x,y,z)


def js_read_orientation(prim):
    """Mirror of readOrientationAttribute in usdValueReaders.js."""
    order_attr = prim.GetAttribute('xformOpOrder')
    order = list(order_attr.Get() or []) if order_attr else []
    if order:
        combined = np.array([1.0, 0.0, 0.0, 0.0])
        found = False
        for op in order:
            op = op.replace('!invert!', '')
            if op == 'xformOp:orient' or op.startswith('xformOp:orient:'):
                q = prim.GetAttribute(op)
                if q and q.Get() is not None:
                    combined = mul(combined, js_quat_from_matrix_usda(q.Get()))
                    found = True
            elif op.startswith('xformOp:rotate'):
                rot = prim.GetAttribute(op)
                if rot and rot.Get() is not None:
                    combined = mul(combined, euler_usd(op, rot.Get()))
                    found = True
        if found:
            return norm(combined)
    for op in ('xformOp:orient', 'xformOp:rotateXYZ', 'xformOp:rotateXZY', 'xformOp:rotateYXZ',
               'xformOp:rotateYZX', 'xformOp:rotateZXY', 'xformOp:rotateZYX'):
        attr = prim.GetAttribute(op)
        if attr and attr.Get() is not None:
            if op == 'xformOp:orient':
                return norm(np.array(attr.Get(), float))
            return norm(euler_usd(op, attr.Get()))
    return None


def mul(a, b):
    """Quaternion product a*b in (w,x,y,z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def norm(q):
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1.0, 0, 0, 0])


def axis_angle(axis, deg):
    h = math.radians(deg) / 2
    return np.array([math.cos(h), *(math.sin(h) * np.array(axis, float))])


def euler_usd(op, degrees):
    axes = {'X': (1, 0, 0), 'Y': (0, 1, 0), 'Z': (0, 0, 1)}
    order = op[len('xformOp:rotate'):][:3]
    q = np.array([1.0, 0, 0, 0])
    for name, deg in zip(order, degrees):
        # pxr's rotateXYZ matrix equals Rz * Ry * Rx in column-vector form,
        # matching the loader's combined = axisRotation * combined loop.
        q = mul(axis_angle(axes[name], deg), q)
    return q


def quat_from_matrix(m):
    # pxr GfMatrix4d uses the row-vector convention: translation is row 3,
    # and the column-vector rotation is the transpose of the upper-left 3x3.
    t = np.array(m, float)
    r = t[:3, :3].T
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2
    y = math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2
    z = math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2
    x = math.copysign(x, r[2, 1] - r[1, 2])
    y = math.copysign(y, r[0, 2] - r[2, 0])
    z = math.copysign(z, r[1, 0] - r[0, 1])
    return norm(np.array([w, x, y, z]))


def local_matrix(prim):
    return np.array(UsdGeom.Xformable(prim).GetLocalTransformation(), float)


def world_matrix(prim):
    chain = []
    cur = prim
    while cur and cur.GetPath() != Sdf.Path('/'):
        chain.append(cur)
        cur = cur.GetParent()
    m = np.eye(4)
    for node in reversed(chain):  # root -> ... -> prim
        # Row-vector convention: world = local * parent * ... * root.
        m = np.array(UsdGeom.Xformable(node).GetLocalTransformation(), float) @ m
    return m


def main():
    worst_pos = 0.0
    worst_ang = 0.0
    failures = []
    checked = 0
    for path in SCENES:
        stage = Usd.Stage.Open(str(path))
        for prim in stage.Traverse():
            tags_attr = prim.GetAttribute('ecs:tags')
            tags = list(tags_attr.Get() or []) if tags_attr else []
            if not tags:
                continue
            if not any(t in tags for t in ('Spool', 'Wheel', 'Anchor', 'Pinhole', 'Eyelet', 'Attachment')):
                continue
            tr = prim.GetAttribute('xformOp:translate')
            if not tr or tr.Get() is None:
                continue
            checked += 1
            sim_pos = np.array(tr.Get(), float)
            usd_m = world_matrix(prim)
            usd_pos = usd_m[3, :3]
            usd_q = quat_from_matrix(usd_m)
            sim_q = js_read_orientation(prim)
            if sim_q is None:
                sim_q = np.array([1.0, 0, 0, 0])
            dot = abs(float(np.dot(sim_q, usd_q)))
            ang = math.degrees(2 * math.acos(min(1.0, dot)))
            pos_err = 1000 * float(np.linalg.norm(usd_pos - sim_pos))
            worst_pos = max(worst_pos, pos_err)
            worst_ang = max(worst_ang, ang)
            if pos_err > POS_TOL_MM or ang > ANG_TOL_DEG:
                order = list(prim.GetAttribute('xformOpOrder').Get() or []) if prim.GetAttribute('xformOpOrder') else []
                failures.append((path.name, prim.GetName(), tags, pos_err, ang,
                                 sim_pos, usd_pos, order))

    print(f'checked {checked} tagged bodies in {len(SCENES)} scenes')
    print(f'worst position error {worst_pos:.3f} mm, worst orientation error {worst_ang:.3f} deg')
    for row in failures[:25]:
        scene, name, tags, pos_err, ang, sim_pos, usd_pos, order = row
        print(f'{scene} {name} {tags}: pos_err={pos_err:.1f} mm ang_err={ang:.2f} deg order={order}')
    print(f'{len(failures)} bodies disagree')
    if failures:
        print('FAIL: loader transform differs from composed USD transform')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
