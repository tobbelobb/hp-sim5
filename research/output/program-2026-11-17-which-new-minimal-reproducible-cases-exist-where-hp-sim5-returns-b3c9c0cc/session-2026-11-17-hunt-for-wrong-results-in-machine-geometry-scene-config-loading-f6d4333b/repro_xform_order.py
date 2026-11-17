"""Compare USD-correct prim placement with the placement the simulator uses.

The simulator (hp-sim-3d/app/scene/machineEntityBuilders.js and the baker in
src/js/usd/cable_scene_baker.js) reads xformOp:translate directly as the prim's
local position and ignores xformOpOrder.  USD composes the listed ops, so for
xformOpOrder = ["xformOp:rotateXYZ", "xformOp:translate"] the translation is
itself rotated about the parent origin (M = R * T).

For every tagged machine body in HP4 and HP5-pulley-reduced this script prints
the USD world position (pxr) and the position the simulator would use, and exits
nonzero when they disagree by more than 1 mm.
"""
from pathlib import Path
import sys
import numpy as np
from pxr import Usd, UsdGeom, Gf

ROOT = Path('/home/torbjorn/repos/hp-sim5')
SCENES = ['public/usd_scenes/hp4.usda', 'public/usd_scenes/hp4_worn_bearings.usda',
          'public/usd_scenes/hp5_pulley_reduced.usda']
TOL_MM = 1.0


def world_matrix(prim):
    """USD local-to-world as the product of local transforms down from the root."""
    chain = []
    cur = prim
    while cur and cur.GetPath() != '/':
        chain.append(cur)
        cur = cur.GetParent()
    matrix = Gf.Matrix4d(1.0)
    for node in reversed(chain):
        matrix = matrix * UsdGeom.Xformable(node).GetLocalTransformation()
    return matrix


def main():
    worst = 0.0
    failures = []
    for rel in SCENES:
        stage = Usd.Stage.Open(str(ROOT / rel))
        for prim in stage.Traverse():
            tags_attr = prim.GetAttribute('ecs:tags')
            tags = tags_attr.Get() if tags_attr else None
            if not tags:
                continue
            tags = list(tags)
            kind = next((t for t in ('Wheel', 'Pinhole', 'Spool', 'Anchor') if t in tags), None)
            if kind is None:
                continue
            tr = prim.GetAttribute('xformOp:translate')
            if not tr or tr.Get() is None:
                continue
            authored = np.array(tr.Get(), float)
            usd_world = np.array(world_matrix(prim).ExtractTranslation(), float)
            # Simulator: local translate is the position (scene root has no transform).
            sim_world = authored
            err_mm = 1000.0 * float(np.linalg.norm(usd_world - sim_world))
            worst = max(worst, err_mm)
            if err_mm > TOL_MM:
                order = prim.GetAttribute('xformOpOrder').Get()
                failures.append((rel, prim.GetName(), kind, err_mm, authored, usd_world, list(order or [])))

    print(f'checked {len(SCENES)} scenes; worst placement disagreement {worst:.3f} mm')
    for rel, name, kind, err, authored, usd, order in failures[:40]:
        print(f'{rel} {name} [{kind}] error={err:.1f} mm')
        print(f'   translate={np.round(authored,6).tolist()} order={order}')
        print(f'   sim={np.round(authored,6).tolist()} usd={np.round(usd,6).tolist()}')
    print(f'{len(failures)} prims disagree with USD by > {TOL_MM} mm')
    if failures:
        print('FAIL: authored xformOpOrder and simulator placement disagree')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
