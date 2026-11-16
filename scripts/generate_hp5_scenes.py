"""Build square HP5 movers using the production HP3/HP4 routing templates."""
from pathlib import Path
import math
import re

from pxr import Gf, Sdf, Usd

ROOT = Path(__file__).resolve().parents[1]
BASE = '/World/HangprinterScene/'
CORNERS = [(-.25,-.25,.1), (.25,-.25,.1), (.25,.25,.1), (-.25,.25,.1)]
NAMES = ['CornerSW','CornerSE','CornerNE','CornerNW']


def rotate(vector, angle):
    c, s = math.cos(angle), math.sin(angle)
    x,y,z = vector
    return (c*x-s*y, s*x+c*y, z)


def build(reduced):
    source = Usd.Stage.Open(str(ROOT / f'public/usd_scenes/{"hp4" if reduced else "hp3"}.usda'))
    layer = Sdf.Layer.CreateAnonymous('.usda')
    layer.ImportFromString(source.GetRootLayer().ExportToString())
    stage = Usd.Stage.Open(layer)
    root = stage.GetPrimAtPath(BASE[:-1])
    for prim in list(root.GetChildren()):
        stage.RemovePrim(prim.GetPath())
    machine = 'hp5_pulley_reduced' if reduced else 'hp5'
    root.GetAttribute('machine:catalog').Set(machine)
    layer.documentation = f'Square Hangprinter 5, {"HP4 reeving" if reduced else "direct lines"}'

    def copy(name, renamed, mapping, transform=None):
        Sdf.CopySpec(source.GetRootLayer(), BASE+name, layer, BASE+renamed)
        prim = stage.GetPrimAtPath(BASE+renamed)
        for rel in prim.GetRelationships():
            targets = rel.GetTargets()
            # SetTargets([]) authors USD None, which the JS baker cannot serialize.
            # Leave unauthored relationships absent instead of inventing null values.
            if targets:
                rel.SetTargets([Sdf.Path(mapping.get(str(path), str(path))) for path in targets])
        if transform:
            attr = prim.GetAttribute('xformOp:translate')
            if attr:
                attr.Set(Gf.Vec3d(*transform(tuple(attr.Get()))))
        return prim

    corner_source = 'CornerAL' if reduced else 'AttachAL'
    for name, point in zip(NAMES,CORNERS):
        prim = copy(corner_source, name, {}, lambda _: point)
        prim.GetAttribute('physics:mass').Set(.00745 if reduced else .02025)
    old_corner = tuple(source.GetPrimAtPath(BASE+corner_source).GetAttribute('xformOp:translate').Get())
    low_names = [p.GetName() for p in source.GetPrimAtPath(BASE[:-1]).GetChildren()
                 if re.match(r'^(Spool|Anchor|Attach|Corner|Wheel|Pinhole|Joint|CablePath)A', p.GetName())]
    for axis_index, axis in enumerate('ABCD'):
        angle = axis_index*math.pi/2
        renamed = {name: re.sub(r'^(Spool|Anchor|Attach|Corner|Wheel|Pinhole|Joint|CablePath)A',
                               lambda m: m[1]+axis, name) for name in low_names}
        for side,index in [('L',axis_index), ('R',(axis_index+1)%4)]:
            renamed[f'{"Corner" if reduced else "Attach"}A{side}'] = NAMES[index]
        mapping = {BASE+name:BASE+new for name,new in renamed.items()}
        for name in low_names:
            if name.startswith('Corner' if reduced else 'Attach'):
                continue
            side = re.match(r'^(?:Anchor|Wheel|Pinhole)A([LR])', name)
            shift = (0.,0.,0.) if not side else ((-.25 if side[1]=='L' else .25)
                -(-abs(old_corner[0]) if side[1]=='L' else abs(old_corner[0])), -.05, 0.)
            prim = copy(name, renamed[name], mapping,
                        lambda p: rotate(tuple(p[i]+shift[i] for i in range(3)), angle))
            orient = prim.GetAttribute('xformOp:rotateXYZ')
            if orient:
                xyz=orient.Get(); orient.Set(Gf.Vec3d(xyz[0],xyz[1],xyz[2]+math.degrees(angle)))
            stored = prim.GetAttribute('cablePath:stored')
            if stored:
                values=list(stored.Get());values[0]=4. if reduced else 2.;stored.Set(values)

    top_names = [p.GetName() for p in source.GetPrimAtPath(BASE[:-1]).GetChildren()
                 if re.match(r'^(Anchor|Attach|Wheel|Joint|CablePath)D0',p.GetName())]
    copy('SpoolD','SpoolI',{})
    for index,point in enumerate(CORNERS):
        angle = math.atan2(point[1],point[0])-math.atan2(old_corner[1],old_corner[0])
        renamed={name:name.replace('D0',f'I{index}') for name in top_names}
        mapping={BASE+name:BASE+new for name,new in renamed.items()}
        mapping.update({BASE+'SpoolD':BASE+'SpoolI', BASE+corner_source:BASE+NAMES[index]})
        for name in top_names:
            def transform(p):
                delta=rotate((p[0]-old_corner[0],p[1]-old_corner[1],0.),angle)
                return (point[0]+delta[0],point[1]+delta[1],p[2])
            prim=copy(name,renamed[name],mapping,transform)
            orient=prim.GetAttribute('xformOp:rotateXYZ')
            if orient:
                xyz=orient.Get();orient.Set(Gf.Vec3d(xyz[0],xyz[1],xyz[2]+math.degrees(angle)))
            if name.startswith('Attach'):
                prim.GetAttribute('ecs:tags').Set(['Anchor'])  # fixed ceiling knot, not a mover corner
            stored=prim.GetAttribute('cablePath:stored')
            if stored:
                values=list(stored.Get());values[0]=4. if reduced else 2.;stored.Set(values)
    extruder=copy('Extruder','Extruder',{})
    extruder.GetRelationship('machine:centerSources').SetTargets([BASE+name for name in NAMES])
    members=[BASE+name for name in NAMES]
    for prim in root.GetChildren():
        mass=prim.GetAttribute('physics:mass')
        if mass and mass.Get()>0 and str(prim.GetPath()) not in members:
            members.append(str(prim.GetPath()))
    effector=stage.DefinePrim(BASE+'Effector','RigidBody')
    effector.CreateRelationship('rigidBody:members',custom=True).SetTargets(members)
    effector.CreateRelationship('rigidBody:frameCorners',custom=True).SetTargets([BASE+name for name in NAMES])
    (ROOT/f'public/usd_scenes/{machine}.usda').write_text(layer.ExportToString().rstrip()+'\n')
    print(machine, 'corners', len(NAMES), 'mover members',len(members),
          'mass',sum(stage.GetPrimAtPath(p).GetAttribute('physics:mass').Get() for p in members))


if __name__=='__main__':
    build(False)
    build(True)
