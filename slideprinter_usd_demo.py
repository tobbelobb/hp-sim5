try:
    from pxr import Usd, UsdGeom  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    Usd = None
    UsdGeom = None

try:
    import warp  # type: ignore
    import warp.sim  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    warp = None


def _parse_usda_ascii(path):
    """Very small USDA parser used when ``pxr`` is unavailable."""
    import re

    with open(path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    entities = {}
    joints = []
    paths = []
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line.startswith("def ") and '"' in line:
            name = line.split('"')[1]
            if line.rstrip().endswith('{'):
                # Container Xform like the scene root
                i += 1
                continue
            purpose = None
            i += 1
            # parse optional customData section
            while i < len(lines):
                l = lines[i].strip()
                m = re.search(r'purpose\s*=\s*"([^"]+)"', l)
                if m:
                    purpose = m.group(1)
                if l.endswith(')'):
                    i += 1
                    break
                i += 1

            attrs = {}
            if i < len(lines) and lines[i].strip() == '{':
                i += 1
                while i < len(lines):
                    l = lines[i].strip()
                    if l == '}':
                        break
                    if 'xformOp:translate' in l:
                        m = re.search(r'\(([^)]+)\)', l)
                        if m:
                            vals = [float(x) for x in m.group(1).split(',')]
                            attrs['pos'] = vals[:2]
                    for attr in ['radius','mass','angVel','velX','velY','restLength']:
                        if l.startswith(f'double {attr}'):
                            attrs[attr] = float(l.split('=')[1])
                    if l.startswith('token entityA'):
                        attrs['entityA'] = l.split('=')[1].strip().strip('"')
                    if l.startswith('token entityB'):
                        attrs['entityB'] = l.split('=')[1].strip().strip('"')
                    if l.startswith('double3 attachA'):
                        m = re.search(r'\(([^)]+)\)', l)
                        attrs['attachA'] = [float(x) for x in m.group(1).split(',')]
                    if l.startswith('double3 attachB'):
                        m = re.search(r'\(([^)]+)\)', l)
                        attrs['attachB'] = [float(x) for x in m.group(1).split(',')]
                    if l.startswith('token[] joints'):
                        m = re.search(r'\[([^]]+)\]', l)
                        attrs['joints'] = [s.strip().strip('"') for s in m.group(1).split(',')]
                    if l.startswith('token[] linkTypes'):
                        m = re.search(r'\[([^]]+)\]', l)
                        attrs['linkTypes'] = [s.strip().strip('"') for s in m.group(1).split(',')]
                    if l.startswith('bool[] cw'):
                        m = re.search(r'\[([^]]+)\]', l)
                        attrs['cw'] = [s.strip().lower() == 'true' for s in m.group(1).split(',')]
                    if l.startswith('double[] stored'):
                        m = re.search(r'\[([^]]+)\]', l)
                        attrs['stored'] = [float(s.strip()) for s in m.group(1).split(',')]
                    i += 1
                # skip closing brace
                if i < len(lines) and lines[i].strip() == '}':
                    i += 1

            if purpose == 'cable_joint':
                joint = {'name': name}
                joint.update(attrs)
                joints.append(joint)
            elif purpose == 'cable_path':
                path = {'name': name}
                path.update(attrs)
                paths.append(path)
            elif purpose:
                info = {'name': name, 'type': purpose}
                info.update(attrs)
                entities[name] = info
        else:
            i += 1

    return entities, joints, paths


def parse_slideprinter(path):
    """Parse a USD scene describing Slideprinter-style mechanics.

    The function recognizes generic entities, cable joints and cable paths using
    ``customData.purpose`` metadata. When the optional ``pxr`` and ``warp``
    modules are available, they are used to load the scene and build a Warp
    model. Otherwise a small USDA parser is used so tests still run on minimal
    environments.
    """

    if Usd is not None and warp is not None:
        stage = Usd.Stage.Open(path)
        builder = warp.sim.ModelBuilder()
        try:
            data = warp.sim.parse_usd(stage, builder)
            model = builder.finalize()
        except Exception:
            data = None
            model = None

        entities = {}
        joints = []
        paths = []

        for prim in stage.Traverse():
            purpose = prim.GetCustomData().get("purpose") if prim.HasCustomData() else None
            if not purpose:
                continue

            name = prim.GetName()
            xform = UsdGeom.Xformable(prim)
            pos = [0.0, 0.0]
            for op in xform.GetOrderedXformOps():
                if op.GetOpName() == "xformOp:translate":
                    v = op.Get()
                    pos = [float(v[0]), float(v[1])]

            if purpose in {"spool", "anchor", "ball", "obstacle", "flipper"}:
                info = {"name": name, "type": purpose, "pos": pos}
                for attr in ["radius", "mass", "angVel", "velX", "velY"]:
                    usd_attr = prim.GetAttribute(attr)
                    if usd_attr:
                        val = usd_attr.Get()
                        if val is not None:
                            info[attr] = val
                entities[name] = info

            elif purpose == "cable_joint":
                joint = {
                    "name": name,
                    "entityA": prim.GetAttribute("entityA").Get(),
                    "entityB": prim.GetAttribute("entityB").Get(),
                    "restLength": prim.GetAttribute("restLength").Get(),
                    "attachA": list(prim.GetAttribute("attachA").Get()),
                    "attachB": list(prim.GetAttribute("attachB").Get()),
                }
                joints.append(joint)

            elif purpose == "cable_path":
                path = {
                    "name": name,
                    "joints": list(prim.GetAttribute("joints").Get() or []),
                    "linkTypes": list(prim.GetAttribute("linkTypes").Get() or []),
                    "cw": list(prim.GetAttribute("cw").Get() or []),
                    "stored": list(prim.GetAttribute("stored").Get() or []),
                }
                paths.append(path)

        return model, entities, joints, paths, data

    # Fallback ASCII parsing when pxr or warp is missing
    entities, joints, paths = _parse_usda_ascii(path)
    return None, entities, joints, paths, None


def run_sim(model, steps=10, dt=1.0/200.0):
    """Run a tiny Warp simulation just to show the USD data works."""
    if warp is None or model is None:
        return None
    warp.set_device("cpu")
    integrator = warp.sim.SemiImplicitIntegrator()
    state_in = model.state()
    state_out = model.state()
    for _ in range(steps):
        integrator.simulate(model, state_in, state_out, dt)
        state_in, state_out = state_out, state_in
    return state_in


def main():
    model, entities, joints, paths, _ = parse_slideprinter('slideprinter/slideprinter.usda')
    if model is not None:
        print(f"Loaded model with {model.body_count} bodies")
    else:
        print("Loaded scene without Warp model")
    print('Entities:', list(entities.keys()))
    print('Joints:', [j['name'] for j in joints])
    print('Paths:', [p['name'] for p in paths])
    state = run_sim(model)
    if state is not None:
        print('First body position after sim:', state.body_q[0])


if __name__ == '__main__':
    main()
