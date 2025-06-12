from pxr import Usd, UsdGeom
import warp.sim


def parse_slideprinter(path):
    """Parse a USD scene describing Slideprinter-style mechanics.

    The function now recognizes generic entities, cable joints and cable paths
    using ``customData.purpose`` metadata. It returns a Warp model along with
    dictionaries describing the parsed scene.
    """

    stage = Usd.Stage.Open(path)
    builder = warp.sim.ModelBuilder()
    try:
        data = warp.sim.parse_usd(stage, builder)
        model = builder.finalize()
    except Exception:
        # USD scene may not contain primitives recognized by Warp. In that case
        # still allow parsing of custom metadata.
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


def run_sim(model, steps=10, dt=1.0/200.0):
    """Run a tiny Warp simulation just to show the USD data works."""
    integrator = warp.sim.SemiImplicitIntegrator(model)
    state = model.state()
    for _ in range(steps):
        integrator.step(dt, state)
    return state


def main():
    model, entities, joints, paths, _ = parse_slideprinter('slideprinter/slideprinter.usda')
    print(f"Loaded model with {model.body_count} bodies")
    print('Entities:', list(entities.keys()))
    print('Joints:', [j['name'] for j in joints])
    print('Paths:', [p['name'] for p in paths])
    state = run_sim(model)
    if model.body_count:
        print('First body position after sim:', state.body_q[0])


if __name__ == '__main__':
    main()
