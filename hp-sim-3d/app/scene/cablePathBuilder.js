import { getAttribute, getRelationship } from '../../../src/js/usd/stage.js';
import { MachineTagComponent, SceneEntityInfoComponent } from '../../../src/js/cable_joints_3d/ecs.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import { CablePathComponent, PinholeGateComponent, GateWheelComponent } from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { scopedKeyFromPath } from './sceneNaming.js';
import { readNumericAttribute, readSpoolAxisLocal } from './usdValueReaders.js';

export function buildCablePaths(context, registry) {
    if (context.options.remote) {
        return [];
    }
    return [(world) => applyCablePaths(world, context, registry)];
}

function applyCablePaths(world, context, registry) {
    const machineId = context.options.namespace || 'default';
    const namespace = context.options.namespace || null;

    for (const prim of registry.pathPrims) {
        const jointPaths = getRelationship(prim, "cablePath:joints");
        if (!jointPaths) continue;

        const jointEntities = jointPaths
            .map((path) => registry.jointEntityMap.get(scopedKeyFromPath(namespace, context.sceneRootPath, path)))
            .filter(Boolean);
        const pathComp = new CablePathComponent(
            world,
            jointEntities,
            getAttribute(prim, "cablePath:linkTypes") ? [...getAttribute(prim, "cablePath:linkTypes")] : null,
            getAttribute(prim, "cablePath:clockwise") ? [...getAttribute(prim, "cablePath:clockwise")] : null,
            getAttribute(prim, "cablePath:stiffness") ?? Infinity,
            getAttribute(prim, "cablePath:stored") ? [...getAttribute(prim, "cablePath:stored")] : null,
            getAttribute(prim, "cablePath:halfWidth") ?? 0.0,
            readNumericAttribute(prim, "cablePath:damping") ?? 0.0,
            readNumericAttribute(prim, "cablePath:solverIterations") ?? 1,
        );
        const cablePath = world.createEntity();
        world.addComponent(cablePath, pathComp);
        world.addComponent(cablePath, new MachineTagComponent(machineId));
        world.addComponent(cablePath, new SceneEntityInfoComponent(prim.name, ['CablePath']));
        for (let i = 1; i < pathComp.linkTypes.length - 1; i++) {
            if (pathComp.linkTypes[i] !== 'pinhole_gate') continue;
            const jointPrim = context.stage.GetPrimAtPath(jointPaths[i - 1]);
            const portPath = getRelationship(jointPrim, 'physics:body1')[0];
            const portPrim = context.stage.GetPrimAtPath(portPath);
            const entry = registry.nameToEntityId.get(scopedKeyFromPath(namespace, context.sceneRootPath, portPath));
            const gate = world.getComponent(entry, PinholeGateComponent);
            gate.visualRoute = (getAttribute(portPrim, 'pinholeGate:visualRoute') || []).map(p => new Vector3(...p));
            gate.visible = getAttribute(portPrim, 'pinholeGate:visible') !== false;
            gate.effectiveInertia = readNumericAttribute(portPrim, 'pinholeGate:effectiveInertia') ?? gate.effectiveInertia;
            gate.damping = Math.max(0.0, readNumericAttribute(portPrim, 'pinholeGate:damping') ?? gate.damping);
            gate.radius = readNumericAttribute(portPrim, 'pinholeGate:radius') ?? gate.radius;
            const coulombTorque = readNumericAttribute(portPrim, 'pinholeGate:coulombTorque');
            gate.coulombForce = coulombTorque === null || !(gate.radius > 0.0)
                ? 0.0
                : Math.max(0.0, coulombTorque / gate.radius);
            for (const wheelPath of getRelationship(portPrim, 'pinholeGate:wheels') || []) {
                const wheel = registry.nameToEntityId.get(scopedKeyFromPath(namespace, context.sceneRootPath, wheelPath));
                if (wheel === undefined) continue;
                const wheelPrim = context.stage.GetPrimAtPath(wheelPath);
                const radius = readNumericAttribute(wheelPrim, 'radius') ?? gate.radius;
                const rotationScale = readNumericAttribute(wheelPrim, 'pinholeGate:rotationScale') ?? 1.0;
                world.addComponent(wheel, new GateWheelComponent(
                    entry, cablePath, radius, readSpoolAxisLocal(wheelPrim), rotationScale, pathComp.stored[0],
                ));
            }
        }
    }
}
