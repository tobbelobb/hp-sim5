import { sceneSettingsForMachine } from '../../../machines/catalog.js';
import { getAttribute, getRelationship } from '../../../src/js/usd/stage.js';
import { MachineTagComponent, SceneEntityInfoComponent } from '../../../src/js/cable_joints_3d/ecs.js';
import { CablePathComponent } from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { scopedKeyFromPath } from './sceneNaming.js';
import { readNumericAttribute } from './usdValueReaders.js';

export function buildCablePaths(context, registry) {
    if (context.options.remote) {
        return [];
    }
    const machineId = context.options.namespace || 'default';
    const namespace = context.options.namespace || null;
    const iterations = sceneSettingsForMachine(getAttribute(context.sceneRoot, "machine:catalog")).solver_iterations;

    return registry.pathPrims.map((prim) => {
        const jointKeys = getRelationship(prim, "cablePath:joints").map((path) => {
            const key = scopedKeyFromPath(namespace, context.sceneRootPath, path);
            if (!registry.jointEntityMap.has(key)) {
                throw new Error(`${context.sceneRootPath}/${prim.name}: cablePath:joints target ${path} did not resolve to a CableJoint.`);
            }
            return key;
        });
        return (world) => applyCablePath(world, prim, jointKeys, registry, machineId, iterations);
    });
}

function applyCablePath(world, prim, jointKeys, registry, machineId, iterations) {
    const jointEntities = jointKeys.map((key) => registry.jointEntityMap.get(key));
    const pathComp = new CablePathComponent(
        world,
        jointEntities,
        getAttribute(prim, "cablePath:linkTypes") ? [...getAttribute(prim, "cablePath:linkTypes")] : null,
        getAttribute(prim, "cablePath:clockwise") ? [...getAttribute(prim, "cablePath:clockwise")] : null,
        getAttribute(prim, "cablePath:stiffnessPerUnitLength") ?? Infinity,
        getAttribute(prim, "cablePath:stored") ? [...getAttribute(prim, "cablePath:stored")] : null,
        getAttribute(prim, "cablePath:halfWidth") ?? 0.0,
        readNumericAttribute(prim, "cablePath:dampingPerUnitLength") ?? 0.0,
        iterations ?? readNumericAttribute(prim, "cablePath:solverIterations") ?? 1,
        // The baked half-width already encodes the initial geometry, including zero for an unlayered bake.
        { initialLayeringEnabled: true },
    );
    const hystFrac = readNumericAttribute(prim, "cablePath:hysteresisFraction");
    if (hystFrac !== null && hystFrac > 0) pathComp.hysteresisFraction = hystFrac;
    const hystRec = readNumericAttribute(prim, "cablePath:hysteresisRecoveryTime");
    if (hystRec !== null && hystRec >= 0) pathComp.hysteresisRecoveryTime = hystRec;
    const radialFlat = readNumericAttribute(prim, "cablePath:spoolRadialFlattening");
    if (radialFlat !== null && radialFlat >= 0) pathComp.spoolRadialFlattening = radialFlat;
    const strainedSpool = getAttribute(prim, "cablePath:strainedSpoolStorage");
    if (typeof strainedSpool === "boolean") pathComp.strainedSpoolStorage = strainedSpool;
    else if (typeof strainedSpool === "number" && Number.isFinite(strainedSpool)) {
        pathComp.strainedSpoolStorage = strainedSpool !== 0;
    }
    const initStorageTension = readNumericAttribute(prim, "cablePath:initialStorageTension");
    if (initStorageTension !== null && initStorageTension >= 0) pathComp.initialStorageTension = initStorageTension;
    const cablePath = world.createEntity();
    world.addComponent(cablePath, pathComp);
    world.addComponent(cablePath, new MachineTagComponent(machineId));
    world.addComponent(cablePath, new SceneEntityInfoComponent(prim.name, ['CablePath']));
}
