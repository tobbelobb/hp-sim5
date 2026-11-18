import { createRigidTransformReader } from '../../../src/js/usd/rigid_transforms.js';
import {
    getAttribute,
    getChild,
    getChildren,
    getRelationship,
} from '../../../src/js/usd/stage.js';

export function createSceneBuildRegistry() {
    return {
        nameToEntityId: new Map(),
        bodyPrims: [],
        jointPrims: [],
        pathPrims: [],
        distanceJointPrims: [],
        rigidBodyPrims: [],
        entityToRigidBody: new Map(),
        jointEntityMap: new Map(),
        extruder: {
            prim: null,
            authoredPos: null,
            centerPaths: [],
            tipOffset: null,
            coldEndOffset: null,
        },
    };
}

export function discoverScenePrims(context, registry) {
    if (context.options.remote || !context.sceneRoot) {
        return;
    }

    const readWorldTransform = createRigidTransformReader(path => context.stage.GetPrimAtPath(path));
    for (const prim of getChildren(context.sceneRoot)) {
        const apiSchemas = getAttribute(prim, "apiSchemas");
        if (apiSchemas && apiSchemas.includes("CablePathAPI")) {
            registry.pathPrims.push(prim);
        }

        if (prim.type === 'definition' && prim.defType === 'CableJoint') {
            registry.jointPrims.push(prim);
        }

        if (prim.type === 'definition' && prim.defType === 'DistancePhysicsJoint') {
            registry.distanceJointPrims.push(prim);
        }

        if (isRigidBodyPrim(prim)) {
            registry.rigidBodyPrims.push(prim);
        }

        const tags = getAttribute(prim, "ecs:tags") || [];
        if (tags.length === 0) {
            continue;
        }

        const path = `${context.sceneRootPath}/${prim.name}`;
        const centerPaths = getRelationship(prim, 'machine:centerSources');
        if (tags.includes("Extruder") || centerPaths.length > 0) {
            const { position } = readWorldTransform(path);
            const sceneOrigin = readWorldTransform(context.sceneRootPath).position;
            const offset = child => child
                ? readWorldTransform(`${path}/${child.name}`).position.clone().subtract(position)
                : null;
            registry.extruder.prim = prim;
            registry.extruder.authoredPos = position.clone().subtract(sceneOrigin);
            registry.extruder.centerPaths = centerPaths;
            registry.extruder.tipOffset = offset(getChild(prim, 'Tip') || getChild(prim, 'HotEnd'));
            registry.extruder.coldEndOffset = offset(getChild(prim, 'ColdEnd') || getChild(prim, 'Bottom'));
            continue;
        }

        registry.bodyPrims.push({ prim, readTransform: () => readWorldTransform(path) });
    }
}

function isRigidBodyPrim(prim) {
    if (prim.type === 'definition' && (prim.defType === 'RigidBody' || prim.defType === 'RigidGroup')) {
        return true;
    }
    const maybeRigidBodyMembers = getRelationship(prim, 'rigidBody:members');
    const maybeRigidGroupMembers = getRelationship(prim, 'rigidGroup:members');
    return (
        (maybeRigidBodyMembers && maybeRigidBodyMembers.length > 0)
        || (maybeRigidGroupMembers && maybeRigidGroupMembers.length > 0)
    );
}
