import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
    getAttribute,
    getChild,
    getChildren,
    getRelationship,
} from '../../../src/js/usd/stage.js';
import { readVectorAttribute } from './usdValueReaders.js';

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

        const posArr = getAttribute(prim, "xformOp:translate");
        if (!posArr) {
            continue;
        }

        const centerPaths = getRelationship(prim, 'machine:centerSources');
        if (tags.includes("Extruder") || centerPaths.length > 0) {
            registry.extruder.prim = prim;
            registry.extruder.authoredPos = new Vector3(posArr[0], posArr[1], posArr[2] || 0);
            registry.extruder.centerPaths = centerPaths;
            const tipPrim = getChild(prim, 'Tip') || getChild(prim, 'HotEnd');
            const coldEndPrim = getChild(prim, 'ColdEnd') || getChild(prim, 'Bottom');
            registry.extruder.tipOffset = readVectorAttribute(tipPrim, 'xformOp:translate') || new Vector3(0.0, 0.0, 0.0);
            registry.extruder.coldEndOffset = readVectorAttribute(coldEndPrim, 'xformOp:translate');
            continue;
        }

        registry.bodyPrims.push(prim);
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
