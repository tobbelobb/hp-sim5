import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import { PositionComponent } from '../../../src/js/cable_joints_3d/ecs.js';
import { ExtruderComponent } from '../hangprinter_extruder.js';
import { normalizeToPlainArray } from './usdValueReaders.js';
import { scopedKeyFromPath } from './sceneNaming.js';

export function buildExtruderBindings(context, registry) {
    if (context.options.remote) {
        return [];
    }
    return [(world) => applyExtruderBindings(world, context, registry)];
}

function applyExtruderBindings(world, context, registry) {
    const machineId = context.options.namespace || 'default';
    const namespace = context.options.namespace || null;
    const extruderCenterEntityIds = resolveExtruderCenterEntityIds(context, registry, namespace);
    const extruderCenterOffset = registry.extruder.prim && registry.extruder.authoredPos
        ? registry.extruder.authoredPos.clone()
        : null;
    const extruderCenterSourceOffsets = resolveExtruderCenterSourceOffsets(world, extruderCenterEntityIds);

    if (world.query([ExtruderComponent]).length === 0) {
        const extruderEntity = world.createEntity();
        world.addComponent(extruderEntity, new ExtruderComponent());
    }

    const extruderComp = firstExtruderComponent(world);
    if (!extruderComp) {
        return;
    }

    ensureExtruderMaps(extruderComp);
    setOrDeleteArray(extruderComp.centerSources, machineId, extruderCenterEntityIds);
    setOrDeleteVector(extruderComp.centerOffsets, machineId, extruderCenterOffset);
    if (extruderCenterSourceOffsets.length > 0) {
        extruderComp.centerSourceOffsets[machineId] = extruderCenterSourceOffsets.map((offset) => offset.clone());
    } else if (extruderComp.centerSourceOffsets[machineId]) {
        delete extruderComp.centerSourceOffsets[machineId];
    }
    setOrDeleteVector(extruderComp.tipOffsets, machineId, registry.extruder.tipOffset);
    setOrDeleteVector(extruderComp.coldEndOffsets, machineId, registry.extruder.coldEndOffset);
}

function resolveExtruderCenterEntityIds(context, registry, namespace) {
    const rels = normalizeToPlainArray(registry.extruder.centerPaths);
    if (!Array.isArray(rels) || rels.length === 0) {
        return [];
    }
    return rels
        .map((path) => registry.nameToEntityId.get(scopedKeyFromPath(namespace, context.sceneRootPath, path)))
        .filter((id) => id !== undefined);
}

function resolveExtruderCenterSourceOffsets(world, extruderCenterEntityIds) {
    if (extruderCenterEntityIds.length === 0) {
        return [];
    }
    const averagePos = resolveAveragePosition(world, extruderCenterEntityIds);
    if (!averagePos) {
        return [];
    }
    return extruderCenterEntityIds
        .map((entityId) => {
            const pos = world.getComponent(entityId, PositionComponent)?.pos;
            return pos ? pos.clone().subtract(averagePos) : null;
        })
        .filter((offset) => offset instanceof Vector3);
}

function resolveAveragePosition(world, entityIds) {
    const sum = new Vector3();
    let count = 0;
    for (const entityId of entityIds) {
        const pos = world.getComponent(entityId, PositionComponent)?.pos;
        if (pos) {
            sum.add(pos);
            count += 1;
        }
    }
    return count > 0 ? sum.scale(1.0 / count) : null;
}

function firstExtruderComponent(world) {
    for (const extruderEntity of world.query([ExtruderComponent])) {
        const extruderComp = world.getComponent(extruderEntity, ExtruderComponent);
        if (extruderComp) {
            return extruderComp;
        }
    }
    return null;
}

function ensureExtruderMaps(extruderComp) {
    for (const key of ['centerSources', 'centerOffsets', 'centerSourceOffsets', 'tipOffsets', 'coldEndOffsets']) {
        if (!extruderComp[key] || typeof extruderComp[key] !== 'object') {
            extruderComp[key] = {};
        }
    }
}

function setOrDeleteArray(target, key, values) {
    if (values.length > 0) {
        target[key] = values.slice();
    } else if (target[key]) {
        delete target[key];
    }
}

function setOrDeleteVector(target, key, value) {
    if (value) {
        target[key] = value.clone();
    } else if (target[key]) {
        delete target[key];
    }
}
