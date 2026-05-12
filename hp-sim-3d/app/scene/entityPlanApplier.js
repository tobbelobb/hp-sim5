import { applySceneResources } from './sceneResources.js';

export function applyEntityPlan(world, plan) {
    applySceneResources(world, plan.resources);
    applyEntities(world, plan.entities);
    runPostApplyHooks(world, plan.postApply);
}

export function applyEntities(world, entities) {
    for (const applyEntity of entities) {
        applyEntity(world);
    }
}

export function runPostApplyHooks(world, hooks) {
    for (const hook of hooks) {
        hook(world);
    }
}
