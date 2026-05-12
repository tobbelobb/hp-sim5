export function parseStage(stage) {
    return { stage };
}

export function readMachineSceneSpec(parsedStage, scenePrimPath = '/World/SlideprinterScene', options = {}) {
    const remote = Boolean(options.remote);
    const normalizedScenePrimPath = typeof scenePrimPath === 'string' && scenePrimPath.length > 0
        ? scenePrimPath
        : '/World/SlideprinterScene';
    const stage = parsedStage?.stage ?? parsedStage;
    return {
        parsedStage: { stage },
        scenePrimPath: normalizedScenePrimPath,
        sceneRoot: remote ? null : stage?.GetPrimAtPath?.(normalizedScenePrimPath) ?? null,
        remote,
    };
}

export function validateMachineSceneSpec(spec) {
    const sceneRootPath = spec.scenePrimPath.endsWith('/')
        ? spec.scenePrimPath.slice(0, -1)
        : spec.scenePrimPath;
    if (!spec.remote && !spec.sceneRoot) {
        return {
            ...spec,
            sceneRootPath,
            valid: false,
            warnings: [`setupScene: Unable to find scene root at ${spec.scenePrimPath}.`],
        };
    }
    return {
        ...spec,
        sceneRootPath,
        valid: true,
        warnings: [],
    };
}

export function createSceneReadContext(checkedSpec, options = {}) {
    return {
        stage: checkedSpec.parsedStage.stage,
        sceneRoot: checkedSpec.sceneRoot,
        sceneRootPath: checkedSpec.sceneRootPath,
        options: {
            ...options,
            remote: checkedSpec.remote,
            scenePrimPath: checkedSpec.scenePrimPath,
        },
    };
}
