import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import { getAttribute } from '../../../src/js/usd/stage.js';
import { SimulationErrorStateComponent } from '../../../src/js/cable_joints_3d/ecs.js';

const DEFAULT_PLANE_NORMAL = new Vector3(0, 0, 1);

export class PauseStateComponent {
    constructor(paused = false) {
        this.paused = paused;
    }
}

export function buildSceneResources(context) {
    const { stage, options } = context;
    const isRemote = Boolean(options.remote);
    const append = Boolean(options.append);
    const canvas = options.canvas;
    const simHeight = 1.7;
    const canvasWidth = canvas?.clientWidth ?? canvas?.width ?? 0;
    const canvasHeight = canvas?.clientHeight ?? canvas?.height ?? 0;
    const cScale = canvasHeight / simHeight;
    const simWidth = cScale > 0 ? canvasWidth / cScale : 0;
    const machineId = options.namespace || 'default';

    let physics = null;
    if (!isRemote && !append) {
        const physicsScene = stage.GetPrimAtPath("/World/PhysicsScene");
        const gravityDir = getAttribute(physicsScene, "physics:gravityDirection");
        const gravityMag = getAttribute(physicsScene, "physics:gravityMagnitude");
        const dtAssignment = stage.ast.descriptor.assignments.find(
            (s) => s.type === 'assignment' && s.identifier === 'timeCodesPerSecond',
        );
        physics = {
            gravity: new Vector3(
                gravityDir[0] * gravityMag,
                gravityDir[1] * gravityMag,
                (gravityDir[2] || 0) * gravityMag,
            ),
            dt: 1.0 / dtAssignment.value,
            defaultPlaneNormal: DEFAULT_PLANE_NORMAL.clone(),
        };
    }

    return {
        append,
        remote: isRemote,
        canvas,
        canvasWidth,
        canvasHeight,
        simWidth,
        simHeight,
        physics,
        machineColors: {
            clear: !append,
            machineId,
            tintColor: options.tintColor || null,
            extrusionColor: options.extrusionColor || null,
        },
    };
}

export function applySceneResources(world, resources) {
    const existingRenderSystem = world.getResource('renderSystem');
    if (!resources.append && typeof existingRenderSystem?.resetVisuals === 'function') {
        existingRenderSystem.resetVisuals();
    }

    if (!resources.remote && !resources.append) {
        world.clear();
    }

    if (resources.canvas) {
        resources.canvas.width = resources.canvasWidth;
        resources.canvas.height = resources.canvasHeight;
    }

    let machineColors = world.getResource('machineColors');
    if (!machineColors || typeof machineColors.set !== 'function') {
        machineColors = new Map();
        world.setResource('machineColors', machineColors);
    }
    if (resources.machineColors.clear) {
        machineColors.clear();
    }
    machineColors.set(resources.machineColors.machineId, {
        tintColor: resources.machineColors.tintColor,
        extrusionColor: resources.machineColors.extrusionColor,
    });

    if (resources.physics) {
        world.setResource('gravity', resources.physics.gravity);
        world.setResource('dt', resources.physics.dt);
        world.setResource('defaultPlaneNormal', resources.physics.defaultPlaneNormal);
    }

    if (!resources.append) {
        const existingPauseState = world.getResource('pauseState');
        const paused = typeof existingPauseState?.paused === 'boolean'
            ? existingPauseState.paused
            : false;
        world.setResource('simWidth', resources.simWidth);
        world.setResource('simHeight', resources.simHeight);
        world.setResource('pauseState', new PauseStateComponent(paused));
        world.setResource('debugRenderPoints', {});
        world.setResource('errorState', new SimulationErrorStateComponent(false));
        world.setResource('grabbedBall', null);
    }
}
