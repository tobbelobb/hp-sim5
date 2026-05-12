import Vector3 from '../../src/js/cable_joints_3d/vector3.js';
import { CableAttachmentUpdateSystem, PBDCableConstraintSolver } from '../../src/js/cable_joints_3d/cable_joints_core.js';
import { PBDResolveCableOverCorrections } from '../../src/js/cable_joints_3d/pbdResolveCableOverCorrections.js';
import { CableAttachmentCacheSystem } from '../../src/js/cable_joints_3d/cable_attachment_cache_system.js';
import { CableFrictionSystem } from '../../src/js/cable_joints_3d/cable_friction_system.js';
import { InputSystem, RemoteInputSystem } from './hangprinter_input.js';
import { ExtruderSystem } from './hangprinter_extruder.js';
import { RemoteSpoolSystem } from './remoteSpoolSystem.js';
import { StepperMotorSystem } from './hangprinter_stepper_motor.js';
import { TorqueModeSystem } from './torqueModeSystem.js';
import { MissedStepTrackingSystem } from './motor-diagnostics.js';
import { RenderSystem3D } from '../../src/js/cable_joints_3d/render_system_3d.js';
import {
    PrevFinalPosSystem,
    PrevFinalOrientationSystem,
    EncoderUpdateSystem,
    GravitySystem,
    MovementSystem,
    AngularMovementSystem,
    PBDVelocityUpdateSystem,
    PBDAngularVelocityUpdateSystem,
    RigidBodySyncSystem,
} from '../../src/js/cable_joints_3d/commonSystems.js';

const DEFAULT_PLANE_NORMAL = new Vector3(0, 0, 1);

export function registerSceneSystems(world, { canvas, mode = '3d', remote = false, ws = null } = {}) {
    if (mode !== '3d') {
        throw new Error(`registerSceneSystems: unsupported mode "${mode}"`);
    }

    const existingRenderSystem = world.getResource('renderSystem');
    if (world.systems.length === 0) {
        const pauseBtn = document.getElementById("pauseBtn");
        let inputSys;
        if (remote) {
            inputSys = new RemoteInputSystem(canvas, world, ws);
        } else {
            inputSys = new InputSystem(canvas, world, pauseBtn);
        }
        inputSys.scaleMultiplier = 1.1;
        inputSys.viewOffsetX = 0.0;
        inputSys.viewOffsetY = -0.0;
        world.registerSystem(inputSys);

        if (!remote) {
            world.registerSystem(new PrevFinalPosSystem());
            world.registerSystem(new PrevFinalOrientationSystem());
            world.registerSystem(new RemoteSpoolSystem());
            world.registerSystem(new StepperMotorSystem());
            world.registerSystem(new GravitySystem());
            world.registerSystem(new MovementSystem());
            world.registerSystem(new AngularMovementSystem());
            world.registerSystem(new RigidBodySyncSystem());
            world.registerSystem(new CableAttachmentUpdateSystem(false));
            world.registerSystem(new CableAttachmentCacheSystem());
            world.registerSystem(new CableFrictionSystem());
            world.registerSystem(new PBDCableConstraintSolver());
            world.registerSystem(new PBDResolveCableOverCorrections());
            world.registerSystem(new PBDVelocityUpdateSystem());
            world.registerSystem(new PBDAngularVelocityUpdateSystem());
            world.registerSystem(new TorqueModeSystem());
            world.registerSystem(new ExtruderSystem());
            world.registerSystem(new EncoderUpdateSystem());
            world.registerSystem(new MissedStepTrackingSystem());
        }

        const renderSystem = new RenderSystem3D(canvas, {
            planeNormal: DEFAULT_PLANE_NORMAL,
            targetX: 0.0,
            targetY: 0.0,
            cameraZ: 2.2 / Math.max(0.02, inputSys.scaleMultiplier),
            initialOrbitAzimuth: -Math.PI * 0.25,
            initialOrbitPolar: 1.05,
            controlsEnabled: false,
            renderOnSimulationStep: false
        });
        renderSystem.setViewTransform?.({
            scaleMultiplier: inputSys.scaleMultiplier,
            offsetX: inputSys.viewOffsetX,
            offsetY: inputSys.viewOffsetY,
        });
        world.setResource('renderSystem', renderSystem);
    } else if (existingRenderSystem instanceof RenderSystem3D) {
        existingRenderSystem.setCanvasSize(canvas.clientWidth, canvas.clientHeight);
    }

    if (!remote) {
        const extruderSystem = world.systems.find((system) => system instanceof ExtruderSystem);
        if (extruderSystem && typeof extruderSystem.update === 'function') {
            extruderSystem.update(world, 0);
        }
    }

    const renderSystem = world.getResource('renderSystem');
    if (renderSystem instanceof RenderSystem3D && typeof renderSystem.update === 'function') {
        renderSystem.update(world, 0);
    }
}
