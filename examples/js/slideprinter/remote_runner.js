import { dumpWorldState } from '../../../src/js/cable_joints/debugUtils.js';
import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
    PositionComponent, VelocityComponent, RadiusComponent, MassComponent,
    OrientationComponent, AngularVelocityComponent, MomentOfInertiaComponent,
    PrevFinalOrientationComponent, PrevFinalPosComponent, RestitutionComponent,
    CoefficientOfFrictionComponent, SimulationErrorStateComponent, RenderableComponent,
    DistanceConstraintComponent,
} from "../../../src/js/cable_joints/ecs.js";
import {
    CableLinkComponent, CableJointComponent, CablePathComponent
} from '../../../src/js/cable_joints/cable_joints_core.js';
import { SpoolTagComponent, ExtruderComponent } from './slideprinter_common.js';

const componentRegistry = {
    PositionComponent, VelocityComponent, RadiusComponent, MassComponent,
    OrientationComponent, AngularVelocityComponent, MomentOfInertiaComponent,
    PrevFinalOrientationComponent, PrevFinalPosComponent, RestitutionComponent,
    CoefficientOfFrictionComponent, SimulationErrorStateComponent, RenderableComponent,
    DistanceConstraintComponent, CableLinkComponent, CableJointComponent,
    CablePathComponent, SpoolTagComponent, ExtruderComponent
};

function rebuildWorldFromState(world, state) {
    let existingExtrusions = [];
    const extruderEntities = world.query([ExtruderComponent]);
    if (extruderEntities.length > 0) {
        const extruderComp = world.getComponent(extruderEntities[0], ExtruderComponent);
        if (extruderComp) {
            existingExtrusions = extruderComp.extrusions;
        }
    }

    world.clear();

    if (state.resources) {
        for (const key in state.resources) {
            if (key === 'pauseState') continue;
            if (key === 'gravity' && Array.isArray(state.resources[key])) {
                 world.setResource(key, new Vector2(state.resources[key][0], state.resources[key][1]));
            } else {
                 world.setResource(key, state.resources[key]);
            }
        }
    }

    if (state.entities) {
        for (const entityIdStr in state.entities) {
            const entityId = parseInt(entityIdStr, 10);
            if (entityId >= world.nextEntityId) world.nextEntityId = entityId + 1;
            world.entities.set(entityId, new Set());
        }
    }

    if (state.components) {
        for (const compName in state.components) {
            const ComponentClass = componentRegistry[compName];
            if (!ComponentClass) continue;

            const allCompData = state.components[compName];
            for (const entityIdStr in allCompData) {
                const entityId = parseInt(entityIdStr, 10);
                const compData = allCompData[entityIdStr];
                let componentInstance;

                if (compName === 'CableJointComponent') {
                    componentInstance = new CableJointComponent(
                        compData.entity_a, compData.entity_b, compData.rest_length,
                        new Vector2(compData.attachment_point_a_world[0], compData.attachment_point_a_world[1]),
                        new Vector2(compData.attachment_point_b_world[0], compData.attachment_point_b_world[1])
                    );
                    if (compData.attachment_point_a_world) {
                        componentInstance.attachmentPointA_world = new Vector2(compData.attachment_point_a_world[0], compData.attachment_point_a_world[1]);
                    }
                    if (compData.attachment_point_b_world) {
                        componentInstance.attachmentPointB_world = new Vector2(compData.attachment_point_b_world[0], compData.attachment_point_b_world[1]);
                    }
                } else if (compName === 'CablePathComponent') {
                     componentInstance = new CablePathComponent(world, compData.joint_entities, compData.link_types, compData.cw, compData.spring_constant, compData.stored);
                     componentInstance.totalRestLength = compData.total_rest_length;
                } else if (compName === 'DistanceConstraintComponent') {
                    componentInstance = new DistanceConstraintComponent(compData.entity_a, compData.entity_b, compData.rest_length, compData.compliance);
                } else {
                    componentInstance = new ComponentClass();
                    for (const py_prop in compData) {
                        const js_prop = py_prop.replace(/_([a-z])/g, g => g[1].toUpperCase());
                        const value = compData[py_prop];
                        if (js_prop === 'pos' || js_prop === 'vel' || js_prop === 'prevFinalPos') {
                            componentInstance[js_prop] = new Vector2(value[0], value[1]);
                        } else {
                            componentInstance[js_prop] = value;
                        }
                    }
                }
                world.addComponent(entityId, componentInstance);
            }
        }
    }

    const newExtruderEntities = world.query([ExtruderComponent]);
    if (newExtruderEntities.length > 0) {
        const extruderComp = world.getComponent(newExtruderEntities[0], ExtruderComponent);
        if (extruderComp) {
            const incremental = extruderComp.extrusions || [];
            extruderComp.extrusions = existingExtrusions.concat(incremental);
        }
    }
}

export function runRemoteGame(world, internalSetupScene, ws) {
    const pauseBtn = document.getElementById("pauseBtn");
    const resetBtn = document.getElementById("resetBtn");
    const stepBtn = document.getElementById("stepBtn");
    const dumpBtn = document.getElementById('dumpBtn');
    const dtEl = document.getElementById('dt');
    const speedEl = document.getElementById('speed');

    let lastTime = 0;
    let accumulator = 0.0;
    let speedSamples = [];
    const numSpeedSamples = 60;
    let frameCounter = 0;
    let isPaused = true;
    let dt = 1 / 200;
    let maxAccumulator = dt * 500;

    ws.onmessage = (event) => {
        const state = JSON.parse(event.data);
        rebuildWorldFromState(world, state);

        const serverPauseState = state.resources.pauseState;
        if (serverPauseState) {
            isPaused = serverPauseState.paused;
            const pauseState = world.getResource('pauseState');
            if (pauseState) pauseState.paused = isPaused;
            pauseBtn.textContent = isPaused ? (pauseBtn.textContent === "Start" ? "Start" : "Resume") : "Pause";
        }

        const serverDt = world.getResource('dt');
        if (serverDt) {
            dt = serverDt;
            maxAccumulator = dt * 500;
            if (dtEl.textContent === 'N/A') {
                dtEl.textContent = `${(dt * 1000).toFixed(2)}ms`;
            }
        }
    };

    function gameLoop(currentTime) {
        if (!lastTime) lastTime = currentTime;

        const speedScale = 1.0;
        const frameSec = speedScale * (currentTime - lastTime) / 1000.0;
        let stepsToTake = 0;

        if (frameSec > 0) {
            lastTime = currentTime;
            if (!isPaused && ws && ws.readyState === WebSocket.OPEN) {
                accumulator = Math.min(accumulator + frameSec, maxAccumulator);
                stepsToTake = Math.floor(accumulator / dt);
                if (stepsToTake > 0) {
                    accumulator -= stepsToTake * dt;
                    ws.send(JSON.stringify({ action: 'step', steps: stepsToTake }));
                }
            }
        }

        if (frameSec > 1e-6 && stepsToTake > 0) {
            const currentSpeed = (stepsToTake * dt) / frameSec;
            speedSamples.push(currentSpeed);
            if (speedSamples.length > numSpeedSamples) speedSamples.shift();
        }

        frameCounter++;
        if (frameCounter % 10 === 0 && speedEl && speedSamples.length > 0) {
            const avgSpeed = speedSamples.reduce((a, b) => a + b, 0) / speedSamples.length;
            speedEl.textContent = `${avgSpeed.toFixed(2)}x`;
        }

        world.update(0);
        const renderSystem = world.getResource('renderSystem');
        if (renderSystem) {
            renderSystem.update(world, 0);
        }

        requestAnimationFrame(gameLoop);
    }

    pauseBtn.addEventListener('click', (e) => {
        e.preventDefault();
        if (ws && ws.readyState === WebSocket.OPEN) {
            isPaused = !isPaused;
            ws.send(JSON.stringify({ action: 'pause', paused: isPaused }));
            if (!isPaused) lastTime = performance.now();
        }
    });

    resetBtn.addEventListener('click', (e) => {
        e.preventDefault();
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ action: 'reset' }));

            const extruderEntities = world.query([ExtruderComponent]);
            if (extruderEntities.length > 0) {
                const extruderComp = world.getComponent(extruderEntities[0], ExtruderComponent);
                if (extruderComp) {
                    extruderComp.extrusions = [];
                }
            }

            isPaused = true;
            pauseBtn.textContent = "Start";
            lastTime = 0;
            accumulator = 0;
            speedSamples = [];
            frameCounter = 0;
            if (speedEl) speedEl.textContent = 'N/A';
            if (dtEl) dtEl.textContent = 'N/A';
        }
    });

    stepBtn.addEventListener('click', (e) => {
        e.preventDefault();
        if (isPaused && ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ action: 'step', steps: 1 }));
        }
    });

    dumpBtn.addEventListener('click', (e) => {
        e.preventDefault();
        console.log(dumpWorldState(world));
    });

    internalSetupScene(ws);
    pauseBtn.textContent = "Start";
    requestAnimationFrame(gameLoop);
}
