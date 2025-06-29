import { dumpWorldState } from '../../../src/js/cable_joints/debugUtils.js';
import { RenderSystem } from '../flipper/renderSystem.js';
import { InputSystem } from './slideprinter_common.js';


export function runGameRemote(world, internalSetupScene, ws, applyState) {
    const pauseBtn = document.getElementById("pauseBtn");
    const resetBtn = document.getElementById("resetBtn");
    const stepBtn = document.getElementById("stepBtn");
    const dumpBtn = document.getElementById('dumpBtn');
    const dtEl = document.getElementById('dt');
    const speedEl = document.getElementById('speed');

    let lastTime = 0;
    let accumulator = 0.0;
    let doStep = true;
    let speedSamples = [];
    const numSpeedSamples = 60;
    let frameCounter = 0;
    let pendingResolve = null;

    function send(action) {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify(action));
        }
    }

    if (ws) {
        ws.addEventListener('message', (event) => {
            const data = JSON.parse(event.data);
            if (applyState) applyState(world, data);
            if (pendingResolve) { pendingResolve(); pendingResolve = null; }
        });
    }

    async function gameLoop(currentTime) {
        const dt = world.getResource('dt');
        if (dtEl && dtEl.textContent === 'N/A') {
            dtEl.textContent = `${(dt * 1000).toFixed(2)}ms`;
        }
        const pauseState = world.getResource('pauseState');

        if (lastTime === 0) {
            lastTime = currentTime;
        }
        const speedScale = 1.0;
        let frameSec = speedScale * (currentTime - lastTime) / 1000;
        let simTimeProcessed = 0;

        if (frameSec >= dt) {
            lastTime = currentTime;
            const maxSteps = 500;
            const maxAccum = dt * maxSteps;
            accumulator = Math.min(accumulator + frameSec, maxAccum);
            const stepsToTake = Math.floor(accumulator / dt);
            if (stepsToTake > 0) {
                accumulator -= stepsToTake * dt;
                if (!pauseState.paused || doStep) {
                    if (doStep) pauseState.paused = false;
                    if (ws && ws.readyState === WebSocket.OPEN) {
                        await new Promise(res => { pendingResolve = res; ws.send(JSON.stringify({ action: 'step', steps: stepsToTake })); });
                    }
                    simTimeProcessed += stepsToTake * dt;
                    if (doStep) {
                        pauseState.paused = true;
                        doStep = false;
                    }
                } else {
                    accumulator = 0;
                }
            }
        }

        if (frameSec > 1e-6 && simTimeProcessed > 0) {
            const currentSpeed = simTimeProcessed / frameSec;
            speedSamples.push(currentSpeed);
            if (speedSamples.length > numSpeedSamples) {
                speedSamples.shift();
            }
        }

        frameCounter++;
        if (frameCounter % 10 === 0 && speedEl && speedSamples.length > 0) {
            const avgSpeed = speedSamples.reduce((a, b) => a + b, 0) / speedSamples.length;
            speedEl.textContent = `${avgSpeed.toFixed(2)}x`;
        }


        const renderSystem = world.systems.find(s => s instanceof RenderSystem);
        if (renderSystem) {
            renderSystem.update(world, 0);
        }

        requestAnimationFrame(gameLoop);
    }

    pauseBtn.addEventListener('click', (e) => {
        e.preventDefault();
        const pauseState = world.getResource('pauseState');
        if (pauseState) {
            pauseState.paused = !pauseState.paused;
            pauseBtn.textContent = pauseState.paused ? "Resume" : "Pause";
            send({ action: 'pause', paused: pauseState.paused });
            if (!pauseState.paused) {
                lastTime = performance.now();
                requestAnimationFrame(gameLoop);
            }
        }
    });

    resetBtn.addEventListener('click', (e) => {
        e.preventDefault();
        internalSetupScene();
        send({ action: 'reset' });
        for (const sys of world.systems) {
            if (sys instanceof InputSystem) {
                if (typeof sys.reset === 'function') sys.reset();
            }
        }
        lastTime = 0;
        accumulator = 0;
        speedSamples = [];
        frameCounter = 0;
        if (speedEl) speedEl.textContent = 'N/A';
        const pauseState = world.getResource('pauseState');
        if (pauseState) pauseState.paused = true;
        pauseBtn.textContent = "Start";
        doStep = true;
        requestAnimationFrame(gameLoop);
    });

    stepBtn.addEventListener('click', (e) => {
        e.preventDefault();
        const pauseState = world.getResource('pauseState');
        if (pauseState && pauseState.paused) {
            doStep = true;
            requestAnimationFrame(gameLoop);
        }
    });

    dumpBtn.addEventListener('click', (e) => {
        e.preventDefault();
        console.log(dumpWorldState(world));
    });

    internalSetupScene();
    send({ action: 'reset' });
    const pauseState = world.getResource('pauseState');
    pauseBtn.textContent = pauseState.paused ? "Start" : "Pause";
    requestAnimationFrame(gameLoop);
}
