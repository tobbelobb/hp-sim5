import { dumpWorldState } from '../../../src/js/cable_joints/debugUtils.js';
import { InputSystem } from './slideprinter_common.js';


export function runGame(world, internalSetupScene) {
    const pauseBtn = document.getElementById("pauseBtn");
    const resetBtn = document.getElementById("resetBtn");
    const stepBtn = document.getElementById("stepBtn");
    const dumpBtn = document.getElementById('dumpBtn');
    const dtEl = document.getElementById('dt');
    const speedEl = document.getElementById('speed');

    let lastTime = 0;
    let accumulator = 0.0;
    let doStep = false;
    let speedSamples = [];
    const numSpeedSamples = 60;
    let frameCounter = 0;

    function gameLoop(currentTime) {
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
            while (accumulator >= dt) {
                if (!pauseState.paused || doStep) {
                    if (doStep) pauseState.paused = false;
                    world.update(dt);
                    simTimeProcessed += dt;
                    if (doStep) {
                        pauseState.paused = true;
                        doStep = false;
                    }
                }
                if (pauseState.paused) {
                    accumulator = 0;
                    break;
                }
                accumulator -= dt;
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


        const renderSystem = world.getResource('renderSystem');
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
            if (!pauseState.paused) {
                lastTime = performance.now();
                requestAnimationFrame(gameLoop);
            }
        }
    });

    resetBtn.addEventListener('click', (e) => {
        e.preventDefault();
        internalSetupScene();
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
        doStep = false;
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
    const pauseState = world.getResource('pauseState');
    pauseBtn.textContent = pauseState.paused ? "Start" : "Pause";
    requestAnimationFrame(gameLoop);
}
