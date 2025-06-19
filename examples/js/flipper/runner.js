import { dumpWorldState } from '../../../src/js/cable_joints/debugUtils.js';
import { RenderSystem } from './renderSystem.js';
import { InputSystem, InputReplaySystem } from './flipper_common.js';


export function runGame(world, setupScene, sceneData) {
    const pauseBtn = document.getElementById("pauseBtn");
    const resetBtn = document.getElementById("resetBtn");
    const stepBtn = document.getElementById("stepBtn");
    const dumpBtn = document.getElementById('dumpBtn');

    let lastTime = 0;
    let accumulator = 0.0;
    let doStep = true;

    function gameLoop(currentTime) {
        const dt = world.getResource('dt');
        const pauseState = world.getResource('pauseState');

        if (lastTime === 0) {
            lastTime = currentTime;
        }
        const speedScale = window._flipperSpeedScale ?? 1.0;
        let frameSec = speedScale * (currentTime - lastTime) / 1000;
        if (frameSec >= dt) {
            lastTime = currentTime;
            const maxSteps = window._flipperMaxSubSteps ?? 5;
            const maxAccum = dt * maxSteps;
            accumulator = Math.min(accumulator + frameSec, maxAccum);
            while (accumulator >= dt) {
                if (!pauseState.paused || doStep) {
                    if (doStep) pauseState.paused = false;
                    world.update(dt);
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

        // Always render, even if paused or if no physics step was taken.
        const renderSystem = world.systems.find(s => s instanceof RenderSystem);
        if (renderSystem) {
            renderSystem.update(world, 0); // dt=0 as we only want to render
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
        setupScene(sceneData);
        // reset any input related state
        for (const sys of world.systems) {
            if (sys instanceof InputSystem || sys instanceof InputReplaySystem) {
                if (typeof sys.reset === 'function') sys.reset();
            }
        }
        lastTime = 0;
        accumulator = 0;
        const pauseState = world.getResource('pauseState');
        if (pauseState) pauseState.paused = true;
        pauseBtn.textContent = "Start";
        doStep = true;
        requestAnimationFrame(gameLoop); // to render the reset state
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

    setupScene(sceneData);
    const pauseState = world.getResource('pauseState');
    pauseBtn.textContent = pauseState.paused ? "Start" : "Pause";
    requestAnimationFrame(gameLoop);
}
