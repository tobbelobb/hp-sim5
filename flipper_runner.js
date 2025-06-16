import { dumpWorldState } from './cable_joints/debugUtils.js';
import { InputSystem, InputReplaySystem } from './cable_joints/commonSystems.js';

export function runGame(world, setupScene, sceneData) {
    const originalScene = sceneData ? (typeof structuredClone === 'function' ?
        structuredClone(sceneData) : JSON.parse(JSON.stringify(sceneData))) : null;
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
        const speedScale = window._flipperSpeedScale ?? 0.6;
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
        const data = originalScene ? (typeof structuredClone === 'function' ?
            structuredClone(originalScene) : JSON.parse(JSON.stringify(originalScene))) : sceneData;
        setupScene(data);
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
