import { dumpWorldState } from '../../../src/js/cable_joints/debugUtils.js';
import { RenderSystem } from './renderSystem.js';
import { InputSystem, InputReplaySystem } from './flipper_common.js';


export function runGame(world, setupScene, sceneData) {
    const pauseBtn = document.getElementById("pauseBtn");
    const resetBtn = document.getElementById("resetBtn");
    const stepBtn = document.getElementById("stepBtn");
    const dumpBtn = document.getElementById('dumpBtn');
    const dtEl = document.getElementById('dt');
    const speedEl = document.getElementById('speed');

    let lastTime = 0;
    let accumulator = 0.0;
    let doStep = true;
    let stepWhileKeyHeld = false;
    let speedSamples = [];
    const numSpeedSamples = 60;
    let frameCounter = 0;

    const requestSingleStep = () => {
        const pauseState = world.getResource('pauseState');
        if (pauseState && pauseState.paused) {
            doStep = true;
            requestAnimationFrame(gameLoop);
        }
    };

    function gameLoop(currentTime) {
        const dt = world.getResource('dt');
        if (dtEl && dtEl.textContent === 'N/A') {
            dtEl.textContent = `${(dt * 1000).toFixed(2)}ms`;
        }
        const pauseState = world.getResource('pauseState');

        if (lastTime === 0) {
            lastTime = currentTime;
        }
        const speedScale = window._flipperSpeedScale ?? 1.0;
        let frameSec = speedScale * (currentTime - lastTime) / 1000;
        let simTimeProcessed = 0;

        if (frameSec >= dt) {
            lastTime = currentTime;
            const maxSteps = window._flipperMaxSubSteps ?? 500;
            const maxAccum = dt * maxSteps;
            accumulator = Math.min(accumulator + frameSec, maxAccum);
            while (accumulator >= dt) {
                const shouldStepWhilePaused = doStep || stepWhileKeyHeld;
                if (!pauseState.paused || shouldStepWhilePaused) {
                    const steppingWhilePaused = pauseState.paused && shouldStepWhilePaused;
                    if (steppingWhilePaused) pauseState.paused = false;
                    world.update(dt);
                    simTimeProcessed += dt;
                    if (steppingWhilePaused) pauseState.paused = true;
                    if (doStep) {
                        doStep = false;
                    }
                }
                if (pauseState.paused && !stepWhileKeyHeld) {
                    accumulator = 0;
                    break;
                }
                accumulator -= dt;
            }
        }

        // Speed calculation - only record if simulation time actually advanced
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
        speedSamples = [];
        frameCounter = 0;
        if (speedEl) speedEl.textContent = 'N/A';
        const pauseState = world.getResource('pauseState');
        if (pauseState) pauseState.paused = true;
        pauseBtn.textContent = "Start";
        doStep = true;
        requestAnimationFrame(gameLoop); // to render the reset state
    });

    stepBtn.addEventListener('click', (e) => {
        e.preventDefault();
        requestSingleStep();
    });

    document.addEventListener('keydown', (e) => {
        const targetTag = e.target && e.target.tagName;
        const isEditableTarget = targetTag === 'INPUT' || targetTag === 'TEXTAREA' || targetTag === 'SELECT' || e.target?.isContentEditable;
        if (isEditableTarget || e.repeat) {
            return;
        }
        const key = (typeof e.key === 'string') ? e.key.toLowerCase() : '';
        if (key === 's' || key === 't') {
            e.preventDefault();
            stepWhileKeyHeld = true;
            requestSingleStep();
        }
    });

    document.addEventListener('keyup', (e) => {
        const targetTag = e.target && e.target.tagName;
        const isEditableTarget = targetTag === 'INPUT' || targetTag === 'TEXTAREA' || targetTag === 'SELECT' || e.target?.isContentEditable;
        if (isEditableTarget) {
            return;
        }
        const key = (typeof e.key === 'string') ? e.key.toLowerCase() : '';
        if (key === 's' || key === 't') {
            stepWhileKeyHeld = false;
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
