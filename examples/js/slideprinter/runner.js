import { dumpWorldState } from '../../../src/js/cable_joints/debugUtils.js';
import { InputSystem } from './slideprinter_common.js';

export function runGame(world, internalSetupScene, options = {}) {
    const pauseBtn = document.getElementById('pauseBtn');
    const resetBtn = document.getElementById('resetBtn');
    const stepBtn = document.getElementById('stepBtn');
    const dumpBtn = document.getElementById('dumpBtn');
    const dtEl = document.getElementById('dt');
    const speedEl = document.getElementById('speed');

    const {
        initialTimeScale = 1.0,
        onTimeScaleChange,
    } = options;

    function sanitizeTimeScale(value, fallback) {
        if (!Number.isFinite(value)) {
            return fallback;
        }
        if (value <= 0) {
            return fallback;
        }
        return value;
    }

    let lastTime = 0;
    let accumulator = 0.0;
    let doStep = false;
    let frameCounter = 0;
    let startTime = 0;
    let totalSim = 0;
    let hasStarted = false;
    let targetTimeScale = sanitizeTimeScale(initialTimeScale, 1.0);
    world.setResource('timeScale', targetTimeScale);

    const getPauseState = () => world.getResource('pauseState');

    const updatePauseButtonLabel = () => {
        const pauseState = getPauseState();
        if (!pauseBtn || !pauseState) {
            return;
        }
        if (!pauseState.paused) {
            pauseBtn.textContent = 'Pause';
        } else {
            pauseBtn.textContent = hasStarted ? 'Resume' : 'Start';
        }
    };

    function gameLoop(currentTime) {
        const dt = world.getResource('dt');
        if (dtEl && dtEl.textContent === 'N/A') {
            dtEl.textContent = `${(dt * 1000).toFixed(2)}ms`;
        }
        const pauseState = getPauseState();

        if (lastTime === 0) {
            lastTime = currentTime;
        }
        const speedScale = targetTimeScale;
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

        totalSim += simTimeProcessed;

        frameCounter++;
        if (frameCounter % 10 === 0 && speedEl) {
            if (startTime >= 0) {
                const elapsed = (performance.now() - startTime) / 1000;
                if (elapsed > 0) {
                    const avgSpeed = totalSim / elapsed;
                    speedEl.textContent = `${avgSpeed.toFixed(2)}x`;
                }
            }
        }

        const renderSystem = world.getResource('renderSystem');
        if (renderSystem) {
            renderSystem.update(world, 0);
        }

        requestAnimationFrame(gameLoop);
    }

    function resetGame({ autoPause = true } = {}) {
        internalSetupScene();
        for (const sys of world.systems) {
            if (sys instanceof InputSystem) {
                if (typeof sys.reset === 'function') sys.reset();
            }
        }
        lastTime = 0;
        accumulator = 0;
        frameCounter = 0;
        if (speedEl) speedEl.textContent = 'N/A';
        totalSim = 0;
        const pauseState = getPauseState();
        if (autoPause) {
            startTime = 0;
            hasStarted = false;
            if (pauseState) {
                pauseState.paused = true;
            }
        } else {
            startTime = performance.now();
            hasStarted = true;
            if (pauseState) {
                pauseState.paused = false;
            }
            lastTime = performance.now();
        }
        doStep = false;
        updatePauseButtonLabel();
        requestAnimationFrame(gameLoop);
    }

    function setTimeScale(scale) {
        const clamped = sanitizeTimeScale(scale, targetTimeScale);
        if (Math.abs(clamped - targetTimeScale) < 1e-6) {
            return;
        }
        targetTimeScale = clamped;
        world.setResource('timeScale', targetTimeScale);
        lastTime = 0;
        frameCounter = 0;
        totalSim = 0;
        if (speedEl) {
            speedEl.textContent = 'N/A';
        }
        const pauseState = getPauseState();
        if (pauseState && !pauseState.paused) {
            startTime = performance.now();
        } else {
            startTime = 0;
        }
        if (typeof onTimeScaleChange === 'function') {
            onTimeScaleChange(targetTimeScale);
        }
    }

    function getTimeScale() {
        return targetTimeScale;
    }

    if (pauseBtn) {
        pauseBtn.addEventListener('click', (e) => {
            e.preventDefault();
            const pauseState = getPauseState();
            if (!pauseState) {
                return;
            }
            if (pauseState.paused) {
                pauseState.paused = false;
                if (!hasStarted) {
                    startTime = performance.now();
                    totalSim = 0;
                    hasStarted = true;
                }
                lastTime = performance.now();
                updatePauseButtonLabel();
                requestAnimationFrame(gameLoop);
            } else {
                pauseState.paused = true;
                updatePauseButtonLabel();
            }
        });
    }

    if (resetBtn) {
        resetBtn.addEventListener('click', (e) => {
            e.preventDefault();
            resetGame({ autoPause: true });
        });
    }

    if (stepBtn) {
        stepBtn.addEventListener('click', (e) => {
            e.preventDefault();
            const pauseState = getPauseState();
            if (pauseState && pauseState.paused) {
                doStep = true;
                requestAnimationFrame(gameLoop);
            }
        });
    }

    if (dumpBtn) {
        dumpBtn.addEventListener('click', (e) => {
            e.preventDefault();
            console.log(dumpWorldState(world));
        });
    }

    resetGame({ autoPause: false });
    if (typeof onTimeScaleChange === 'function') {
        onTimeScaleChange(targetTimeScale);
    }

    return {
        reset: resetGame,
        setTimeScale,
        getTimeScale,
    };
}
