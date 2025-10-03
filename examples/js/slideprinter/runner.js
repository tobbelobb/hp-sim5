import { dumpWorldState } from '../../../src/js/cable_joints/debugUtils.js';
import { InputSystem } from './slideprinter_common.js';

const ASAP_THRESHOLD_SCALE = 50;
const ASAP_RENDER_STRIDE = 10;
const ASAP_SLICE_BUDGET_MS = 28;
const ASAP_MIN_STEPS_PER_SLICE = 64;
const ASAP_INPUT_CHECK_MASK = 0x3f;
const ASAP_PAUSED_DELAY_MS = 12;
const SPEED_UPDATE_PERIOD = 10;
const SIMULATION_PLAYBACK_RESOURCE = 'simulationPlayback';

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

    const sanitizeRenderOverride = (value) => {
        if (value == null) {
            return null;
        }
        if (!Number.isFinite(value)) {
            return Number.POSITIVE_INFINITY;
        }
        return Math.max(1, Math.floor(value));
    };

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

    const updatePlaybackResource = () => {
        world.setResource(SIMULATION_PLAYBACK_RESOURCE, {
            mode: playbackMode,
            renderEveryNth: renderStride,
            asapThreshold: ASAP_THRESHOLD_SCALE,
        });
    };

    const recomputeRenderStride = () => {
        const base = renderStrideBase;
        const next = renderStrideOverride != null ? renderStrideOverride : base;
        const normalized = Number.isFinite(next) ? Math.max(1, Math.floor(next)) : Number.POSITIVE_INFINITY;
        if (renderStride !== normalized) {
            renderStride = normalized;
            renderCounter = 0;
        }
        updatePlaybackResource();
    };

    const setPlaybackMode = (mode) => {
        const normalized = mode === 'asap' ? 'asap' : 'linear';
        if (normalized === playbackMode) {
            return false;
        }
        playbackMode = normalized;
        renderStrideBase = playbackMode === 'asap' ? ASAP_RENDER_STRIDE : 1;
        lastTime = 0;
        accumulator = 0;
        renderCounter = 0;
        recomputeRenderStride();
        stopLoops();
        updatePlaybackResource();
        return true;
    };

    const maybeRender = () => {
        const renderSystem = world.getResource('renderSystem');
        if (!renderSystem || renderSystem.drawingSuspended) {
            return;
        }
        if (renderStride === Number.POSITIVE_INFINITY) {
            return;
        }
        renderCounter += 1;
        if (renderStride > 1 && renderCounter < renderStride) {
            return;
        }
        renderCounter = 0;
        renderSystem.update(world, 0);
    };

    const updateSpeedMeter = () => {
        if (!speedEl) {
            return;
        }
        if (!hasStarted || startTime <= 0) {
            speedEl.textContent = 'N/A';
            return;
        }
        const elapsed = (performance.now() - startTime) / 1000;
        if (elapsed > 0) {
            const avgSpeed = totalSim / elapsed;
            speedEl.textContent = `${avgSpeed.toFixed(2)}x`;
        }
    };

    const runLinearFrame = (currentTime) => {
        const dt = world.getResource('dt');
        if (dtEl && dtEl.textContent === 'N/A') {
            dtEl.textContent = `${(dt * 1000).toFixed(2)}ms`;
        }
        const pauseState = getPauseState();
        if (!pauseState) {
            maybeRender();
            return;
        }
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
                    if (doStep) {
                        pauseState.paused = false;
                    }
                    world.update(dt);
                    simTimeProcessed += dt;
                    if (doStep) {
                        pauseState.paused = true;
                        doStep = false;
                        break;
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
        frameCounter += 1;
        if (frameCounter % SPEED_UPDATE_PERIOD === 0) {
            updateSpeedMeter();
        }
        maybeRender();
    };

    const runAsapBatch = () => {
        const dt = world.getResource('dt');
        if (dtEl && dtEl.textContent === 'N/A') {
            dtEl.textContent = `${(dt * 1000).toFixed(2)}ms`;
        }
        const pauseState = getPauseState();
        if (!pauseState) {
            maybeRender();
            return ASAP_PAUSED_DELAY_MS;
        }

        let stepsRun = 0;
        const sliceDeadline = performance.now() + ASAP_SLICE_BUDGET_MS;
        const checkInput = typeof navigator !== 'undefined'
            && navigator !== null
            && navigator.scheduling
            && typeof navigator.scheduling.isInputPending === 'function'
            ? () => navigator.scheduling.isInputPending()
            : null;

        while (true) {
            if (pauseState.paused && !doStep) {
                break;
            }
            if (doStep) {
                pauseState.paused = false;
            }
            world.update(dt);
            totalSim += dt;
            stepsRun += 1;
            if (doStep) {
                pauseState.paused = true;
                doStep = false;
                break;
            }
            if (pauseState.paused) {
                break;
            }
            if (stepsRun >= ASAP_MIN_STEPS_PER_SLICE) {
                if (performance.now() >= sliceDeadline) {
                    break;
                }
                if (checkInput && (stepsRun & ASAP_INPUT_CHECK_MASK) === 0 && checkInput()) {
                    break;
                }
            }
        }

        frameCounter += 1;
        if (frameCounter % SPEED_UPDATE_PERIOD === 0) {
            updateSpeedMeter();
        }
        maybeRender();

        if (pauseState.paused && !doStep) {
            return ASAP_PAUSED_DELAY_MS;
        }
        return 0;
    };

    const startLinearLoop = (token) => {
        const step = (time) => {
            if (token !== loopToken) {
                return;
            }
            runLinearFrame(time);
            rafHandle = requestAnimationFrame(step);
        };
        rafHandle = requestAnimationFrame(step);
    };

    const startAsapLoop = (token) => {
        const iterate = () => {
            if (token !== loopToken) {
                return;
            }
            const delay = runAsapBatch();
            if (token !== loopToken) {
                return;
            }
            asapTimer = setTimeout(iterate, delay);
        };
        iterate();
    };

    const startActiveLoop = () => {
        const currentToken = loopToken;
        if (playbackMode === 'asap') {
            startAsapLoop(currentToken);
        } else {
            startLinearLoop(currentToken);
        }
    };

    const stopLoops = () => {
        if (rafHandle != null) {
            cancelAnimationFrame(rafHandle);
            rafHandle = null;
        }
        if (asapTimer != null) {
            clearTimeout(asapTimer);
            asapTimer = null;
        }
        loopToken += 1;
    };

    const resetGame = ({ autoPause = true } = {}) => {
        internalSetupScene();
        for (const sys of world.systems) {
            if (sys instanceof InputSystem) {
                if (typeof sys.reset === 'function') {
                    sys.reset();
                }
            }
        }
        lastTime = 0;
        accumulator = 0;
        frameCounter = 0;
        renderCounter = 0;
        if (speedEl) {
            speedEl.textContent = 'N/A';
        }
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
        stopLoops();
        startActiveLoop();
    };

    const setRenderEveryNth = (value) => {
        const sanitized = sanitizeRenderOverride(value);
        if (renderStrideOverride === sanitized) {
            return;
        }
        renderStrideOverride = sanitized;
        renderCounter = 0;
        recomputeRenderStride();
    };

    const setTimeScale = (scale) => {
        const clamped = sanitizeTimeScale(scale, targetTimeScale);
        const desiredMode = clamped >= ASAP_THRESHOLD_SCALE ? 'asap' : 'linear';
        const modeChanged = setPlaybackMode(desiredMode);
        if (Math.abs(clamped - targetTimeScale) < 1e-6 && !modeChanged) {
            return;
        }
        targetTimeScale = clamped;
        world.setResource('timeScale', targetTimeScale);
        lastTime = 0;
        accumulator = 0;
        frameCounter = 0;
        renderCounter = 0;
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
        if (modeChanged) {
            startActiveLoop();
        }
    };

    const getTimeScale = () => targetTimeScale;

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
                stopLoops();
                startActiveLoop();
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
                stopLoops();
                startActiveLoop();
            }
        });
    }

    if (dumpBtn) {
        dumpBtn.addEventListener('click', (e) => {
            e.preventDefault();
            console.log(dumpWorldState(world));
        });
    }

    let targetTimeScale = sanitizeTimeScale(initialTimeScale, 1.0);
    world.setResource('timeScale', targetTimeScale);

    let lastTime = 0;
    let accumulator = 0.0;
    let doStep = false;
    let frameCounter = 0;
    let renderCounter = 0;
    let startTime = 0;
    let totalSim = 0;
    let hasStarted = false;
    let playbackMode = 'linear';
    let renderStrideBase = 1;
    let renderStrideOverride = null;
    let renderStride = 1;
    let rafHandle = null;
    let asapTimer = null;
    let loopToken = 0;

    recomputeRenderStride();
    setPlaybackMode(targetTimeScale >= ASAP_THRESHOLD_SCALE ? 'asap' : 'linear');
    resetGame({ autoPause: false });
    if (typeof onTimeScaleChange === 'function') {
        onTimeScaleChange(targetTimeScale);
    }

    return {
        reset: resetGame,
        setTimeScale,
        getTimeScale,
        setRenderEveryNth,
    };
}
