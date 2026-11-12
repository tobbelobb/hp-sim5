import { setSolverIterations } from './solver-iterations.js';
import { setPhysicsSubsteps } from './physics-substeps.js';
import { setClosedLoopMotorFeatureFlags } from './closed-loop-flags.js';
import { setLineLayeringFeatureFlags } from './line-layering-flags.js';

export function createFeatureFlagsController({ world, state, toggles = {}, onLineLayeringChanged = null }) {
  if (state) {
    state.lineLayeringEnabled = toggles.lineLayeringToggle ? Boolean(toggles.lineLayeringToggle.checked) : true;
    state.closedLoopMotorsEnabled = toggles.closedLoopMotorsToggle ? Boolean(toggles.closedLoopMotorsToggle.checked) : false;
    state.showConstraintForces = toggles.showForcesToggle ? Boolean(toggles.showForcesToggle.checked) : false;
  }

  function setShowConstraintForcesState(enabled, { fromToggle = false } = {}) {
    const next = Boolean(enabled);
    if (state) {
      state.showConstraintForces = next;
    }
    world.setResource('showConstraintForces', next);
    if (!fromToggle && toggles.showForcesToggle) {
      toggles.showForcesToggle.checked = next;
    }
    const renderSystem = world.getResource('renderSystem');
    if (renderSystem && typeof renderSystem.requestRender === 'function') {
      renderSystem.requestRender(world);
    }
  }

  function setHighlightSlackLines(enabled) {
    world.setResource('highlightSlackLines', Boolean(enabled));
    world.getResource('renderSystem')?.requestRender?.(world);
  }

  function setLineLayeringEnabledState(enabled, { fromToggle = false } = {}) {
    const next = Boolean(enabled);
    const changed = (state?.lineLayeringEnabled ?? true) !== next;
    if (state) {
      state.lineLayeringEnabled = next;
    }
    setLineLayeringFeatureFlags(world, next);
    world.getResource('cableSolverBackend')?.invalidate?.();
    if (!fromToggle && toggles.lineLayeringToggle) {
      toggles.lineLayeringToggle.checked = next;
    }
    if (changed && fromToggle && typeof onLineLayeringChanged === 'function') {
      onLineLayeringChanged();
    }
  }

  function setClosedLoopMotorsEnabledState(enabled, { fromToggle = false } = {}) {
    const next = Boolean(enabled);
    if (state) {
      state.closedLoopMotorsEnabled = next;
    }
    setClosedLoopMotorFeatureFlags(world, next);
    if (!fromToggle && toggles.closedLoopMotorsToggle) {
      toggles.closedLoopMotorsToggle.checked = next;
    }
  }

  function setPhysicsSubstepsState(count, options) {
    const next = setPhysicsSubsteps(world, count, options);
    if (state) state.physicsSubsteps = next;
    if (toggles.physicsSubsteps) {
      // Remote settings also accept counts between the standard menu choices.
      const select = toggles.physicsSubsteps;
      if (![...select.options].some(option => Number(option.value) === next)) {
        const option = select.ownerDocument.createElement('option');
        option.value = option.textContent = String(next);
        select.add(option);
      }
      select.value = String(next);
    }
  }

  function syncSolverIterationsState() {
    const count = world.getResource('machineSolverIterations') ?? 1;
    const override = world.getResource('solverIterationsOverride');
    if (state) state.solverIterations = override;
    const select = toggles.solverIterations;
    if (!select) return;
    select.options[0].textContent = `Machine default (${count})`;
    const value = override == null ? '' : String(override);
    if (![...select.options].some(option => option.value === value)) {
      const option = select.ownerDocument.createElement('option');
      option.value = option.textContent = value;
      select.add(option);
    }
    select.value = value;
  }

  function setSolverIterationsState(count) {
    const next = setSolverIterations(world, count);
    syncSolverIterationsState();
    return next;
  }

  function bindUi() {
    toggles.solverIterations?.addEventListener('change', () => {
      setSolverIterationsState(toggles.solverIterations.value === '' ? null : Number(toggles.solverIterations.value));
    });
    toggles.physicsSubsteps?.addEventListener('change', () => {
      setPhysicsSubstepsState(Number(toggles.physicsSubsteps.value));
    });
    if (toggles.showForcesToggle) {
      toggles.showForcesToggle.addEventListener('change', () => {
        setShowConstraintForcesState(toggles.showForcesToggle.checked, { fromToggle: true });
      });
      setShowConstraintForcesState(toggles.showForcesToggle.checked, { fromToggle: true });
    } else {
      setShowConstraintForcesState(false);
    }
    if (toggles.highlightSlackToggle) {
      toggles.highlightSlackToggle.addEventListener('change', () => {
        setHighlightSlackLines(toggles.highlightSlackToggle.checked);
      });
      setHighlightSlackLines(toggles.highlightSlackToggle.checked);
    }
    if (toggles.lineLayeringToggle) {
      toggles.lineLayeringToggle.addEventListener('change', () => {
        setLineLayeringEnabledState(toggles.lineLayeringToggle.checked, { fromToggle: true });
      });
      setLineLayeringEnabledState(toggles.lineLayeringToggle.checked, { fromToggle: true });
    } else {
      setLineLayeringEnabledState(true);
    }
    if (toggles.closedLoopMotorsToggle) {
      toggles.closedLoopMotorsToggle.addEventListener('change', () => {
        setClosedLoopMotorsEnabledState(toggles.closedLoopMotorsToggle.checked, { fromToggle: true });
      });
      setClosedLoopMotorsEnabledState(toggles.closedLoopMotorsToggle.checked, { fromToggle: true });
    } else {
      setClosedLoopMotorsEnabledState(false);
    }
  }

  if (state?.solverIterations !== undefined) setSolverIterationsState(state.solverIterations);
  else syncSolverIterationsState();
  setPhysicsSubstepsState(state?.physicsSubsteps ?? world.getResource('physicsSubsteps') ?? 1,
    { override: state?.physicsSubsteps !== undefined });
  setShowConstraintForcesState(state?.showConstraintForces ?? false);
  setLineLayeringEnabledState(state?.lineLayeringEnabled ?? true);
  setClosedLoopMotorsEnabledState(state?.closedLoopMotorsEnabled ?? false);

  return {
    get lineLayeringEnabled() {
      return state?.lineLayeringEnabled ?? true;
    },
    get closedLoopMotorsEnabled() {
      return state?.closedLoopMotorsEnabled ?? false;
    },
    get showConstraintForces() {
      return state?.showConstraintForces ?? false;
    },
    setSolverIterationsState,
    syncSolverIterationsState,
    setPhysicsSubstepsState,
    setShowConstraintForcesState,
    setLineLayeringEnabledState,
    setClosedLoopMotorsEnabledState,
    bindUi,
  };
}
