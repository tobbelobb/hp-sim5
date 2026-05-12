import { _updateAttachmentPoints } from '../../src/js/cable_joints_3d/cable_joints_core.js';
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

  function setLineLayeringEnabledState(enabled, { fromToggle = false } = {}) {
    const next = Boolean(enabled);
    const changed = (state?.lineLayeringEnabled ?? true) !== next;
    if (state) {
      state.lineLayeringEnabled = next;
    }
    setLineLayeringFeatureFlags(world, next);
    _updateAttachmentPoints(world);
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

  function bindUi() {
    if (toggles.showForcesToggle) {
      toggles.showForcesToggle.addEventListener('change', () => {
        setShowConstraintForcesState(toggles.showForcesToggle.checked, { fromToggle: true });
      });
      setShowConstraintForcesState(toggles.showForcesToggle.checked, { fromToggle: true });
    } else {
      setShowConstraintForcesState(false);
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
    setShowConstraintForcesState,
    setLineLayeringEnabledState,
    setClosedLoopMotorsEnabledState,
    bindUi,
  };
}
