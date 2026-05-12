import { _updateAttachmentPoints } from '../../src/js/cable_joints_3d/cable_joints_core.js';
import { setClosedLoopMotorFeatureFlags } from './closed-loop-flags.js';
import { setLineLayeringFeatureFlags } from './line-layering-flags.js';

export function createFeatureFlagsController({ world, toggles = {}, onLineLayeringChanged = null }) {
  let lineLayeringEnabled = toggles.lineLayeringToggle ? Boolean(toggles.lineLayeringToggle.checked) : true;
  let closedLoopMotorsEnabled = toggles.closedLoopMotorsToggle ? Boolean(toggles.closedLoopMotorsToggle.checked) : false;
  let showConstraintForces = toggles.showForcesToggle ? Boolean(toggles.showForcesToggle.checked) : false;

  function setShowConstraintForcesState(enabled, { fromToggle = false } = {}) {
    const next = Boolean(enabled);
    showConstraintForces = next;
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
    const changed = lineLayeringEnabled !== next;
    lineLayeringEnabled = next;
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
    closedLoopMotorsEnabled = next;
    setClosedLoopMotorFeatureFlags(world, next);
    if (!fromToggle && toggles.closedLoopMotorsToggle) {
      toggles.closedLoopMotorsToggle.checked = next;
    }
  }

  return {
    get lineLayeringEnabled() {
      return lineLayeringEnabled;
    },
    get closedLoopMotorsEnabled() {
      return closedLoopMotorsEnabled;
    },
    get showConstraintForces() {
      return showConstraintForces;
    },
    setShowConstraintForcesState,
    setLineLayeringEnabledState,
    setClosedLoopMotorsEnabledState,
  };
}

