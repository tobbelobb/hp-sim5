const MOBILE_MACHINES_MENU_MARGIN_PX = 12;
const MACHINE_MENU_HOVER_CLOSE_DELAY_MS = 3000;

export function createMachineMenuController({
  document,
  window,
  dom,
  isMobileLayout = () => false,
} = {}) {
  let machineMenuOpen = false;
  let machineMenuHoverCloseTimeout = null;
  let machineMenuHoverTrackingEnabled = false;
  let machineMenuPointerInside = false;
  let machineMenuFocusInside = false;

  function clearInlinePosition() {
    if (!dom.machinesMenu) {
      return;
    }
    dom.machinesMenu.style.left = '';
    dom.machinesMenu.style.right = '';
    dom.machinesMenu.style.transform = '';
  }

  function positionForMobile() {
    if (!dom.machinesMenu || !dom.machinesToggle || !isMobileLayout()) {
      clearInlinePosition();
      return;
    }
    const parentRect = dom.machinesContainer?.getBoundingClientRect() ?? dom.machinesMenu.parentElement?.getBoundingClientRect();
    const toggleRect = dom.machinesToggle.getBoundingClientRect();
    const menuRect = dom.machinesMenu.getBoundingClientRect();
    const viewportWidth = window.innerWidth || document.documentElement.clientWidth || 0;
    if (!parentRect || !menuRect || viewportWidth <= 0) {
      return;
    }
    const maxTargetLeft = Math.max(MOBILE_MACHINES_MENU_MARGIN_PX, viewportWidth - menuRect.width - MOBILE_MACHINES_MENU_MARGIN_PX);
    const targetLeft = Math.max(
      MOBILE_MACHINES_MENU_MARGIN_PX,
      Math.min(toggleRect.left + toggleRect.width / 2 - menuRect.width / 2, maxTargetLeft)
    );
    dom.machinesMenu.style.left = `${targetLeft - (parentRect.left || 0)}px`;
    dom.machinesMenu.style.right = 'auto';
    dom.machinesMenu.style.transform = 'none';
  }

  function syncPlacement() {
    if (!machineMenuOpen) {
      if (!isMobileLayout()) {
        clearInlinePosition();
      }
      return;
    }
    if (isMobileLayout()) {
      positionForMobile();
    } else {
      clearInlinePosition();
    }
  }

  function schedulePlacementSync() {
    const sync = () => syncPlacement();
    if (typeof window.requestAnimationFrame === 'function') {
      window.requestAnimationFrame(sync);
    } else {
      window.setTimeout(sync, 0);
    }
  }

  function isHoverCapableDesktop() {
    if (!window.matchMedia) {
      return !isMobileLayout();
    }
    try {
      return window.matchMedia('(hover: hover) and (pointer: fine)').matches;
    } catch (_error) {
      return !isMobileLayout();
    }
  }

  function isPointerWithinMenu() {
    for (const element of [dom.machinesMenu, dom.machinesToggle]) {
      if (element instanceof HTMLElement && element.matches(':hover')) {
        return true;
      }
    }
    return false;
  }

  function clearHoverTimeout() {
    if (machineMenuHoverCloseTimeout !== null) {
      window.clearTimeout(machineMenuHoverCloseTimeout);
      machineMenuHoverCloseTimeout = null;
    }
  }

  function scheduleHoverClose() {
    if (!machineMenuHoverTrackingEnabled) {
      return;
    }
    clearHoverTimeout();
    machineMenuHoverCloseTimeout = window.setTimeout(() => {
      machineMenuHoverCloseTimeout = null;
      if (!machineMenuHoverTrackingEnabled || !machineMenuOpen || machineMenuPointerInside || machineMenuFocusInside) {
        return;
      }
      if (dom.machinesMenu?.contains(document.activeElement)) {
        machineMenuFocusInside = true;
        return;
      }
      close();
    }, MACHINE_MENU_HOVER_CLOSE_DELAY_MS);
  }

  function startHoverTracking() {
    if (!dom.machinesContainer) {
      machineMenuHoverTrackingEnabled = false;
      clearHoverTimeout();
      return;
    }
    machineMenuHoverTrackingEnabled = isHoverCapableDesktop();
    if (!machineMenuHoverTrackingEnabled) {
      clearHoverTimeout();
      return;
    }
    machineMenuPointerInside = isPointerWithinMenu();
    if (machineMenuPointerInside || machineMenuFocusInside) {
      clearHoverTimeout();
    } else {
      scheduleHoverClose();
    }
  }

  function stopHoverTracking() {
    machineMenuHoverTrackingEnabled = false;
    machineMenuPointerInside = false;
    machineMenuFocusInside = false;
    clearHoverTimeout();
  }

  function open() {
    if (!dom.machinesMenu || !dom.machinesToggle || machineMenuOpen) {
      return;
    }
    machineMenuOpen = true;
    dom.machinesMenu.classList.remove('sim-hidden');
    dom.machinesToggle.setAttribute('aria-expanded', 'true');
    dom.machinesContainer?.setAttribute('data-open', 'true');
    schedulePlacementSync();
    const firstFocusable = dom.machinesMenu.querySelector('input, button');
    firstFocusable?.focus?.({ preventScroll: true });
    addOutsideListeners();
    startHoverTracking();
  }

  function close({ focusToggle = false } = {}) {
    if (!dom.machinesMenu || !dom.machinesToggle || !machineMenuOpen) {
      return;
    }
    machineMenuOpen = false;
    dom.machinesMenu.classList.add('sim-hidden');
    dom.machinesToggle.setAttribute('aria-expanded', 'false');
    dom.machinesContainer?.setAttribute('data-open', 'false');
    clearInlinePosition();
    removeOutsideListeners();
    stopHoverTracking();
    if (focusToggle) {
      dom.machinesToggle.focus({ preventScroll: true });
    }
  }

  function handleOutsideInteraction(event) {
    const target = event.target;
    if (!machineMenuOpen || !(target instanceof Node)) {
      return;
    }
    if (dom.machinesMenu?.contains(target) || dom.machinesToggle?.contains(target)) {
      return;
    }
    close();
  }

  function handleKeydown(event) {
    if (machineMenuOpen && (event.key === 'Escape' || event.key === 'Esc')) {
      event.preventDefault();
      close({ focusToggle: true });
    }
  }

  function addOutsideListeners() {
    document.addEventListener('mousedown', handleOutsideInteraction, true);
    document.addEventListener('touchstart', handleOutsideInteraction, true);
    document.addEventListener('click', handleOutsideInteraction, true);
    document.addEventListener('keydown', handleKeydown, true);
  }

  function removeOutsideListeners() {
    document.removeEventListener('mousedown', handleOutsideInteraction, true);
    document.removeEventListener('touchstart', handleOutsideInteraction, true);
    document.removeEventListener('click', handleOutsideInteraction, true);
    document.removeEventListener('keydown', handleKeydown, true);
  }

  function bindUi() {
    if (dom.machinesMenu) {
      dom.machinesMenu.setAttribute('tabindex', '-1');
      dom.machinesMenu.addEventListener('mouseenter', () => {
        machineMenuPointerInside = true;
        clearHoverTimeout();
      });
      dom.machinesMenu.addEventListener('mouseleave', (event) => {
        if (!machineMenuHoverTrackingEnabled) {
          return;
        }
        const nextTarget = event?.relatedTarget;
        if (nextTarget instanceof Node && (dom.machinesMenu.contains(nextTarget) || dom.machinesToggle?.contains(nextTarget))) {
          return;
        }
        machineMenuPointerInside = false;
        scheduleHoverClose();
      });
      dom.machinesMenu.addEventListener('focusin', () => {
        machineMenuFocusInside = true;
        clearHoverTimeout();
      });
      dom.machinesMenu.addEventListener('focusout', (event) => {
        const nextTarget = event?.relatedTarget;
        if (nextTarget instanceof Node && (dom.machinesMenu.contains(nextTarget) || dom.machinesToggle?.contains(nextTarget))) {
          return;
        }
        machineMenuFocusInside = false;
        if (machineMenuHoverTrackingEnabled) {
          scheduleHoverClose();
        }
      });
    }
    if (dom.machinesToggle) {
      dom.machinesToggle.addEventListener('click', (event) => {
        event.preventDefault();
        if (machineMenuOpen) {
          close();
        } else {
          open();
        }
      });
      dom.machinesToggle.addEventListener('mouseenter', () => {
        machineMenuPointerInside = true;
        clearHoverTimeout();
      });
      dom.machinesToggle.addEventListener('mouseleave', () => {
        if (machineMenuHoverTrackingEnabled) {
          machineMenuPointerInside = false;
          scheduleHoverClose();
        }
      });
      dom.machinesToggle.addEventListener('focusin', () => {
        machineMenuFocusInside = true;
        clearHoverTimeout();
      });
      dom.machinesToggle.addEventListener('focusout', () => {
        machineMenuFocusInside = false;
        if (machineMenuHoverTrackingEnabled) {
          scheduleHoverClose();
        }
      });
    }
  }

  return {
    bindUi,
    open,
    close,
    syncPlacement,
  };
}
