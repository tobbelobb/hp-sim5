import { QualityMonitor } from './quality-monitor.js';
import {
  getMachineMotorDiagnostics,
  resetMachineMotorDiagnostics,
} from './motor-diagnostics.js';

export const QUALITY_HISTORY_MAX_ENTRIES = 20;

export function createQualityController({
  document,
  world,
  state,
  hudElement,
  historyHud,
  historyToggle,
  historyList,
  qualityToggle,
  getMachines,
  getMachineDisplayName,
  getReferenceOverlayState,
}) {
  const machineQualityMonitors = new Map();
  const qualityHistoryRecords = [];
  if (state) {
    state.qualityEnabled = qualityToggle ? Boolean(qualityToggle.checked) : false;
  }
  let qualityHistoryExpanded = false;

  const isEnabled = () => Boolean(state?.qualityEnabled);

  function forEachQualityMonitor(callback) {
    if (typeof callback !== 'function') {
      return;
    }
    for (const entry of machineQualityMonitors.values()) {
      if (entry?.monitor) {
        callback(entry.monitor, entry);
      }
    }
  }

  function updateQualityHudVisibility() {
    if (!hudElement) {
      return;
    }
    const hasVisibleMonitor = Array.from(machineQualityMonitors.values()).some((entry) => {
      const card = entry?.monitor?.hudElement;
      return card instanceof HTMLElement && !card.classList.contains('sim-hidden');
    });
    hudElement.classList.toggle('sim-hidden', !hasVisibleMonitor);
  }

  function ensureQualityMonitorForMachine(machine) {
    if (!machine || !hudElement) {
      return null;
    }
    const existing = machineQualityMonitors.get(machine.id);
    if (existing?.monitor) {
      existing.monitor.setMachineContext({
        id: machine.id,
        label: getMachineDisplayName(machine),
        tintColor: machine.tintColor || null,
      });
      existing.monitor.setMotorDiagnosticsProvider(() => getMachineMotorDiagnostics(world, machine.id));
      existing.monitor.setEnabled(isEnabled());
      return existing.monitor;
    }
    const card = document.createElement('div');
    card.className = 'quality-hud__card sim-hidden';
    card.dataset.machineId = machine.id;
    hudElement.appendChild(card);
    const monitor = new QualityMonitor({ hudElement: card });
    monitor.setVisibilityCallback(updateQualityHudVisibility);
    monitor.setMachineContext({
      id: machine.id,
      label: getMachineDisplayName(machine),
      tintColor: machine.tintColor || null,
    });
    monitor.setMotorDiagnosticsProvider(() => getMachineMotorDiagnostics(world, machine.id));
    monitor.setEnabled(isEnabled());
    machineQualityMonitors.set(machine.id, { monitor });
    const referenceOverlayState = getReferenceOverlayState?.();
    if (referenceOverlayState?.segments) {
      monitor.setReferenceSegments(referenceOverlayState.segments, referenceOverlayState.metadata);
    }
    updateQualityHudVisibility();
    return monitor;
  }

  function removeQualityMonitor(machineId) {
    const entry = machineQualityMonitors.get(machineId);
    if (!entry) {
      return;
    }
    const monitor = entry.monitor;
    const card = monitor?.hudElement || null;
    monitor?.setVisibilityCallback(null);
    monitor?.detachRemoteSystem();
    if (card instanceof HTMLElement && card.parentElement) {
      card.parentElement.removeChild(card);
    }
    monitor?.dispose();
    machineQualityMonitors.delete(machineId);
    updateQualityHudVisibility();
  }

  function clearQualityMonitors() {
    for (const machineId of Array.from(machineQualityMonitors.keys())) {
      removeQualityMonitor(machineId);
    }
  }

  function setQualityEnabledState(enabled, { fromToggle = false } = {}) {
    const next = Boolean(enabled);
    if (state) {
      state.qualityEnabled = next;
    }
    forEachQualityMonitor((monitor) => monitor.setEnabled(next));
    if (!fromToggle && qualityToggle) {
      qualityToggle.checked = next;
    }
    updateQualityHudVisibility();
  }

  function resetQualityMonitors(options = {}) {
    resetMachineMotorDiagnostics(world);
    forEachQualityMonitor((monitor) => monitor.reset(options));
  }

  function refreshAllQualityMonitors(force = false) {
    forEachQualityMonitor((monitor) => monitor.refreshHud(force));
  }

  function updateQualityHistoryUI() {
    if (!historyHud || !historyToggle || !historyList) {
      return;
    }
    if (qualityHistoryRecords.length === 0) {
      historyHud.classList.add('sim-hidden');
      historyToggle.setAttribute('aria-expanded', 'false');
      historyToggle.textContent = 'Quality History ▼';
      historyList.classList.add('sim-hidden');
      historyList.innerHTML = '';
      qualityHistoryExpanded = false;
      return;
    }
    const arrow = qualityHistoryExpanded ? '▲' : '▼';
    historyHud.classList.remove('sim-hidden');
    historyToggle.textContent = `Quality History (${qualityHistoryRecords.length}) ${arrow}`;
    historyToggle.setAttribute('aria-expanded', qualityHistoryExpanded ? 'true' : 'false');
    historyList.classList.toggle('sim-hidden', !qualityHistoryExpanded);
    if (qualityHistoryExpanded) {
      historyList.innerHTML = qualityHistoryRecords.map((record) => {
        const label = `${record.machineLabel}, ${record.jobLabel}`
          .replace(/&/g, '&amp;')
          .replace(/</g, '&lt;')
          .replace(/>/g, '&gt;')
          .replace(/"/g, '&quot;')
          .replace(/'/g, '&#39;');
        const score = Number.isFinite(record.score) ? Math.round(record.score).toString() : '--';
        return `<div class="quality-history__item"><span class="quality-history__label">${label}</span><span class="quality-history__score">${score}</span></div>`;
      }).join('');
    }
  }

  function toggleQualityHistory() {
    if (qualityHistoryRecords.length === 0) {
      return;
    }
    qualityHistoryExpanded = !qualityHistoryExpanded;
    updateQualityHistoryUI();
  }

  function recordQualityHistoryEntry({ activeJobId, lastRecordedJobId, currentJobDescriptor }) {
    if (activeJobId == null || lastRecordedJobId === activeJobId) {
      return false;
    }
    const machines = getMachines();
    const jobLabel = currentJobDescriptor?.label || currentJobDescriptor?.name || currentJobDescriptor?.key || 'Program';
    const timestamp = Date.now();
    let added = false;
    for (const [machineId, entry] of machineQualityMonitors.entries()) {
      const monitor = entry?.monitor;
      const metrics = typeof monitor?.getMetrics === 'function' ? monitor.getMetrics() : monitor?.metrics;
      const score = metrics?.score;
      if (!Number.isFinite(score)) {
        continue;
      }
      const machine = machines.find((candidate) => candidate.id === machineId);
      qualityHistoryRecords.push({
        machineId,
        jobLabel,
        machineLabel: getMachineDisplayName(machine),
        score,
        timestamp,
      });
      added = true;
    }
    if (added) {
      qualityHistoryRecords.sort((a, b) => (b.score !== a.score ? b.score - a.score : b.timestamp - a.timestamp));
      if (qualityHistoryRecords.length > QUALITY_HISTORY_MAX_ENTRIES) {
        qualityHistoryRecords.length = QUALITY_HISTORY_MAX_ENTRIES;
      }
      updateQualityHistoryUI();
    }
    return true;
  }

  function attachToRemoteSystem(remoteSystem) {
    if (!remoteSystem) {
      return;
    }
    forEachQualityMonitor((monitor) => {
      monitor.attachRemoteSystem(remoteSystem);
      monitor.refreshHud();
    });
  }

  function runFinalQualityChecks(jobState = {}) {
    forEachQualityMonitor((monitor) => monitor.runFinalCheck());
    return recordQualityHistoryEntry(jobState);
  }

  function bindUi() {
    if (!qualityToggle) {
      if (state) {
        state.qualityEnabled = false;
      }
      return;
    }
    if (!hudElement) {
      qualityToggle.checked = false;
      qualityToggle.disabled = true;
      qualityToggle.setAttribute('aria-disabled', 'true');
      if (state) {
        state.qualityEnabled = false;
      }
      return;
    }
    qualityToggle.addEventListener('change', () => {
      setQualityEnabledState(qualityToggle.checked, { fromToggle: true });
    });
    setQualityEnabledState(qualityToggle.checked, { fromToggle: true });
  }

  return {
    get enabled() {
      return isEnabled();
    },
    get monitors() {
      return machineQualityMonitors;
    },
    get historyRecords() {
      return qualityHistoryRecords;
    },
    forEachQualityMonitor,
    updateQualityHudVisibility,
    ensureQualityMonitorForMachine,
    removeQualityMonitor,
    clearQualityMonitors,
    setQualityEnabledState,
    resetQualityMonitors,
    refreshAllQualityMonitors,
    updateQualityHistoryUI,
    toggleQualityHistory,
    recordQualityHistoryEntry,
    attachToRemoteSystem,
    runFinalQualityChecks,
    bindUi,
  };
}
