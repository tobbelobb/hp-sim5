import { applyForceModeState, FixedAnchorDriftError } from '../primitives/uncalibrated_actions.mjs';

// Retry the whole direction: never mix points from before/after a slip.
export async function collectWithSlipRecovery(send, collect, {
  motorIds, forceLow, forceMid, forceMax, sensorCollectionForce,
  onRecovery, maxRetries = 3,
}) {
  let forces = { forceLow, forceMid, forceMax, sensorCollectionForce };
  const recoveries = [];
  for (let attempt = 0; ; attempt += 1) {
    try {
      const result = await collect(forces, attempt);
      return { ...result, forces, recoveries };
    } catch (error) {
      if (!(error instanceof FixedAnchorDriftError)) throw error;
      await applyForceModeState(send, { motorIds, modes: motorIds.map(() => 'position') });
      if (attempt >= maxRetries) throw new Error(`Slip recovery exhausted after ${maxRetries} retries: ${error.message}`);
      const lower = force => Math.min(force, Math.max(forceLow, force * .5));
      const reduced = {
        forceLow,
        forceMid: lower(forces.forceMid),
        forceMax: lower(forces.forceMax),
        sensorCollectionForce: Number.isFinite(forces.sensorCollectionForce)
          ? lower(forces.sensorCollectionForce) : forces.sensorCollectionForce,
      };
      if (reduced.forceMid === forces.forceMid && reduced.forceMax === forces.forceMax
        && reduced.sensorCollectionForce === forces.sensorCollectionForce) {
        throw new Error(`Cannot reduce sweep forces below idle preload: ${error.message}`);
      }
      const recovery = { discarded_attempt: attempt, anchor: error.anchor, drift_deg: error.driftDeg,
        previous_forces: forces, reduced_forces: reduced };
      recoveries.push(recovery);
      console.log(`; slip recovery ${attempt + 1}/${maxRetries}: ${error.message}; lowering forces and restarting direction`);
      await onRecovery?.(recovery);
      forces = reduced;
    }
  }
}
