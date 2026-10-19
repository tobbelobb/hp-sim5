// Wall deadlines for waits expressed in simulated time.
//
// A simulated clock advances only as fast as the physics can step, so a fixed
// wall budget measures host load rather than a stuck trial. Requested playback
// speed does not establish actual throughput; a simulated window can outlive
// its wall budget on a slow host. Keep the configured
// budget as a floor, but extend it by the wall time actually needed for the
// remaining simulated window. Only forward clock progress earns an extension,
// so a clock that has stopped still reaches the floor deadline, and the hard
// cap bounds pathological cases.
export function projectSimulationWallDeadline({
  wallDeadlineMs, hardWallDeadlineMs, remainingClockMs,
  clockAdvancedMs, wallElapsedMs, nowMs,
}) {
  if (!(clockAdvancedMs > 0) || !(wallElapsedMs > 0)) return wallDeadlineMs;
  // Nothing left to wait for: never hand out the safety margin on its own, or a
  // wall-clock-driven host could keep extending its deadline indefinitely.
  if (!(remainingClockMs > 0)) return wallDeadlineMs;
  const projectedMs = Math.max(0, remainingClockMs) * (wallElapsedMs / clockAdvancedMs);
  return Math.min(hardWallDeadlineMs, Math.max(wallDeadlineMs, nowMs + projectedMs + 5000));
}

export function simulationHardWallDeadline({ wallStartMs, wallTimeoutMs, clockBudgetMs }) {
  const budget = Number.isFinite(clockBudgetMs) && clockBudgetMs > 0 ? clockBudgetMs : 0;
  return wallStartMs + Math.max(wallTimeoutMs, budget * 20);
}
