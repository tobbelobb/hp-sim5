// Verify only the local interval just probed, under one sensor preload. Encoder
// coupling is evidence of a tensioned transmission, not an empty-spool sensor.
export class UnmeasurableSweepError extends Error {
  constructor(message, evidence = {}) { super(message); this.evidence = evidence; }
}

export async function probeSpoolInterval(moveAndSample, {
  start, away, toward, driveAnchor, sensorAnchor, mmPerDeg, sigmaFloorDeg, driveToleranceMm = 0,
}) {
  const base = await moveAndSample(start);
  const farther = await moveAndSample(away);
  const returned = await moveAndSample(start);
  const threshold = (a, b, axis) => 5 * Math.hypot(
    Math.max(sigmaFloorDeg, a.sigmaByMotorDeg[axis]),
    Math.max(sigmaFloorDeg, b.sigmaByMotorDeg[axis]),
  );
  const tracks = (sample, target) => Number.isFinite(sample.muByMotorDeg[driveAnchor])
    && Math.abs(sample.muByMotorDeg[driveAnchor] - target / mmPerDeg[driveAnchor])
       <= Math.max(5 * Math.max(sigmaFloorDeg, sample.sigmaByMotorDeg[driveAnchor]), driveToleranceMm / mmPerDeg[driveAnchor]);
  const recovers = (a, b, axes = [driveAnchor, sensorAnchor]) => axes.every(axis =>
    Number.isFinite(a.muByMotorDeg[axis]) && Number.isFinite(b.muByMotorDeg[axis])
    && Math.abs(a.muByMotorDeg[axis] - b.muByMotorDeg[axis]) <= Math.max(threshold(a, b, axis),
      axis === driveAnchor ? driveToleranceMm / mmPerDeg[axis] : 0));
  const outward = farther.muByMotorDeg[sensorAnchor] - base.muByMotorDeg[sensorAnchor];
  if (!recovers(base, returned, [driveAnchor])) throw new Error('Spool probe did not restore the held drive');
  if (!recovers(base, returned)) return { accepted: false, reason: 'hysteretic sensor return' };
  if (!tracks(base, start) || !tracks(farther, away)
      || Math.abs(outward) <= threshold(base, farther, sensorAnchor)) {
    return { accepted: false, reason: 'unresolved coupling or return mismatch' };
  }
  const nearer = await moveAndSample(toward);
  const restored = await moveAndSample(start);
  if (!recovers(base, restored, [driveAnchor])) throw new Error('Spool probe did not restore the held drive');
  if (!recovers(base, restored)) return { accepted: false, reason: 'hysteretic sensor return' };
  const inward = nearer.muByMotorDeg[sensorAnchor] - base.muByMotorDeg[sensorAnchor];
  const accepted = tracks(nearer, toward)
    && Math.abs(inward) > threshold(base, nearer, sensorAnchor) && outward * inward < 0;
  return { accepted, reason: accepted ? 'reversible encoder coupling' : 'flat, reversed or hysteretic response',
    sample: restored,
    tested_drive_mm: [start, toward], sensor_response_deg: inward };
}
