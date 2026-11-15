// Verify only the local interval just probed, under one sensor preload. Encoder
// coupling is evidence of a tensioned transmission, not an empty-spool sensor.
export async function probeSpoolInterval(moveAndSample, {
  start, away, toward, driveAnchor, sensorAnchor, mmPerDeg, sigmaFloorDeg,
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
       <= 5 * Math.max(sigmaFloorDeg, sample.sigmaByMotorDeg[driveAnchor]);
  const recovers = (a, b) => [driveAnchor, sensorAnchor].every(axis =>
    Number.isFinite(a.muByMotorDeg[axis]) && Number.isFinite(b.muByMotorDeg[axis])
    && Math.abs(a.muByMotorDeg[axis] - b.muByMotorDeg[axis]) <= threshold(a, b, axis));
  const outward = farther.muByMotorDeg[sensorAnchor] - base.muByMotorDeg[sensorAnchor];
  if (!recovers(base, returned)) throw new Error('Spool probe did not return to the reserved encoder state');
  if (!tracks(base, start) || !tracks(farther, away)
      || Math.abs(outward) <= threshold(base, farther, sensorAnchor)) {
    return { accepted: false, reason: 'unresolved coupling or return mismatch' };
  }
  const nearer = await moveAndSample(toward);
  const restored = await moveAndSample(start);
  if (!recovers(base, restored)) throw new Error('Spool probe did not return to the reserved encoder state');
  const inward = nearer.muByMotorDeg[sensorAnchor] - base.muByMotorDeg[sensorAnchor];
  const accepted = tracks(nearer, toward)
    && Math.abs(inward) > threshold(base, nearer, sensorAnchor) && outward * inward < 0;
  return { accepted, reason: accepted ? 'reversible encoder coupling' : 'flat, reversed or hysteretic response',
    tested_drive_mm: [start, toward], sensor_response_deg: inward };
}
