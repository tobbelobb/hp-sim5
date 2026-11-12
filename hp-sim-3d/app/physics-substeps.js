// Shared by browser controls and remote/headless settings.
export function setPhysicsSubsteps(world, count, { override = true } = {}) {
  if (!Number.isInteger(count) || count < 1 || count > 16) {
    throw new RangeError('physicsSubsteps must be an integer from 1 to 16');
  }
  if (override) world.setResource('physicsSubstepsOverride', count);
  world.setResource('physicsSubsteps', count);
  return count;
}
