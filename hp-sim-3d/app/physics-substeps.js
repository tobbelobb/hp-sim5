// Shared by browser controls and remote/headless settings.
export function setPhysicsSubsteps(world, count) {
  if (!Number.isInteger(count) || count < 1 || count > 16) {
    throw new RangeError('physicsSubsteps must be an integer from 1 to 16');
  }
  world.setResource('physicsSubsteps', count);
  return count;
}
