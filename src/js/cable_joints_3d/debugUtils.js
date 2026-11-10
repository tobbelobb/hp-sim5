export function dumpWorldState(world) {
  const summarize = value => {
    if (value === null || typeof value !== 'object') return value;
    if (value instanceof Map) return [...value.entries()];
    if (Array.isArray(value)) return value;
    if (typeof value.toJSON === 'function') return value;
    return value.constructor?.name ?? typeof value;
  };
  return JSON.stringify({
    resources: Object.fromEntries(Object.entries(world.resources).map(([key, value]) => [key, summarize(value)])),
    entities: [...world.entities.keys()].map(id => ({
      id,
      components: [...world.entities.get(id)].map(componentClass => componentClass.name),
    })),
  }, null, 2);
}
