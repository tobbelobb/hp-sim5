// A collector can exit between sweeps; the service keeps the physics world alive.
export async function createHeadlessBridge(url) {
  let timeMs = 0;
  let backend;
  let pending = Promise.resolve();
  async function request(operation, args = {}) {
    const response = await fetch(`${url}/${operation}`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(args),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error);
    timeMs = result.collector_time_s * 1000;
    backend = result.backend;
    return result;
  }
  await request('status');
  const simulationClock = {
    backend,
    now: () => timeMs,
    sleep: async ms => { await pending; await request('advance', { seconds: ms / 1000 }); },
  };
  return {
    simulationClock,
    sendGcodeLine: async line => { await pending; return (await request('gcode', { line })).result; },
    broadcast: payload => { pending = pending.then(() => request('payload', { payload })); },
    waitForHpSimConnection: async () => { await pending; },
    close() {},
  };
}
