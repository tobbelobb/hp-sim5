import { settleOnTargets } from '../../primitives/uncalibrated_actions.mjs';

// A motor whose encoder follows its command up to a reachable limit, like a
// saturated closed-loop motor pulling against another held line.
function motor({ command, limit }) {
  const state = { command, limit };
  const encoder = () => Math.max(state.command, state.limit);
  let now = 0;
  const sent = [];
  const send = async line => {
    sent.push(line);
    const move = line.match(/G1 H2 V([-\d.]+)/);
    if (move) state.command += Number(move[1]);
    return { reply: line.startsWith('M569.3') ? `0 ${encoder()}` : '' };
  };
  send.simulationClock = { now: () => now, sleep: async ms => { now += ms; } };
  return { send, sent, state, encoder };
}

const options = { motorIds: ['40.0', '44.0'], axes: ['X', 'V'], mmPerDeg: [1, 1], feed: 1000 };

describe('settleOnTargets', () => {
  test('a reached target needs no move', async () => {
    const { send, sent } = motor({ command: -300, limit: -1000 });
    const held = await settleOnTargets(send, { ...options, targets: [null, -300], starts: [0, 0] });
    expect(held[1]).toBeCloseTo(-300);
    expect(sent.some(line => line.startsWith('G1'))).toBe(false);
  });

  test('a line within tolerance holds its planned target, not the encoder read', async () => {
    // Force profiles are keyed by the planned target; a 0.005 mm read difference must not change it.
    const { send } = motor({ command: -300.005, limit: -1000 });
    const held = await settleOnTargets(send, { ...options, targets: [null, -300], starts: [0, 0] });
    expect(held[1]).toBe(-300);
  });

  test('a saturated motor backs off and reports the length it holds', async () => {
    // Commanded to -620, it stopped at -600 and keeps straining.
    const { send, state, encoder } = motor({ command: -620, limit: -600 });
    const held = await settleOnTargets(send, { ...options, targets: [null, -620], starts: [0, 0] });
    // Backed off by three times its 20 mm miss, and the command now matches the encoder.
    expect(held[1]).toBeCloseTo(-540);
    expect(state.command).toBeCloseTo(encoder());
  });
});
