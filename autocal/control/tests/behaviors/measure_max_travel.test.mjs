import { measureMaxTravelMm } from '../../behaviors/sweep_data_collection.mjs';

const motorIds = ['40.0', '41.0', '42.0', '43.0'];
const hold = `M569.4 P${motorIds.join(':')} T0.0:0.0:0.0:0.0`;
const options = {
  motorIds, mmPerDeg: [0.5, 0.5, 0.5, 0.5],
  pairAnchors: [1, 2], forbiddenForceAnchors: [3],
  forceLow: 0.011363575731199994, forceMax: 12.884911335493843,
  speedup: 25,
};

function movingMachine({ actualSpeed = 1, stopMs = 40000, stalledClock = false } = {}) {
  let simMs = 0;
  let wallMs = 0;
  let pulling = false;
  const commands = [];
  const send = jest.fn(async command => {
    commands.push(command);
    if (command.startsWith('M569.4')) pulling = command !== hold;
    return { reply: `${pulling ? Math.min(simMs, stopMs) / 40 : 0} 0 0 0` };
  });
  send.simulationClock = {
    now: () => simMs,
    // The browser's ordinary settling budget stays at 30 seconds.
    settlingTimeoutMs: 30000,
    sleep: async ms => {
      if (!stalledClock) simMs += ms;
      wallMs += stalledClock ? 1000 : ms / actualSpeed;
    },
  };
  return { send, commands, now: () => simMs, wallNow: () => wallMs };
}

function settling(machine) {
  return { wallNowFn: machine.wallNow, onProgress: jest.fn() };
}

describe('workspace-size measurement', () => {
  test.each([0.57, 25])('allows active travel beyond 30 seconds at achieved speed %sx', async actualSpeed => {
    const machine = movingMachine({ actualSpeed });
    const distance = await measureMaxTravelMm(machine.send, {
      ...options, settleOptions: settling(machine),
    });
    expect(distance).toBe(500);
    expect(machine.now()).toBeGreaterThan(40000);
    expect(machine.commands.at(-1)).toBe(hold);
    expect(machine.commands).toContain(
      'M569.4 P40.0:41.0:42.0:43.0 T0.011363575731199994:9.663683501620383:9.663683501620383:0.0',
    );
  });

  test('keeps the active-travel deadline finite and holds motors on timeout', async () => {
    const machine = movingMachine({ stopMs: Infinity });
    await expect(measureMaxTravelMm(machine.send, {
      ...options, settleOptions: { ...settling(machine), wallTimeoutMs: 200000 },
    })).rejects.toThrow('after 120000ms');
    expect(machine.commands.at(-1)).toBe(hold);
  });

  test('a stopped simulation reaches its independent wall deadline and holds motors', async () => {
    const machine = movingMachine({ stalledClock: true });
    await expect(measureMaxTravelMm(machine.send, {
      ...options, settleOptions: { ...settling(machine), wallTimeoutMs: 6000 },
    })).rejects.toThrow('wall-clock deadline');
    expect(machine.commands.at(-1)).toBe(hold);
  });

  test.each(['ramp', 'encoder'])('holds motors if the %s transport fails', async failure => {
    const machine = movingMachine();
    const original = machine.send.getMockImplementation();
    let rampCommands = 0;
    machine.send.mockImplementation(async command => {
      if (command.startsWith('M569.4') && command !== hold) rampCommands += 1;
      if ((failure === 'ramp' && rampCommands === 2 && command !== hold)
        || (failure === 'encoder' && command.startsWith('M569.3') && machine.now() > 0)) {
        throw new Error(`${failure} disconnected`);
      }
      return original(command);
    });
    await expect(measureMaxTravelMm(machine.send, {
      ...options, settleOptions: settling(machine),
    })).rejects.toThrow(`${failure} disconnected`);
    expect(machine.commands.at(-1)).toBe(hold);
  });
});
