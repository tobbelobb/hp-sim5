import fs from 'node:fs';
import { pos_to_motor_pos_samples_deg } from '../../../hp-sim-3d/app/workers/kinematics.js';

test('positive buildup matches the spool equation and its zero-buildup limit', () => {
  const anchors = [[100, 0, 0], [200, 0, 0]];
  const params = { U: [1, 4], O: [1, 1], H: [20, 20], L: [20, 20] };
  const angles = pos_to_motor_pos_samples_deg(anchors, [[-10, 0, 0]], 0, false, .5, [30, 30], params)[0];
  for (const [i, ma] of [1, 4].entries()) {
    const expected = 2 * (ma*360/(2*Math.PI)) / (-ma*.5) * (Math.sqrt(900-10*ma*.5)-30);
    expect(angles[i]).toBeCloseTo(expected, 10);
  }
  const linear = pos_to_motor_pos_samples_deg(anchors, [[-10, 0, 0]], 0, false, 0, [30, 30], params)[0];
  expect(linear[1]).toBeCloseTo(4*10/30*180/Math.PI);
  expect(() => pos_to_motor_pos_samples_deg(anchors, [[-1000, 0, 0]], 0, false, 1, [30, 30], params))
    .toThrow('insufficient line');
});

let MoveCommander;
beforeAll(async () => {
  global.self = { addEventListener() {} };
  global.postMessage = jest.fn();
  ({ MoveCommander } = await import('../../../hp-sim-3d/app/workers/moveCommander.js'));
});
afterAll(() => { delete global.self; delete global.postMessage; });

function gcodeStream(source) {
  return new ReadableStream({ start(controller) {
    controller.enqueue(new TextEncoder().encode(source));
    controller.close();
  } });
}

test('tracks absolute E across resets, relative moves, and retractions', async () => {
  const commander = new MoveCommander();
  commander.configure('M669 A100:0:0 B0:100:0 C0:0:100\nM666 R30:30:30');
  const commands = [];
  commander.sendCommand = async command => { commands.push(command); };

  await commander.run(gcodeStream([
    'G1 E4',
    'G1 E6',
    'G92 E10; reset without movement',
    'G1 E13',
    '  m83 ; use relative E',
    'G1 E-2',
    'G1 E1',
    'M82',
    'G1 E14',
    'G92 X5 E20',
    'G1 E21',
  ].join('\n')));

  expect(commands.map(({ type, E }) => [type, E])).toEqual([
    ['Move', 4], ['Move', 2], ['Move', 3], ['Move', 1], ['Move', 2],
    ['Add to reference', undefined], ['Move', 1],
  ]);
  expect(commander.current_pos_mm.X).toBe(5);
});

test('clears extrusion mode and position for each configured or streamed job', async () => {
  const commander = new MoveCommander();
  const config = 'M669 A100:0:0 B0:100:0 C0:0:100\nM666 R30:30:30';
  commander.configure(config);
  const extrusion = [];
  commander.sendCommand = async command => { extrusion.push(command.E); };

  for (const reconfigure of [true, false]) {
    await commander.run(gcodeStream('M83\nG1 E100'));
    if (reconfigure) {
      commander.configure(config);
      expect(commander.absolute_extrusion).toBe(true);
      expect(commander.last_e).toBe(0);
    }
    await commander.run(gcodeStream('G1 E5\nG1 E10'));
  }

  expect(extrusion).toEqual([100, 5, 5, 100, 5, 5]);
});

for (const name of ['hp5', 'hp5_pulley_reduced']) {
  test(`${name} emits all configured winches, including D and I`, async () => {
    const commander = new MoveCommander();
    commander.configure(fs.readFileSync(`RRF/run/vsd/sys/config_${name}.g`, 'utf8'));
    commander.setAsapMode(true);
    const commands = [];
    commander.sendCommand = async command => { commands.push(command); };
    await commander.run(gcodeStream('G1 X10 Y20 Z30 F60000\nG6 D1 I1 F60000\n'));
    const moves = commands.filter(command => command.type === 'Move');
    expect(moves.length).toBeGreaterThan(0);
    for (const move of moves) {
      expect(Object.keys(move).sort()).toEqual(['A','B','C','D','I','type']);
      expect([...'ABCDI'].every(axis => Number.isFinite(move[axis]))).toBe(true);
    }
    const delta = commands.at(-1);
    expect(delta.type).toBe('Add to reference');
    expect(delta.D).toBeCloseTo((name === 'hp5' ? 1 : 2) / 30);
    expect(delta.I).toBeCloseTo((name === 'hp5' ? 1 : 4) / 30);
  });
}
