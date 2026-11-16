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

for (const name of ['hp5', 'hp5_pulley_reduced']) {
  test(`${name} emits all configured winches, including D and I`, async () => {
    const commander = new MoveCommander();
    commander.configure(fs.readFileSync(`RRF/run/vsd/sys/config_${name}.g`, 'utf8'));
    commander.setAsapMode(true);
    const commands = [];
    commander.sendCommand = async command => { commands.push(command); };
    const source = 'G1 X10 Y20 Z30 F60000\nG6 D1 I1 F60000\n';
    await commander.run(new ReadableStream({ start(controller) {
      controller.enqueue(new TextEncoder().encode(source)); controller.close();
    } }));
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
