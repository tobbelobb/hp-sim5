import fs from 'node:fs';

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
