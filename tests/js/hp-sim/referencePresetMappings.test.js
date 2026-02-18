import fs from 'fs';
import path from 'path';

function readWorkspaceFile(filePath) {
  const absolutePath = path.resolve(process.cwd(), filePath);
  return fs.readFileSync(absolutePath, 'utf8');
}

describe('hp-sim reference preset mappings', () => {
  test('registers draw_squares_bigger as its own reference preset', () => {
    const source = readWorkspaceFile('hp-sim/assets/hp-sim.js');

    expect(source).toContain('straightMovesBigger');
    expect(source).toContain('draw_squares_bigger.gcode');
    expect(source).toContain("label: 'Draw Bigger Squares (G-code)'");
  });

  test('checks draw_squares_bigger before draw_squares in default matching', () => {
    const source = readWorkspaceFile('hp-sim/assets/hp-sim.js');
    const biggerIndex = source.indexOf("{ substring: 'draw_squares_bigger', presetKey: 'straightMovesBigger' }");
    const defaultIndex = source.indexOf("{ substring: 'draw_squares', presetKey: 'straightMoves' }");

    expect(biggerIndex).toBeGreaterThan(-1);
    expect(defaultIndex).toBeGreaterThan(-1);
    expect(biggerIndex).toBeLessThan(defaultIndex);
  });

  test('keeps draw_squares_bigger ahead of draw_squares in html upload mapping', () => {
    const html = readWorkspaceFile('hp-sim/index.html');
    const biggerIndex = html.indexOf('draw_squares_bigger=straightMovesBigger');
    const defaultIndex = html.indexOf('draw_squares=straightMoves');

    expect(biggerIndex).toBeGreaterThan(-1);
    expect(defaultIndex).toBeGreaterThan(-1);
    expect(biggerIndex).toBeLessThan(defaultIndex);
  });

  test('draw_squares_bigger reference gcode closes five squares', () => {
    const gcode = readWorkspaceFile('public/examples/gcode/draw_squares_bigger.gcode');
    const closureMoves = gcode.match(/G1 X(?:200|300|400|500|600) Y(?:200|300|400|500|600) Z0 E(?:50|60|70|80)/g) || [];

    expect(closureMoves).toHaveLength(5);
  });
});
