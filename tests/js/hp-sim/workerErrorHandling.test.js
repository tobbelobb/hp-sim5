import fs from 'fs';
import path from 'path';

function readHpSimSource() {
  const scriptPath = path.resolve(process.cwd(), 'hp-sim/assets/hp-sim.js');
  return fs.readFileSync(scriptPath, 'utf8');
}

describe('hp-sim worker error handling', () => {
  test('registers onerror handlers for all print workers', () => {
    const source = readHpSimSource();

    expect(source).toContain('klipperMcuCommandPlayerWorker.onerror = (event) => {');
    expect(source).toContain('rrfCanPlayerWorker.onerror = (event) => {');
    expect(source).toContain('moveCommanderWorker.onerror = (event) => {');
  });

  test('clears active print state when workers report runtime errors', () => {
    const source = readHpSimSource();

    expect(source).toContain("if (event.data.type === 'error') {");
    expect(source).toContain('setPrintActive(false);');
  });
});
