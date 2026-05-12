import fs from 'fs';
import path from 'path';

function readWorkspaceFile(filePath) {
  return fs.readFileSync(path.resolve(process.cwd(), filePath), 'utf8');
}

describe('hp-sim quality monitor motor diagnostics wiring', () => {
  test('connects motor diagnostics into the quality monitor cards', () => {
    const source = readWorkspaceFile('hp-sim/app/appBootstrap.js');

    expect(source).toContain("import { getMachineMotorDiagnostics } from './motor-diagnostics.js';");
    expect(source).toContain('setMotorDiagnosticsProvider(() => getMachineMotorDiagnostics(world, machine.id));');
  });
});
