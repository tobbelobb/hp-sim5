import fs from 'fs';
import path from 'path';

function readWorkspaceFile(filePath) {
  return fs.readFileSync(path.resolve(process.cwd(), filePath), 'utf8');
}

describe('hp-sim closed-loop motor toggle', () => {
  test('adds a closed-loop motor toggle beside the existing motor controls', () => {
    const html = readWorkspaceFile('hp-sim/index.html');
    const lineLayeringToggleIndex = html.indexOf('id="lineLayeringToggleWrapper"');
    const closedLoopToggleIndex = html.indexOf('id="closedLoopMotorsToggleWrapper"');

    expect(lineLayeringToggleIndex).toBeGreaterThan(-1);
    expect(closedLoopToggleIndex).toBeGreaterThan(lineLayeringToggleIndex);
    expect(html).toContain('<input id="closedLoopMotorsToggle" type="checkbox">');
    expect(html).toContain('<span>Closed Loop Motors</span>');
  });

  test('wires the toggle through the app shell feature flags', () => {
    const source = readWorkspaceFile('hp-sim/app/appBootstrap.js');

    expect(source).toContain("import { setClosedLoopMotorFeatureFlags } from './closed-loop-flags.js';");
    expect(source).toContain("const closedLoopMotorsToggle = document.getElementById('closedLoopMotorsToggle');");
    expect(source).toContain('closedLoopMotorsToggle.checked = false;');
    expect(source).toContain('setClosedLoopMotorFeatureFlags(world, next);');
    expect(source).toContain("setClosedLoopMotorsEnabledState(closedLoopMotorsToggle.checked, { fromToggle: true });");
  });
});
