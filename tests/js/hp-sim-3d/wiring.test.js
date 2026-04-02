import fs from 'fs';
import path from 'path';

function readWorkspaceFile(filePath) {
  return fs.readFileSync(path.resolve(process.cwd(), filePath), 'utf8');
}

describe('hp-sim-3d wiring', () => {
  test('points the app shell at the 3D cable joints and slideprinter modules', () => {
    const source = readWorkspaceFile('hp-sim-3d/app/hp-sim-3d.js');

    expect(source).toContain("../../src/js/cable_joints_3d/ecs.js");
    expect(source).toContain("../../src/js/cable_joints_3d/cable_joints_core.js");
    expect(source).toContain("./runner.js");
    expect(source).toContain("./setupScene.js");
    expect(source).toContain("./hangprinter_extruder.js");
    expect(source).toContain("./remoteSpoolSystem.js");
    expect(source).toContain("./hangprinter_input.js");
    expect(source).toContain("./replay_state.js");
  });

  test('declares a Three.js import map in the 3D html entrypoint', () => {
    const html = readWorkspaceFile('hp-sim-3d/index.html');

    expect(html).toContain('<script type="importmap">');
    expect(html).toContain('"three": "../node_modules/three/build/three.module.js"');
    expect(html).toContain('"three/addons/": "../node_modules/three/examples/jsm/"');
  });

  test('adds and wires the closed-loop motor toggle in the 3D app shell', () => {
    const html = readWorkspaceFile('hp-sim-3d/index.html');
    const source = readWorkspaceFile('hp-sim-3d/app/hp-sim-3d.js');
    const lineLayeringToggleIndex = html.indexOf('id="lineLayeringToggleWrapper"');
    const closedLoopToggleIndex = html.indexOf('id="closedLoopMotorsToggleWrapper"');

    expect(lineLayeringToggleIndex).toBeGreaterThan(-1);
    expect(closedLoopToggleIndex).toBeGreaterThan(lineLayeringToggleIndex);
    expect(html).toContain('<input id="closedLoopMotorsToggle" type="checkbox">');
    expect(html).toContain('<span>Closed Loop Motors</span>');
    expect(source).toContain("import { setClosedLoopMotorFeatureFlags } from './closed-loop-flags.js';");
    expect(source).toContain("const closedLoopMotorsToggle = document.getElementById('closedLoopMotorsToggle');");
    expect(source).toContain('closedLoopMotorsToggle.checked = false;');
    expect(source).toContain('setClosedLoopMotorFeatureFlags(world, next);');
    expect(source).toContain("setClosedLoopMotorsEnabledState(closedLoopMotorsToggle.checked, { fromToggle: true });");
  });

  test('connects motor diagnostics into the 3D quality monitor cards', () => {
    const source = readWorkspaceFile('hp-sim-3d/app/hp-sim-3d.js');

    expect(source).toContain("import { getMachineMotorDiagnostics } from './motor-diagnostics.js';");
    expect(source).toContain('setMotorDiagnosticsProvider(() => getMachineMotorDiagnostics(world, machine.id));');
  });
});
