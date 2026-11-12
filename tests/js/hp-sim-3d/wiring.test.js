import fs from 'fs';
import path from 'path';

function readWorkspaceFile(filePath) {
  return fs.readFileSync(path.resolve(process.cwd(), filePath), 'utf8');
}

describe('hp-sim-3d wiring', () => {







  test('declares a Three.js import map in the 3D html entrypoint', () => {
    const html = readWorkspaceFile('hp-sim-3d/index.html');

    expect(html).toContain('<script type="importmap">');
    expect(html).toContain('"three": "../node_modules/three/build/three.module.js"');
    expect(html).toContain('"three/addons/": "../node_modules/three/examples/jsm/"');
  });

  test('adds and wires the closed-loop motor toggle in the 3D app shell', () => {
    const html = readWorkspaceFile('hp-sim-3d/index.html');
    const bootstrap = readWorkspaceFile('hp-sim-3d/app/appBootstrap.js');
    const featureFlags = readWorkspaceFile('hp-sim-3d/app/featureFlagsController.js');
    const lineLayeringToggleIndex = html.indexOf('id="lineLayeringToggleWrapper"');
    const closedLoopToggleIndex = html.indexOf('id="closedLoopMotorsToggleWrapper"');

    expect(lineLayeringToggleIndex).toBeGreaterThan(-1);
    expect(closedLoopToggleIndex).toBeGreaterThan(lineLayeringToggleIndex);
    expect(html).toContain('<input id="closedLoopMotorsToggle" type="checkbox">');
    expect(html).toContain('<span>Closed Loop Motors</span>');
    expect(bootstrap).toContain("closedLoopMotorsToggle: ownerDocument.getElementById('closedLoopMotorsToggle')");
    expect(featureFlags).toContain('setClosedLoopMotorFeatureFlags');
    expect(featureFlags).toContain('setClosedLoopMotorFeatureFlags(world, next);');
    expect(featureFlags).toContain("setClosedLoopMotorsEnabledState(toggles.closedLoopMotorsToggle.checked, { fromToggle: true });");
  });

  test('adds and wires the force sign toggle with the other checkbox controls', () => {
    const html = readWorkspaceFile('hp-sim-3d/index.html');
    const bootstrap = readWorkspaceFile('hp-sim-3d/app/appBootstrap.js');
    const featureFlags = readWorkspaceFile('hp-sim-3d/app/featureFlagsController.js');
    const qualityToggleIndex = html.indexOf('id="qualityToggleWrapper"');
    const showForcesToggleIndex = html.indexOf('id="showForcesToggleWrapper"');
    const lineLayeringToggleIndex = html.indexOf('id="lineLayeringToggleWrapper"');

    expect(html).toContain('<div id="settingsMenu" class="sim-settings-menu sim-hidden"');
    expect(showForcesToggleIndex).toBeGreaterThan(qualityToggleIndex);
    expect(lineLayeringToggleIndex).toBeGreaterThan(showForcesToggleIndex);
    expect(html).toContain('<input id="showForcesToggle" type="checkbox">');
    expect(html).toContain('<span>Show Forces</span>');
    expect(bootstrap).toContain("showForcesToggle: ownerDocument.getElementById('showForcesToggle')");
    expect(featureFlags).toContain("world.setResource('showConstraintForces', next);");
    expect(featureFlags).toContain("setShowConstraintForcesState(toggles.showForcesToggle.checked, { fromToggle: true });");
  });

  test('connects motor diagnostics into the 3D quality monitor cards', () => {
    const source = readWorkspaceFile('hp-sim-3d/app/qualityController.js');

    expect(source).toContain('getMachineMotorDiagnostics,');
    expect(source).toContain('resetMachineMotorDiagnostics');
    expect(source).toContain('setMotorDiagnosticsProvider(() => getMachineMotorDiagnostics(world, machine.id));');
    expect(source).toContain('resetMachineMotorDiagnostics(world);');
  });

  test('keeps preview-safe worker URL imports for bundled print workers', () => {
    const source = readWorkspaceFile('hp-sim-3d/app/workerController.js');

    expect(source).toContain('?worker&url');
    expect(source).toContain('klipperMcuCommandPlayerWorkerUrl');
    expect(source).toContain('rrfCanPlayerWorkerUrl');
    expect(source).toContain('moveCommanderWorkerUrl');
  });
});
