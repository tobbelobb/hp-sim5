import fs from 'fs';
import path from 'path';

function readWorkspaceFile(filePath) {
  return fs.readFileSync(path.resolve(process.cwd(), filePath), 'utf8');
}

describe('hp-sim-3d wiring', () => {
  test('points the app shell at the 3D cable joints and slideprinter modules', () => {
    const source = readWorkspaceFile('hp-sim-3d/app/appBootstrap.js');

    expect(source).toContain("../../src/js/cable_joints_3d/ecs.js");
    expect(source).toContain("../../src/js/cable_joints_3d/cable_joints_core.js");
    expect(source).toContain("./runner.js");
    expect(source).toContain("./setupScene.js");
    expect(source).toContain("./hangprinter_extruder.js");
    expect(source).toContain("./remoteSpoolSystem.js");
    expect(source).toContain("./hangprinter_input.js");
  });

  test('keeps setupScene as a scene-loading phase coordinator', () => {
    const source = readWorkspaceFile('hp-sim-3d/app/setupScene.js');

    expect(source).toContain('readMachineSceneSpec');
    expect(source).toContain('validateMachineSceneSpec');
    expect(source).toContain('buildEntityPlan');
    expect(source).toContain('applyEntityPlan');
    expect(source).toContain('registerSceneSystems');
    expect(source).not.toContain('getAttribute(');
    expect(source).not.toContain('getRelationship(');
    expect(source).not.toContain('GetPrimAtPath(');
  });

  test('declares a Three.js import map in the 3D html entrypoint', () => {
    const html = readWorkspaceFile('hp-sim-3d/index.html');

    expect(html).toContain('<script type="importmap">');
    expect(html).toContain('"three": "../node_modules/three/build/three.module.js"');
    expect(html).toContain('"three/addons/": "../node_modules/three/examples/jsm/"');
  });

  test('adds and wires the closed-loop motor toggle in the 3D app shell', () => {
    const html = readWorkspaceFile('hp-sim-3d/index.html');
    const source = readWorkspaceFile('hp-sim-3d/app/appBootstrap.js');
    const lineLayeringToggleIndex = html.indexOf('id="lineLayeringToggleWrapper"');
    const closedLoopToggleIndex = html.indexOf('id="closedLoopMotorsToggleWrapper"');

    expect(lineLayeringToggleIndex).toBeGreaterThan(-1);
    expect(closedLoopToggleIndex).toBeGreaterThan(lineLayeringToggleIndex);
    expect(html).toContain('<input id="closedLoopMotorsToggle" type="checkbox">');
    expect(html).toContain('<span>Closed Loop Motors</span>');
    expect(source).toContain('setClosedLoopMotorFeatureFlags');
    expect(source).toContain("const closedLoopMotorsToggle = document.getElementById('closedLoopMotorsToggle');");
    expect(source).toContain('closedLoopMotorsToggle.checked = false;');
    expect(source).toContain('setClosedLoopMotorFeatureFlags(world, next);');
    expect(source).toContain("setClosedLoopMotorsEnabledState(closedLoopMotorsToggle.checked, { fromToggle: true });");
  });

  test('adds and wires the force sign toggle with the other checkbox controls', () => {
    const html = readWorkspaceFile('hp-sim-3d/index.html');
    const source = readWorkspaceFile('hp-sim-3d/app/appBootstrap.js');
    const qualityToggleIndex = html.indexOf('id="qualityToggleWrapper"');
    const showForcesToggleIndex = html.indexOf('id="showForcesToggleWrapper"');
    const lineLayeringToggleIndex = html.indexOf('id="lineLayeringToggleWrapper"');

    expect(html).toContain('class="sim-toggle-group"');
    expect(showForcesToggleIndex).toBeGreaterThan(qualityToggleIndex);
    expect(lineLayeringToggleIndex).toBeGreaterThan(showForcesToggleIndex);
    expect(html).toContain('<input id="showForcesToggle" type="checkbox">');
    expect(html).toContain('<span>Show Forces</span>');
    expect(source).toContain("const showForcesToggle = document.getElementById('showForcesToggle');");
    expect(source).toContain("world.setResource('showConstraintForces', next);");
    expect(source).toContain("setShowConstraintForcesState(showForcesToggle.checked, { fromToggle: true });");
  });

  test('connects motor diagnostics into the 3D quality monitor cards', () => {
    const source = readWorkspaceFile('hp-sim-3d/app/appBootstrap.js');

    expect(source).toContain('getMachineMotorDiagnostics,');
    expect(source).toContain('resetMachineMotorDiagnostics');
    expect(source).toContain('setMotorDiagnosticsProvider(() => getMachineMotorDiagnostics(world, machine.id));');
    expect(source).toContain('resetMachineMotorDiagnostics(world);');
  });

  test('keeps preview-safe worker URL imports for bundled print workers', () => {
    const source = readWorkspaceFile('hp-sim-3d/app/appBootstrap.js');

    expect(source).toContain('?worker&url";');
    expect(source).toContain('klipperMcuCommandPlayerWorkerUrl');
    expect(source).toContain('rrfCanPlayerWorkerUrl');
    expect(source).toContain('moveCommanderWorkerUrl');
  });
});
