import fs from 'fs';
import path from 'path';

function readWorkspaceFile(filePath) {
  return fs.readFileSync(path.resolve(process.cwd(), filePath), 'utf8');
}

describe('hp-sim-3d wiring', () => {
  test('keeps appBootstrap as a small composition root', () => {
    const source = readWorkspaceFile('hp-sim-3d/app/appBootstrap.js');
    const bannedBootstrapDetails = [
      'function initHpSim',
      'handleFileUpload',
      'parseGcodeText',
      'connectExternalCommandStream',
      'addUsdMachineFromFile',
      'addUsdaFromCatalog',
      'ensureQualityMonitorForMachine',
      'setReferenceSegments',
      'setLineLayeringEnabledState',
      'setPrintActive',
    ];

    expect(source.split('\n').length).toBeLessThan(300);
    for (const detail of bannedBootstrapDetails) {
      expect(source).not.toContain(detail);
    }
    expect(source).toContain('createMachineSceneController');
    expect(source).toContain('createMachineMenuController');
    expect(source).toContain('createCommandJobController');
    expect(source).toContain('createInspectionToolsController');
  });

  test('points the app shell at the 3D cable joints and slideprinter modules', () => {
    const bootstrap = readWorkspaceFile('hp-sim-3d/app/appBootstrap.js');
    const featureFlags = readWorkspaceFile('hp-sim-3d/app/featureFlagsController.js');
    const sceneRuntime = readWorkspaceFile('hp-sim-3d/app/sceneController.js');
    const machineScene = readWorkspaceFile('hp-sim-3d/app/machineSceneController.js');
    const view = readWorkspaceFile('hp-sim-3d/app/viewController.js');
    const commandJobs = readWorkspaceFile('hp-sim-3d/app/commandJobController.js');

    expect(bootstrap).toContain("../../src/js/cable_joints_3d/ecs.js");
    expect(featureFlags).toContain("../../src/js/cable_joints_3d/cable_joints_core.js");
    expect(sceneRuntime).toContain("./runner.js");
    expect(machineScene).toContain("./setupScene.js");
    expect(readWorkspaceFile('hp-sim-3d/app/scene/extruderSceneBinding.js')).toContain("../hangprinter_extruder.js");
    expect(commandJobs).toContain("./remoteSpoolSystem.js");
    expect(view).toContain("./hangprinter_input.js");
  });

  test('keeps machine menu behavior out of the machine scene controller', () => {
    const machineScene = readWorkspaceFile('hp-sim-3d/app/machineSceneController.js');
    const machineMenu = readWorkspaceFile('hp-sim-3d/app/machineMenuController.js');

    expect(machineScene).not.toContain('MACHINE_MENU_HOVER_CLOSE_DELAY_MS');
    expect(machineScene).not.toContain('syncMachineMenuPlacement');
    expect(machineScene).not.toContain('clearMachineMenuHoverTimeout');
    expect(machineMenu).toContain('function syncPlacement');
    expect(machineMenu).toContain("dom.machinesToggle.addEventListener('click'");
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

  test('keeps scene pipeline separate from system registration', () => {
    const pipeline = readWorkspaceFile('hp-sim-3d/app/scene/machineScenePipeline.js');
    const applier = readWorkspaceFile('hp-sim-3d/app/scene/entityPlanApplier.js');
    const entityBuilders = readWorkspaceFile('hp-sim-3d/app/scene/machineEntityBuilders.js');

    for (const source of [pipeline, applier, entityBuilders]) {
      expect(source).not.toContain('registerSystem');
      expect(source).not.toContain('registerSystems');
      expect(source).not.toContain('RenderSystem3D');
      expect(source).not.toContain('InputSystem');
      expect(source).not.toContain('RemoteInputSystem');
      expect(source).not.toContain('document.getElementById');
    }
    expect(pipeline).toContain('buildSceneResources');
    expect(pipeline).toContain('applyEntityPlan');
  });

  test('keeps scene feature builders as component owners', () => {
    const files = [
      'hp-sim-3d/app/scene/machineEntityBuilders.js',
      'hp-sim-3d/app/scene/rigidBodyBuilder.js',
      'hp-sim-3d/app/scene/distanceJointBuilder.js',
      'hp-sim-3d/app/scene/cableJointBuilder.js',
      'hp-sim-3d/app/scene/cablePathBuilder.js',
      'hp-sim-3d/app/scene/extruderSceneBinding.js',
    ];
    const sources = Object.fromEntries(files.map((file) => [file, readWorkspaceFile(file)]));
    const filesContaining = (needle) => files.filter((file) => sources[file].includes(needle));

    expect(filesContaining('CableJointComponent')).toEqual(['hp-sim-3d/app/scene/cableJointBuilder.js']);
    expect(filesContaining('CablePathComponent')).toEqual(['hp-sim-3d/app/scene/cablePathBuilder.js']);
    expect(filesContaining('DistanceConstraintComponent')).toEqual(['hp-sim-3d/app/scene/distanceJointBuilder.js']);
    expect(filesContaining('RigidBodyComponent')).toEqual(['hp-sim-3d/app/scene/rigidBodyBuilder.js']);
    expect(filesContaining('RigidBodyMemberComponent')).toEqual(['hp-sim-3d/app/scene/rigidBodyBuilder.js']);
    expect(filesContaining('ExtruderComponent')).toEqual(['hp-sim-3d/app/scene/extruderSceneBinding.js']);

    const machineBodiesLineCount = sources['hp-sim-3d/app/scene/machineEntityBuilders.js'].split('\n').length;
    expect(machineBodiesLineCount).toBeLessThan(350);
    expect(sources['hp-sim-3d/app/scene/machineEntityBuilders.js']).not.toContain('function readVectorAttribute');
    expect(sources['hp-sim-3d/app/scene/machineEntityBuilders.js']).not.toContain('function scopedKey');
  });

  test('keeps sceneSystems separate from USD scene interpretation', () => {
    const source = readWorkspaceFile('hp-sim-3d/app/sceneSystems.js');

    expect(source).not.toContain('getAttribute(');
    expect(source).not.toContain('getRelationship(');
    expect(source).not.toContain('GetPrimAtPath(');
    expect(source).not.toContain('CableJoint');
    expect(source).not.toContain('RigidGroup');
    expect(source).not.toContain('DistancePhysicsJoint');
  });

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

    expect(html).toContain('class="sim-toggle-group"');
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
