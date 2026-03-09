import fs from 'fs';
import path from 'path';

function readWorkspaceFile(filePath) {
  return fs.readFileSync(path.resolve(process.cwd(), filePath), 'utf8');
}

describe('hp-sim-3d wiring', () => {
  test('points the app shell at the 3D cable joints and slideprinter modules', () => {
    const source = readWorkspaceFile('hp-sim-3d/assets/hp-sim.js');

    expect(source).toContain("../../src/js/cable_joints_3d/ecs.js");
    expect(source).toContain("../../src/js/cable_joints_3d/cable_joints_core.js");
    expect(source).toContain("../../examples/js/slideprinter_3d/runner.js");
    expect(source).toContain("../../examples/js/slideprinter_3d/setupScene.js");
    expect(source).toContain("../../examples/js/slideprinter_3d/slideprinter_common.js");
  });

  test('declares a Three.js import map in the 3D html entrypoint', () => {
    const html = readWorkspaceFile('hp-sim-3d/index.html');

    expect(html).toContain('<script type="importmap">');
    expect(html).toContain('"three": "../node_modules/three/build/three.module.js"');
    expect(html).toContain('"three/addons/": "../node_modules/three/examples/jsm/"');
  });
});
