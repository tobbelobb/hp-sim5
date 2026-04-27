jest.mock('../../../src/js/cable_joints_3d/render_system_3d.js', () => {
  class MockRenderSystem3D {
    constructor(canvas, options) {
      this.canvas = canvas;
      this.options = options;
      this.setViewTransform = jest.fn();
    }

    resetVisuals() {}
    setCanvasSize() {}
    update() {}
  }

  return { RenderSystem3D: MockRenderSystem3D };
});

import { readFileSync } from 'fs';
import path from 'path';
import { Open as UsdOpen } from '../../../src/js/usd/stage.js';
import { bakeCableSceneUsdaSource } from '../../../src/js/usd/cable_scene_baker.js';
import {
  World,
  RigidBodyComponent,
  RigidBodyMemberComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { setupScene } from '../../../hp-sim-3d/app/setupScene.js';

function createCanvas() {
  return {
    clientWidth: 640,
    clientHeight: 960,
    width: 0,
    height: 0,
    style: {},
    setAttribute() {},
    focus() {},
    addEventListener() {},
    removeEventListener() {},
    getBoundingClientRect() {
      return {
        left: 0,
        top: 0,
        width: this.clientWidth,
        height: this.clientHeight,
      };
    },
  };
}

describe('rigid-body USDA scene loading', () => {
  const originalDocument = global.document;
  const originalWindow = global.window;

  beforeEach(() => {
    global.document = {
      getElementById() {
        return { textContent: 'Pause' };
      },
      addEventListener() {},
      removeEventListener() {},
      body: { style: {} },
      documentElement: { style: {} },
    };
    global.window = {
      addEventListener() {},
      removeEventListener() {},
    };
  });

  afterEach(() => {
    global.document = originalDocument;
    global.window = originalWindow;
  });

  test.each([
    ['slideprinter_multi_unit_rigid_body.usda', 3],
    ['slideprinter_single_pinholes_rigid_body.usda', 6],
    ['slideprinter_hexagon_rigid_body.usda', 9],
  ])('loads %s into a single rigid-body assembly', async (fileName, expectedMembers) => {
    const source = readFileSync(path.resolve(process.cwd(), 'public/usd_scenes', fileName), 'utf8');
    const baked = bakeCableSceneUsdaSource(source).source;
    const stage = await UsdOpen(
      baked,
    );
    const world = new World();

    setupScene(world, stage, createCanvas());

    expect(world.query([RigidBodyComponent])).toHaveLength(1);
    expect(world.query([RigidBodyMemberComponent])).toHaveLength(expectedMembers);
    expect(world.query([CableJointComponent]).length).toBeGreaterThan(0);
    expect(world.query([CablePathComponent]).length).toBeGreaterThan(0);
  });
});
