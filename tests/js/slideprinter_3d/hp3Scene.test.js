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
import { World } from '../../../src/js/cable_joints_3d/ecs.js';
import {
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { setupScene } from '../../../examples/js/slideprinter_3d/setupScene.js';
import { ExtruderComponent } from '../../../examples/js/slideprinter_3d/slideprinter_common.js';

const usdPath = path.resolve(process.cwd(), 'examples/usd_scenes/hp3.usda');

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

describe('hp3 USDA scene loading', () => {
  let stage;
  const originalDocument = global.document;
  const originalWindow = global.window;

  beforeAll(async () => {
    stage = await UsdOpen(readFileSync(usdPath, 'utf8'));
  });

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

  test('loads the Hangprinter scene root with resolved joints, paths, and extruder height', () => {
    const world = new World();

    setupScene(world, stage, createCanvas(), { scenePrimPath: '/World/HangprinterScene' });

    expect(world.query([CableJointComponent]).length).toBe(12);
    expect(world.query([CablePathComponent]).length).toBe(6);

    const extruderEntity = world.query([ExtruderComponent])[0];
    expect(extruderEntity).toBeDefined();

    const extruder = world.getComponent(extruderEntity, ExtruderComponent);
    const extruderSystem = world.systems.find((system) => system.constructor.name === 'ExtruderSystem');
    expect(extruderSystem).toBeDefined();

    extruderSystem.update(world, 0);

    expect(extruder.centerSources.default).toHaveLength(3);
    expect(extruder.centerPos.z).toBeCloseTo(1.0, 6);
    expect(extruder.centerOffsets.default.z).toBeCloseTo(0.0, 6);
  });
});
