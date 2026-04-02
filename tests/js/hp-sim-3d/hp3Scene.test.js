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
import {
  World,
  GravityAffectedComponent,
  MassComponent,
  PositionComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { setupScene } from '../../../hp-sim-3d/app/setupScene.js';
import { ExtruderComponent } from '../../../hp-sim-3d/app/hangprinter_extruder.js';

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

    expect(world.query([CableJointComponent]).length).toBe(18);
    expect(world.query([CablePathComponent]).length).toBe(9);

    const extruderEntity = world.query([ExtruderComponent])[0];
    expect(extruderEntity).toBeDefined();

    const extruder = world.getComponent(extruderEntity, ExtruderComponent);
    const extruderSystem = world.systems.find((system) => system.constructor.name === 'ExtruderSystem');
    expect(extruderSystem).toBeDefined();

    extruderSystem.update(world, 0);

    expect(extruder.centerSources.default).toHaveLength(3);
    expect(extruder.centerPos.z).toBeCloseTo(0.0, 6);
    expect(extruder.centerOffsets.default.z).toBeCloseTo(-0.1, 6);
  });

  test('marks the hp3 attachment bodies with positive mass as gravity affected', () => {
    const world = new World();

    setupScene(world, stage, createCanvas(), { scenePrimPath: '/World/HangprinterScene' });

    const expectedAttachmentPositions = [
      [-0.346410161513775, -0.2, 0.1],
      [0.346410161513775, -0.2, 0.1],
      [0.0, 0.4, 0.1],
    ];

    const dynamicAttachmentEntities = world.query([MassComponent, PositionComponent]).filter((entityId) => {
      const mass = world.getComponent(entityId, MassComponent)?.mass;
      const pos = world.getComponent(entityId, PositionComponent)?.pos;
      if (!(typeof mass === 'number' && mass > 0.0 && pos)) {
        return false;
      }
      return expectedAttachmentPositions.some(([x, y, z]) =>
        Math.abs(pos.x - x) < 1e-9 &&
        Math.abs(pos.y - y) < 1e-9 &&
        Math.abs(pos.z - z) < 1e-9
      );
    });

    expect(dynamicAttachmentEntities).toHaveLength(3);
    for (const entityId of dynamicAttachmentEntities) {
      expect(world.getComponent(entityId, GravityAffectedComponent)).toBeDefined();
    }
  });
});
