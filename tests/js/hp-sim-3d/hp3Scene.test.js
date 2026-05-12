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
  RigidBodyComponent,
  GravityAffectedComponent,
  MassComponent,
  PositionComponent,
  RigidBodyMemberComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { setupScene } from '../../../hp-sim-3d/app/setupScene.js';
import { ExtruderComponent } from '../../../hp-sim-3d/app/hangprinter_extruder.js';

import { bakeCableSceneUsdaSource } from '../../../src/js/usd/cable_scene_baker.js';

const usdPath = path.resolve(process.cwd(), 'public/usd_scenes/hp3_rigid_body.usda');

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

  beforeEach(async () => {
    const source = readFileSync(usdPath, 'utf8');
    stage = await UsdOpen(bakeCableSceneUsdaSource(source).source);
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
    expect(extruder.centerPos.z).toBeCloseTo(0.1, 6);
    expect(extruder.centerOffsets.default.z).toBeCloseTo(0.0, 6);
    expect(extruder.tipOffsets.default.z).toBeCloseTo(-0.1, 6);
    expect(extruder.coldEndOffsets.default.z).toBeCloseTo(0.0, 6);
    expect(extruder.tipPos.z).toBeCloseTo(0.0, 6);
    expect(extruder.coldEndPos.z).toBeCloseTo(0.1, 6);
  });

  test('groups the hp3 attachment bodies into a rigid effector', () => {
    const world = new World();

    setupScene(world, stage, createCanvas(), { scenePrimPath: '/World/HangprinterScene' });

    const rigidBodyEntities = world.query([RigidBodyComponent]);
    expect(rigidBodyEntities).toHaveLength(1);

    const rigidBody = world.getComponent(rigidBodyEntities[0], RigidBodyComponent);
    expect(rigidBody.members).toHaveLength(3);

    const memberStates = rigidBody.members.map((entityId) => ({
      entityId,
      member: world.getComponent(entityId, RigidBodyMemberComponent),
      mass: world.getComponent(entityId, MassComponent)?.mass,
      pos: world.getComponent(entityId, PositionComponent)?.pos,
      gravityAffected: world.getComponent(entityId, GravityAffectedComponent),
    }));

    expect(memberStates.every(({ member }) => member)).toBe(true);
    expect(memberStates.every(({ mass }) => mass === 0)).toBe(true);
    expect(memberStates.every(({ gravityAffected }) => gravityAffected === undefined)).toBe(true);

    const expectedLocalPositions = [
      [-0.346410161513775, -0.2, 0.0],
      [0.346410161513775, -0.2, 0.0],
      [0.0, 0.4, 0.0],
    ];

    memberStates.forEach(({ member }, index) => {
      const pos = member.localPosition;
      const [expectedX, expectedY, expectedZ] = expectedLocalPositions[index];
      expect(pos.x).toBeCloseTo(expectedX, 12);
      expect(pos.y).toBeCloseTo(expectedY, 12);
      expect(pos.z).toBeCloseTo(expectedZ, 12);
    });
  });
});
