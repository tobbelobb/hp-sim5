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
    requestRender() {}
  }

  return { RenderSystem3D: MockRenderSystem3D };
});

const fs = require('node:fs');
const path = require('node:path');

const { bakeCableSceneUsdaSource } = require('../../../src/js/usd/cable_scene_baker.js');
const { OpenText } = require('../../../src/js/usd/stage.js');
const { setupScene } = require('../../../hp-sim-3d/app/setupScene.js');
const { setLineLayeringFeatureFlags } = require('../../../hp-sim-3d/app/line-layering-flags.js');
const {
  World,
  RadiusComponent,
} = require('../../../src/js/cable_joints_3d/ecs.js');
const {
  CableJointComponent,
  CableLinkComponent,
  CablePathComponent,
} = require('../../../src/js/cable_joints_3d/cable_joints_core.js');
const {
  getEntityWorldOrientation,
  getEntityWorldPosition,
} = require('../../../src/js/cable_joints_3d/rigid_bodies.js');
const { signedArcLengthOnWheel } = require('../../../src/js/cable_joints_3d/geometry3.js');
const Vector3 = require('../../../src/js/cable_joints_3d/vector3.js').default;

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
      return { left: 0, top: 0, width: this.clientWidth, height: this.clientHeight };
    },
  };
}

function isRollingLike(linkType) {
  return linkType === 'rolling' || linkType === 'hybrid';
}

function cablePlaneNormal(world, entityId) {
  const linkComp = world.getComponent(entityId, CableLinkComponent);
  if (linkComp?.cablePlaneNormalLocal) {
    const orientation = getEntityWorldOrientation(world, entityId);
    if (orientation) {
      const worldAxis = orientation.transformVector(linkComp.cablePlaneNormalLocal.clone());
      if (worldAxis.lengthSq() > 1e-12) {
        return worldAxis.normalize();
      }
    }
    return linkComp.cablePlaneNormalLocal.clone().normalize();
  }
  return linkComp?.cablePlaneNormal ?? null;
}

function geometricCablePathLength(world, pathComp) {
  let length = 0.0;

  for (const jointEntity of pathComp.jointEntities) {
    const joint = world.getComponent(jointEntity, CableJointComponent);
    length += joint.attachmentPointA_world.distanceTo(joint.attachmentPointB_world);
  }

  for (let linkIndex = 1; linkIndex < pathComp.linkTypes.length - 1; linkIndex += 1) {
    if (!isRollingLike(pathComp.linkTypes[linkIndex])) {
      continue;
    }

    const previousJoint = world.getComponent(pathComp.jointEntities[linkIndex - 1], CableJointComponent);
    const nextJoint = world.getComponent(pathComp.jointEntities[linkIndex], CableJointComponent);
    const linkEntity = previousJoint.entityB;
    expect(linkEntity).toBe(nextJoint.entityA);

    const center = getEntityWorldPosition(world, linkEntity);
    const radius = world.getComponent(linkEntity, RadiusComponent)?.radius ?? 0.0;
    const planeNormal = cablePlaneNormal(world, linkEntity);
    const arcLength = signedArcLengthOnWheel(
      previousJoint.attachmentPointB_world,
      nextJoint.attachmentPointA_world,
      center,
      radius,
      Boolean(pathComp.cw[linkIndex]),
      planeNormal,
      true
    );
    length += arcLength;
  }

  const lastLinkIndex = pathComp.linkTypes.length - 1;
  for (const linkIndex of [0, lastLinkIndex]) {
    if (isRollingLike(pathComp.linkTypes[linkIndex])) {
      length += pathComp.stored[linkIndex] ?? 0.0;
    }
  }

  return length;
}

function expectAllCablePathsTight(world, phase) {
  const cablePathEntities = world.query([CablePathComponent]);
  expect(cablePathEntities.length).toBeGreaterThan(0);

  for (const pathEntity of cablePathEntities) {
    const pathComp = world.getComponent(pathEntity, CablePathComponent);
    const geometricLength = geometricCablePathLength(world, pathComp);
    try {
      expect(pathComp.totalRestLength).toBeCloseTo(geometricLength, 9);
    } catch (error) {
      error.message = `${phase}: ${error.message}`;
      throw error;
    }
  }
}

describe('hp4 no-line-layering bake tightness', () => {
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

  test('baked hp4 cable paths are exactly tight before and after one simulation step', () => {
    const sourcePath = path.resolve(__dirname, '../../../public/usd_scenes/hp4_rigid_body.usda');
    const source = fs.readFileSync(sourcePath, 'utf8');
    const baked = bakeCableSceneUsdaSource(source, { cablePathHalfWidthOverride: 0.0, deriveAll: true });
    const stage = OpenText(baked.source);
    const world = new World();

    setLineLayeringFeatureFlags(world, false);
    setupScene(world, stage, createCanvas(), {
      scenePrimPath: '/World/HangprinterScene',
    });
    setLineLayeringFeatureFlags(world, false);

    expectAllCablePathsTight(world, 'initial');
    world.setResource('gravity', new Vector3(0.0, 0.0, 0.0));
    world.update(world.getResource('dt'));
    expectAllCablePathsTight(world, 'after one step');
  });
});
