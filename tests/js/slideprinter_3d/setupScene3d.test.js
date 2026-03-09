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

jest.mock('../../../src/js/usd/stage.js', () => ({
  getAttribute: jest.fn((prim, attr) => {
    if (prim?.path === '/World/PhysicsScene') {
      if (attr === 'physics:gravityDirection') return [0.0, -1.0, 0.0];
      if (attr === 'physics:gravityMagnitude') return 9.82;
    }
    if (prim?.path === '/World/SlideprinterScene/SpoolA') {
      if (attr === 'ecs:tags') return ['Spool'];
      if (attr === 'xformOp:translate') return [0.2, 0.3, 0.0];
      if (attr === 'radius') return 0.05;
      if (attr === 'physics:mass') return 1.0;
      if (attr === 'physics:inertiaTensor') return [[0, 0, 0], [0, 0, 0], [0, 0, 0.01]];
      if (attr === 'physics:velocity') return [0.0, 0.0, 0.0];
      if (attr === 'physics:angularVelocity') return [0.0, 0.0, 0.0];
      if (attr === 'stepper:holdingTorque') return 0.7;
      if (attr === 'stepper:numPolePairs') return 50;
      if (attr === 'stepper:dampingCoeff') return 0.01;
      if (attr === 'stepper:maxSpeedRad') return 600;
      if (attr === 'cable:linkable') return true;
    }
    return null;
  }),
  getChild: jest.fn(() => null),
  getChildren: jest.fn((prim) => {
    if (prim?.path === '/World/SlideprinterScene') {
      return [{
        path: '/World/SlideprinterScene/SpoolA',
        name: 'SpoolA',
        type: 'definition',
        defType: 'Sphere'
      }];
    }
    return [];
  }),
  getRelationship: jest.fn(() => []),
  materialProperties: jest.fn(() => ({ color: '#8899aa', friction: null, restitution: null }))
}));

const { World } = require('../../../src/js/cable_joints_3d/ecs.js');
const { setupScene } = require('../../../examples/js/slideprinter_3d/setupScene.js');
const { RenderSystem3D } = require('../../../src/js/cable_joints_3d/render_system_3d.js');

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
        height: this.clientHeight
      };
    }
  };
}

describe('slideprinter 3D setupScene', () => {
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
      documentElement: { style: {} }
    };
    global.window = {
      addEventListener() {},
      removeEventListener() {}
    };
  });

  afterEach(() => {
    global.document = originalDocument;
    global.window = originalWindow;
  });

  test('uses the 3D render system and registers 3D constraint systems', () => {
    const world = new World();
    const stage = {
      GetPrimAtPath(path) {
        return { path, name: path.split('/').pop() };
      },
      ast: {
        descriptor: {
          assignments: [
            { type: 'assignment', identifier: 'timeCodesPerSecond', value: 500 }
          ]
        }
      }
    };

    setupScene(world, stage, createCanvas());

    const renderSystem = world.getResource('renderSystem');
    expect(renderSystem).toBeInstanceOf(RenderSystem3D);
    expect(renderSystem.options.targetX).toBe(0.0);
    expect(renderSystem.options.targetY).toBe(0.0);
    expect(renderSystem.options.initialOrbitAzimuth).toBeCloseTo(-Math.PI * 0.25, 6);
    expect(renderSystem.options.initialOrbitPolar).toBeCloseTo(1.05, 6);
    expect(renderSystem.setViewTransform).toHaveBeenCalledWith({
      scaleMultiplier: 1.1,
      offsetX: 0.0,
      offsetY: -0.0,
    });

    const planeNormal = world.getResource('defaultPlaneNormal');
    expect(planeNormal?.x).toBeCloseTo(0.0, 6);
    expect(planeNormal?.y).toBeCloseTo(0.0, 6);
    expect(planeNormal?.z).toBeCloseTo(1.0, 6);

    const systemNames = world.systems.map((system) => system.constructor.name);
    expect(systemNames).toContain('RigidGroupSystem');
    expect(systemNames).toContain('XPBDDistanceConstraintSystem');
    expect(systemNames).toContain('ExtruderSystem');
    expect(systemNames).toContain('StepperMotorSystem');
  });
});
