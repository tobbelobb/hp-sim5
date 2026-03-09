jest.mock('../../../src/js/cable_joints_3d/render_system_3d.js', () => {
  class MockRenderSystem3D {
    resetVisuals() {}
    setComponentClasses() {}
    setCanvasSize() {}
  }
  return { RenderSystem3D: MockRenderSystem3D };
});

jest.mock('../../../src/js/usd/stage.js', () => ({
  getAttribute: jest.fn((prim, attr) => {
    if (prim?.path === '/World/PhysicsScene') {
      if (attr === 'physics:gravityDirection') return [0.0, -1.0, 0.0];
      if (attr === 'physics:gravityMagnitude') return 9.82;
    }
    if (prim?.path === '/World/FlipperScene/Border' && attr === 'points') {
      return [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0]
      ];
    }
    return null;
  }),
  getChild: jest.fn((parent, name) => {
    if (parent?.path === '/World/FlipperScene' && name === 'Border') {
      return { path: '/World/FlipperScene/Border', name: 'Border' };
    }
    return null;
  }),
  getChildren: jest.fn(() => []),
  getRelationship: jest.fn(() => []),
  materialProperties: jest.fn(() => ({ color: '#000000', friction: null, restitution: null }))
}));

const { World } = require('../../../src/js/cable_joints_3d/ecs.js');
const { setupScene } = require('../../../examples/js/flipper_3d/setupScene.js');

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
    contains() {
      return false;
    },
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

describe('setupScene 3D planar registration', () => {
  const originalDocument = global.document;
  const originalWindow = global.window;

  beforeEach(() => {
    global.document = {
      getElementById() {
        return { textContent: '' };
      },
      addEventListener() {},
      removeEventListener() {}
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

  test('registers a single final PlanarConstraintSystem3D pass', () => {
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

    const systemNames = world.systems.map((system) => system.constructor.name);
    const planarIndices = systemNames
      .map((name, index) => (name === 'PlanarConstraintSystem3D' ? index : -1))
      .filter((index) => index >= 0);

    expect(planarIndices).toHaveLength(1);
    expect(planarIndices[0]).toBeGreaterThan(systemNames.indexOf('BallBorderOrFlipperVelocityContactSystem3D'));
    expect(planarIndices[0]).toBeLessThan(systemNames.indexOf('ScoreSystem'));
  });
});
