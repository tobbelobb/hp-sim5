import {
  InputSystem,
  SpoolTagComponent,
} from '../../../examples/js/slideprinter_3d/slideprinter_common.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';

function createCanvas() {
  return {
    clientWidth: 640,
    clientHeight: 480,
    width: 640,
    height: 480,
    style: {},
    setAttribute() {},
    focus() {},
    setPointerCapture() {},
    releasePointerCapture() {},
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

function createPointerEvent(canvas, { clientX, clientY, pointerId = 1 } = {}) {
  return {
    target: canvas,
    clientX,
    clientY,
    pointerId,
    pointerType: 'mouse',
    button: 0,
    preventDefault() {},
  };
}

describe('slideprinter 3D InputSystem orbit interaction', () => {
  const originalDocument = global.document;
  const originalWindow = global.window;

  beforeEach(() => {
    global.document = {
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

  test('left-drag on empty canvas rotates the 3D render system', () => {
    const world = new World();
    const renderSystem = {
      rotateOrbitByPixels: jest.fn(),
    };
    world.setResource('simHeight', 1.7);
    world.setResource('renderSystem', renderSystem);

    const canvas = createCanvas();
    const inputSystem = new InputSystem(canvas, world, null);

    inputSystem.handlePointerDown(createPointerEvent(canvas, { clientX: 100, clientY: 120, pointerId: 7 }));
    expect(inputSystem.isOrbiting).toBe(true);

    inputSystem.handlePointerMove(createPointerEvent(canvas, { clientX: 135, clientY: 150, pointerId: 7 }));
    expect(renderSystem.rotateOrbitByPixels).toHaveBeenCalledWith(35, 30);

    inputSystem.handlePointerUp(createPointerEvent(canvas, { clientX: 135, clientY: 150, pointerId: 7 }));
    expect(inputSystem.isOrbiting).toBe(false);
  });

  test('pan mode follows the projected screen-plane motion', () => {
    const world = new World();
    const renderSystem = {
      projectClientToSim: jest.fn((clientX, clientY) => {
        if (clientX === 100 && clientY === 120) {
          return { x: 1.0, y: 2.0 };
        }
        if (clientX === 140 && clientY === 150) {
          return { x: -2.0, y: 5.0 };
        }
        return { x: 0.0, y: 0.0 };
      }),
      rotateOrbitByPixels: jest.fn(),
    };
    world.setResource('simHeight', 1.7);
    world.setResource('renderSystem', renderSystem);

    const canvas = createCanvas();
    const inputSystem = new InputSystem(canvas, world, null);
    const onViewChange = jest.fn();
    inputSystem.setInteractionMode('pan');
    inputSystem.setViewChangeListener(onViewChange);

    inputSystem.handlePointerDown(createPointerEvent(canvas, { clientX: 100, clientY: 120, pointerId: 9 }));
    inputSystem.handlePointerMove(createPointerEvent(canvas, { clientX: 140, clientY: 150, pointerId: 9 }));

    expect(inputSystem.viewOffsetX).toBeCloseTo(3.0, 6);
    expect(inputSystem.viewOffsetY).toBeCloseTo(-3.0, 6);
    expect(onViewChange).toHaveBeenCalledWith({
      scale: 1.0,
      offsetX: 3.0,
      offsetY: -3.0,
    });
  });

  test('grabs a spool without orbiting and moves it on a camera-parallel drag plane', () => {
    const world = new World();
    const renderSystem = {
      getCameraPlaneNormal: jest.fn(() => ({ x: 0.0, y: 0.0, z: 1.0 })),
      projectClientToPlane: jest.fn((clientX, clientY, planePoint) => {
        if (clientX === 100 && clientY === 120) {
          return { x: planePoint.x + 0.01, y: planePoint.y - 0.02, z: planePoint.z };
        }
        if (clientX === 140 && clientY === 160) {
          return { x: planePoint.x + 0.08, y: planePoint.y + 0.03, z: planePoint.z + 0.15 };
        }
        return { x: planePoint.x, y: planePoint.y, z: planePoint.z };
      }),
      projectClientToSim: jest.fn(() => ({ x: 0.0, y: 0.0 })),
      rotateOrbitByPixels: jest.fn(),
    };
    world.setResource('simHeight', 1.7);
    world.setResource('renderSystem', renderSystem);
    world.setResource('grabbedBall', null);

    const spoolId = world.createEntity();
    world.addComponent(spoolId, new SpoolTagComponent());
    world.addComponent(spoolId, new PositionComponent(0.4, 0.5, 0.2));
    world.addComponent(spoolId, new RadiusComponent(0.05));

    const canvas = createCanvas();
    const inputSystem = new InputSystem(canvas, world, null);

    expect(() => {
      inputSystem.handlePointerDown(createPointerEvent(canvas, { clientX: 100, clientY: 120, pointerId: 11 }));
    }).not.toThrow();

    expect(inputSystem.isOrbiting).toBe(false);
    expect(inputSystem.grabSpring?.ballE).toBe(spoolId);
    expect(world.getResource('grabbedBall')).toBe(spoolId);
    expect(inputSystem.grabPlanePoint).toEqual(expect.objectContaining({ x: 0.4, y: 0.5, z: 0.2 }));

    const ptrPos = world.getComponent(inputSystem.grabSpring.ptrE, PositionComponent)?.pos;
    expect(ptrPos?.x).toBeCloseTo(0.41, 6);
    expect(ptrPos?.y).toBeCloseTo(0.48, 6);
    expect(ptrPos?.z).toBeCloseTo(0.2, 6);

    inputSystem.handlePointerMove(createPointerEvent(canvas, { clientX: 140, clientY: 160, pointerId: 11 }));

    expect(renderSystem.rotateOrbitByPixels).not.toHaveBeenCalled();
    expect(ptrPos?.x).toBeCloseTo(0.48, 6);
    expect(ptrPos?.y).toBeCloseTo(0.53, 6);
    expect(ptrPos?.z).toBeCloseTo(0.35, 6);
  });
});
