import {
  InputSystem,
} from '../../../hp-sim-3d/app/hangprinter_input.js';
import {
  SpoolTagComponent,
} from '../../../hp-sim-3d/app/hangprinter_spools.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';

function createCanvas(overrides = {}) {
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
    ...overrides,
  };
}

function createPointerEvent(canvas, { clientX, clientY, pointerId = 1, pointerType = 'mouse', button = 0 } = {}) {
  return {
    target: canvas,
    clientX,
    clientY,
    pointerId,
    pointerType,
    button,
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

  test('single-finger touch drag on empty canvas orbits the 3D render system', () => {
    const world = new World();
    const renderSystem = {
      rotateOrbitByPixels: jest.fn(),
    };
    world.setResource('simHeight', 480);
    world.setResource('renderSystem', renderSystem);

    const canvas = createCanvas();
    const inputSystem = new InputSystem(canvas, world, null);

    inputSystem.handlePointerDown(createPointerEvent(canvas, {
      clientX: 100,
      clientY: 120,
      pointerId: 17,
      pointerType: 'touch',
    }));
    expect(inputSystem.isOrbiting).toBe(true);
    expect(inputSystem.isPanning).toBe(false);

    inputSystem.handlePointerMove(createPointerEvent(canvas, {
      clientX: 135,
      clientY: 150,
      pointerId: 17,
      pointerType: 'touch',
    }));
    expect(renderSystem.rotateOrbitByPixels).toHaveBeenCalledWith(35, 30);

    inputSystem.handlePointerUp(createPointerEvent(canvas, {
      clientX: 135,
      clientY: 150,
      pointerId: 17,
      pointerType: 'touch',
    }));
    expect(inputSystem.isOrbiting).toBe(false);
    expect(inputSystem.activeGrabPointerId).toBe(null);
  });

  test('a second touch cancels orbit and starts the pinch gesture state', () => {
    const world = new World();
    const renderSystem = {
      rotateOrbitByPixels: jest.fn(),
    };
    world.setResource('simHeight', 480);
    world.setResource('renderSystem', renderSystem);

    const canvas = createCanvas();
    const inputSystem = new InputSystem(canvas, world, null);

    inputSystem.handlePointerDown(createPointerEvent(canvas, {
      clientX: 100,
      clientY: 120,
      pointerId: 21,
      pointerType: 'touch',
    }));
    expect(inputSystem.isOrbiting).toBe(true);

    inputSystem.handlePointerDown(createPointerEvent(canvas, {
      clientX: 200,
      clientY: 120,
      pointerId: 22,
      pointerType: 'touch',
    }));

    expect(inputSystem.isOrbiting).toBe(false);
    expect(inputSystem.pinchActive).toBe(true);
    expect(inputSystem.activePointers.size).toBe(2);
  });

  test('two-finger drag pans by translating the gesture midpoint', () => {
    const world = new World();
    const renderSystem = {
      rotateOrbitByPixels: jest.fn(),
    };
    world.setResource('simHeight', 480);
    world.setResource('renderSystem', renderSystem);

    const canvas = createCanvas();
    const inputSystem = new InputSystem(canvas, world, null);
    const onViewChange = jest.fn();
    inputSystem.setViewChangeListener(onViewChange);

    inputSystem.handlePointerDown(createPointerEvent(canvas, {
      clientX: 100,
      clientY: 120,
      pointerId: 31,
      pointerType: 'touch',
    }));
    inputSystem.handlePointerDown(createPointerEvent(canvas, {
      clientX: 200,
      clientY: 120,
      pointerId: 32,
      pointerType: 'touch',
    }));

    inputSystem.handlePointerMove(createPointerEvent(canvas, {
      clientX: 120,
      clientY: 120,
      pointerId: 31,
      pointerType: 'touch',
    }));
    inputSystem.handlePointerMove(createPointerEvent(canvas, {
      clientX: 220,
      clientY: 120,
      pointerId: 32,
      pointerType: 'touch',
    }));

    expect(inputSystem.scaleMultiplier).toBeCloseTo(1.0, 6);
    expect(inputSystem.viewOffsetX).toBeCloseTo(-20.0, 6);
    expect(inputSystem.viewOffsetY).toBeCloseTo(0.0, 6);
    const [viewState, options] = onViewChange.mock.calls.at(-1);
    expect(viewState.scale).toBeCloseTo(1.0, 6);
    expect(viewState.offsetX).toBeCloseTo(-20.0, 6);
    expect(viewState.offsetY).toBeCloseTo(0.0, 6);
    expect(viewState.offsetZ).toBeCloseTo(0.0, 6);
    expect(options).toEqual({ gesture: 'pinch' });
  });

  test('two-finger drag pans along the camera-parallel plane basis', () => {
    const diagonal = Math.SQRT1_2;
    const world = new World();
    const renderSystem = {
      rotateOrbitByPixels: jest.fn(),
      getViewPlaneMetrics: jest.fn(() => ({
        right: { x: diagonal, y: 0.0, z: diagonal },
        up: { x: 0.0, y: 1.0, z: 0.0 },
        worldUnitsPerPixel: 1.0,
      })),
    };
    world.setResource('simHeight', 480);
    world.setResource('renderSystem', renderSystem);

    const canvas = createCanvas();
    const inputSystem = new InputSystem(canvas, world, null);
    const onViewChange = jest.fn();
    inputSystem.setViewChangeListener(onViewChange);

    inputSystem.handlePointerDown(createPointerEvent(canvas, {
      clientX: 100,
      clientY: 120,
      pointerId: 35,
      pointerType: 'touch',
    }));
    inputSystem.handlePointerDown(createPointerEvent(canvas, {
      clientX: 200,
      clientY: 120,
      pointerId: 36,
      pointerType: 'touch',
    }));

    inputSystem.handlePointerMove(createPointerEvent(canvas, {
      clientX: 120,
      clientY: 120,
      pointerId: 35,
      pointerType: 'touch',
    }));
    inputSystem.handlePointerMove(createPointerEvent(canvas, {
      clientX: 220,
      clientY: 120,
      pointerId: 36,
      pointerType: 'touch',
    }));

    expect(inputSystem.viewOffsetX).toBeCloseTo(-20.0 * diagonal, 6);
    expect(inputSystem.viewOffsetY).toBeCloseTo(0.0, 6);
    expect(inputSystem.viewOffsetZ).toBeCloseTo(-20.0 * diagonal, 6);
    const [viewState, options] = onViewChange.mock.calls.at(-1);
    expect(viewState.offsetX).toBeCloseTo(-20.0 * diagonal, 6);
    expect(viewState.offsetY).toBeCloseTo(0.0, 6);
    expect(viewState.offsetZ).toBeCloseTo(-20.0 * diagonal, 6);
    expect(options).toEqual({ gesture: 'pinch' });
  });

  test('pinch zoom scales around the gesture midpoint', () => {
    const world = new World();
    const renderSystem = {
      rotateOrbitByPixels: jest.fn(),
    };
    world.setResource('simHeight', 480);
    world.setResource('renderSystem', renderSystem);

    const canvas = createCanvas();
    const inputSystem = new InputSystem(canvas, world, null);
    const onViewChange = jest.fn();
    inputSystem.setViewChangeListener(onViewChange);

    inputSystem.handlePointerDown(createPointerEvent(canvas, {
      clientX: 100,
      clientY: 120,
      pointerId: 41,
      pointerType: 'touch',
    }));
    inputSystem.handlePointerDown(createPointerEvent(canvas, {
      clientX: 200,
      clientY: 120,
      pointerId: 42,
      pointerType: 'touch',
    }));

    inputSystem.handlePointerMove(createPointerEvent(canvas, {
      clientX: 90,
      clientY: 120,
      pointerId: 41,
      pointerType: 'touch',
    }));
    inputSystem.handlePointerMove(createPointerEvent(canvas, {
      clientX: 210,
      clientY: 120,
      pointerId: 42,
      pointerType: 'touch',
    }));

    expect(inputSystem.scaleMultiplier).toBeCloseTo(1.2, 6);
    expect(inputSystem.viewOffsetX).toBeCloseTo(-28.333333, 6);
    expect(inputSystem.viewOffsetY).toBeCloseTo(20.0, 6);
    const [viewState, options] = onViewChange.mock.calls.at(-1);
    expect(viewState.scale).toBeCloseTo(1.2, 6);
    expect(viewState.offsetX).toBeCloseTo(-28.333333333333343, 6);
    expect(viewState.offsetY).toBeCloseTo(20.0, 6);
    expect(viewState.offsetZ).toBeCloseTo(0.0, 6);
    expect(options).toEqual({ gesture: 'pinch' });
  });

  test('pan mode uses the axis-aligned camera-plane fallback when no metrics are available', () => {
    const world = new World();
    const renderSystem = {
      rotateOrbitByPixels: jest.fn(),
    };
    world.setResource('simHeight', 480);
    world.setResource('renderSystem', renderSystem);

    const canvas = createCanvas();
    const inputSystem = new InputSystem(canvas, world, null);
    const onViewChange = jest.fn();
    inputSystem.setInteractionMode('pan');
    inputSystem.setViewChangeListener(onViewChange);

    inputSystem.handlePointerDown(createPointerEvent(canvas, { clientX: 100, clientY: 120, pointerId: 9 }));
    inputSystem.handlePointerMove(createPointerEvent(canvas, { clientX: 140, clientY: 150, pointerId: 9 }));

    expect(inputSystem.viewOffsetX).toBeCloseTo(-40.0, 6);
    expect(inputSystem.viewOffsetY).toBeCloseTo(30.0, 6);
    const [viewState] = onViewChange.mock.calls.at(-1);
    expect(viewState.scale).toBeCloseTo(1.0, 6);
    expect(viewState.offsetX).toBeCloseTo(-40.0, 6);
    expect(viewState.offsetY).toBeCloseTo(30.0, 6);
    expect(viewState.offsetZ).toBeCloseTo(0.0, 6);
  });

  test('pan fallback respects client-to-canvas scaling on high-DPR canvases', () => {
    const world = new World();
    world.setResource('simHeight', 480);

    const canvas = createCanvas({
      clientWidth: 640,
      clientHeight: 480,
      width: 960,
      height: 720,
    });
    const inputSystem = new InputSystem(canvas, world, null);
    const onViewChange = jest.fn();
    inputSystem.setInteractionMode('pan');
    inputSystem.setViewChangeListener(onViewChange);

    inputSystem.handlePointerDown(createPointerEvent(canvas, { clientX: 100, clientY: 120, pointerId: 13 }));
    inputSystem.handlePointerMove(createPointerEvent(canvas, { clientX: 140, clientY: 150, pointerId: 13 }));

    expect(inputSystem.viewOffsetX).toBeCloseTo(-40.0, 6);
    expect(inputSystem.viewOffsetY).toBeCloseTo(30.0, 6);
    expect(inputSystem.viewOffsetZ).toBeCloseTo(0.0, 6);
    const [viewState] = onViewChange.mock.calls.at(-1);
    expect(viewState.offsetX).toBeCloseTo(-40.0, 6);
    expect(viewState.offsetY).toBeCloseTo(30.0, 6);
    expect(viewState.offsetZ).toBeCloseTo(0.0, 6);
  });

  test('pan mode can move along the camera-parallel plane instead of only XY', () => {
    const diagonal = Math.SQRT1_2;
    const world = new World();
    const renderSystem = {
      getViewPlaneMetrics: jest.fn(() => ({
        right: { x: diagonal, y: 0.0, z: diagonal },
        up: { x: 0.0, y: 1.0, z: 0.0 },
        worldUnitsPerPixel: 1.0,
      })),
      rotateOrbitByPixels: jest.fn(),
    };
    world.setResource('simHeight', 480);
    world.setResource('renderSystem', renderSystem);

    const canvas = createCanvas();
    const inputSystem = new InputSystem(canvas, world, null);
    const onViewChange = jest.fn();
    inputSystem.setInteractionMode('pan');
    inputSystem.setViewChangeListener(onViewChange);

    inputSystem.handlePointerDown(createPointerEvent(canvas, { clientX: 100, clientY: 120, pointerId: 61 }));
    inputSystem.handlePointerMove(createPointerEvent(canvas, { clientX: 120, clientY: 120, pointerId: 61 }));

    expect(inputSystem.viewOffsetX).toBeCloseTo(-20.0 * diagonal, 6);
    expect(inputSystem.viewOffsetY).toBeCloseTo(0.0, 6);
    expect(inputSystem.viewOffsetZ).toBeCloseTo(-20.0 * diagonal, 6);
    const [viewState] = onViewChange.mock.calls.at(-1);
    expect(viewState.offsetX).toBeCloseTo(-20.0 * diagonal, 6);
    expect(viewState.offsetY).toBeCloseTo(0.0, 6);
    expect(viewState.offsetZ).toBeCloseTo(-20.0 * diagonal, 6);
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

  test('does not grab a spool from a mouse right click', () => {
    const world = new World();
    const renderSystem = {
      getCameraPlaneNormal: jest.fn(() => ({ x: 0.0, y: 0.0, z: 1.0 })),
      projectClientToPlane: jest.fn(() => ({ x: 0.4, y: 0.5, z: 0.2 })),
      projectClientToSim: jest.fn(() => ({ x: 0.4, y: 0.5 })),
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

    inputSystem.handlePointerDown(createPointerEvent(canvas, {
      clientX: 100,
      clientY: 120,
      pointerId: 12,
      button: 2,
    }));

    expect(inputSystem.grabSpring).toBeNull();
    expect(world.getResource('grabbedBall')).toBeNull();
    expect(renderSystem.rotateOrbitByPixels).not.toHaveBeenCalled();
  });
});
