import { InputSystem } from '../../../examples/js/slideprinter_3d/slideprinter_common.js';
import { World } from '../../../src/js/cable_joints_3d/ecs.js';

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
});
