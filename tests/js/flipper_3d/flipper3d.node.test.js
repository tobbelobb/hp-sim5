jest.mock('../../../src/js/cable_joints_3d/render_system_3d.js', () => {
  class MockRenderSystem3D {
    resetVisuals() {}
    projectClientToSim() {
      return { x: 0.0, y: 0.0, z: 0.0 };
    }
    setComponentClasses() {}
    setCanvasSize() {}
    update() {}
  }

  return { RenderSystem3D: MockRenderSystem3D };
});

import fs from 'fs';
import path from 'path';
import {
  World,
  PositionComponent
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  BallTagComponent,
  ScoreComponent
} from '../../../example_apps/js/flipper_3d/flipper_common_3d.js';
import { setupScene } from '../../../example_apps/js/flipper_3d/setupScene.js';
import { OpenCableScene as Open } from '../../../src/js/usd/cable_scene_loader.js';

function createMockCanvas() {
  const width = 460;
  const height = Math.round(width * 1.7);
  return {
    clientWidth: width,
    clientHeight: height,
    width,
    height,
    style: {},
    setAttribute: () => {},
    focus: () => {},
    addEventListener: () => {},
    removeEventListener: () => {},
    getContext: () => ({}),
    getBoundingClientRect: () => ({ left: 0, top: 0 })
  };
}

const mockDocument = {
  addEventListener: () => {},
  removeEventListener: () => {},
  getElementById: () => ({ textContent: '' }),
  createElement: (tag) => {
    if (tag === 'canvas') {
      return createMockCanvas();
    }
    return {};
  }
};

function getGameState(world) {
  const ballEntities = world.query([BallTagComponent, PositionComponent]);
  const balls = ballEntities.map((id) => {
    const pos = world.getComponent(id, PositionComponent).pos;
    return { id, x: pos.x, y: pos.y, z: pos.z };
  });
  const scoreId = world.query([ScoreComponent])[0];
  const scoreComp = scoreId !== undefined ? world.getComponent(scoreId, ScoreComponent) : null;
  return {
    balls,
    score: scoreComp ? scoreComp.value : -1
  };
}

function debugStepsConfig() {
  const maxStepsRaw = Number(process.env.FLIPPER3D_NODE_MAX_STEPS);
  const maxSteps = Number.isFinite(maxStepsRaw) && maxStepsRaw > 0 ? Math.floor(maxStepsRaw) : 2500;
  const checkpointsRaw = process.env.FLIPPER3D_NODE_CHECKPOINTS;
  const checkpoints = checkpointsRaw
    ? checkpointsRaw
      .split(',')
      .map((value) => Number(value.trim()))
      .filter((value) => Number.isFinite(value) && value > 0)
    : [500, 1000, 1500, 2000, 2500];
  return { maxSteps, checkpoints };
}

describe('Flipper 3D Node Simulation', () => {
  const originalDocument = global.document;
  const originalWindow = global.window;

  beforeAll(() => {
    global.document = mockDocument;
    global.window = {
      addEventListener: () => {},
      removeEventListener: () => {}
    };
  });

  afterAll(() => {
    global.document = originalDocument;
    global.window = originalWindow;
  });

  test('steps the full 3D scene in node with mocked rendering', async () => {
    const usdaPath = path.resolve(__dirname, '../../../public/usd_scenes/flipper_scene.usda');
    const source = fs.readFileSync(usdaPath, 'utf8');
    const stage = await Open(source);

    const world = new World();
    const canvas = createMockCanvas();

    setupScene(world, stage, canvas);
    world.getResource('pauseState').paused = false;

    const dt = world.getResource('dt');
    let state = getGameState(world);
    const checkpoints = [];
    const { maxSteps, checkpoints: checkpointSteps } = debugStepsConfig();
    for (let step = 1; step <= maxSteps; step += 1) {
      world.update(dt);
      if (checkpointSteps.includes(step)) {
        state = getGameState(world);
        checkpoints.push({ step, score: state.score });
      }
    }

    if (process.env.FLIPPER3D_DEBUG_LOGS === '1') {
      // Useful when comparing node-side traces without depending on the browser harness.
      // eslint-disable-next-line no-console
      console.log(JSON.stringify({ checkpoints, final: state }, null, 2));
    }

    expect(checkpoints.length).toBeGreaterThan(0);
    expect(state.score).toBeGreaterThan(0);
    expect(state.score).toBeLessThan(100);
    expect(state.balls.every((ball) => Math.abs(ball.z) < 1e-6)).toBe(true);
  });
});
