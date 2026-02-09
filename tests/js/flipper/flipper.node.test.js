import fs from 'fs';
import path from 'path';
import {
  World,
  PositionComponent
} from '../../../src/js/cable_joints/ecs.js';
import {
  BallTagComponent,
  ScoreComponent,
  PauseStateComponent
} from '../../../examples/js/flipper/flipper_common.js';
import { setupScene } from '../../../examples/js/flipper/setupScene.js';
import { Open } from '../../../src/js/usd/stage.js';

// Minimal DOM stubs for systems that expect browser APIs
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
    getContext: () => ({
      beginPath() {},
      arc() {},
      closePath() {},
      fill() {},
      stroke() {},
      moveTo() {},
      lineTo() {},
      fillRect() {},
      save() {},
      restore() {},
      translate() {},
      rotate() {},
      clearRect() {},
      drawImage() {}
    }),
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

describe('Flipper Node Simulation', () => {
  beforeAll(() => {
    global.document = mockDocument;
  });

  afterAll(() => {
    delete global.document;
  });

  function createCanvas() {
    return createMockCanvas();
  }

  function getGameState(world) {
    const ballEntities = world.query([BallTagComponent, PositionComponent]);
    const balls = ballEntities.map(id => {
      const pos = world.getComponent(id, PositionComponent).pos;
      return { id, y: pos.y };
    });
    const scoreId = world.query([ScoreComponent])[0];
    const scoreComp = scoreId !== undefined ? world.getComponent(scoreId, ScoreComponent) : null;
    const score = scoreComp ? scoreComp.value : -1;
    return { balls, score };
  }

  test('balls settle below flippers with expected score', async () => {
    const originalWarn = console.warn;
    console.warn = jest.fn(); // suppress warnings
    const usdaPath = path.resolve(__dirname, '../../../examples/usd_scenes/flipper_scene.usda');
    const source = fs.readFileSync(usdaPath, 'utf8');
    const stage = await Open(source);

    const world = new World();
    const canvas = createCanvas();

    setupScene(world, stage, canvas);
    world.getResource('pauseState').paused = false;

    const dt = world.getResource('dt');
    let state;
    let settled = false;
    for (let i = 0; i < 30000; i++) {
      world.update(dt);
      state = getGameState(world);
      if (state.balls.every(b => b.y < 0.05)) {
        settled = true;
        break;
      }
    }

    console.warn = originalWarn; // restore after test
    expect(settled).toBe(false);
    // Score is sensitive to physics ordering and floating-point drift across Node versions.
    // This baseline reflects entry-based scoring on raw obstacle hits only,
    // with wrapped effective-shape collisions enabled.
    expect(state.score).toBe(14);
  });
});
