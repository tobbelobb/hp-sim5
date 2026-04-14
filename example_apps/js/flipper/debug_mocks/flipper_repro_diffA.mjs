import fs from 'fs';
import path from 'path';
import { World } from '../home/torbjorn/repos/hp-sim5/src/js/cable_joints/ecs.js';
import { setupScene } from '../home/torbjorn/repos/hp-sim5/examples/js/flipper/setupScene.js';
import { Open } from '../home/torbjorn/repos/hp-sim5/src/js/usd/stage.js';

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
    releasePointerCapture: () => {},
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
      drawImage() {},
      strokeText() {},
      fillText() {},
      scale() {},
      setLineDash() {}
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

global.document = mockDocument;

const repoRoot = '/home/torbjorn/repos/hp-sim5';
const usdaPath = path.resolve(repoRoot, 'examples/usd_scenes/flipper_scene.usda');
const source = fs.readFileSync(usdaPath, 'utf8');
const stage = await Open(source);

const world = new World();
const canvas = createMockCanvas();
setupScene(world, stage, canvas);
world.getResource('pauseState').paused = false;
const dt = world.getResource('dt');

for (let i = 0; i < 900; i++) {
  world.update(dt);
}
