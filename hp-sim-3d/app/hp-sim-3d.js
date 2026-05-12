import { createHpSimApp } from './appBootstrap.js';

async function bootHpSim3d() {
  const canvas = document.getElementById('myCanvas');
  const controlsRoot = document.getElementById('controls');
  const app = createHpSimApp({ document, canvas, controlsRoot });
  await app.loadDefaultScene();
  app.bindUi();
  app.start();
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', () => {
    bootHpSim3d().catch((error) => {
      console.error('hp-sim-3d bootstrap failed:', error);
    });
  }, { once: true });
} else {
  bootHpSim3d().catch((error) => {
    console.error('hp-sim-3d bootstrap failed:', error);
  });
}
