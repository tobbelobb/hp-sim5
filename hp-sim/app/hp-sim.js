import { createHpSimApp } from './appBootstrap.js';

async function bootHpSim() {
  const canvas = document.getElementById('myCanvas');
  const controlsRoot = document.getElementById('controls');
  const app = createHpSimApp({ document, canvas, controlsRoot });
  await app.loadDefaultScene();
  app.bindUi();
  app.start();
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', () => {
    bootHpSim().catch((error) => {
      console.error('hp-sim bootstrap failed:', error);
    });
  }, { once: true });
} else {
  bootHpSim().catch((error) => {
    console.error('hp-sim bootstrap failed:', error);
  });
}
