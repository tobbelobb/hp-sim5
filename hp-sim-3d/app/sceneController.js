export function createSceneChangeQueue({ onError = console.error } = {}) {
  let sceneChangeQueue = Promise.resolve();

  function enqueueSceneChange(task) {
    const run = sceneChangeQueue.then(() => task());
    sceneChangeQueue = run.catch((error) => {
      onError('hp-sim-3d: scene change task failed', error);
    });
    return run;
  }

  return { enqueueSceneChange };
}

