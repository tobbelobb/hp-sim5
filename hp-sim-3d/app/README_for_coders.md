# README for coders: `hp-sim-3d/app`

This directory is the browser app layer for the 3D simulator.

The app has been split so each file has a clear owner role. Keep it that way: new code should go where the responsibility already lives, not back into one large app file.

## Core idea

`hp-sim-3d.js` should be boring.

It should start the app, connect the modules, and wire browser events. It should not own feature logic, long state machines, upload policy, worker internals, or scene-specific rules.

The other files exist so each concern has one home.

## File ownership

### `hp-sim-3d.js`

Main app entry point.

Owns:

- app startup
- top-level wiring
- connecting controllers/modules
- minimal browser bootstrapping

Does not own:

- feature internals
- upload routing
- worker lifecycle details
- scene setup details
- preset logic
- app state structure

### `appState.js`

The app-owned state tree.

Owns:

- UI/session state
- currently loaded machine/job state
- view state
- feature toggle state
- controller-visible app state

Rule: app-level state should be findable here, not scattered as unrelated `let` variables.

### `machineCatalog.js`

Known machines and machine metadata.

Owns:

- built-in machine list
- machine labels/names
- default machine choices
- static machine metadata used by the app

Does not own scene loading behavior.

### `commandPresets.js`

Built-in command/demo presets.

Owns:

- demo command files
- preset labels
- preset-to-machine matching
- feature-dependent preset variants

Does not own command playback.

### `uploadPipeline.js`

User file upload routing.

Owns:

- detecting uploaded file type
- deciding whether a file is a scene or command file
- forwarding scene files to the scene controller
- forwarding command files to the command controller

Rule: upload button handlers should call this file, not implement file-type branches themselves.

### `sceneController.js`

Loaded scene/machine lifecycle.

Owns:

- loading default scenes
- loading uploaded/custom scenes
- resetting/replacing scenes
- deciding when scene reload is needed
- coordinating with `setupScene.js`

Does not own UI policy or command playback.

### `workerController.js`

Background worker lifecycle.

Owns:

- creating workers
- stopping workers
- restarting workers
- routing messages to/from workers
- cleaning up stale worker state

Rule: if a worker is started here, it must also be stopped here.

### `viewController.js`

Camera, canvas, and navigation behavior.

Owns:

- view scale/offset
- pointer/camera interactions
- navigation cursor behavior
- view reset/focus behavior

Does not own simulation state.

### `qualityController.js`

Quality/debug monitor wiring.

Owns:

- enabling/disabling quality diagnostics
- connecting quality monitor output to the app
- quality-related UI state

Does not own physics behavior.

### `featureFlagsController.js`

Feature toggle coordination.

Owns:

- app-level feature enable/disable logic
- applying feature flags to the world
- deciding whether a feature change requires reload/reset
- keeping feature flag behavior out of random UI handlers

## Rule of thumb

When adding code, ask:

> Which file owns this responsibility?

If the answer is “several files,” the boundary is probably wrong.

If the answer is “just add it to `hp-sim-3d.js`,” stop and create or use the proper owner module instead.

## Desired shape

The app should read like this:

```js
const state = createAppState();
const sceneController = createSceneController({ state, world });
const workerController = createWorkerController({ state });
const uploadPipeline = createUploadPipeline({
  state,
  sceneController,
  commandController,
});

wireUi({
  state,
  sceneController,
  workerController,
  uploadPipeline,
});
```

That is the goal: small owners, explicit state, boring wiring.
