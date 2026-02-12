# Notes On Funnel Scene Debugging

This is a quick handoff for debugging instability in:
- `examples/js/flipper/spool_energy_funnel_debug.html`
- `examples/js/flipper/spool_energy_debug.html`
- `examples/js/flipper/index.html`

## 1) Manual Debug Loop (Browser)

1. Open `examples/js/flipper/spool_energy_funnel_debug.html`.
2. Use `Pause`, `Step`, and `Reset`.
3. While paused:
   - tap `s` for one step
   - hold `s` for press-and-hold stepping
   - (`t` is also mapped the same way)
4. Use `Dump State` to print full ECS state.

`world` is exposed globally in all 3 pages, so console inspection works.

## 2) Cable Checkboxes To Sweep

All checkboxes are defined in `examples/js/flipper/cable_feature_flags.js`.

Current ordering detail (important for trace interpretation):
- `CableAttachmentUpdateSystem` now runs hybrid endpoint updates both before and after the split/merge passes.
- `OverlayRadiusAndCircleSectorSystem` is registered both before and after `CableAttachmentUpdateSystem` in the funnel/debug setups and stability harness.
- Contact handling now runs through `PBDUnifiedContactManifoldSystem` (single GS manifold pass per substep across border/ball/obstacle/flipper contacts).
- In the node harness (`tests/js/flipper/spoolEnergyFunnel.stability.node.test.js`) these two overlay passes are keyed as `overlayPre` and `overlayPost` (legacy `overlay` disables both).
- In the node harness the contact pass key is `contactManifold`; legacy disable aliases (`ballBorder`, `borderSector`, `ballBall`, `ballBallSector`, `ballObstacle`, `obstacleSector`, `ballFlipper`, `flipperSector`) all map to it.

Most important for layering instability:
- `cable/updateAttachment` (`layeringAttachmentUpdatePoints`)
- `cable/mergeJoints` (`layeringMergeJoints`)
- `cable/splitJoints` (`layeringSplitJoints`)
- `cable/hybridLinkStates` (`layeringHybridLinkStates`)

Notes:
- `spool_energy_debug.html` and `spool_energy_funnel_debug.html` auto-reset on checkbox changes.
- `index.html` does not auto-reset on checkbox change; hit `Reset` manually.

## 3) Runtime Trace Tools (Console)

Enable focused event traces around a suspect window:

```js
// Example: around early spike window
world.setResource('cableHybridTransitionStep', 0);
world.setResource('cableHybridTransitionTrace', true);
world.setResource('cableHybridTransitionTraceStepMin', 200);
world.setResource('cableHybridTransitionTraceStepMax', 260);
world.setResource('cableHybridTransitionTraceLimit', 4096);
world.setResource('cableHybridTransitionTraceTruncated', false);
world.setResource('cableHybridTransitionTraceBuffer', []);

world.setResource('cableEventTrace', true);
world.setResource('cableEventTraceStepMin', 200);
world.setResource('cableEventTraceStepMax', 260);
world.setResource('cableEventTraceLimit', 64000);
world.setResource('cableEventTraceTruncated', false);
world.setResource('cableEventTraceBuffer', []);
```

Read buffers:
- `world.getResource('cableHybridTransitionTraceBuffer')`
- `world.getResource('cableEventTraceBuffer')`

High-value event types:
- `hybrid-rub-check`
- `hybrid-transition`
- `split`
- `split-abort`
- `merge`
- `rest-length-anomaly`

## 4) Automated Node Diagnostics

Main targeted test:

```bash
npm test -- tests/js/flipper/spoolEnergyFunnel.stability.node.test.js -t "greedy cable-step disabling isolates spike source around step 240"
```

Long-horizon routed-path test (green pair, around 17450-17850):

```bash
SPOOL_LONG=1 npm test -- tests/js/flipper/spoolEnergyFunnel.stability.node.test.js -t "long-horizon green-pair routed-path diagnostic"
```

Useful log blocks to parse from test output:
- `SPOOL_FUNNEL_EVENT_TRACE`
- `SPOOL_FUNNEL_HYBRID_TRACE`
- `SPOOL_FUNNEL_STABILITY_SWEEP`
- `SPOOL_FUNNEL_LONG_HORIZON_TRACE`

## 5) What To Look For First

1. `rest-length-anomaly` events right before angular jumps.
2. `hybrid-rub-check` with `sameJointPath: true` and `bothEndpointsHybridLike: true` (pinch/rubbing cases).
3. `split` events producing very small `minNewRestLength` / `dAS` / `dSB`.
