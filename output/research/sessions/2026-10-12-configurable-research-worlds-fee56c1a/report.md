# Configurable HP3/HP4 research worlds through MCP

The MCP configuration API was checked against production scene construction and the existing RRF collector. HP4-to-HP3 reconfiguration, reset retaining accepted settings, source reload, active-job rejection and failure recovery passed. The final physical HP3 trial produced six points with known 30 mm radii, zero buildup and all layering flags disabled. Its baked cable half-widths were zero. Real encoder travel was 182.974794° on A and 183.025909° on B. These are simulated measurement-ingestion and lifecycle results; no anchor/radius fit or hardware accuracy was measured.

## Method and measured results

E1: the repository tests compare prepared HP3/HP4 geometry byte-for-byte with `createHeadlessWorld` and the production browser's shared baker/options, then step the actual JS worker. Tests also inspect physical cable half-widths, custom unlayered spool radii and corresponding frozen firmware. A temporary repository changes authored scene winding and JS worker source; ordinary reset observes stale hashes, loads both edits, preserves settings and leaves earlier baked evidence unchanged. Fault injection checks preparation failure preserving the live world and firmware-start failure recovering through the last accepted configuration. Active collection/autocal jobs require explicit cancellation before reset/reconfiguration.

E2: failed before movement (0 fixed steps). The first collector caller incorrectly selected `hangprinter_3`; production HP3 uses the four-anchor Hangprinter collector. Evidence inspection also found that RRF booted as Cartesian because the long artifact/VSD path exceeded its 120-character host filename buffer. Both failures are preserved in [the failed manifest](native/c9bd24bcb3514e8b8cfb6c4f8838a355/62b3db82e9384f7f8ba390bc8579abfd/manifest.json) and [the trial log](E2-failed/validation.log). The final implementation uses the established `hangprinter_4` collector profile, a short private VSD and observed M669/M666 startup checks.

E3: successful retry, six points in 6.878117 s wall time. [Manifest](E3-complete/collection-status.json) records the trial and numerical validation. A/B encoder ranges and physical sensor spans match E4.

E4: final-code MCP collection `7c7c2180a90348e783d73aae2c5367a1`. It started on HP4, called `configure_session` for HP3 with layering disabled, then called ordinary reset and collected one configuration with three points in each direction. M669 identified Hangprinter kinematics; M666 reported Q0 and four 30 mm radii. The generation manifest recorded all requested/effective settings and authored/design/baked scene and firmware hashes. The assigned directory stayed fixed while world IDs changed.

| Measurement | E4 value |
| --- | --- |
| Measurement points | 6 |
| Fixed steps executed | 28212 |
| Simulation time | 56.424 s |
| Wall time | 6.605359 s |
| Encoder A/B range | 182.974794 / 183.025909 degrees |
| Physical sensor span, directions 1/2 | 48.621382 / 48.595202 mm |
| Drive span | 95.493945 mm |
| Autocal residual ingestion | 6 finite residuals |
| Baked half-widths | [0.0] m |
| Layering flags | {'enableLayering': False, 'layeringFrictionEffectiveRadius': False, 'layeringRenderWraps': False} |

## Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| Layering-off configuration removes geometric width and matches production construction | E1, E4 | Supported | [Regression suite](../../../../tests/python/test_world_configuration.py), [baked scene](native-final/2eab243afbdd4c06aa4ce36cdea60ad0/scene.usda) |
| Coordinated reset preserves settings and creates fresh firmware/encoder references | E1, E3, E4 | Supported | [Before/after statuses](reset-status.json), [lifecycle tests](../../../../tests/python/test_world_configuration.py) |
| The existing HP3 collector produces real encoder motion and ingested measurements | E2, E3, E4 | Supported after caller/firmware fixes | [Failed trial](native/c9bd24bcb3514e8b8cfb6c4f8838a355/62b3db82e9384f7f8ba390bc8579abfd/manifest.json), [dataset](native-final/2eab243afbdd4c06aa4ce36cdea60ad0/7c7c2180a90348e783d73aae2c5367a1/sweeps.json), [encoder movement](encoder-movement.json) |
| Calibration accuracy improves | None | Not tested | No fit or held-out accuracy experiment was performed |

## Verification and limits

102 Python regressions passed with the normal slow-test exclusions. A separate 23-check run included actual HP3 collection and lifecycle/source-reload validation. Four additional slow tests covered real HP4 collection, cancellation, desktop attachment/reconnection and service-only launcher shutdown. Five JS checks covered shared production geometry construction and browser research controls. [Commands and results](verification.md) record their scope.

This checks production construction parity and numerical collection, without opening a live rendered browser. Actual network-denied command sandbox execution was not exercised; the API uses MCP's supervised service transport without issuing command-sandbox HTTP calls, and launcher dry-run tests preserve explicit networking overrides. Launcher network-enabled defaults remain unchanged.

Supported fresh worlds are stock HP3/HP4 scenes on the selected continuing backend. Custom unlayered radii (1–200 mm) are supported. Layered designs are limited to stock 30 mm base radii and buildup 2/pi; custom anchor layouts and arbitrary layered buildup/radius combinations are explicitly unsupported. Stock firmware effective layered radii are existing model settings, not a claim that the discrete winding geometry exactly equals RRF's continuous spool predictor. No independent hardware comparison, calibration accuracy, GPU parity or throughput benchmark was performed. Recording was disabled for E2–E4; JSON events retain numerical evidence. Existing RRD lifecycle tests still passed.

## References

- [E4 collection manifest](native-final/2eab243afbdd4c06aa4ce36cdea60ad0/7c7c2180a90348e783d73aae2c5367a1/manifest.json): inputs, settings, source hashes, generation identity, measured work and validation.
- [E4 dataset](native-final/2eab243afbdd4c06aa4ce36cdea60ad0/7c7c2180a90348e783d73aae2c5367a1/sweeps.json) and [encoder/command trace](native-final/2eab243afbdd4c06aa4ce36cdea60ad0/7c7c2180a90348e783d73aae2c5367a1/events.jsonl): actual simulated measurement and movement evidence.
- [HP3 status](hp3-status.json), [reset status](reset-status.json): coherent lifecycle and firmware replies.
- [Validation script](validate.py): real stdio MCP calls; [trial output](validation.log): completed collection result.
- [Research notebook](research.md): hypotheses, budgets, failed trial and decisions.
- [Production constructor](../../../../hp-sim-3d/app/headlessWorld.js) and [browser baker path](../../../../hp-sim-3d/app/machineSceneController.js): matched scene construction.
- [Implementation prompt](../../../../research/prompts/tooling_improvement_2.txt) and [integration rationale](../../../../research/investigation.md): task scope and retained architecture.
- Git base `08996a8228530e1dc549b05bbf3db0eb13307b86` plus [working source snapshot](source-hashes.json) and [diff](implementation.diff): implementation provenance. Trial manifests record the loaded Python/JS/bridge hashes; timestamps and source changes are not calibration evidence.
