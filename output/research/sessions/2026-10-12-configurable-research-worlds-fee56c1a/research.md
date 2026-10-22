# Configurable research worlds

Objective: validate MCP HP4-to-HP3 reconfiguration, coherent firmware/encoder restart and layering-off geometry using the established production collector.

Hypothesis H1: settings applied before baking produce zero cable half-width and match production headless/browser geometry. H2: coordinated reset retains settings and starts a fresh generation/encoder origin. H3: a small real HP3 collection with 30 mm radii and zero buildup produces actual encoder motion and valid measurements.

Constraints: stock HP3/HP4 scenes; headless JS; recording disabled for the numeric trial. No calibration algorithm changes and no hardware/accuracy claims. Budget: one six-point collection, 900 seconds wall time; retain failures. Lifecycle fault injection and source reload tests are tool checks.

Experiments: E1 production geometry and lifecycle regression suite; E2 real MCP HP3 collection (pending). Accepted steering: research/prompts/tooling_improvement_2.txt. Next decision: inspect E2 dataset, baked geometry, encoder movement and manifests before accepting the feature.

E2 failed before physics (0 fixed steps): collector rejected hangprinter_3; inspection additionally found Cartesian firmware boot due to RRF host path truncation. Evidence retained in E2-failed/ and native/. Corrected caller profile and short private VSD; E3 retry authorized within a second 900-second budget. Verify actual M669/M666 startup before collecting.

E3 completed: six production HP3 points in 6.878 s wall time. Encoder ranges A/B: 182.975/183.026 degrees; physical sensor spans 48.621/48.595 mm. All layering flags false and baked half-width zero. E4 repeats this final-code collection after status/provenance and source-reload validation changes; one further 900-second budget. No fit or accuracy claim.

E4 completed: 7c7c2180a90348e783d73aae2c5367a1; six points in 6.605359 s wall time, 28212 physical fixed steps. Same encoder and sensor spans as E3. E1 tests supported coherent reset, source reload, backend-preparation failure preserving a live world and recoverable post-retirement failure. Implementation accepted with documented stock-design and accuracy limits. Final decision: complete tooling work; future accuracy/held-out experiments remain untested.
