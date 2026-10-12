# Configurable continuing research worlds

Verified 2026-10-12. `configure_session(configuration)` makes a fresh HP3/HP4
continuing world through the existing MCP attachment. Settings are immutable per
generation; ordinary reset retains them and reloads source. Layering disabled
uses zero-width geometry before physics construction. Paired firmware, collector
references and encoder origin restart together. Invalid/preparation failures keep
the live world; post-retirement failures report a reset recovery path.

A final real HP3/RRF/production-JS collection with four 30 mm radii, zero buildup
and layering disabled produced six points in 6.605 s wall time.
Baked half-widths were zero; moving encoder ranges were
182.975/183.026 degrees. Tests compare HP3/HP4 baking with the shared production
headless/browser construction path and exercise reset, active-job rejection,
source reload, failure recovery, cancellation and attachment. A failed zero-step
trial is retained; it exposed an incorrect HP3 collector profile and RRF's host
filename length limit. This is simulated collection/tooling evidence, without
calibration-accuracy or hardware claims.

Supported scenes are stock HP3/HP4. Custom radii require layering disabled;
layered worlds use stock radii/buildup. Custom anchors and arbitrary layered
spool designs are explicitly unsupported. See [API examples and recovery](README.md).

[Full report, hypotheses, source provenance and preserved failures](../output/research/sessions/2026-10-12-configurable-research-worlds-fee56c1a/report.md).
