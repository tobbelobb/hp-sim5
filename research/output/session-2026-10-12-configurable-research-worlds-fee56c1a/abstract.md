# Configurable HP3/HP4 research worlds through MCP

Status: complete.

## Abstract

MCP world configuration was tested against production scene construction and the existing RRF collector. Tests validated HP4-to-HP3 configuration, layering-off geometry, reset retaining settings, source reload, active-job rejection and failed-start recovery. The final HP3 collection with known 30 mm spool radii, zero buildup and layering disabled produced six measurement points in 6.605 seconds wall time. Baked cable half-widths were zero; A/B encoder ranges were 182.975/183.026 degrees. The dataset passed schema and finite residual-ingestion checks. An earlier zero-step failed trial exposed the incorrect collector profile and RRF host path truncation; both were corrected and the failed evidence retained. This establishes simulated configuration/collection behavior, without demonstrating calibration accuracy, hardware transfer or rendered-browser collection parity.

## Topics

HP3, HP4, MCP, world configuration, layering, scene baking, source reload, encoder origin, RRF, collection, lifecycle

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Layering-off geometry matches production construction | E1, E4 | Supported | [Report](report.md), [baked scene](native-final/2eab243afbdd4c06aa4ce36cdea60ad0/scene.usda) |
| Reset preserves settings and coherent generation identity | E1, E3, E4 | Supported | [Statuses](reset-status.json), [tests](../../../../tests/python/test_world_configuration.py) |
| Existing HP3 collector produces actual encoder movement and ingested measurements | E2, E3, E4 | Supported after failed caller trial | [Dataset](native-final/2eab243afbdd4c06aa4ce36cdea60ad0/7c7c2180a90348e783d73aae2c5367a1/sweeps.json), [failed manifest](native/c9bd24bcb3514e8b8cfb6c4f8838a355/62b3db82e9384f7f8ba390bc8579abfd/manifest.json) |
| Calibration accuracy improves | None | Not tested | No fitting or held-out accuracy trial |

## References

- [Full report](report.md): measured outcomes and limitations.
- [Research notebook](research.md): decisions, budgets and preserved failures.
- [E4 manifest](native-final/2eab243afbdd4c06aa4ce36cdea60ad0/7c7c2180a90348e783d73aae2c5367a1/manifest.json): source/input hashes and generation provenance.
- [Verification](verification.md): test commands and results.
