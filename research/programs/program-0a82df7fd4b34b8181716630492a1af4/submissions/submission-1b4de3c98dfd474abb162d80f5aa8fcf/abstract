# HP3 lower sensor measurement force under motor drag and encoder noise

Status: controlled partial.

We compare the fixed4×midpoint sensor measurement force with the current5×midpoint rule in six fresh paired HP3 autocalibrations. The nominal and development worlds precede a heldout evaluation pair. All worlds use known30mm radii, zero buildup, no layering or guide friction, native Rust, and seeded Gaussian encoder-read noise. Motor Coulomb and smoothed-stiction factors vary by axis. 4 of six primary collections have completed or failed. Default force adoption is blocked by the preserved nominal candidate workflow failure. Evaluation pair NOT TESTED. The manager will fund only the two unrun evaluation jobs and focused transport follow-up in a fresh bounded continuation. No successful run will be repeated. A small hypothetical three-world simulation cannot establish hardware reliability or a lower failure rate. Applied anchor errors come from the rounded M669 command and a checked firmware readback. Every intermediate fit, failure and full command/noise journal is preserved separately from final applied geometry.

Topics: HP3, autocalibration, measurement force, motor friction, encoder noise, applied anchors.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Lower sensor force improves applied accuracy across the frozen worlds without new failure or >20% extra effort | n-default, n-candidate, d-default, d-candidate | Inconclusive for transfer; development max improves22.81% with6.41% extra simulated effort; nominal candidate workflow failed | [Applied results](evidence/measurements.json) |
| An explicit encoder deadline is silently capped at5s in the shared transport | source and nominal failed trace only | Not tested by the authorized real-WebSocket discriminator yet | [Failure excerpt](evidence/n-candidate-transport-excerpt.jsonl) |

## Decision handoff

- Scope: four fresh simulated HP3 attempts at basea68d6adf, known30mm radii, zero buildup/guide friction, layering off and the fixed nominal/development motors and noise.
- Review: producer hash/readback and offline geometry checks; independent fixed-evidence review pending.
- Decision: keep production5mid force default. The nominal failure stays in every denominator. Evaluation pair NOT TESTED in the controlled partial.
- Next: only the two frozen evaluation jobs and the separate bounded transport follow-up. Do not repeat successful jobs, retune policy, restore an old world or erase the failed trial.

## References

- [Full report](report.md) gives per-anchor geometry, effort and intermediate tails.
- [Research notebook](research.md) records the frozen decision, manager steering and reporting corrections.
- [Frozen policy](policy-frozen.json) preserves exact worlds, code/native hashes and pre-result adoption margin.
