# HP3 lower sensor force and encoder reply budgets

Status: complete. Six frozen primary attempts compare 4mid with stock 5mid sensor measurement force under declared motor drag and Gaussian encoder noise. 6 primary attempts are recorded. Keep the production 5mid measurement-force default. The preserved nominal 4mid workflow failure violates the no-new-failure gate. No later pass can erase it. Development applied maximum error was 3.6959 to 2.8529 mm at 1.0641 times simulated effort. Evaluation applied maximum error was 3.4116 to 2.7518 mm at 1.0709 times simulated effort. Evaluation remains separate from development. The separately tested WebSocket transport patch honors explicit encoder budgets while preserving the effective 5-second command default and shorter deadlines. One invalid path-rebasing startup is preserved and counted in the hard seven-collection allocation. No full-autocal run with the transport patch was authorized after that startup consumed the optional slot. These hypothetical simulated worlds and software transport checks do not establish hardware reliability.

Topics: HP3, autocalibration, sensor force, motor friction, encoder noise, WebSocket, reply timeout.

## Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| Fixed 4mid improves accuracy across the frozen worlds without new failure or excess effort | n-default, n-candidate, d-default, d-candidate, e-default, e-candidate | Rejected for production; nominal candidate workflow failure remains | [Measurements](measurements.json) |
| Explicit encoder reply budgets can be honored while preserving bounded failures and routing | Fresh-process real WebSocket contrast and regression checks | Supported within software transport scope | [Transport evidence](evidence/transport-summary.json) |

## Decision handoff

- Scope: hypothetical HP3 motors and noisy encoders, known 30 mm radii, zero guide friction/buildup, layering off. Real WebSocket tests use a declared firmware HTTP stub.
- Review: producer hash/readback and numerical recomputation; manager owns independent fixed review and integration.
- Decision: retain 5mid. Deliver only the supported encoder reply-budget patch after software checks pass.
- Next: fixed review of the exact source candidate and evidence. No further primary or confirmation collections are authorized in this attempt.

## References

- [Full report](report.md) separates applied geometry, intermediate fits, effort and transport evidence.
- [Research notebook](research.md) records the frozen protocol and manager decisions.
- [Frozen policy](policy-frozen.json) and [budget ledger](budget-ledger.json) preserve source/native identities and hard caps.
- The original independently reviewed four-attempt partial remains at Git commit 57658240.
