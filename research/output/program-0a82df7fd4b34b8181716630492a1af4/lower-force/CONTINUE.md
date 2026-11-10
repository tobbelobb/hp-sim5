# Frozen lower-force matrix completed

All six primary attempts are complete. Do not repeat n-default, n-candidate, d-default, d-candidate, e-default or e-candidate. Keep production 5mid. The nominal 4mid timeout remains a workflow failure and violates the no-new-failure gate despite development and evaluation accuracy gains.

Evaluation maximum error improved from 3.411564 to 2.751784 mm, and RMSE from 2.834960 to 2.174766 mm, at 1.070851 times simulated effort. Both arms collected 140 points and stopped normally with all anchors below 5 mm. B worsened from 1.140885 to 2.409836 mm. Intermediate tails and all failures remain preserved.

Checkpoint hashes: prior four 53a6a9ba, ddc0e91a, 28b10980 and 9746964a; invalid startup f8d4fd02; evaluation default 0c0de5a9; evaluation candidate 4d144da5. Every raw archive was hash-readback verified and pushed before the next collection. The prior partial report and independent review remain at commit 57658240.

The invalid import-path startup used 71,980 steps, zero points and zero fits. It counts as collection 5. Corrected evaluation used slots 6 and 7, and the optional full-autocal transport confirmation was withdrawn. Aggregate usage is seven collections, 47 fits, 12,651,622 physics steps including the prior 3003-step probe and 30 noncollection verification steps, and 750 physical points. No eighth collection is authorized.

The separate transport contrast used real WebSocket traffic, a declared local firmware HTTP stub and fresh Node processes. With explicit 120-second configured and command budgets, stock failed after 5009.345 ms while the fix returned the identical finite payload after 6018.729 ms. The historical host-delay cause remains unknown. Final code-only candidate 95a0af8f63f4d0d257629ea7eaaa295c2638f276 has direct base a68d6adf. The 45 JavaScript and two headless checks ran on 903b00d4. Only README wording changed afterward; production and test blobs remain identical. No full-autocal-with-fix confirmation is claimed.

Read the [report](report.md), [measurements](measurements.json), [pairs](pairs.json), [budget](budget-ledger.json) and [combined evidence manifest](combined-review-evidence-manifest.json). Full per-run raw archives omit nothing. The compact view omits only duplicate event journals with exact full archive hash references. The manager owns fixed review, integration and controller history exports. No primary research work remains in this allocation.
