# Paused after completed experiment and producer-tested transport patch

The human requested pause and push on 2026-11-05. Do not start more collection.
All six frozen primary attempts are finished. One invalid setup attempt consumed
the spare slot. All seven attempts and full raw evidence are pushed. Final review,
integration checks and production adoption are NOT DONE.

## Concrete results

| World | Stock 5mid maximum mm | Frozen 4mid maximum mm | Candidate simulated effort |
| --- | ---: | ---: | ---: |
| Nominal | 4.5472 | Failed before fitting after 50 points | Not comparable |
| Development | 3.6959 | 2.8529 | 1.064117 times baseline |
| Evaluation | 3.4116 | 2.7518 | 1.070853 times baseline |

All five completed calibrations accepted every anchor below 5 mm. No completed
baseline had an anchor above 20 mm. The nominal candidate transport failure
remains a practical failure and blocks adopting the lower-force default.
Keep production measurement force at 5mid. Do not erase failures or repeat
completed experiments. Per-anchor errors, fits/tails, readbacks, effort and
limitations are in lower-force/report.md and measurements.json.

The invalid path-rebasing startup used 71,980 steps and zero points/fits. It is
preserved separately and counts against the hard seven-collection budget. Total
use is 7 collections, 47 fits, 12,651,622 physics steps including probe and
verification steps, and 750 points. There is NO collection slot left.

## Production candidate, not adopted

Code-only candidate: 95a0af8f63f4d0d257629ea7eaaa295c2638f276.
Explicit base: a68d6adffec3aea86a55a17d6b4d65a27badd42a, still origin/main at last fetch.
Remote code branch: research/encoder-reply-budget-docs-b46f63c3.
Producer branch: research/bounded-research-assignment-ca60a579.
Latest producer report/evidence commit: db16b4ede1ca3d9d97b041433b8b356aae1db4a6.
The combined source/evidence checkpoint is on research/autocal-finish-0a82df7f.
Do not force-push or overwrite newer main commits.

The minimal patch honors explicit finite encoder reply budgets, preserving the
effective five-second command default, shorter command deadlines and request-ID
routing. Real WebSocket tests with a disclosed local firmware HTTP stub reproduced
stock failure at 5009.345 ms versus the identical delayed payload succeeding at
6018.729 ms. The original host-delay cause is unknown. No physics ran in that
contrast, and no full-autocal run with the patch was authorized or performed.

45 JavaScript checks across six suites and two noncollection headless checks
passed at 903b00d47c579bda68425e6000ed16a7341b2813. Candidate 95a0af8f differs
only in README wording; production and test blobs are identical. Actual tested
commits and initial failed checks are preserved. Do not rewrite those receipts
as checks against a later commit. A separate final fixed-evidence review and
exact combined-candidate integration checks remain required.

Final combined compact bundle: lower-force/combined-review-evidence.tar.gz.
SHA256 c015b06839f6cfb9cb03682b54405eb3f8b3be4d11811a3f764a5a550b5c35c2,
515 members, 5,304,588 bytes. Its duplicate view omits large events.jsonl only;
every complete journal remains in the hash-verified pushed per-run raw archives.
Presentation-only corrections followed successful numerical recomputation.

## Review and continuation state

The FOUR-attempt partial alone was independently reviewed and kept, report only:
submission-1b4de3c98dfd474abb162d80f5aa8fcf,
review-6d368c8db8d14e0e9166b495cee60436,
admission-410f1328caf0460aa2142d5339f7ccc2.
It does not admit the later evaluation or transport patch. An earlier partial
review timed out before its actor initialized and produced no verdict; its
cancellation is preserved. Do not confuse it with the completed review above
or repeat the historical preload review.

Latest attempt: attempt-b46f63c3e41b42338c0e719a9837ce52,
assignment-75a29344551b4d1e96b3acb643eced4d.
Session: output/research/sessions/attempt-b46f63c3e41b42338c0e719a9837ce52.
Read its final-message.md and the tracked lower-force report/evidence. On resume,
use the current cloud-research-client skill and supported program controller.
Reconcile current upstream and runtime state. Do not adopt historical PIDs or
restore an old physics world. No numerical experiment needs repeating.

Remaining work is to seal the complete report, candidate95 and selected compact
assets; obtain a separate review; apply the real verdict; integrate against an
explicit current base and run required checks; update final documentation/catalog;
finish cleanup and save/push. Keep force5mid, failures and scope limits explicit.
No hardware-reliability or reduced >20 mm failure-rate claim is supported.
